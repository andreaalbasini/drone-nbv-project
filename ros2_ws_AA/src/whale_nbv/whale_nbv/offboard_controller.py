#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleOdometry

from whale_nbv.utils import quaternion_to_yaw, distance_3d, speed_norm, ros_time_us

# Controlled descent speed [m/s in NED — positive = descending]
LAND_SPEED = 0.30
# Margin above ground_z: stops descending and pushes to ground_z [m]
LAND_STOP_MARGIN = 0.15
# Maximum descent timeout [s]: after this time the waiting phase begins
LAND_TIMEOUT_S = 30.0
# Extra seconds publishing setpoint at ground_z after stopping the descent
# (gives PX4 time to touch ground and auto-disarm before we stop)
POST_LAND_WAIT_S = 8.0


class OffboardController(Node):
    def __init__(self):
        super().__init__('offboard_controller')

        # Target
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0
        self.has_received_target = False

        # Home: saved from the first goal, used for RTL
        self.home_x = None
        self.home_y = None
        self.home_z = None

        # Actual State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self.has_odometry = False

        # Auto-hover
        self.declare_parameter('auto_hover_z', float('nan'))
        self.auto_hover_z = self.get_parameter('auto_hover_z').value
        self.auto_hover_generated = False

        # Goal check
        self.goal_tolerance_xy = 0.15
        self.goal_tolerance_z  = 0.20
        self.velocity_tolerance = 0.20
        self.goal_reached = False
        self.goal_reached_reported = False

        # Debug log every second
        self.last_debug_time_ns = 0
        self.debug_period_ns = int(1.0 * 1e9)

        # RTL / Landing state
        self.rtl_active = False
        self.rtl_phase = 0          # 1 = return to home, 2 = controlled descent
        self.landing = False
        self.landing_z_target = None
        self.landing_start_time = None
        self.post_landing_start = None
        self.ground_z = None        # received from nbv_planner
        self.mission_done = False   # when True stops publishing setpoints

        # Publisher PX4
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.goal_reached_publisher = self.create_publisher(
            Bool, '/whale_nbv/goal_reached', 10)

        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped, '/whale_nbv/goal_pose', self.goal_pose_callback, 10)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback, qos_profile)

        self.mission_complete_sub = self.create_subscription(
            Bool, '/whale_nbv/mission_complete', self.mission_complete_callback, 10)
        self.land_now_sub = self.create_subscription(
            Bool, '/whale_nbv/land_now', self.land_now_callback, 10)
        self.ground_z_sub = self.create_subscription(
            Float32, '/whale_nbv/ground_z', self.ground_z_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.counter = 0
        self.offboard_enabled = False
        self.armed = False
        self.waiting_log_printed = False

        if not math.isnan(self.auto_hover_z):
            self.get_logger().info(
                f'Auto-hover mode: will climb to z={self.auto_hover_z:.2f} m (NED) '
                f'from current position as soon as odometry is received.')
        else:
            self.get_logger().info(
                'Offboard controller started. Waiting for goal on /whale_nbv/goal_pose ...')

    # ── Callback ────────────────────────────────────────────────────────────

    def ground_z_callback(self, msg: Float32):
        self.ground_z = float(msg.data)

    def land_now_callback(self, msg: Bool):
        if msg.data and not self.landing and not self.mission_done:
            self.get_logger().info('LAND NOW received — starting controlled descent.')
            self._start_landing()

    def mission_complete_callback(self, msg: Bool):
        if msg.data and not self.rtl_active:
            self.rtl_active = True
            self.rtl_phase = 1
            home_x = self.home_x if self.home_x is not None else self.current_x
            home_y = self.home_y if self.home_y is not None else self.current_y
            rtl_z = self.home_z if self.home_z is not None else self.current_z
            self.get_logger().info(
                f'Mission complete: RTL phase 1 — returning to home ({home_x:.2f}, {home_y:.2f}) at z={rtl_z:.2f}m'
            )
            self._set_target(home_x, home_y, rtl_z, self.current_yaw)

    def goal_pose_callback(self, msg: PoseStamped):
        if self.mission_done or self.landing or self.rtl_active:
            return

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        first_target = not self.has_received_target
        if first_target:
            self.home_x = float(msg.pose.position.x)
            self.home_y = float(msg.pose.position.y)
            self.home_z = float(msg.pose.position.z)

        self._set_target(
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            quaternion_to_yaw(qx, qy, qz, qw)
        )

        if first_target:
            self.counter = 0
            self.offboard_enabled = False
            self.armed = False
            self.get_logger().info(
                f'First goal received: home=({self.home_x:.2f}, {self.home_y:.2f}). Starting OFFBOARD sequence.'
            )

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        self.current_x = float(msg.position[0])
        self.current_y = float(msg.position[1])
        self.current_z = float(msg.position[2])
        self.current_vx = float(msg.velocity[0])
        self.current_vy = float(msg.velocity[1])
        self.current_vz = float(msg.velocity[2])
        self.current_yaw = quaternion_to_yaw(
            float(msg.q[1]), float(msg.q[2]), float(msg.q[3]), float(msg.q[0]))
        self.has_odometry = True

        if (not math.isnan(self.auto_hover_z)
                and not self.auto_hover_generated
                and not self.has_received_target):
            self.auto_hover_generated = True
            self.home_x = self.current_x
            self.home_y = self.current_y
            goal_z = self.current_z + self.auto_hover_z
            self._set_target(self.current_x, self.current_y, goal_z, self.current_yaw)
            self.counter = 0
            self.offboard_enabled = False
            self.armed = False
            self.get_logger().info(
                f'Auto-hover goal: ({self.current_x:.2f}, {self.current_y:.2f}, '
                f'{goal_z:.2f}) [ground_z={self.current_z:.2f} + offset={self.auto_hover_z:.2f}], '
                f'yaw={self.current_yaw:.2f}')

    # ── Helper ──────────────────────────────────────────────────────────────

    def _set_target(self, x: float, y: float, z: float, yaw: float):
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_yaw = yaw
        self.has_received_target = True
        self.goal_reached = False
        self.goal_reached_reported = False
        self.publish_goal_reached(False)
        self.get_logger().info(
            f'Target: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}')

    def _start_landing(self):
        self.landing = True
        self.landing_z_target = float(self.current_z)
        self.landing_start_time = time.monotonic()
        home_x = self.home_x if self.home_x is not None else self.current_x
        home_y = self.home_y if self.home_y is not None else self.current_y
        self.get_logger().info(
            f'RTL phase 2: controlled OFFBOARD descent at {LAND_SPEED:.1f} m/s '
            f'from z={self.landing_z_target:.2f} | ground_z={self.ground_z}')
        # Keep XY fixed at home during descent
        self.target_x = home_x
        self.target_y = home_y

    def _update_landing(self):
        elapsed = time.monotonic() - self.landing_start_time

        if self.post_landing_start is not None:
            # Phase 2: push the drone toward the ground, wait for it to actually land
            if self.ground_z is not None:
                self.target_z = self.ground_z
            post_elapsed = time.monotonic() - self.post_landing_start
            if post_elapsed >= POST_LAND_WAIT_S:
                self.get_logger().info(
                    f'Post-landing complete ({post_elapsed:.1f}s). Stopping setpoint.')
                self.mission_done = True
                self.landing = False
            return

        # Phase 1: controlled descent
        self.landing_z_target += LAND_SPEED * 0.1  # 0.1 s per tick
        self.target_z = self.landing_z_target

        near_ground = (
            self.ground_z is not None and
            self.landing_z_target >= self.ground_z - LAND_STOP_MARGIN
        )
        timeout = elapsed >= LAND_TIMEOUT_S

        if near_ground or timeout:
            reason = 'ground near' if near_ground else f'timeout {elapsed:.0f}s'
            self.get_logger().info(
                f'Landing phase 2 ({reason}): target_z → ground_z={self.ground_z}, '
                f'waiting {POST_LAND_WAIT_S:.0f}s before stopping setpoint.')
            self.post_landing_start = time.monotonic()
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def compute_distance_to_goal(self) -> float:
        return distance_3d(
            self.current_x, self.current_y, self.current_z,
            self.target_x, self.target_y, self.target_z)

    def compute_speed_norm(self) -> float:
        return speed_norm(self.current_vx, self.current_vy, self.current_vz)

    def maybe_print_debug(self, distance: float, speed: float):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_debug_time_ns >= self.debug_period_ns:
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            dz = self.target_z - self.current_z
            self.get_logger().info(
                f'Pos  curr=({self.current_x:+.2f}, {self.current_y:+.2f}, {self.current_z:+.2f}) '
                f'goal=({self.target_x:+.2f}, {self.target_y:+.2f}, {self.target_z:+.2f})'
            )
            self.get_logger().info(
                f'Err  dx={dx:+.2f} dy={dy:+.2f} dz={dz:+.2f} | '
                f'dist={distance:.2f}m speed={speed:.2f}m/s reached={self.goal_reached}'
            )
            self.last_debug_time_ns = now_ns

    # ── Main timer ──────────────────────────────────────────────────────────

    def timer_callback(self):
        try:
            self._timer_callback_impl()
        except Exception as e:
            self.get_logger().error(f'CRASH in timer_callback: {e}', throttle_duration_sec=2.0)

    def _timer_callback_impl(self):
        if self.mission_done:
            return

        if not self.has_received_target:
            if not self.waiting_log_printed:
                self.get_logger().info('Waiting for the first goal...')
                self.waiting_log_printed = True
            return

        if self.landing:
            self._update_landing()
            if self.mission_done:
                return

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.counter == 10 and not self.offboard_enabled:
            self.engage_offboard_mode()
            self.offboard_enabled = True
            self.get_logger().info('Sent OFFBOARD command.')

        if self.counter == 12 and not self.armed:
            self.arm()
            self.armed = True
            self.get_logger().info('Sent ARM command.')

        if self.has_odometry and not self.landing:
            distance = self.compute_distance_to_goal()
            speed = self.compute_speed_norm()

            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            dz = self.target_z - self.current_z
            dist_xy = math.hypot(dx, dy)
            dist_z  = abs(dz)

            if (dist_xy <= self.goal_tolerance_xy
                    and dist_z  <= self.goal_tolerance_z
                    and speed   <= self.velocity_tolerance):
                self.goal_reached = True
                self.publish_goal_reached(True)
                if not self.goal_reached_reported:
                    self.get_logger().info(
                        f'Goal reached | xy={dist_xy:.2f} z={dist_z:.2f} m | speed={speed:.2f} m/s'
                        f' | curr_z={self.current_z:.2f} target_z={self.target_z:.2f}')
                    self.goal_reached_reported = True
            else:
                self.goal_reached = False
                self.goal_reached_reported = False
                self.publish_goal_reached(False)

            self.maybe_print_debug(distance, speed)

        if self.rtl_active and self.goal_reached and self.rtl_phase == 1:
            self.rtl_phase = 2
            self.goal_reached = False
            self.goal_reached_reported = False
            self._start_landing()

        self.counter += 1

    # ── Publish helpers ─────────────────────────────────────────────────────

    def publish_goal_reached(self, value: bool):
        msg = Bool()
        msg.data = value
        self.goal_reached_publisher.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = ros_time_us(self)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = ros_time_us(self)
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = self.target_yaw
        self.trajectory_setpoint_publisher.publish(msg)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = ros_time_us(self)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
