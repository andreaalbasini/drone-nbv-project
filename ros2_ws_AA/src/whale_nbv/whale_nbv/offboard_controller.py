#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleOdometry

from whale_nbv.utils import quaternion_to_yaw, distance_3d, speed_norm, ros_time_us


class OffboardController(Node):
    def __init__(self):
        super().__init__('offboard_controller')

        # Target
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0
        self.has_received_target = False

        # Actual State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self.has_odometry = False

        # Goal check
        self.goal_tolerance = 0.30
        self.velocity_tolerance = 0.20
        self.goal_reached = False
        self.goal_reached_reported = False

        # Debug log every second
        self.last_debug_time_ns = 0
        self.debug_period_ns = int(1.0 * 1e9)
        
        self.goal_reached_time_ns = 0
        self.stabilization_delay_ns = int(1.5 * 1e9)  # 1.5 secondi di stabilizzazione

        # Publisher PX4
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # Publisher mission state
        self.goal_reached_publisher = self.create_publisher(
            Bool,
            '/whale_nbv/goal_reached',
            10
        )

        # Subscriber goal
        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/whale_nbv/goal_pose',
            self.goal_pose_callback,
            10
        )

        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber odometry
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            self.vehicle_odometry_callback,
            qos_profile
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.counter = 0
        self.offboard_enabled = False
        self.armed = False
        self.waiting_log_printed = False

        self.get_logger().info('Offboard controller started.')
        self.get_logger().info('Waiting the first goal pose on /whale_nbv/goal_pose ...')

        self.mission_complete_sub = self.create_subscription(
            Bool, '/whale_nbv/mission_complete',
            self.mission_complete_callback, 10)

        self.rtl_active = False
        self.home_z = -4.0  # quota di rientro sicura
        self.rtl_phase = 0       # 1 = sposta XY, 2 = invia LAND
        self.landing_sent = False

    def mission_complete_callback(self, msg: Bool):
        if msg.data and not self.rtl_active:
            self.rtl_active = True
            self.rtl_phase = 1
            self.get_logger().info('RTL fase 1: torno su XY mantenendo quota attuale')
            self.target_x = 0.0
            self.target_y = 0.0
            self.target_z = self.current_z
            self.target_yaw = 0.0
            self.goal_reached = False
            self.goal_reached_reported = False

    def goal_pose_callback(self, msg: PoseStamped):
        self.target_x = float(msg.pose.position.x)
        self.target_y = float(msg.pose.position.y)
        self.target_z = float(msg.pose.position.z)

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.target_yaw = quaternion_to_yaw(qx, qy, qz, qw)

        first_target = not self.has_received_target
        self.has_received_target = True

        self.goal_reached = False
        self.goal_reached_time_ns = 0  # reset timer stabilizzazione
        self.goal_reached_reported = False
        self.publish_goal_reached(False)

        self.get_logger().info(
            f'New target: x={self.target_x:.2f}, y={self.target_y:.2f}, '
            f'z={self.target_z:.2f}, yaw={self.target_yaw:.2f}'
        )

        if first_target:
            self.counter = 0
            self.offboard_enabled = False
            self.armed = False
            self.get_logger().info('Received first target: starting sequence OFFBOARD.')

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        self.current_x = float(msg.position[0])
        self.current_y = float(msg.position[1])
        self.current_z = float(msg.position[2])

        self.current_vx = float(msg.velocity[0])
        self.current_vy = float(msg.velocity[1])
        self.current_vz = float(msg.velocity[2])

        self.has_odometry = True

    def compute_distance_to_goal(self) -> float:
        return distance_3d(
            self.current_x, self.current_y, self.current_z,
            self.target_x, self.target_y, self.target_z
        )

    def compute_speed_norm(self) -> float:
        return speed_norm(self.current_vx, self.current_vy, self.current_vz)

    def maybe_print_debug(self, distance: float, speed: float):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_debug_time_ns >= self.debug_period_ns:
            self.get_logger().info(
                f'Goal State | distance={distance:.2f} m | '
                f'speed={speed:.2f} m/s | reached={self.goal_reached}'
            )
            self.last_debug_time_ns = now_ns

    def timer_callback(self):
        if not self.has_received_target:
            if not self.waiting_log_printed:
                self.get_logger().info('Waiting goal_pose before sending setpoint to PX4.')
                self.waiting_log_printed = True
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

        if self.has_odometry:
            distance = self.compute_distance_to_goal()
            speed = self.compute_speed_norm()

            if distance <= self.goal_tolerance:
                   now_ns = self.get_clock().now().nanoseconds
                   if self.goal_reached_time_ns == 0:
                          self.goal_reached_time_ns = now_ns  # inizia il timer
    
                   elapsed = now_ns - self.goal_reached_time_ns
                   if elapsed >= self.stabilization_delay_ns:
                          self.goal_reached = True
                          self.publish_goal_reached(True)
                          if not self.goal_reached_reported:
                                 self.get_logger().info(
                                      f'Goal reached | distance={distance:.2f} m | '
                                      f'stabilized for {elapsed/1e9:.1f}s'
                                 )  
                                 self.goal_reached_reported = True
            else:
                  # Drone lontano dal goal — reset timer
                  self.goal_reached_time_ns = 0
                  self.goal_reached = False
                  self.publish_goal_reached(False)

            self.maybe_print_debug(distance, speed)

        if self.rtl_active and self.goal_reached:
            if self.rtl_phase == 1:
                self.rtl_phase = 2
                self.get_logger().info('RTL fase 2: invio LAND')
                self.target_z = self.home_z  # scendi alla quota hover prima del land
                self.goal_reached = False
                self.goal_reached_reported = False

            elif self.rtl_phase == 2 and not self.landing_sent:
                self.landing_sent = True
                self.get_logger().info('Quota raggiunta: invio LAND')
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        self.counter += 1

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
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )

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
