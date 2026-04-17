#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, Float32
from px4_msgs.msg import VehicleOdometry

from whale_nbv.utils import (
    quaternion_to_yaw,
    yaw_to_quaternion,
    quat_wxyz_to_rotmat,
    make_camera_matrix,
    pixel_to_camera_ray,
    intersect_ray_with_plane,
)


class ArucoController(Node):
    def __init__(self):
        super().__init__('aruco_controller')

        # -----------------------------
        # State ArUco
        # -----------------------------
        self.aruco_detected = False
        self.aruco_center_u = 0.0
        self.aruco_center_v = 0.0
        self.aruco_confidence = 0.0

        # -----------------------------
        # State mission/ goal
        # -----------------------------
        self.goal_reached = False
        self.goal_z = -5.0
        self.has_goal = False

        # -----------------------------
        # State drone from odometry PX4
        # -----------------------------
       
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.t_wb = np.zeros(3, dtype=float)   # position body in world PX4
        self.R_wb = np.eye(3, dtype=float)     # rotation body -> world
        self.has_odometry = False

        # -----------------------------
        # Camera Parameters
        # -----------------------------
        self.image_width = 640.0
        self.image_height = 480.0
        self.hfov = 1.047  # rad

        self.cx = self.image_width / 2.0
        self.cy = self.image_height / 2.0
        self.fx = self.image_width / (2.0 * math.tan(self.hfov / 2.0))
        self.fy = self.fx

        self.K = make_camera_matrix(self.fx, self.fy, self.cx, self.cy)

        # Camera optical frame C -> body frame B (PX4 FRD)
        # Assumptions downward camera:
        # x_C = right image
        # y_C = down image
        # z_C = optical axis (downward toward ground)
        #
        # In body FRD:
        # x_B = forward
        # y_B = right
        # z_B = down
        #
        # Mapping:
        # z_C -> z_B
        # x_C -> y_B
        # y_C -> -x_B

        self.R_bc = np.array([
            [0.0, -1.0,  0.0],
            [1.0,  0.0,  0.0],
            [0.0,  0.0,  1.0],
        ], dtype=float)

        # Traslation camera respect to body in frame body FRD
        # SDF: x=0.12, y=0, z=0.02 in frame type base_link Gazebo (z up).
        # In FRD : forward 0.12, right 0.0, down -0.02

        self.t_bc = np.array([0.12, 0.0, -0.02], dtype=float)

       
        # Plane known in world PX4
        # Horizontal plane considered: z = 0

        self.plane_normal_w = np.array([0.0, 0.0, 1.0], dtype=float)
        self.plane_offset_d = 0.0


        self.dry_run = False

        # -----------------------------
        # Parameters control
        # -----------------------------
        self.confidence_target = 0.65
        self.min_confidence_to_move = 0.20


        # Gain on the plane
        self.k_xy = 0.20

        # Step Saturation 
        self.max_step_xy_far = 0.10
        self.max_step_xy_mid = 0.06
        self.max_step_xy_near = 0.03

        self.step_mid_distance = 1.0
        self.step_near_distance = 0.30

        # Delay between commands
        self.command_cooldown_ns = int(2.0 * 1e9)
        self.last_command_time_ns = 0

        # After one command, it waits again goal_reached=True
        self.waiting_after_command = False

        # -----------------------------
        # Publisher target
        # -----------------------------
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/whale_nbv/goal_pose',
            10
        )

        # -----------------------------
        # Subscriber detector
        # -----------------------------
        self.detected_sub = self.create_subscription(
            Bool,
            '/aruco/detected',
            self.detected_callback,
            10
        )

        self.center_sub = self.create_subscription(
            Point,
            '/aruco/center',
            self.center_callback,
            10
        )

        self.confidence_sub = self.create_subscription(
            Float32,
            '/aruco/confidence',
            self.confidence_callback,
            10
        )

        # -----------------------------
        # Subscriber state mission
        # -----------------------------
        self.goal_reached_sub = self.create_subscription(
            Bool,
            '/whale_nbv/goal_reached',
            self.goal_reached_callback,
            10
        )

        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/whale_nbv/goal_pose',
            self.goal_pose_callback,
            10
        )

        # -----------------------------
        # Subscriber odometry PX4
        # -----------------------------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback,
            qos_profile
        )

        # -----------------------------
        # Timer control
        # -----------------------------
        self.timer = self.create_timer(0.5, self.control_loop)

        self.get_logger().info('Started Aruco controller.')
        self.get_logger().info('Control with monocular camera + known plane.')
        self.get_logger().info('Publishes target on /whale_nbv/goal_pose')

    # =========================================================
    # CALLBACK
    # =========================================================

    def detected_callback(self, msg: Bool):
        self.aruco_detected = msg.data

    def center_callback(self, msg: Point):
        self.aruco_center_u = float(msg.x)
        self.aruco_center_v = float(msg.y)

    def confidence_callback(self, msg: Float32):
        self.aruco_confidence = float(msg.data)

    def goal_reached_callback(self, msg: Bool):
        self.goal_reached = msg.data
        if self.goal_reached:
            self.waiting_after_command = False

    def goal_pose_callback(self, msg: PoseStamped):
        self.goal_z = float(msg.pose.position.z)
        self.has_goal = True

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        self.current_x = float(msg.position[0])
        self.current_y = float(msg.position[1])
        self.current_z = float(msg.position[2])

        self.t_wb = np.array([
            self.current_x,
            self.current_y,
            self.current_z
        ], dtype=float)

        qw = float(msg.q[0])
        qx = float(msg.q[1])
        qy = float(msg.q[2])
        qz = float(msg.q[3])

        self.R_wb = quat_wxyz_to_rotmat(qw, qx, qy, qz)
        self.current_yaw = quaternion_to_yaw(qx, qy, qz, qw)

        self.has_odometry = True

    # =========================================================
    # GEOMETRY
    # =========================================================


    
    def compute_camera_pose_in_world(self):
        R_wc = self.R_wb @ self.R_bc
        t_wc = self.t_wb + self.R_wb @ self.t_bc
        return R_wc, t_wc


    def intersect_pixel_with_plane(self, u: float, v: float):
        if not self.has_odometry:
            return None

        ray_c = pixel_to_camera_ray(u, v, self.K)

        R_wc, t_wc = self.compute_camera_pose_in_world()
        ray_w = R_wc @ ray_c

        point_w = intersect_ray_with_plane(
            ray_origin_w=t_wc,
            ray_dir_w=ray_w,
            plane_normal_w=self.plane_normal_w,
            plane_offset_d=self.plane_offset_d
        )
        return point_w


    def compute_world_delta_from_image(self):
        p_target = self.intersect_pixel_with_plane(self.aruco_center_u, self.aruco_center_v)
        if p_target is None:
            self.get_logger().warn('Intersection target-plane not valid.')
            return None

        p_center = self.intersect_pixel_with_plane(self.cx, self.cy)
        if p_center is None:
            self.get_logger().warn('Intersection center-plane not valid.')
            return None

        delta_w = p_target - p_center
        return p_target, p_center, delta_w
        
    def shape_xy_step(self, delta_x: float, delta_y: float):
        dist = math.sqrt(delta_x * delta_x + delta_y * delta_y)

        if dist < 1e-9:
            return 0.0, 0.0, dist, 0.0

        raw_dx = self.k_xy * delta_x
        raw_dy = self.k_xy * delta_y
        raw_norm = math.sqrt(raw_dx * raw_dx + raw_dy * raw_dy)

        if dist > self.step_mid_distance:
            max_step = self.max_step_xy_far
        elif dist > self.step_near_distance:
            max_step = self.max_step_xy_mid
        else:
            max_step = self.max_step_xy_near

        if raw_norm <= max_step:
            return raw_dx, raw_dy, dist, max_step

        scale = max_step / raw_norm
        dx = raw_dx * scale
        dy = raw_dy * scale
        return dx, dy, dist, max_step    

    # =========================================================
    # CONTROL LOOP
    # =========================================================

    def control_loop(self):
        now_ns = self.get_clock().now().nanoseconds

        if not self.has_goal:
            return

        if not self.has_odometry:
            return

        if not self.goal_reached:
            return

        if not self.aruco_detected:
            self.get_logger().info('Goal reached but marker non relieved: waiting...')
            return

        if self.waiting_after_command:
            return

        if now_ns - self.last_command_time_ns < self.command_cooldown_ns:
            return

        self.get_logger().info(
            f'State ArUco | center=({self.aruco_center_u:.1f}, {self.aruco_center_v:.1f}) '
            f'confidence={self.aruco_confidence:.3f} '
            f'odom=({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) '
            f'yaw={self.current_yaw:.2f}'
        )

        if self.aruco_confidence >= self.confidence_target:
            self.get_logger().info(
                f'Confidence {self.aruco_confidence:.3f} >= target {self.confidence_target:.3f}. '
                f'No new movement.'
            )
            return

        if self.aruco_confidence < self.min_confidence_to_move:
            self.get_logger().warn(
                f'Confidence too low ({self.aruco_confidence:.3f}) to move in reliable way.'
            )
            return

        result = self.compute_world_delta_from_image()
        if result is None:
            return

        p_target, p_center, delta_w = result

        delta_x = float(delta_w[0])
        delta_y = float(delta_w[1])

        self.get_logger().info(
            f'Geometry ray-plane | '
            f'P_target=({p_target[0]:.3f}, {p_target[1]:.3f}, {p_target[2]:.3f}) | '
            f'P_center=({p_center[0]:.3f}, {p_center[1]:.3f}, {p_center[2]:.3f}) | '
            f'delta_w=({delta_x:.3f}, {delta_y:.3f}, {delta_w[2]:.3f})'
        )

        if self.dry_run:
            self.get_logger().info('DRY RUN active: do not publish new goal.')
            return

        dx_world, dy_world, dist_xy, max_step_used = self.shape_xy_step(delta_x, delta_y)

        if abs(dx_world) < 1e-6 and abs(dy_world) < 1e-6:
            self.get_logger().info('Negligible correction: no new goal.')
            return

        new_goal_x = self.current_x + dx_world
        new_goal_y = self.current_y + dy_world
        new_goal_z = self.goal_z
        new_goal_yaw = self.current_yaw

        self.publish_goal(new_goal_x, new_goal_y, new_goal_z, new_goal_yaw)

        self.waiting_after_command = True
        self.last_command_time_ns = now_ns

        self.get_logger().info(
            f'Published new goal | '
            f'x={new_goal_x:.2f}, y={new_goal_y:.2f}, z={new_goal_z:.2f}, yaw={new_goal_yaw:.2f} | '
            f'dx={dx_world:.3f}, dy={dy_world:.3f} | '
            f'dist_xy={dist_xy:.3f}, max_step={max_step_used:.3f}'
        )

    # =========================================================
    # PUBLISH
    # =========================================================

    def publish_goal(self, x: float, y: float, z: float, yaw: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        self.goal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()