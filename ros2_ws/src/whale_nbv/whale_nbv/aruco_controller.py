#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, Float32
from px4_msgs.msg import VehicleOdometry

from whale_nbv.utils import quaternion_to_yaw


class ArucoController(Node):
    def __init__(self):
        super().__init__('aruco_controller')

        # -----------------------------
        # Stato ArUco
        # -----------------------------
        self.aruco_detected = False
        self.aruco_center_u = 0.0
        self.aruco_center_v = 0.0
        self.aruco_confidence = 0.0

        # -----------------------------
        # Stato missione / goal
        # -----------------------------
        self.goal_reached = False
        self.goal_z = -5.0
        self.has_goal = False

        # -----------------------------
        # Stato drone da odometria PX4
        # -----------------------------
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.has_odometry = False

        # -----------------------------
        # Parametri camera
        # -----------------------------
        self.image_width = 640.0
        self.image_height = 480.0
        self.hfov = 1.047  # rad

        self.cx = self.image_width / 2.0
        self.cy = self.image_height / 2.0
        self.fx = self.image_width / (2.0 * math.tan(self.hfov / 2.0))
        self.fy = self.fx

        # -----------------------------
        # Piano noto
        # In simulazione: ArUco a terra, piano z = 0
        # -----------------------------
        self.plane_z = 0.0

        # -----------------------------
        # Parametri controllo
        # -----------------------------
        self.confidence_target = 0.75
        self.min_confidence_to_move = 0.20

        self.pixel_deadband = 15.0

        # Guadagno metrico sul piano
        self.k_xy = 0.35

        # Saturazione passo
        self.max_step_xy = 0.15

        # Delay tra un comando e il successivo
        self.command_cooldown_ns = int(2.0 * 1e9)
        self.last_command_time_ns = 0

        # Antirimbalzo: dopo un comando, aspetta di nuovo goal_reached=True
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
        # Subscriber stato missione
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
        # Subscriber odometria PX4
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
        # Timer controllo
        # -----------------------------
        self.timer = self.create_timer(0.5, self.control_loop)

        self.get_logger().info('Aruco controller avviato.')
        self.get_logger().info('Controllo metrico con camera monoculare + piano noto.')
        self.get_logger().info('Pubblica target su /whale_nbv/goal_pose')

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

        qx = float(msg.q[0])
        qy = float(msg.q[1])
        qz = float(msg.q[2])
        qw = float(msg.q[3])
        self.current_yaw = quaternion_to_yaw(qx, qy, qz, qw)

        self.has_odometry = True

    # =========================================================
    # GEOMETRIA
    # =========================================================

    def altitude_above_plane(self) -> float:
        # In PX4/NED tipicamente z negativa sopra il piano z=0
        return abs(self.current_z - self.plane_z)

    def pixel_to_camera_ground_offset(self, u: float, v: float, height_m: float):
        """
        Converte il centro detection (u,v) in offset metrico sul piano,
        nel frame camera/body locale, assumendo camera verso il basso
        e piano noto.
        """
        du = u - self.cx
        dv = v - self.cy

        if abs(du) < self.pixel_deadband:
            du = 0.0
        if abs(dv) < self.pixel_deadband:
            dv = 0.0

        x_cam = du / self.fx
        y_cam = dv / self.fy

        offset_u_m = x_cam * height_m
        offset_v_m = y_cam * height_m

        return offset_u_m, offset_v_m

    def camera_offsets_to_world_offsets(self, offset_u_m: float, offset_v_m: float):
        """
        Mapping empirico iniziale:
        - u (destra immagine) influenza asse laterale
        - v (basso immagine) influenza asse longitudinale

        Prima costruiamo un offset nel frame orizzontale locale del drone,
        poi lo ruotiamo nel world con la yaw.
        """

        # Segni empirici iniziali:
        # se marker è a destra, il drone deve andare a destra
        # se marker è in basso, il drone deve andare "indietro/avanti"
        # Questi segni possono richiedere una singola inversione empirica.
        dx_body = offset_v_m
        dy_body = -offset_u_m

        c = math.cos(self.current_yaw)
        s = math.sin(self.current_yaw)

        dx_world = c * dx_body - s * dy_body
        dy_world = s * dx_body + c * dy_body

        return dx_world, dy_world

    def compute_world_step(self):
        if not self.has_odometry:
            return None

        h = self.altitude_above_plane()
        if h < 0.05:
            self.get_logger().warn('Quota sopra il piano troppo piccola.')
            return None

        offset_u_m, offset_v_m = self.pixel_to_camera_ground_offset(
            self.aruco_center_u,
            self.aruco_center_v,
            h
        )

        dx_world, dy_world = self.camera_offsets_to_world_offsets(offset_u_m, offset_v_m)

        # Guadagno
        dx_world *= self.k_xy
        dy_world *= self.k_xy

        # Saturazione
        dx_world = max(-self.max_step_xy, min(self.max_step_xy, dx_world))
        dy_world = max(-self.max_step_xy, min(self.max_step_xy, dy_world))

        return dx_world, dy_world, h, offset_u_m, offset_v_m

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
            self.get_logger().info('Goal raggiunto ma marker non rilevato: aspetto...')
            return

        if self.waiting_after_command:
            return

        if now_ns - self.last_command_time_ns < self.command_cooldown_ns:
            return

        self.get_logger().info(
            f'Stato ArUco | center=({self.aruco_center_u:.1f}, {self.aruco_center_v:.1f}) '
            f'confidence={self.aruco_confidence:.3f} '
            f'odom=({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) '
            f'yaw={self.current_yaw:.2f}'
        )

        if self.aruco_confidence >= self.confidence_target:
            self.get_logger().info(
                f'Confidence {self.aruco_confidence:.3f} >= target {self.confidence_target:.3f}. '
                f'Nessun nuovo movimento.'
            )
            return

        if self.aruco_confidence < self.min_confidence_to_move:
            self.get_logger().warn(
                f'Confidence troppo bassa ({self.aruco_confidence:.3f}) per muovere in modo affidabile.'
            )
            return

        result = self.compute_world_step()
        if result is None:
            return

        dx_world, dy_world, h, offset_u_m, offset_v_m = result

        if abs(dx_world) < 1e-6 and abs(dy_world) < 1e-6:
            self.get_logger().info('Marker gia` vicino al centro immagine: nessun nuovo movimento.')
            return

        # IMPORTANTISSIMO:
        # il nuovo goal parte dalla posa ATTUALE del drone, non dal vecchio goal
        new_goal_x = self.current_x + dx_world
        new_goal_y = self.current_y + dy_world
        new_goal_z = self.goal_z
        new_goal_yaw = 0.0

        self.publish_goal(new_goal_x, new_goal_y, new_goal_z, new_goal_yaw)

        self.waiting_after_command = True
        self.last_command_time_ns = now_ns

        self.get_logger().info(
            f'Nuovo goal metrico pubblicato: '
            f'x={new_goal_x:.2f}, y={new_goal_y:.2f}, z={new_goal_z:.2f} | '
            f'dx_world={dx_world:.3f}, dy_world={dy_world:.3f} | '
            f'offset_u_m={offset_u_m:.3f}, offset_v_m={offset_v_m:.3f} | '
            f'h={h:.2f}'
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

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

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