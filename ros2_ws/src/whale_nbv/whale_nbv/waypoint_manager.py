#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.target_pose_publisher = self.create_publisher(
            PoseStamped,
            '/whale_nbv/target_pose',
            10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback,
            qos_profile
        )

        # Lista waypoint: (x, y, z, yaw)
        self.waypoints = [
            (0.0, 0.0, -5.0, 0.0),
            (2.0, 0.0, -5.0, 0.0),
            (2.0, 2.0, -5.0, math.pi / 2.0),
            (0.0, 2.0, -5.0, math.pi),
        ]

        self.current_index = 0
        self.current_position = None
        self.current_velocity = None

        self.position_tolerance = 0.10   # 10 cm
        self.velocity_tolerance = 0.10   # 10 cm/s

        self.active = True

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info('Waypoint manager avviato.')
        self.get_logger().info(
            f'Tolleranza posizione: {self.position_tolerance} m'
        )
        self.get_logger().info(
            f'Tolleranza velocita: {self.velocity_tolerance} m/s'
        )

        self.publish_current_waypoint()

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        self.current_position = (
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2]),
        )

        self.current_velocity = (
            float(msg.velocity[0]),
            float(msg.velocity[1]),
            float(msg.velocity[2]),
        )

    def yaw_to_quaternion(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw

    def publish_current_waypoint(self):
        x, y, z, yaw = self.waypoints[self.current_index]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.target_pose_publisher.publish(msg)

        self.get_logger().info(
            f'Waypoint pubblicato #{self.current_index}: '
            f'x={x}, y={y}, z={z}, yaw={yaw:.2f}'
        )

    def distance_to_current_waypoint(self):
        if self.current_position is None:
            return None

        x, y, z, _ = self.waypoints[self.current_index]
        cx, cy, cz = self.current_position

        dx = x - cx
        dy = y - cy
        dz = z - cz

        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def speed_norm(self):
        if self.current_velocity is None:
            return None

        vx, vy, vz = self.current_velocity
        return math.sqrt(vx * vx + vy * vy + vz * vz)

    def timer_callback(self):
        if not self.active:
            return

        distance = self.distance_to_current_waypoint()
        speed = self.speed_norm()

        if distance is None or speed is None:
            return

        self.get_logger().info(
            f'Waypoint #{self.current_index} | distanza={distance:.2f} m | velocita={speed:.2f} m/s'
        )

        if distance <= self.position_tolerance and speed <= self.velocity_tolerance:
            self.get_logger().info(
                f'Waypoint #{self.current_index} raggiunto e stabilizzato.'
            )

            if self.current_index < len(self.waypoints) - 1:
                self.current_index += 1
                self.publish_current_waypoint()
            else:
                self.get_logger().info(
                    'Ultimo waypoint raggiunto. Missione completata.'
                )
                self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C ricevuto: chiusura waypoint manager.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()