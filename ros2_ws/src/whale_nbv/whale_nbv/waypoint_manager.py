#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.goal_pose_publisher = self.create_publisher(
            PoseStamped,
            '/whale_nbv/goal_pose',
            10
        )

        self.goal_reached_subscriber = self.create_subscription(
            Bool,
            '/whale_nbv/goal_reached',
            self.goal_reached_callback,
            10
        )

        # Lista waypoint: (x, y, z, yaw)
        self.waypoints = [
            (0.0, 0.0, -5.0, 0.0),
            (2.0, 0.0, -5.0, 0.0),
            (2.0, 2.0, -5.0, math.pi / 2.0),
            (0.0, 2.0, -5.0, math.pi),
        ]

        self.current_index = 0
        self.active = True

        self.waiting_for_goal = False
        self.last_goal_reached = False

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info('Waypoint manager avviato.')
        self.get_logger().info(f'Numero waypoint caricati: {len(self.waypoints)}')

        self.publish_current_waypoint()

    def goal_reached_callback(self, msg: Bool):
        self.last_goal_reached = bool(msg.data)

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

        self.goal_pose_publisher.publish(msg)

        self.waiting_for_goal = True
        self.last_goal_reached = False

        self.get_logger().info(
            f'Waypoint pubblicato #{self.current_index}: '
            f'x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}'
        )

    def timer_callback(self):
        if not self.active:
            return

        if not self.waiting_for_goal:
            return

        if self.last_goal_reached:
            self.get_logger().info(
                f'Waypoint #{self.current_index} raggiunto.'
            )

            if self.current_index < len(self.waypoints) - 1:
                self.current_index += 1
                self.publish_current_waypoint()
            else:
                self.get_logger().info(
                    'Ultimo waypoint raggiunto. Missione completata.'
                )
                self.active = False
                self.waiting_for_goal = False


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