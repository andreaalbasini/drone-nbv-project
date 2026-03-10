#!/usr/bin/env python3

import math
import signal
import sys
import threading
import termios
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand


class OffboardController(Node):
    def __init__(self):
        super().__init__('offboard_controller')

        # Parametri di fallback iniziali
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', -5.0)
        self.declare_parameter('target_yaw', 0.0)

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_z = float(self.get_parameter('target_z').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)

        self.get_logger().info(
            f'Target iniziale: x={self.target_x}, y={self.target_y}, z={self.target_z}, yaw={self.target_yaw}'
        )

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

        self.target_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/whale_nbv/target_pose',
            self.target_pose_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.counter = 0
        self.offboard_enabled = False
        self.armed = False
        self.landing_requested = False
        self.exit_requested = False

        self.get_logger().info('Offboard controller avviato.')
        self.get_logger().info("Premi 'l' per LAND, 'q' per uscire.")

    def target_pose_callback(self, msg: PoseStamped):
        self.target_x = float(msg.pose.position.x)
        self.target_y = float(msg.pose.position.y)
        self.target_z = float(msg.pose.position.z)

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        self.target_yaw = self.quaternion_to_yaw(qx, qy, qz, qw)

        self.get_logger().info(
            f'Nuovo target ricevuto da topic: x={self.target_x:.2f}, '
            f'y={self.target_y:.2f}, z={self.target_z:.2f}, yaw={self.target_yaw:.2f}'
        )

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def timer_callback(self):
        if self.exit_requested:
            raise SystemExit

        if self.landing_requested:
            return

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.counter == 10 and not self.offboard_enabled:
            self.engage_offboard_mode()
            self.offboard_enabled = True
            self.get_logger().info('Comando OFFBOARD inviato.')

        if self.counter == 12 and not self.armed:
            self.arm()
            self.armed = True
            self.get_logger().info('Comando ARM inviato.')

        self.counter += 1

    def request_landing(self):
        if not self.landing_requested:
            self.landing_requested = True
            self.get_logger().info('Richiesta landing ricevuta.')
            self.land()

    def request_exit(self):
        if not self.exit_requested:
            self.exit_requested = True
            self.get_logger().info('Richiesta uscita ricevuta.')

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_timestamp()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_timestamp()
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

    def land(self):
        self.get_logger().info('Comando LAND inviato.')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
            0.0,
            0.0
        )

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_timestamp()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

    def get_timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)


def keyboard_listener(node):
    fd = sys.stdin.fileno()

    if not sys.stdin.isatty():
        return

    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        while rclpy.ok():
            key = sys.stdin.read(1)

            # EOF / terminale chiuso
            if key == '':
                node.request_exit()
                break

            if key.lower() == 'l':
                node.request_landing()

            elif key.lower() == 'q':
                node.request_exit()
                break

    except Exception:
        node.request_exit()

    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()

    def handle_signal(signum, frame):
        node.request_exit()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGHUP, handle_signal)

    key_thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    key_thread.start()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()