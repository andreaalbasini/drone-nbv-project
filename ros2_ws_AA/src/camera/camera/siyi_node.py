#!/usr/bin/env python3
"""
SIYI A8 Camera Node
- Connects SDK gimbal and sets pitch to -90 (nadir)
- Reads RTSP stream via ffmpeg
- Publishes /camera/image_raw  (sensor_msgs/Image)
- Publishes /siyi/attitude     (geometry_msgs/Vector3)
"""

import threading
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
import rclpy

from camera.siyi_sdk import SIYISDK

RTSP_URL = "rtsp://192.168.144.25:8554/main.264"
CAM_IP   = "192.168.144.25"
PORT     = 37260


class SiyiNode(Node):
    def __init__(self):
        super().__init__('siyi_node')
        self.get_logger().info('Starting SIYI ROS2 node...')

        # --- image publisher ---
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', image_qos)
        self.att_pub   = self.create_publisher(Vector3, '/siyi/attitude', 10)
        self.bridge    = CvBridge()

        # --- frame shared between stream thread and publish timer ---
        self.latest_frame = None
        self.frame_lock   = threading.Lock()

        # --- gimbal SDK ---
        self.cam = SIYISDK(server_ip=CAM_IP, port=PORT, debug=False)
        if not self.cam.connect(maxWaitTime=5.0):
            self.get_logger().error('Cannot connect to SIYI camera')
            raise RuntimeError('SIYI connection failed')
        self.get_logger().info('SIYI SDK connected')

        # move the gimbal to nadir in a separate thread (does not block ROS)
        self.gimbal_ready = False
        threading.Thread(target=self._init_gimbal, daemon=True).start()

        # --- RTSP acquisition thread ---
        self.running = True
        threading.Thread(target=self._stream_loop, daemon=True).start()

        # --- image publish timer @ 10 Hz ---
        self.create_timer(0.1, self._publish_image)

        # --- attitude publish timer @ 2 Hz ---
        self.create_timer(0.5, self._publish_attitude)

        self.get_logger().info('SiyiNode ready. Publishing /camera/image_raw @ 10Hz')

    # ----------------------------------------------------------------
    # GIMBAL: move to nadir in separate thread
    # ----------------------------------------------------------------
    def _init_gimbal(self):
        time.sleep(2.0)  # let the SDK connection stabilize
        self.get_logger().info('Moving gimbal to nadir (pitch=-90)...')
        try:
            self.cam.setGimbalRotation(yaw=0.0, pitch=-90.0, err_thresh=2.0, kp=4)
            self.get_logger().info('Gimbal at nadir OK')
            self.gimbal_ready = True
        except Exception as e:
            self.get_logger().error(f'Gimbal init error: {e}')

    # ----------------------------------------------------------------
    # RTSP STREAM: reads frames in a loop and stores them in latest_frame
    # ----------------------------------------------------------------
    def _stream_loop(self):
        self.get_logger().info(f'Opening RTSP stream: {RTSP_URL}')
        cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            self.get_logger().error('Cannot open RTSP stream!')
            return
        self.get_logger().info('RTSP stream open')

        while self.running and rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn('Frame lost, retrying...')
                time.sleep(1.0)
                cap.release()
                cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
                continue
            with self.frame_lock:
                self.latest_frame = frame

        cap.release()

    # ----------------------------------------------------------------
    # PUBLISH IMAGE (ROS timer @ 10Hz)
    # ----------------------------------------------------------------
    def _publish_image(self):
        with self.frame_lock:
            frame = self.latest_frame
        if frame is None:
            return
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Image publish error: {e}')

    # ----------------------------------------------------------------
    # PUBLISH GIMBAL ATTITUDE (ROS timer @ 2Hz)
    # ----------------------------------------------------------------
    def _publish_attitude(self):
        try:
            yaw, pitch, roll = self.cam.getAttitude()
            msg = Vector3()
            msg.x = float(roll)
            msg.y = float(pitch)
            msg.z = float(yaw)
            self.att_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Attitude not available: {e}')

    # ----------------------------------------------------------------
    # CLEANUP
    # ----------------------------------------------------------------
    def destroy_node(self):
        self.running = False
        try:
            self.cam.requestGimbalSpeed(0, 0)
            self.cam.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SiyiNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f'[FATAL] {e}')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
