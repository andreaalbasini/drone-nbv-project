#!/usr/bin/env python3
"""
SIYI A8 Camera Node
- Connette SDK gimbal e porta pitch a -90 (nadir)
- Legge stream RTSP via ffmpeg
- Pubblica /camera/image_raw  (sensor_msgs/Image)
- Pubblica /siyi/attitude     (geometry_msgs/Vector3)
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

        # --- publisher immagini ---
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', image_qos)
        self.att_pub   = self.create_publisher(Vector3, '/siyi/attitude', 10)
        self.bridge    = CvBridge()

        # --- frame condiviso tra thread stream e timer publish ---
        self.latest_frame = None
        self.frame_lock   = threading.Lock()

        # --- SDK gimbal ---
        self.cam = SIYISDK(server_ip=CAM_IP, port=PORT, debug=False)
        if not self.cam.connect(maxWaitTime=5.0):
            self.get_logger().error('Cannot connect to SIYI camera')
            raise RuntimeError('SIYI connection failed')
        self.get_logger().info('SIYI SDK connected')

        # porta il gimbal a nadir in un thread separato (non blocca ROS)
        self.gimbal_ready = False
        threading.Thread(target=self._init_gimbal, daemon=True).start()

        # --- thread acquisizione RTSP ---
        self.running = True
        threading.Thread(target=self._stream_loop, daemon=True).start()

        # --- timer pubblicazione immagini @ 10 Hz ---
        self.create_timer(0.1, self._publish_image)

        # --- timer pubblicazione attitude @ 2 Hz ---
        self.create_timer(0.5, self._publish_attitude)

        self.get_logger().info('SiyiNode ready. Publishing /camera/image_raw @ 10Hz')

    # ----------------------------------------------------------------
    # GIMBAL: porta a nadir in thread separato
    # ----------------------------------------------------------------
    def _init_gimbal(self):
        time.sleep(2.0)  # lascia stabilizzare la connessione SDK
        self.get_logger().info('Moving gimbal to nadir (pitch=-90)...')
        try:
            self.cam.setGimbalRotation(yaw=0.0, pitch=-90.0, err_thresh=2.0, kp=4)
            self.get_logger().info('Gimbal at nadir OK')
            self.gimbal_ready = True
        except Exception as e:
            self.get_logger().error(f'Gimbal init error: {e}')

    # ----------------------------------------------------------------
    # STREAM RTSP: legge frame in loop, li mette in latest_frame
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
    # PUBLISH IMMAGINE (timer ROS @ 10Hz)
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
    # PUBLISH ATTITUDE GIMBAL (timer ROS @ 2Hz)
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
