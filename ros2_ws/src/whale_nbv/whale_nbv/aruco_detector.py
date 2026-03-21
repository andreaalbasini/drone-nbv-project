import os
from datetime import datetime

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()
        self.latest_frame = None
        self.capture_done_for_current_true = False

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            image_qos
        )

        self.flag_sub = self.create_subscription(
            Bool,
            '/position_reached',
            self.flag_callback,
            10
        )

        self.output_dir = '/ws_host/aruco_output'
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info('Aruco detector pronto')
        self.get_logger().info(f'Salvataggio in: {self.output_dir}')

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # Compatibilità OpenCV vecchio/nuovo
        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.use_aruco_detector = hasattr(cv2.aruco, 'ArucoDetector')
        if self.use_aruco_detector:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.detector = None

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Errore conversione immagine: {e}')

    def flag_callback(self, msg: Bool):
        if not msg.data:
            self.capture_done_for_current_true = False
            return

        if self.capture_done_for_current_true:
            return

        if self.latest_frame is None:
            self.get_logger().warn('Flag true ricevuto ma nessun frame disponibile')
            return

        self.capture_done_for_current_true = True
        self.process_and_save_frame()

    def process_and_save_frame(self):
        frame = self.latest_frame.copy()
        annotated = frame.copy()

        if self.use_aruco_detector:
            corners, ids, _ = self.detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                frame,
                self.aruco_dict,
                parameters=self.aruco_params
            )

        detected = 0
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            detected = len(ids)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        raw_path = os.path.join(self.output_dir, f'raw_{timestamp}.png')
        annotated_path = os.path.join(self.output_dir, f'annotated_{timestamp}.png')

        ok_raw = cv2.imwrite(raw_path, frame)
        ok_ann = cv2.imwrite(annotated_path, annotated)

        self.get_logger().info(f'Marker rilevati: {detected}')
        self.get_logger().info(f'Salvataggio raw riuscito: {ok_raw} -> {raw_path}')
        self.get_logger().info(f'Salvataggio annotated riuscito: {ok_ann} -> {annotated_path}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()