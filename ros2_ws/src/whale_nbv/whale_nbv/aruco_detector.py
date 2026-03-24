import os
from datetime import datetime
import math

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, Float32
from geometry_msgs.msg import Point


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

        # Publisher dati ArUco
        self.detected_pub = self.create_publisher(Bool, '/aruco/detected', 10)
        self.marker_id_pub = self.create_publisher(Int32, '/aruco/marker_id', 10)
        self.center_pub = self.create_publisher(Point, '/aruco/center', 10)
        self.error_pub = self.create_publisher(Point, '/aruco/error', 10)
        self.confidence_pub = self.create_publisher(Float32, '/aruco/confidence', 10)
        self.bbox_area_pub = self.create_publisher(Float32, '/aruco/bbox_area', 10)

        # Publisher di debug per i singoli score
        self.size_score_pub = self.create_publisher(Float32, '/aruco/size_score', 10)
        self.center_score_pub = self.create_publisher(Float32, '/aruco/center_score', 10)
        self.shape_score_pub = self.create_publisher(Float32, '/aruco/shape_score', 10)
        self.border_score_pub = self.create_publisher(Float32, '/aruco/border_score', 10)

        self.output_dir = '/ws_host/aruco_output'
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info('Aruco detector pronto')
        self.get_logger().info(f'Salvataggio in: {self.output_dir}')

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

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
            self.detect_and_publish()
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

    def detect_markers(self, frame):
        if self.use_aruco_detector:
            corners, ids, _ = self.detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                frame,
                self.aruco_dict,
                parameters=self.aruco_params
            )
        return corners, ids

    def compute_confidence(self, frame, pts):
        h, w = frame.shape[:2]
        img_area = float(w * h)

        # centro marker
        cx = float(pts[:, 0].mean())
        cy = float(pts[:, 1].mean())

        # area quadrilatero
        quad_area = abs(cv2.contourArea(pts.astype('float32')))
        size_score = min(1.0, quad_area / img_area * 25.0)

        # distanza dal centro immagine
        cx_img = w / 2.0
        cy_img = h / 2.0
        dist = math.sqrt((cx - cx_img) ** 2 + (cy - cy_img) ** 2)
        dist_max = math.sqrt((cx_img) ** 2 + (cy_img) ** 2)
        dist_norm = dist / dist_max if dist_max > 0 else 1.0
        center_score = max(0.0, 1.0 - dist_norm)

        # qualità forma: rapporto lato corto / lato lungo
        side_lengths = []
        for i in range(4):
            p1 = pts[i]
            p2 = pts[(i + 1) % 4]
            side_lengths.append(float(math.sqrt(((p2 - p1) ** 2).sum())))

        min_side = min(side_lengths) if side_lengths else 0.0
        max_side = max(side_lengths) if side_lengths else 1.0
        shape_score = (min_side / max_side) if max_side > 0 else 0.0
        shape_score = max(0.0, min(1.0, shape_score))

        # distanza dai bordi
        min_x = float(pts[:, 0].min())
        max_x = float(pts[:, 0].max())
        min_y = float(pts[:, 1].min())
        max_y = float(pts[:, 1].max())

        margin = min(min_x, w - max_x, min_y, h - max_y)
        border_score = max(0.0, margin / min(w, h))
        border_score = min(1.0, border_score * 4.0)

        # confidence totale
        confidence = (
            0.35 * size_score +
            0.20 * center_score +
            0.25 * shape_score +
            0.20 * border_score
        )
        confidence = max(0.0, min(1.0, confidence))

        # bbox area per debug
        bbox_area = max(0.0, (max_x - min_x) * (max_y - min_y))

        return {
            'cx': cx,
            'cy': cy,
            'bbox_area': bbox_area,
            'quad_area': quad_area,
            'size_score': size_score,
            'center_score': center_score,
            'shape_score': shape_score,
            'border_score': border_score,
            'confidence': confidence
        }

    def detect_and_publish(self):
        if self.latest_frame is None:
            return

        frame = self.latest_frame
        h, w = frame.shape[:2]
        cx_img = w / 2.0
        cy_img = h / 2.0

        corners, ids = self.detect_markers(frame)

        detected_msg = Bool()
        center_msg = Point()
        error_msg = Point()
        marker_id_msg = Int32()
        confidence_msg = Float32()
        bbox_area_msg = Float32()

        if ids is not None and len(ids) > 0:
            detected_msg.data = True

            pts = corners[0][0]
            metrics = self.compute_confidence(frame, pts)

            err_x = metrics['cx'] - cx_img
            err_y = metrics['cy'] - cy_img

            center_msg.x = metrics['cx']
            center_msg.y = metrics['cy']
            center_msg.z = 0.0

            error_msg.x = err_x
            error_msg.y = err_y
            error_msg.z = 0.0

            marker_id_msg.data = int(ids[0][0])
            confidence_msg.data = float(metrics['confidence'])
            bbox_area_msg.data = float(metrics['bbox_area'])

            self.detected_pub.publish(detected_msg)
            self.marker_id_pub.publish(marker_id_msg)
            self.center_pub.publish(center_msg)
            self.error_pub.publish(error_msg)
            self.confidence_pub.publish(confidence_msg)
            self.bbox_area_pub.publish(bbox_area_msg)

            self.size_score_pub.publish(Float32(data=float(metrics['size_score'])))
            self.center_score_pub.publish(Float32(data=float(metrics['center_score'])))
            self.shape_score_pub.publish(Float32(data=float(metrics['shape_score'])))
            self.border_score_pub.publish(Float32(data=float(metrics['border_score'])))
        else:
            detected_msg.data = False
            self.detected_pub.publish(detected_msg)

    def process_and_save_frame(self):
        frame = self.latest_frame.copy()
        annotated = frame.copy()

        h, w = frame.shape[:2]
        cx_img = w / 2.0
        cy_img = h / 2.0

        corners, ids = self.detect_markers(frame)

        detected = 0

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            detected = len(ids)

            pts = corners[0][0]
            metrics = self.compute_confidence(frame, pts)

            err_x = metrics['cx'] - cx_img
            err_y = metrics['cy'] - cy_img

            self.get_logger().info(
                f'Aruco id={int(ids[0][0])} '
                f'center=({metrics["cx"]:.1f}, {metrics["cy"]:.1f}) '
                f'error=({err_x:.1f}, {err_y:.1f}) '
                f'bbox_area={metrics["bbox_area"]:.1f} '
                f'conf={metrics["confidence"]:.3f} '
                f'[size={metrics["size_score"]:.3f}, '
                f'center={metrics["center_score"]:.3f}, '
                f'shape={metrics["shape_score"]:.3f}, '
                f'border={metrics["border_score"]:.3f}]'
            )

            cv2.circle(annotated, (int(cx_img), int(cy_img)), 5, (255, 0, 0), -1)
            cv2.circle(annotated, (int(metrics["cx"]), int(metrics["cy"])), 5, (0, 0, 255), -1)
            cv2.line(
                annotated,
                (int(cx_img), int(cy_img)),
                (int(metrics["cx"]), int(metrics["cy"])),
                (0, 255, 0),
                2
            )

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