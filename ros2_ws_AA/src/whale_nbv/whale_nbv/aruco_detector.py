#!/usr/bin/env python3

import os
import math
import json

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()
        self.latest_frame = None
        self.show_debug_window = bool(os.environ.get('DISPLAY', '').strip())
        if self.show_debug_window:
            try:
                cv2.startWindowThread()
            except Exception:
                self.show_debug_window = False
        # Most recent detection state
        self.latest_detections = []
        self.latest_detected = False
        self.last_ids_for_log = None

        # Sticky detection: keeps the last valid detection for N seconds
        # so the NBV planner does not lose the marker when it flickers
        self.sticky_detections = []
        self.sticky_detected = False
        self.last_detected_ns = 0
        self.sticky_timeout_ns = int(1.5 * 1e9)  # 1.5s persistence

        # Photo only on goal_reached (no continuous automatic saving)

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

        self.goal_reached_sub = self.create_subscription(
            Bool,
            '/whale_nbv/goal_reached',
            self.goal_reached_callback,
            10
        )
        self.mission_complete_sub = self.create_subscription(
            Bool,
            '/whale_nbv/mission_complete',
            self.mission_complete_callback,
            10
        )
        self.last_goal_reached_state = False
        self.mission_complete_photo_taken = False
        self.photo_counter = 0

        # Output multi-marker
        self.detected_pub = self.create_publisher(Bool, '/aruco/detected', 10)
        self.detections_json_pub = self.create_publisher(String, '/aruco/detections_json', 10)

        # Cartella debug futura, ma qui non salviamo immagini
        self.output_dir = os.path.expanduser('~/aruco_output')
        os.makedirs(self.output_dir, exist_ok=True)
              
        
        
        

        # ArUco dictionary selectable via ROS parameter:
        #   aruco_dict:=APRILTAG_36h11   (default, markers printed with plastic lamination)
        #   aruco_dict:=4X4_50           (markers reprinted without plastic lamination)
        self.declare_parameter('aruco_dict', '4X4_50')
        dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value
        dict_map = {
            'APRILTAG_36h11': cv2.aruco.DICT_APRILTAG_36h11,
            '4X4_50':         cv2.aruco.DICT_4X4_50,
            '4X4_100':        cv2.aruco.DICT_4X4_100,
            '6X6_250':        cv2.aruco.DICT_6X6_250,
        }
        dict_id = dict_map.get(dict_name, cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        self.get_logger().info(f'Aruco detector ready — dictionary: {dict_name}')
        self.get_logger().info('Multi-marker mode enabled')
        self.get_logger().info('Publishing detections at 2 Hz')

        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Useful parameters for more stable detection
        if hasattr(self.aruco_params, 'cornerRefinementMethod'):
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        if hasattr(self.aruco_params, 'adaptiveThreshWinSizeMin'):
            self.aruco_params.adaptiveThreshWinSizeMin = 3
        if hasattr(self.aruco_params, 'adaptiveThreshWinSizeMax'):
            self.aruco_params.adaptiveThreshWinSizeMax = 23
        if hasattr(self.aruco_params, 'adaptiveThreshWinSizeStep'):
            self.aruco_params.adaptiveThreshWinSizeStep = 10
        if hasattr(self.aruco_params, 'minMarkerPerimeterRate'):
            self.aruco_params.minMarkerPerimeterRate = 0.02
        if hasattr(self.aruco_params, 'maxMarkerPerimeterRate'):
            self.aruco_params.maxMarkerPerimeterRate = 4.0

        self.use_aruco_detector = hasattr(cv2.aruco, 'ArucoDetector')
        if self.use_aruco_detector:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.detector = None

        # Slow publication
        self.publish_period_s = 0.5  # 2 Hz
        self.publish_timer = self.create_timer(self.publish_period_s, self.publish_timer_callback)



    # =========================================================
    # CALLBACKS
    # =========================================================

    def image_callback(self, msg: Image):
       try:
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.update_detections()
       except Exception as e:
        self.get_logger().error(f'Error image conversion: {e}')

    def goal_reached_callback(self, msg: Bool):
        # Fires only on rising edge (False→True), never during manual guidance
        if msg.data and not self.last_goal_reached_state:
            self.save_waypoint_photo()
        self.last_goal_reached_state = msg.data

    def mission_complete_callback(self, msg: Bool):
        if msg.data and not self.mission_complete_photo_taken:
            self.mission_complete_photo_taken = True
            self.save_waypoint_photo(label_prefix='MISSION_COMPLETE')

    def save_waypoint_photo(self, label_prefix='waypoint'):
        import datetime
        if self.latest_frame is None:
            self.get_logger().warn('[PHOTO] No frame available')
            return

        annotated = self.latest_frame.copy()
        for det in self.latest_detections:
            bbox = det['bbox']
            cv2.rectangle(annotated,
                (int(bbox[0]), int(bbox[1])),
                (int(bbox[2]), int(bbox[3])),
                (0, 255, 0), 2)
            label = f"ID {det['id']} | conf={det['confidence']:.2f}"
            cv2.putText(annotated, label,
                (int(bbox[0]), int(bbox[1]) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        ts = datetime.datetime.now().strftime('%H%M%S')
        filename = f'{label_prefix}_{self.photo_counter:04d}_{ts}.jpg'
        path = os.path.join(self.output_dir, filename)
        cv2.imwrite(path, annotated)
        self.photo_counter += 1
        ids = [d['id'] for d in self.latest_detections]
        self.get_logger().info(
            f'[PHOTO] {filename} saved | markers={ids}'
        )

    # =========================================================
    # DETECTION
    # =========================================================

    def detect_markers(self, frame_gray):
        if self.use_aruco_detector:
            corners, ids, _ = self.detector.detectMarkers(frame_gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                frame_gray,
                self.aruco_dict,
                parameters=self.aruco_params
            )
        return corners, ids

    def compute_confidence(self, frame, pts):
        h, w = frame.shape[:2]
        img_area = float(w * h)

        cx = float(pts[:, 0].mean())
        cy = float(pts[:, 1].mean())

        quad_area = abs(cv2.contourArea(pts.astype('float32')))
        image_area_ratio = quad_area / img_area if img_area > 0 else 0.0
        size_score = min(1.0, image_area_ratio * 60.0)

        cx_img = w / 2.0
        cy_img = h / 2.0
        dist = math.sqrt((cx - cx_img) ** 2 + (cy - cy_img) ** 2)
        dist_max = math.sqrt((cx_img) ** 2 + (cy_img) ** 2)
        dist_norm = dist / dist_max if dist_max > 0 else 1.0
        center_score = max(0.0, 1.0 - dist_norm)

        side_lengths = []
        for i in range(4):
            p1 = pts[i]
            p2 = pts[(i + 1) % 4]
            side_lengths.append(float(math.sqrt(((p2 - p1) ** 2).sum())))

        min_side = min(side_lengths) if side_lengths else 0.0
        max_side = max(side_lengths) if side_lengths else 1.0
        shape_score = (min_side / max_side) if max_side > 0 else 0.0
        shape_score = max(0.0, min(1.0, shape_score))

        min_x = float(pts[:, 0].min())
        max_x = float(pts[:, 0].max())
        min_y = float(pts[:, 1].min())
        max_y = float(pts[:, 1].max())

        margin = min(min_x, w - max_x, min_y, h - max_y)
        border_score = max(0.0, margin / min(w, h))
        border_score = min(1.0, border_score * 4.0)

        confidence = (
            0.50 * size_score +  
            0.20 * center_score +
            0.20 * shape_score +
            0.10 * border_score
        )
        confidence = max(0.0, min(1.0, confidence))

        bbox_area = max(0.0, (max_x - min_x) * (max_y - min_y))

        return {
            'cx': cx,
            'cy': cy,
            'min_x': min_x,
            'max_x': max_x,
            'min_y': min_y,
            'max_y': max_y,
            'bbox_area': bbox_area,
            'quad_area': quad_area,
            'image_area_ratio': image_area_ratio,
            'margin_px': margin,
            'size_score': size_score,
            'center_score': center_score,
            'shape_score': shape_score,
            'border_score': border_score,
            'confidence': confidence
        }

    def update_detections(self):
        if self.latest_frame is None:
            self.latest_detections = []
            self.latest_detected = False
            return

        frame = self.latest_frame
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Adaptive gamma: normalizes brightness before any other processing.
        # If the frame is very dark or very bright (sun/shadow), gamma corrects it
        # toward a target of 128 (medium) before CLAHE works on local contrast.
        mean_val = float(np.mean(frame_gray))
        if mean_val > 5.0:
            gamma = math.log(128.0) / math.log(mean_val)
            gamma = max(0.4, min(2.5, gamma))
            inv_gamma = 1.0 / gamma
            lut = np.array([int((i / 255.0) ** inv_gamma * 255)
                            for i in range(256)], dtype=np.uint8)
            frame_gray = cv2.LUT(frame_gray, lut)

        # CLAHE for local contrast (small tiles = better for distant markers)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))
        frame_gray = clahe.apply(frame_gray)
        corners, ids = self.detect_markers(frame_gray)
        
        if self.show_debug_window:
            debug_frame = frame.copy()
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)
            cv2.imshow("ArUco Debug (undistorted)", cv2.resize(debug_frame, (960, 540)))
            cv2.waitKey(1)

        if ids is None or len(ids) == 0:
            self.latest_detections = []
            self.latest_detected = False

            if self.last_ids_for_log != []:
                self.get_logger().info('Raw detected ids: []')
                self.last_ids_for_log = []
            return

        raw_ids = [int(x[0]) for x in ids]
        if raw_ids != self.last_ids_for_log:
            self.get_logger().info(f'Raw detected ids: {raw_ids}')
            self.last_ids_for_log = raw_ids

        detections = []

        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            pts = corners[i][0]
            metrics = self.compute_confidence(frame, pts)

            detection = {
                "id": marker_id,
                "center": [
                    float(metrics["cx"]),
                    float(metrics["cy"])
                ],
                "bbox": [
                    float(metrics["min_x"]),
                    float(metrics["min_y"]),
                    float(metrics["max_x"]),
                    float(metrics["max_y"])
                ],
                "confidence": float(metrics["confidence"]),
                "bbox_area": float(metrics["bbox_area"]),
                "quad_area": float(metrics["quad_area"]),
                "image_area_ratio": float(metrics["image_area_ratio"]),
                "margin_px": float(metrics["margin_px"]),
                "size_score": float(metrics["size_score"]),
                "center_score": float(metrics["center_score"]),
                "shape_score": float(metrics["shape_score"]),
                "border_score": float(metrics["border_score"])
            }

            detections.append(detection)

        detections.sort(key=lambda d: d["id"])

        self.latest_detections = detections
        self.latest_detected = True

        # update sticky: every time we detect something, save timestamp
        self.sticky_detections = detections
        self.sticky_detected = True
        self.last_detected_ns = self.get_clock().now().nanoseconds

    # =========================================================
    # PUBLISH + DEBUG LOG
    # =========================================================

    def publish_timer_callback(self):
        # use sticky detection: if the marker was seen recently, keep it
        now_ns = self.get_clock().now().nanoseconds
        sticky_alive = (self.sticky_detected and
                        (now_ns - self.last_detected_ns) < self.sticky_timeout_ns)

        publish_detections = self.sticky_detections if sticky_alive else self.latest_detections
        publish_detected = sticky_alive or self.latest_detected

        self.detected_pub.publish(Bool(data=publish_detected))

        json_msg = String()
        json_msg.data = json.dumps(publish_detections)
        self.detections_json_pub.publish(json_msg)

        detected_ids = [d["id"] for d in publish_detections]
        sticky_label = '[STICKY]' if (sticky_alive and not self.latest_detected) else ''
        self.get_logger().info(
            f'Published {len(publish_detections)} marker(s): ids={detected_ids} {sticky_label}'
        )

        for det in publish_detections:
            self.get_logger().info(
                f"[MARKER {det['id']}] "
                f"center=({det['center'][0]:.1f}, {det['center'][1]:.1f}) | "
                f"conf={det['confidence']:.3f} | "
                f"bbox_area={det['bbox_area']:.1f} | "
                f"quad_area={det['quad_area']:.1f} | "
                f"img_ratio={det['image_area_ratio']:.5f} | "
                f"margin_px={det['margin_px']:.1f} | "
                f"size={det['size_score']:.3f} | "
                f"center_score={det['center_score']:.3f} | "
                f"shape={det['shape_score']:.3f} | "
                f"border={det['border_score']:.3f}"
            )



def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
