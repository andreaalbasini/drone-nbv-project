#!/usr/bin/env python3

import json
import math
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry

from whale_nbv.utils import (
    quaternion_to_yaw,
    yaw_to_quaternion,
    quat_wxyz_to_rotmat,
    make_camera_matrix,
    pixel_to_camera_ray,
    intersect_ray_with_plane,
)


class NBVPlanner(Node):
    def __init__(self):
        super().__init__('nbv_planner')

        # =====================================================
        # PARAMETRI PRINCIPALI
        # =====================================================
        self.confidence_threshold = 0.70

        # step planner discreti
        self.step_xy = 0.30
        self.step_z_toward_plane = 0.50   # avvicinarsi al piano
        self.min_altitude_above_plane = 3.0

        # piano noto
        self.plane_z = 0.0
        self.plane_normal_w = np.array([0.0, 0.0, 1.0], dtype=float)
        self.plane_offset_d = 0.0

        # camera
        self.image_width = 640.0
        self.image_height = 480.0
        self.hfov = 1.047

        self.cx = self.image_width / 2.0
        self.cy = self.image_height / 2.0
        self.fx = self.image_width / (2.0 * math.tan(self.hfov / 2.0))
        self.fy = self.fx
        self.K = make_camera_matrix(self.fx, self.fy, self.cx, self.cy)

        # extrinseca camera -> body
        self.R_bc = np.array([
            [0.0, -1.0,  0.0],
            [1.0,  0.0,  0.0],
            [0.0,  0.0,  1.0],
        ], dtype=float)

        self.t_bc = np.array([0.12, 0.0, -0.02], dtype=float)

        # planner behavior
        self.goal_reached = True
        self.has_odometry = False
        self.has_goal = False

        # stato drone
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = -4.0
        self.current_yaw = 0.0

        self.t_wb = np.zeros(3, dtype=float)
        self.R_wb = np.eye(3, dtype=float)

        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        self.current_goal_z = -4.0
        self.current_goal_yaw = 0.0

        # stato target
        self.targets: Dict[int, dict] = {}

        # evita di ripubblicare mentre il drone sta ancora eseguendo
        self.waiting_after_publish = False

        # =====================================================
        # PUB/SUB
        # =====================================================
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/whale_nbv/goal_pose',
            10
        )

        self.goal_reached_sub = self.create_subscription(
            Bool,
            '/whale_nbv/goal_reached',
            self.goal_reached_callback,
            10
        )

        self.detections_sub = self.create_subscription(
            String,
            '/aruco/detections_json',
            self.detections_callback,
            10
        )

        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/whale_nbv/goal_pose',
            self.goal_pose_callback,
            10
        )

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

        self.timer = self.create_timer(0.5, self.planning_loop)

        self.get_logger().info('NBV planner avviato')
        self.get_logger().info(f'Confidence threshold = {self.confidence_threshold:.2f}')
        self.get_logger().info('Planner discreto con azioni XY + descend')

    # =====================================================
    # CALLBACKS
    # =====================================================

    def goal_reached_callback(self, msg: Bool):
        self.goal_reached = msg.data
        if self.goal_reached:
            self.waiting_after_publish = False

    def detections_callback(self, msg: String):
        try:
            detections = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Errore parsing detections_json: {e}')
            return

        now_ns = self.get_clock().now().nanoseconds
        new_targets = {}

        for det in detections:
            marker_id = int(det['id'])
            new_targets[marker_id] = {
                'id': marker_id,
                'center': det['center'],
                'bbox': det['bbox'],
                'confidence': float(det['confidence']),
                'bbox_area': float(det['bbox_area']),
                'size_score': float(det.get('size_score', 0.0)),
                'center_score': float(det.get('center_score', 0.0)),
                'shape_score': float(det.get('shape_score', 0.0)),
                'border_score': float(det.get('border_score', 0.0)),
                'timestamp_ns': now_ns
            }

        self.targets = new_targets

    def goal_pose_callback(self, msg: PoseStamped):
        self.current_goal_x = float(msg.pose.position.x)
        self.current_goal_y = float(msg.pose.position.y)
        self.current_goal_z = float(msg.pose.position.z)

        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)
        self.current_goal_yaw = quaternion_to_yaw(qx, qy, qz, qw)

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

    # =====================================================
    # GEOMETRIA
    # =====================================================

    def compute_camera_pose_in_world(self):
        R_wc = self.R_wb @ self.R_bc
        t_wc = self.t_wb + self.R_wb @ self.t_bc
        return R_wc, t_wc

    def intersect_pixel_with_plane_current_pose(self, u: float, v: float) -> Optional[np.ndarray]:
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

    def estimate_target_world_points(self) -> Dict[int, np.ndarray]:
        target_points = {}

        for marker_id, target in self.targets.items():
            u = float(target['center'][0])
            v = float(target['center'][1])
            point_w = self.intersect_pixel_with_plane_current_pose(u, v)
            if point_w is not None:
                target_points[marker_id] = point_w

        return target_points

    def altitude_above_plane_from_z(self, z_value: float) -> float:
        return abs(z_value - self.plane_z)

    # =====================================================
    # PLANNER CORE
    # =====================================================

    def all_targets_above_threshold(self) -> bool:
        if len(self.targets) == 0:
            return False
        return all(t['confidence'] >= self.confidence_threshold for t in self.targets.values())

    def compute_weighted_focus_point(self, target_points_world: Dict[int, np.ndarray]) -> Optional[np.ndarray]:
        if len(target_points_world) == 0:
            return None

        weights = []
        points = []

        for marker_id, point_w in target_points_world.items():
            if marker_id not in self.targets:
                continue

            conf = float(self.targets[marker_id]['confidence'])
            weight = max(0.01, 1.0 - conf)
            weights.append(weight)
            points.append(point_w)

        if len(points) == 0:
            return None

        weights = np.array(weights, dtype=float)
        points = np.array(points, dtype=float)

        focus = np.sum(points * weights[:, None], axis=0) / np.sum(weights)
        return focus

    def generate_candidate_actions(self) -> List[dict]:
        actions = []

        base_moves = [
            ('+x', self.step_xy, 0.0),
            ('-x', -self.step_xy, 0.0),
            ('+y', 0.0, self.step_xy),
            ('-y', 0.0, -self.step_xy),
            ('+x+y', self.step_xy, self.step_xy),
            ('+x-y', self.step_xy, -self.step_xy),
            ('-x+y', -self.step_xy, self.step_xy),
            ('-x-y', -self.step_xy, -self.step_xy),
        ]

        for name, dx, dy in base_moves:
            actions.append({
                'name': name,
                'dx': dx,
                'dy': dy,
                'dz': 0.0
            })

        actions.append({
            'name': 'descend',
            'dx': 0.0,
            'dy': 0.0,
            'dz': self.step_z_toward_plane
        })

        return actions

    def apply_action_to_pose(self, action: dict):
        cand_x = self.current_x + action['dx']
        cand_y = self.current_y + action['dy']
        cand_z = self.current_z + action['dz']

        min_allowed_z = -self.min_altitude_above_plane
        if cand_z > min_allowed_z:
            cand_z = min_allowed_z

        return cand_x, cand_y, cand_z, self.current_yaw

    def predict_confidence_for_target(self, marker_id: int, target_point_w: np.ndarray, candidate_pose: tuple) -> float:
        if marker_id not in self.targets:
            return 0.0

        current_conf = float(self.targets[marker_id]['confidence'])

        cand_x, cand_y, cand_z, _ = candidate_pose

        d_curr = math.sqrt(
            (self.current_x - target_point_w[0]) ** 2 +
            (self.current_y - target_point_w[1]) ** 2
        )

        d_new = math.sqrt(
            (cand_x - target_point_w[0]) ** 2 +
            (cand_y - target_point_w[1]) ** 2
        )

        h_curr = self.altitude_above_plane_from_z(self.current_z)
        h_new = self.altitude_above_plane_from_z(cand_z)

        dist_improvement = max(-1.0, min(1.0, (d_curr - d_new) / max(0.5, d_curr + 1e-6)))
        altitude_improvement = max(-1.0, min(1.0, (h_curr - h_new) / max(0.5, h_curr + 1e-6)))

        close_bonus_curr = max(0.0, 1.0 - d_curr / 2.0)
        close_bonus_new = max(0.0, 1.0 - d_new / 2.0)
        close_bonus_delta = close_bonus_new - close_bonus_curr

        predicted = (
            current_conf
            + 0.30 * dist_improvement
            + 0.20 * altitude_improvement
            + 0.10 * close_bonus_delta
        )

        predicted = max(0.0, min(1.0, predicted))
        return predicted

    def score_action(self, action: dict, target_points_world: Dict[int, np.ndarray]) -> dict:
        candidate_pose = self.apply_action_to_pose(action)

        if len(target_points_world) == 0:
            return {
                'action': action,
                'candidate_pose': candidate_pose,
                'score': -999.0,
                'predicted_confidences': {},
                'debug': {'reason': 'no_target_points'}
            }

        predicted_confidences = {}
        current_confs = []

        for marker_id, point_w in target_points_world.items():
            if marker_id not in self.targets:
                continue

            current_confs.append(float(self.targets[marker_id]['confidence']))
            predicted_confidences[marker_id] = self.predict_confidence_for_target(
                marker_id,
                point_w,
                candidate_pose
            )

        if len(predicted_confidences) == 0:
            return {
                'action': action,
                'candidate_pose': candidate_pose,
                'score': -999.0,
                'predicted_confidences': {},
                'debug': {'reason': 'empty_predictions'}
            }

        pred_values = list(predicted_confidences.values())
        curr_values = current_confs

        min_pred = min(pred_values)
        mean_pred = sum(pred_values) / len(pred_values)
        min_curr = min(curr_values)
        mean_curr = sum(curr_values) / len(curr_values)

        min_gain = min_pred - min_curr
        mean_gain = mean_pred - mean_curr

        num_above = sum(1 for c in pred_values if c >= self.confidence_threshold)
        all_above_bonus = 1.0 if min_pred >= self.confidence_threshold else 0.0

        move_xy_cost = math.sqrt(action['dx'] ** 2 + action['dy'] ** 2)
        move_z_cost = abs(action['dz'])

        score = (
            0.55 * min_pred +
            0.10 * mean_pred +
            0.35 * min_gain +
            0.10 * mean_gain +
            0.30 * all_above_bonus +
            0.05 * num_above -
            0.12 * move_xy_cost -
            0.06 * move_z_cost
        )

        return {
            'action': action,
            'candidate_pose': candidate_pose,
            'score': score,
            'predicted_confidences': predicted_confidences,
            'debug': {
                'min_curr': min_curr,
                'mean_curr': mean_curr,
                'min_pred': min_pred,
                'mean_pred': mean_pred,
                'min_gain': min_gain,
                'mean_gain': mean_gain,
                'num_above': num_above,
                'all_above_bonus': all_above_bonus,
                'move_xy_cost': move_xy_cost,
                'move_z_cost': move_z_cost
            }
        }

    def choose_best_action(self, target_points_world: Dict[int, np.ndarray]) -> Optional[dict]:
        actions = self.generate_candidate_actions()
        scored = []

        self.get_logger().info('--- NBV ACTION EVALUATION START ---')

        for action in actions:
            result = self.score_action(action, target_points_world)
            scored.append(result)

            self.get_logger().info(
                f"[ACTION {action['name']}] "
                f"pose=({result['candidate_pose'][0]:.2f}, {result['candidate_pose'][1]:.2f}, {result['candidate_pose'][2]:.2f}) | "
                f"score={result['score']:.3f} | "
                f"pred={result['predicted_confidences']} | "
                f"dbg={result['debug']}"
            )

        scored.sort(key=lambda r: r['score'], reverse=True)

        self.get_logger().info('--- NBV ACTION RANKING ---')
        for idx, r in enumerate(scored):
            self.get_logger().info(
                f"#{idx+1}: {r['action']['name']} | score={r['score']:.3f} | pose={r['candidate_pose']}"
            )

        if len(scored) == 0:
            return None

        return scored[0]

    # =====================================================
    # PUBLISH
    # =====================================================

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

    # =====================================================
    # LOOP
    # =====================================================

    def planning_loop(self):
        if not self.has_odometry:
            self.get_logger().debug('NBV: waiting odometry...')
            return

        if not self.has_goal:
            self.get_logger().debug('NBV: waiting first goal...')
            return

        if self.waiting_after_publish:
            return

        if not self.goal_reached:
            return

        self.get_logger().info('==================================================')
        self.get_logger().info(
            f'NBV LOOP | odom=({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) '
            f'yaw={self.current_yaw:.2f} | targets={list(self.targets.keys())}'
        )

        if len(self.targets) == 0:
            self.get_logger().warn('NBV: nessun marker rilevato. Mantengo hover.')
            return

        for marker_id, target in self.targets.items():
            self.get_logger().info(
                f"Target {marker_id} | center={target['center']} | conf={target['confidence']:.3f} | bbox={target['bbox']}"
            )

        if self.all_targets_above_threshold():
            self.get_logger().info(
                f'NBV: tutti i target hanno confidence >= {self.confidence_threshold:.2f}. Nessun nuovo goal.'
            )
            return

        target_points_world = self.estimate_target_world_points()

        if len(target_points_world) == 0:
            self.get_logger().warn('NBV: impossibile stimare punti target sul piano. Nessuna azione.')
            return

        for marker_id, point_w in target_points_world.items():
            self.get_logger().info(
                f"Estimated world point target {marker_id}: "
                f"({point_w[0]:.3f}, {point_w[1]:.3f}, {point_w[2]:.3f})"
            )

        focus_point = self.compute_weighted_focus_point(target_points_world)
        if focus_point is not None:
            self.get_logger().info(
                f"Weighted focus point: ({focus_point[0]:.3f}, {focus_point[1]:.3f}, {focus_point[2]:.3f})"
            )

        best = self.choose_best_action(target_points_world)
        if best is None:
            self.get_logger().warn('NBV: nessuna azione valida trovata.')
            return

        best_action = best['action']
        best_pose = best['candidate_pose']

        self.get_logger().info(
            f"NBV BEST ACTION = {best_action['name']} | "
            f"score={best['score']:.3f} | "
            f"new_pose=({best_pose[0]:.2f}, {best_pose[1]:.2f}, {best_pose[2]:.2f})"
        )

        self.publish_goal(best_pose[0], best_pose[1], best_pose[2], best_pose[3])
        self.waiting_after_publish = True

        self.get_logger().info(
            f'NBV: published goal -> x={best_pose[0]:.2f}, y={best_pose[1]:.2f}, z={best_pose[2]:.2f}, yaw={best_pose[3]:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = NBVPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()