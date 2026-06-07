#!/usr/bin/env python3

import json
import math
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Float32, String
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
        # MAIN PARAMETERS
        # =====================================================
        self.declare_parameter('confidence_threshold', 0.70)
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        # discrete planner steps
        self.step_xy = 0.35
        self.step_z_toward_plane = 0.25   # approach to plane (small = safe at low altitude)
        self.min_altitude_above_plane = 0.3

        # known plane — offset_d is updated with ground_z at the end of warmup
        self.plane_z = 0.0
        self.plane_normal_w = np.array([0.0, 0.0, 1.0], dtype=float)
        self.plane_offset_d = 0.0

        # ground_z: NED altitude of the ground, estimated during warmup
        self.ground_z = None
        self.ground_z_samples = []

        # BEFORE (Gazebo simulation)
        #self.image_width  = 640.0
        #self.image_height = 480.0
        #self.hfov         = 1.047  # ~60°

        # AFTER (real SIYI A8)
        self.image_width  = 1280.0
        self.image_height = 720.0
        self.hfov         = 1.2217  # 70° in radians - real SIYI A8 HFOV

        self.fx = 731.42
        self.fy = 734.86
        self.cx = 644.71
        self.cy = 364.32
        self.K = make_camera_matrix(self.fx, self.fy, self.cx, self.cy)


        # Simulation
        #self.cx = self.image_width / 2.0
        #self.cy = self.image_height / 2.0
        #self.fx = self.image_width / (2.0 * math.tan(self.hfov / 2.0))
        #self.fy = self.fx
        #self.K = make_camera_matrix(self.fx, self.fy, self.cx, self.cy)
        
        # SIMULATION
        # extrinseca camera -> body
        #self.R_bc = np.array([
        #    [0.0, -1.0,  0.0],
        #    [1.0,  0.0,  0.0],
        #    [0.0,  0.0,  1.0],
        #], dtype=float)

        #self.t_bc = np.array([0.12, 0.0, -0.02], dtype=float)
        
        # REAL DRONE
        # extrinseca camera -> body
        # camera-x (right in image) -> body -y (LEFT), camera-y (down in image) -> body +x (forward)
        # Determined from flight observation: marker at u>cx moved MORE right when drone moved +y,
        # proving image-right = body-left (camera rotated 90° CW around nadir axis vs. assumed).
        self.R_bc = np.array([
            [0.0,  1.0,  0.0],
            [-1.0, 0.0,  0.0],
            [0.0,  0.0,  1.0],
        ], dtype=float)

        self.t_bc = np.array([-0.12, 0.0, 0.15], dtype=float)

        # planner behavior
        self.goal_reached = True
        self.has_odometry = False
        self.has_goal = False

        # drone state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = -4.0
        self.current_yaw = 0.0

        self.t_wb = np.zeros(3, dtype=float)
        self.R_wb = np.eye(3, dtype=float)

        # Delay
        self.goal_reached_time_ns = 0
        self.stabilization_delay_s = 1.0

        self.last_detection_time_ns = 0

        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        self.current_goal_z = -4.0
        self.current_goal_yaw = 0.0

        # target state
        self.targets: Dict[int, dict] = {}

        # avoid re-publishing while the drone is still executing
        self.waiting_after_publish = False

        self.mission_complete_pub = self.create_publisher(Bool, '/whale_nbv/mission_complete', 10)
        self.ground_z_pub = self.create_publisher(Float32, '/whale_nbv/ground_z', 10)
        self.mission_complete = False

        self.declare_parameter('hover_z', -3.0)
        self.hover_z = self.get_parameter('hover_z').value

        # GPS warm-up: wait N seconds before sending the first goal
        self.declare_parameter('gps_warmup_s', 20.0)
        self.gps_warmup_s = self.get_parameter('gps_warmup_s').value
        self.start_time_ns = None
        self.last_warmup_log_s = -999.0

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

        self.get_logger().info('NBV planner started')
        self.get_logger().info(f'Confidence threshold = {self.confidence_threshold:.2f}')
        self.get_logger().info('Discrete planner with XY + descend actions')

    # =====================================================
    # CALLBACKS
    # =====================================================

    def goal_reached_callback(self, msg: Bool):
        if msg.data and not self.goal_reached:
            self.goal_reached_time_ns = self.get_clock().now().nanoseconds
        self.goal_reached = msg.data
        if self.goal_reached:
            self.waiting_after_publish = False

    def detections_callback(self, msg: String):
        try:
            detections = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Error parsing detections_json: {e}')
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
        self.last_detection_time_ns = self.get_clock().now().nanoseconds

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
        if self.start_time_ns is None:
            self.start_time_ns = self.get_clock().now().nanoseconds
        # accumulate z samples until ground has been estimated
        if self.ground_z is None:
            self.ground_z_samples.append(self.current_z)

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
        _, t_wc = self.compute_camera_pose_in_world()
        cam_z = t_wc[2]

        for marker_id, target in self.targets.items():
            u = float(target['center'][0])
            v = float(target['center'][1])
            point_w = self.intersect_pixel_with_plane_current_pose(u, v)
            if point_w is None:
                continue

            # Clamp z: the marker must lie between the camera and the ground.
            # If it ends up above the camera (z < cam_z) or below ground (z > ground_z)
            # we force z to ground_z — better an approximation than an impossible point.
            if self.ground_z is not None:
                if point_w[2] < cam_z or point_w[2] > self.ground_z:
                    self.get_logger().warn(
                        f'[target {marker_id}] estimated z ({point_w[2]:.2f}) out of range '
                        f'[cam_z={cam_z:.2f}, ground_z={self.ground_z:.2f}] → clamped to ground_z'
                    )
                    point_w[2] = self.ground_z

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

        # in NED: ground_z is positive (e.g. +9.77), climbing = decreasing z
        # min_altitude_above_plane = 2m → do not descend below (ground_z - 2.0)
        if self.ground_z is not None:
            min_allowed_z = self.ground_z - self.min_altitude_above_plane
        else:
            min_allowed_z = -self.min_altitude_above_plane
        if cand_z > min_allowed_z:
            cand_z = min_allowed_z

        return cand_x, cand_y, cand_z, self.current_yaw



    def predict_marker_pixel_size(self, drone_pos, marker_world_pos, marker_real_size=0.5):
        dx = drone_pos[0] - marker_world_pos[0]
        dy = drone_pos[1] - marker_world_pos[1]
        # Camera altitude (camera is t_bc[2]=0.15m lower than body in NED)
        cam_z = drone_pos[2] + self.t_bc[2]
        dz = max(self.ground_z - cam_z, 0.1) if self.ground_z is not None else max(abs(cam_z), 0.1)

        # distanza 3D dalla camera al marker
        dist_3d = math.sqrt(dx**2 + dy**2 + dz**2)

        # dimensione attesa in pixel (proiezione prospettica)
        pixel_size = (marker_real_size * self.fx) / max(dist_3d, 0.1)

        # area attesa in pixel²
        pixel_area = pixel_size ** 2
        image_area = self.image_width * self.image_height

        image_area_ratio = pixel_area / image_area
        size_score = min(1.0, image_area_ratio * 60.0)  # stesso scaling del detector
        return size_score

    def predict_marker_center_score(self, candidate_pose, marker_world_pos):
        cand_x, cand_y, cand_z, cand_yaw = candidate_pose

        # Camera position in world (t_bc offset rotated by yaw)
        cos_y = math.cos(cand_yaw)
        sin_y = math.sin(cand_yaw)
        cam_x = cand_x + cos_y * self.t_bc[0] - sin_y * self.t_bc[1]
        cam_y = cand_y + sin_y * self.t_bc[0] + cos_y * self.t_bc[1]
        cam_z = cand_z + self.t_bc[2]

        # Vector camera -> marker in world frame
        dx = marker_world_pos[0] - cam_x
        dy = marker_world_pos[1] - cam_y
        dz = max(self.ground_z - cam_z, 0.1) if self.ground_z is not None else max(abs(cam_z), 0.1)

        # Rotate into body frame (yaw-aware): body_x = dx*cos+dy*sin, body_y = -dx*sin+dy*cos
        dx_body = dx * cos_y + dy * sin_y
        dy_body = -dx * sin_y + dy * cos_y

        # Applica R_bc^T: con R_bc=[[0,1,0],[-1,0,0],[0,0,1]] → R_bc^T=[[0,-1,0],[1,0,0],[0,0,1]]
        # cam_x = -body_y (image right = body LEFT), cam_y = +body_x (image down = body forward)
        p_cam_x = -dy_body
        p_cam_y =  dx_body

        u = self.cx + self.fx * (p_cam_x / max(dz, 0.1))
        v = self.cy + self.fy * (p_cam_y / max(dz, 0.1))

        # center score: how close to the image center
        cx_img = self.image_width / 2.0
        cy_img = self.image_height / 2.0
        dist = math.sqrt((u - cx_img)**2 + (v - cy_img)**2)
        dist_max = math.sqrt(cx_img**2 + cy_img**2)
        center_score = max(0.0, 1.0 - dist / dist_max)

        # if outside image, confidence zero
        if u < 0 or u > self.image_width or v < 0 or v > self.image_height:
            return 0.0, False

        return center_score, True


    def predict_confidence_for_target(self, marker_id, target_point_w, candidate_pose):
        if marker_id not in self.targets:
            return 0.0

        cand_x, cand_y, cand_z, _ = candidate_pose

        # size score predicted geometrically
        size_score = self.predict_marker_pixel_size(
            (cand_x, cand_y, cand_z),
            target_point_w,
            marker_real_size=0.5  # real marker size in metres
        )

        # center score predicted geometrically
        center_score, visible = self.predict_marker_center_score(
            candidate_pose,
            target_point_w
        )

        if not visible:
            return 0.0  # marker out of field of view: confidence zero, do not go there

        # shape and border are almost always good if the marker is visible
        # we keep them fixed at the current value as a conservative estimate
        current = self.targets[marker_id]
        shape_score = float(current.get('shape_score', 0.8))
        border_score = min(1.0, center_score * 1.5)  # if centred, it is also far from the edges

        predicted = (
            0.50 * size_score +
            0.20 * center_score +
            0.20 * shape_score +
            0.10 * border_score
        )

        return max(0.0, min(1.0, predicted))





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
            0.80 * min_pred +
            0.10 * mean_pred +
            0.35 * min_gain +
            0.10 * mean_gain +
            0.30 * all_above_bonus +
            0.05 * num_above -
            0.12 * move_xy_cost -
            0.10 * move_z_cost
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
        try:
            self._planning_loop_impl()
        except Exception as e:
            self.get_logger().error(f'[planning_loop] UNHANDLED EXCEPTION: {e}', throttle_duration_sec=5.0)
            import traceback
            self.get_logger().error(traceback.format_exc(), throttle_duration_sec=5.0)

    def _planning_loop_impl(self):
        if not self.has_odometry:
            self.get_logger().debug('NBV: waiting odometry...')
            return

        #if not self.has_goal:
        #    self.get_logger().debug('NBV: waiting first goal...')
        #    return

        if not self.has_goal and self.has_odometry:
            elapsed_s = (self.get_clock().now().nanoseconds - self.start_time_ns) / 1e9
            if elapsed_s < self.gps_warmup_s:
                if elapsed_s - self.last_warmup_log_s >= 5.0:
                    self.get_logger().info(
                        f'GPS warm-up: {self.gps_warmup_s - elapsed_s:.0f}s remaining before takeoff '
                        f'(current pos: {self.current_x:.2f}, {self.current_y:.2f}, z={self.current_z:.2f})'
                    )
                    self.last_warmup_log_s = elapsed_s
                return
            # end of warmup: estimate ground_z from samples collected on the ground
            if self.ground_z is None:
                # Use only the last 100 samples (~5s at 20Hz) to avoid stale early GPS readings
                recent = self.ground_z_samples[-100:] if len(self.ground_z_samples) >= 100 else self.ground_z_samples
                self.ground_z = float(np.mean(recent)) if recent else self.current_z
                self.plane_offset_d = -self.ground_z  # plane eq: n·X+d=0 → z=-d → d=-ground_z
                self.plane_z = self.ground_z
                self.get_logger().info(
                    f'Estimated ground z: {self.ground_z:.2f}m NED '
                    f'(mean over last {len(recent)} samples / {len(self.ground_z_samples)} total)'
                )
                self.ground_z_pub.publish(Float32(data=self.ground_z))
            # hover_z is a RELATIVE offset from the ground (negative = up in NED)
            hover_goal_z = self.ground_z + self.hover_z
            self.get_logger().info(
                f'First autonomous hover: ({self.current_x:.2f}, {self.current_y:.2f}, {hover_goal_z:.2f}) '
                f'[ground={self.ground_z:.2f} + offset={self.hover_z:.2f}], yaw={self.current_yaw:.2f}'
            )
            self.publish_goal(self.current_x, self.current_y, hover_goal_z, self.current_yaw)
            return

        if self.waiting_after_publish:
            return

        if not self.goal_reached:
            return
        now_ns = self.get_clock().now().nanoseconds
        elapsed_s = (now_ns - self.goal_reached_time_ns) / 1e9
        if elapsed_s < self.stabilization_delay_s:
            return

        # wait for a detection that arrived AFTER goal_reached
        if self.last_detection_time_ns < self.goal_reached_time_ns:
            return  # no fresh detection yet

        self.get_logger().info('==================================================')
        self.get_logger().info(
            f'NBV LOOP | odom=({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) '
            f'yaw={self.current_yaw:.2f} | targets={list(self.targets.keys())}'
        )

        if len(self.targets) == 0:
            self.get_logger().warn('NBV: no marker detected. Maintaining hover.')
            return

        for marker_id, target in self.targets.items():
            self.get_logger().info(
                f"Target {marker_id} | center={target['center']} | conf={target['confidence']:.3f} | bbox={target['bbox']}"
            )

        if self.all_targets_above_threshold():
            if not self.mission_complete:
                self.mission_complete = True
                self.get_logger().info(
                f'NBV: all targets have confidence >= {self.confidence_threshold:.2f}. Mission complete.'
                )
                self.mission_complete_pub.publish(Bool(data=True))
            return

        target_points_world = self.estimate_target_world_points()

        if len(target_points_world) == 0:
            self.get_logger().warn('NBV: unable to estimate target points on the plane. No action.')
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
            self.get_logger().warn('NBV: no valid action found.')
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
