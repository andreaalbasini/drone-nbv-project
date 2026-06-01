#!/usr/bin/env python3

import math
import numpy as np


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw


def distance_3d(
    x1: float, y1: float, z1: float,
    x2: float, y2: float, z2: float
) -> float:
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def speed_norm(vx: float, vy: float, vz: float) -> float:
    return math.sqrt(vx * vx + vy * vy + vz * vz)


def ros_time_us(node) -> int:
    return int(node.get_clock().now().nanoseconds / 1000)


def quat_wxyz_to_rotmat(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """
    Rotation 3x3 from quaternion PX4 (w, x, y, z).
    It gives back the rotation matrix body -> world.
    """
    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz),     2.0 * (qx * qy - qz * qw),     2.0 * (qx * qz + qy * qw)],
        [    2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz),     2.0 * (qy * qz - qx * qw)],
        [    2.0 * (qx * qz - qy * qw),     2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ], dtype=float)


def make_camera_matrix(fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    return np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ], dtype=float)


def pixel_to_camera_ray(u: float, v: float, K: np.ndarray) -> np.ndarray:
    """
    Radius in the optical camera frame OpenCV:
    x right, y down, z forward.
    """
    p = np.array([u, v, 1.0], dtype=float)
    ray = np.linalg.inv(K) @ p
    norm = np.linalg.norm(ray)
    if norm < 1e-12:
        return ray
    return ray / norm


def intersect_ray_with_plane(
    ray_origin_w: np.ndarray,
    ray_dir_w: np.ndarray,
    plane_normal_w: np.ndarray,
    plane_offset_d: float
):
    """
    Plane: n^T X + d = 0
    Radius: X = origin + lambda * dir
    """
    denom = float(plane_normal_w @ ray_dir_w)
    if abs(denom) < 1e-9:
        return None

    lam = -float(plane_normal_w @ ray_origin_w + plane_offset_d) / denom
    if lam <= 0.0:
        return None

    return ray_origin_w + lam * ray_dir_w