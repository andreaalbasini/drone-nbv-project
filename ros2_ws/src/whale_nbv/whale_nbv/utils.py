#!/usr/bin/env python3

import math


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