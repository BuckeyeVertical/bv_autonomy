"""Targets placed on the ground for the fake camera and detector to see."""

import math
from dataclasses import dataclass

from bv_autonomy.types import GeoPoint, Intrinsics, Pose

_EARTH_R = 6378137.0


@dataclass
class Target:
    label: str
    position: GeoPoint
    size_m: float = 1.5          # width and length on the ground
    color_bgr: tuple[int, int, int] = (40, 40, 220)


def project(target: Target, pose: Pose, intrinsics: Intrinsics, width: int, height: int):
    """Where the target appears in a downward camera's image, as a clipped (x1, y1, x2, y2), or None.

    Same camera as the Bevy sim and bv_core's gimbal: pointing straight down, the image's top
    toward the drone's nose and its right toward the drone's right, roll and pitch ignored.
    """
    h = pose.position.alt
    if h <= 0:
        return None
    east = _EARTH_R * math.radians(target.position.lon - pose.position.lon) * math.cos(math.radians(pose.position.lat))
    north = _EARTH_R * math.radians(target.position.lat - pose.position.lat)
    forward = north * math.cos(pose.yaw) + east * math.sin(pose.yaw)
    right = -north * math.sin(pose.yaw) + east * math.cos(pose.yaw)
    u = intrinsics.fx * right / h + intrinsics.cx
    v = intrinsics.fy * -forward / h + intrinsics.cy
    half_w = intrinsics.fx * target.size_m / h / 2
    half_h = intrinsics.fy * target.size_m / h / 2
    x1, y1, x2, y2 = max(0.0, u - half_w), max(0.0, v - half_h), min(width, u + half_w), min(height, v + half_h)
    if x2 - x1 < 1 or y2 - y1 < 1:
        return None
    return (x1, y1, x2, y2)
