"""Targets placed on the ground for the fake camera and detector to see."""

import math
from dataclasses import dataclass

from bv_autonomy.types import Waypoint, Intrinsics, Pose

_EARTH_R = 6378137.0


@dataclass
class Target:
    label: str
    position: Waypoint
    size_m: float = 1.5
    color_bgr: tuple[int, int, int] = (40, 40, 220)


def project(target: Target, pose: Pose, intrinsics: Intrinsics, width: int, height: int):
    """Returns where a target appears in a downward camera's image, or None if it's out of view."""
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
