"""Data passed between components and missions."""

from dataclasses import dataclass

import numpy as np


@dataclass
class Waypoint:
    lat: float
    lon: float
    alt: float               # meters above the ground; Terrain.msl() gives it above sea level


class WaypointSequence(list[Waypoint]):
    """Waypoints in the order they're flown. Build one from any list: WaypointSequence([a, b, c])."""


@dataclass
class Pose:
    position: Waypoint
    # Radians, aircraft convention (as MAVSDK and PX4 report it): yaw 0 is north and
    # increases clockwise, pitch is positive nose up, roll is positive right wing down.
    roll: float
    pitch: float
    yaw: float
    time: float              # one clock everywhere


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    dist: np.ndarray         # distortion coefficients; bv_core's localizer undistorts with these


@dataclass
class Frame:
    image: np.ndarray        # BGR
    time: float              # capture time, same clock as Pose.time
    pose: Pose               # drone pose at capture
    intrinsics: Intrinsics   # Bevy sends these per frame; a real camera uses its calibration
    sequence: int            # counts up per frame, so a repeated frame is easy to spot


@dataclass
class Detection:
    label: str               # "person", "tent", "marker"; the mission maps labels to actions
    confidence: float
    box: tuple[float, float, float, float]   # x1, y1, x2, y2 in pixels
    frame: Frame             # the frame it came from, so the localizer has its pose


@dataclass
class ApprovalRequest:
    label: str
    position: Waypoint
    confidence: float
    crop: bytes              # JPEG, b"" if none
    drone_position: Waypoint
