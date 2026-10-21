"""Data passed between components and missions."""

from dataclasses import dataclass

import numpy as np


@dataclass
class GeoPoint:
    lat: float
    lon: float
    alt_amsl: float          # meters above sea level


@dataclass
class Pose:
    position: GeoPoint
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
    position: GeoPoint
    confidence: float
    crop: bytes              # JPEG, b"" if none
    drone_position: GeoPoint
