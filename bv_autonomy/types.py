"""Data passed between components and missions."""

from dataclasses import dataclass

import numpy as np


@dataclass
class Waypoint:
    lat: float
    lon: float
    alt: float


class WaypointSequence(list[Waypoint]):
    """Waypoints in the order they're flown. Build one from any list: WaypointSequence([a, b, c])."""

    @classmethod
    def at_alt(cls, alt: float, points: list[tuple[float, float]]) -> "WaypointSequence":
        """Creates waypoints at one altitude from latitude and longitude pairs."""
        return cls(Waypoint(lat, lon, alt) for lat, lon in points)


@dataclass
class Pose:
    position: Waypoint
    roll: float
    pitch: float
    yaw: float
    time: float


@dataclass
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist: np.ndarray


@dataclass
class Frame:
    image: np.ndarray
    time: float
    pose: Pose
    intrinsics: Intrinsics
    sequence: int


@dataclass
class Detection:
    label: str
    confidence: float
    box: tuple[float, float, float, float]
    frame: Frame


@dataclass
class ApprovalRequest:
    label: str
    position: Waypoint
    confidence: float
    crop: bytes
    drone_position: Waypoint
