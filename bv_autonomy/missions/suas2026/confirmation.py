"""Confirming targets over several frames, as bv_core's filtering node did."""

import math
from collections import deque
from dataclasses import dataclass

from bv_autonomy.types import Waypoint

_EARTH_R = 6378137.0


@dataclass
class Sighting:
    label: str
    position: Waypoint


class ConfirmationWindow:
    """A label is found once it's seen in `hits` of the last `frames` frames, each sighting within
    `radius_m` of the one before.

    Hits needn't be consecutive: requiring that threw away real targets on one missed frame, and
    a false positive that flickers between places is caught by the distance check anyway.
    Pass only sightings still worth confirming; the window doesn't know which targets are handled.
    """

    def __init__(self, class_ids: dict[str, int], hits: int, frames: int, radius_m: float):
        """
        Args:
            class_ids: The target labels, with the ids the dashboard shows them under.
        """
        if not 1 <= hits <= frames:
            raise ValueError("hits must be between 1 and frames")
        self._class_ids = dict(class_ids)
        self._hits = hits
        self._radius_m = radius_m
        self._frames: deque[list[Sighting]] = deque(maxlen=frames)

    def clear(self) -> None:
        self._frames.clear()

    def add(self, sightings: list[Sighting]) -> Sighting | None:
        """Add one frame's sightings. Returns the target, at its sightings' centroid, once a label confirms."""
        self._frames.append(sightings)
        for label in self._class_ids:
            # The first sighting of the label in each frame that has one.
            seen = [next(s for s in frame if s.label == label) for frame in self._frames
                    if any(s.label == label for s in frame)]
            if len(seen) < self._hits:
                continue
            if all(distance_m(a.position, b.position) <= self._radius_m for a, b in zip(seen, seen[1:])):
                return Sighting(label, _centroid([s.position for s in seen]))
        return None

    def summary(self, found: set[str]) -> dict:
        """The window in the dashboard's confirm_window format. found: labels already confirmed."""
        return {
            "required": self._hits,
            "window": self._frames.maxlen,
            "frames": len(self._frames),
            "classes": [
                {"class_id": class_id, "name": label, "confirmed": label in found,
                 "hits": [any(s.label == label for s in frame) for frame in self._frames]}
                for label, class_id in self._class_ids.items()
            ],
        }


def wanted(sighting: Sighting, targets: list[Sighting], rejected: list[Sighting], radius_m: float) -> bool:
    """Whether a sighting could be a target still to find: not a label already approved, nor near a rejected spot."""
    if any(t.label == sighting.label for t in targets):
        return False
    return not any(r.label == sighting.label and distance_m(r.position, sighting.position) <= radius_m
                   for r in rejected)


def distance_m(a: Waypoint, b: Waypoint) -> float:
    """Ground distance, flat-earth: fine over a competition field."""
    north = _EARTH_R * math.radians(b.lat - a.lat)
    east = _EARTH_R * math.radians(b.lon - a.lon) * math.cos(math.radians(a.lat))
    return math.hypot(north, east)


def _centroid(points: list[Waypoint]) -> Waypoint:
    return Waypoint(sum(p.lat for p in points) / len(points), sum(p.lon for p in points) / len(points), 0.0)
