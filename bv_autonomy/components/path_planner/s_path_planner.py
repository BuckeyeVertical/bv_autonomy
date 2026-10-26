"""Back-and-forth (S-shaped) scan over a four-corner area, as in bv_core's scan_plan.py."""

import math
from typing import Literal

from bv_autonomy.components.path_planner.base import PathPlanner
from bv_autonomy.types import Intrinsics, Waypoint, WaypointSequence

_EARTH_R = 6378137.0


class SPathPlanner(PathPlanner):
    """Flies rows across the area, alternating direction, spaced so the camera's views overlap.

    Rows run along the area's longer side by default. Row spacing comes from how wide the
    camera sees on the ground at the scan altitude: width * altitude / fx, for a
    downward camera with the image width across the direction of flight.
    """

    def __init__(
        self,
        intrinsics: Intrinsics,
        altitude: float,
        overlap: float,
        sweep: Literal["long", "short"] = "long",
        start: Literal["top", "bottom"] | None = None,
    ):
        """Sets up a planner for a camera flying at the given altitude, with rows spaced so neighboring views
        overlap.
        """
        if altitude <= 0:
            raise ValueError("altitude must be positive")
        if intrinsics.width <= 0 or intrinsics.fx <= 0:
            raise ValueError("image width and focal length must be positive")
        if not 0.0 <= overlap < 1.0:
            raise ValueError("overlap must be in [0, 1)")
        if sweep not in ("long", "short"):
            raise ValueError("sweep must be 'long' or 'short'")
        if start not in (None, "top", "bottom"):
            raise ValueError("start must be 'top' or 'bottom'")

        self._footprint_m = intrinsics.width * altitude / intrinsics.fx
        self._overlap = overlap
        self._altitude = altitude
        self._sweep = sweep
        self._start = start

    def plan(self, boundary: WaypointSequence) -> WaypointSequence:
        """Returns the start and end of each row needed to cover the area inside four corners."""
        if len(boundary) != 4:
            raise ValueError("boundary must contain four ordered corners")
        points = [(p.lat, p.lon) for p in boundary]

        pair_a = (_distance_m(points[0], points[1]) + _distance_m(points[2], points[3])) / 2.0
        pair_b = (_distance_m(points[1], points[2]) + _distance_m(points[3], points[0])) / 2.0
        use_pair_a = pair_a >= pair_b
        if self._sweep == "short":
            use_pair_a = not use_pair_a
        if use_pair_a:
            starts, ends = (points[0], points[3]), (points[1], points[2])
        else:
            starts, ends = (points[1], points[0]), (points[2], points[3])

        cross_span_m = (_distance_m(starts[0], starts[1]) + _distance_m(ends[0], ends[1])) / 2.0
        offsets_m = _row_offsets(cross_span_m, self._footprint_m, self._overlap)

        route = []
        for index, offset_m in enumerate(offsets_m):
            fraction = 0.5 if cross_span_m == 0.0 else offset_m / cross_span_m
            row_start = _interpolate(starts[0], starts[1], fraction)
            row_end = _interpolate(ends[0], ends[1], fraction)
            if index % 2:
                row_start, row_end = row_end, row_start
            route.extend([row_start, row_end])

        if self._start is not None and len(route) >= 2:
            first_is_top = route[0][0] >= route[-1][0]
            if first_is_top != (self._start == "top"):
                route.reverse()

        return WaypointSequence(Waypoint(lat, lon, self._altitude) for lat, lon in route)


def _row_offsets(span_m: float, footprint_m: float, overlap: float) -> list[float]:
    """Returns how far each row's center line is from the first side of the area, in meters."""
    if span_m <= footprint_m:
        return [span_m / 2.0]
    center_span_m = span_m - footprint_m
    maximum_spacing_m = footprint_m * (1.0 - overlap)
    intervals = max(1, math.ceil(center_span_m / maximum_spacing_m))
    spacing_m = center_span_m / intervals
    return [footprint_m / 2.0 + index * spacing_m for index in range(intervals + 1)]


def _distance_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    dx = _EARTH_R * math.radians(a[1] - b[1]) * math.cos(math.radians(b[0]))
    dy = _EARTH_R * math.radians(a[0] - b[0])
    return math.hypot(dx, dy)


def _interpolate(start: tuple[float, float], end: tuple[float, float], fraction: float) -> tuple[float, float]:
    return (start[0] + (end[0] - start[0]) * fraction, start[1] + (end[1] - start[1]) * fraction)
