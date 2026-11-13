import math

import pytest

from bv_autonomy.components.path_planner.s_path_planner import SPathPlanner
from bv_autonomy.types import Waypoint, WaypointSequence


METERS_PER_DEGREE = math.pi * 6378137 / 180


@pytest.fixture
def rectangle():
    return WaypointSequence([
        Waypoint(0, 0, 0), Waypoint(0, 100 / METERS_PER_DEGREE, 0),
        Waypoint(60 / METERS_PER_DEGREE, 100 / METERS_PER_DEGREE, 0),
        Waypoint(60 / METERS_PER_DEGREE, 0, 0),
    ])


@pytest.fixture
def planner(frame):
    return SPathPlanner(frame.intrinsics, altitude=20, overlap=0.5)


@pytest.mark.parametrize("span", [60, 63])
def test_long_rows_alternate_direction_and_cover_area(planner, rectangle, span):
    rectangle[2].lat = rectangle[3].lat = span / METERS_PER_DEGREE
    route = planner.plan(rectangle)
    assert isinstance(route, WaypointSequence)
    assert len(route) >= 2 and len(route) % 2 == 0
    assert all(isinstance(point, Waypoint) and point.alt == 20 for point in route)
    rows = list(zip(route[::2], route[1::2]))
    centers = []
    directions = []
    for start, end in rows:
        assert start.lat == pytest.approx(end.lat)
        assert sorted([start.lon, end.lon]) == pytest.approx([0, 100 / METERS_PER_DEGREE])
        centers.append(start.lat * METERS_PER_DEGREE)
        directions.append(1 if end.lon > start.lon else -1)
    assert directions == [(-1) ** i for i in range(len(rows))]
    assert centers[0] == pytest.approx(10)
    assert centers[-1] == pytest.approx(span - 10)
    assert all(0 < b - a <= 10 + 1e-6 for a, b in zip(centers, centers[1:]))


def test_short_sweep_runs_along_short_side(frame, rectangle):
    planner = SPathPlanner(frame.intrinsics, 20, 0.5, sweep="short")
    route = planner.plan(rectangle)
    for start, end in zip(route[::2], route[1::2]):
        assert start.lon == pytest.approx(end.lon)
        assert sorted([start.lat, end.lat]) == pytest.approx([0, 60 / METERS_PER_DEGREE])


@pytest.mark.parametrize("side", ["top", "bottom"])
def test_requested_start_side_preserves_coverage(frame, rectangle, side):
    default = SPathPlanner(frame.intrinsics, 20, 0.5).plan(rectangle)
    route = SPathPlanner(frame.intrinsics, 20, 0.5, start=side).plan(rectangle)
    assert route[0].lat > route[-1].lat if side == "top" else route[0].lat < route[-1].lat
    assert route == default or route == list(reversed(default))


def test_area_smaller_than_footprint_uses_one_centered_row(frame, rectangle):
    planner = SPathPlanner(frame.intrinsics, 100, 0.5)
    route = planner.plan(rectangle)
    assert len(route) == 2
    assert route[0].lat * METERS_PER_DEGREE == pytest.approx(30)
    assert route[1].lat == route[0].lat


@pytest.mark.parametrize("changes,message", [
    ({"altitude": 0}, "altitude"),
    ({"altitude": -1}, "altitude"),
    ({"overlap": -0.01}, "overlap"),
    ({"overlap": 1}, "overlap"),
    ({"sweep": "diagonal"}, "sweep"),
    ({"start": "left"}, "start"),
])
def test_invalid_planner_settings_rejected(frame, changes, message):
    settings = dict(intrinsics=frame.intrinsics, altitude=20, overlap=0.5)
    settings.update(changes)
    with pytest.raises(ValueError, match=message):
        SPathPlanner(**settings)


@pytest.mark.parametrize("field", ["width", "fx"])
def test_nonpositive_width_or_focal_length_rejected(frame, field):
    setattr(frame.intrinsics, field, 0)
    with pytest.raises(ValueError, match="width and focal length"):
        SPathPlanner(frame.intrinsics, 20, 0.5)


@pytest.mark.parametrize("count", [0, 3, 5])
def test_plan_requires_four_corners(planner, rectangle, count):
    boundary = WaypointSequence((rectangle + rectangle)[:count])
    with pytest.raises(ValueError, match="four"):
        planner.plan(boundary)


def test_rotated_scan_at_field_latitude_with_shifted_corner_order(planner):
    angle = math.pi / 6
    along, across = (math.cos(angle), math.sin(angle)), (-math.sin(angle), math.cos(angle))
    boundary = []
    for x, y in [(0, 0), (100, 0), (100, 60), (0, 60)]:
        east, north = x * along[0] + y * across[0], x * along[1] + y * across[1]
        boundary.append(Waypoint(
            36 + north / METERS_PER_DEGREE,
            -96 + east / (METERS_PER_DEGREE * math.cos(math.radians(36))), 0,
        ))
    boundary = WaypointSequence(boundary[1:] + boundary[:1])
    route = planner.plan(boundary)

    def scan_coordinates(point):
        east = (point.lon + 96) * METERS_PER_DEGREE * math.cos(math.radians(36))
        north = (point.lat - 36) * METERS_PER_DEGREE
        return east * along[0] + north * along[1], east * across[0] + north * across[1]

    centers = []
    for start, end in zip(route[::2], route[1::2]):
        x1, y1 = scan_coordinates(start)
        x2, y2 = scan_coordinates(end)
        assert sorted([x1, x2]) == pytest.approx([0, 100], abs=0.02)
        assert y1 == pytest.approx(y2, abs=0.02)
        centers.append(y1)
    centers.sort()
    assert [centers[0], centers[-1]] == pytest.approx([10, 50], abs=0.02)
    assert all(0 < b - a <= 10.01 for a, b in zip(centers, centers[1:]))
