import math

import pytest

from bv_autonomy.components.localizer.gimbal_nadir_localizer import GimbalNadirLocalizer
from bv_autonomy.types import Detection, Waypoint


@pytest.fixture
def localizer():
    return GimbalNadirLocalizer(mount_xyz=(math.pi, 0, -math.pi / 2))


def detection(frame, u=50, v=50):
    return Detection("person", 0.9, (u - 2, v - 2, u + 2, v + 2), frame)


def test_image_center_projects_directly_below_drone(localizer, frame):
    frame.pose.position = Waypoint(36, -96, 20)
    point = localizer.locate(detection(frame))
    assert point.lat == pytest.approx(36, abs=1e-10)
    assert point.lon == pytest.approx(-96, abs=1e-10)
    assert point.alt == 0


@pytest.mark.parametrize("yaw,u,v,east,north", [
    (0, 100, 50, 10, 0),
    (0, 50, 0, 0, 10),
    (math.pi / 2, 100, 50, 0, -10),
    (math.pi / 2, 50, 0, 10, 0),
    (math.pi, 100, 50, -10, 0),
])
def test_pixel_offsets_and_yaw_match_hand_computed_ground_position(localizer, frame, yaw, u, v, east, north):
    frame.pose.yaw = yaw
    point = localizer.locate(detection(frame, u, v))
    expected_lon = math.degrees(east / 6378137)
    expected_lat = math.degrees(north / 6335439.32729282)
    assert point.lon == pytest.approx(expected_lon, abs=1e-9)
    assert point.lat == pytest.approx(expected_lat, abs=1e-9)
    assert point.alt == 0


def test_gimbal_ignores_drone_roll_and_pitch(localizer, frame):
    target = detection(frame, 75, 25)
    level = localizer.locate(target)
    frame.pose.roll = 0.9
    frame.pose.pitch = -0.7
    assert localizer.locate(target) == level


@pytest.mark.parametrize("altitude", [0, -10])
def test_nonpositive_height_has_no_ground_intersection(localizer, frame, altitude):
    frame.pose.position.alt = altitude
    assert localizer.locate(detection(frame)) is None


def test_upward_ray_has_no_ground_intersection(frame):
    localizer = GimbalNadirLocalizer(mount_xyz=(0, 0, 0))
    assert localizer.locate(detection(frame)) is None


def test_lens_distortion_is_removed_using_each_frames_calibration(localizer, frame):
    frame.intrinsics.fx, frame.intrinsics.fy = 120, 90
    frame.intrinsics.cx, frame.intrinsics.cy = 40, 60
    frame.intrinsics.dist[:] = (0.1, -0.02, 0.003, -0.004, 0.01)
    x, y = 0.25, -0.2
    radius2 = x * x + y * y
    radial = 1 + 0.1 * radius2 - 0.02 * radius2 ** 2 + 0.01 * radius2 ** 3
    distorted_x = x * radial + 2 * 0.003 * x * y - 0.004 * (radius2 + 2 * x * x)
    distorted_y = y * radial + 0.003 * (radius2 + 2 * y * y) - 2 * 0.004 * x * y
    point = localizer.locate(detection(frame, 40 + 120 * distorted_x, 60 + 90 * distorted_y))
    assert point.lon == pytest.approx(math.degrees(5 / 6378137), abs=1e-9)
    assert point.lat == pytest.approx(math.degrees(4 / 6335439.32729282), abs=1e-9)
