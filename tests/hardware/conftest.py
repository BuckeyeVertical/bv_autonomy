"""Checks of the real components, run by hand and never in CI: pytest tests/hardware -s"""

import os
import sys

import cv2
import pytest
import pytest_asyncio
from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.telemetry import TelemetryAsync

from bv_autonomy import paths
from bv_autonomy.components.detector import LtdetrDetector
from bv_autonomy.components.drone import FakeDrone
from bv_autonomy.components.platform import CompDronePlatform, SimPlatform, X650Platform
from bv_autonomy.missions.suas2026.suas2026 import DETECTION_THRESHOLDS
from bv_autonomy.types import Waypoint

PLATFORMS = {"comp_drone": CompDronePlatform, "x650": X650Platform, "sim": SimPlatform}
WINDOW_MAX = (1280, 720)


def pytest_addoption(parser):
    parser.addoption("--platform", choices=list(PLATFORMS), default="comp_drone",
                     help="which aircraft to check, or sim for bv_bevy's simulator")
    parser.addoption("--image", help="an image for the detector to run on instead of a camera frame")
    parser.addoption("--no-show", action="store_true", help="save images without opening a window")


@pytest.fixture
def platform(pytestconfig):
    return PLATFORMS[pytestconfig.getoption("--platform")]()


@pytest_asyncio.fixture
async def telemetry(platform):
    """PX4's telemetry from the platform's address, without waiting for a GPS fix like the mission does."""
    mavsdk = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
    await mavsdk.add_any_connection(platform.drone._address)
    system = await mavsdk.first_autopilot(timeout_s=30.0)
    if system is None:
        mavsdk.destroy()
        pytest.fail(f"no autopilot found at {platform.drone._address} within 30 s")
    telemetry = TelemetryAsync(system)
    yield telemetry
    telemetry.destroy()
    mavsdk.destroy()


@pytest_asyncio.fixture
async def camera(platform, monkeypatch):
    """The platform's camera on its own, started, with frames stamped with a fixed pose instead of the
    Pixhawk's, so it runs without the drone connected.
    """
    monkeypatch.setattr(platform.drone, "pose", FakeDrone(Waypoint(0.0, 0.0, 0.0)).pose)
    await platform.camera.start()
    yield platform.camera
    await platform.camera.stop()


@pytest_asyncio.fixture
async def detector():
    """The detector the mission uses, with its thresholds, loaded."""
    detector = LtdetrDetector(thresholds=DETECTION_THRESHOLDS)
    await detector.start()
    yield detector
    await detector.stop()


@pytest.fixture(scope="session")
def output_dir():
    """The hardware tests' output folder, with the last run's images moved into its old folder."""
    paths.archive_outputs(paths.HARDWARE_TESTS_OUTPUT)
    paths.HARDWARE_TESTS_OUTPUT.mkdir(parents=True, exist_ok=True)
    return paths.HARDWARE_TESTS_OUTPUT


@pytest.fixture
def show(pytestconfig, output_dir):
    """Saves an image to output/hardware_tests, then shows it in a window until a key is pressed or the
    window is closed. With no display, such as over SSH, it only saves it.
    """

    def show(name, image):
        path = output_dir / f"{name}.jpg"
        cv2.imwrite(str(path), image)
        print(f"saved {path}")
        if pytestconfig.getoption("--no-show") or not _has_display():
            return
        height, width = image.shape[:2]
        scale = min(1.0, WINDOW_MAX[0] / width, WINDOW_MAX[1] / height)
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, round(width * scale), round(height * scale))
        cv2.imshow(name, image)
        while cv2.waitKey(100) < 0 and cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) >= 1:
            pass
        cv2.destroyWindow(name)
        cv2.waitKey(1)

    return show


def _has_display() -> bool:
    if sys.platform != "linux":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
