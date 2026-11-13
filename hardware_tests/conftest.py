"""Checks of the real components, run by hand and never in CI: pytest hardware_tests -s"""

import os
import sys

import cv2
import pytest
import pytest_asyncio

from bv_autonomy import paths
from bv_autonomy.components.camera import BevyCamera, Rx0Camera
from bv_autonomy.components.detector import LtdetrDetector
from bv_autonomy.components.drone import FakeDrone
from bv_autonomy.components.platform import PhysicalPlatform, SimPlatform
from bv_autonomy.missions.suas2026.suas2026 import DETECTION_THRESHOLDS
from bv_autonomy.types import Waypoint

WINDOW_MAX = (1280, 720)


def pytest_addoption(parser):
    parser.addoption("--platform", choices=["sim", "real"], default="real",
                     help="what to check: bv_bevy's simulator or the aircraft")
    parser.addoption("--image", help="an image for the detector to run on instead of a camera frame")
    parser.addoption("--no-show", action="store_true", help="save images without opening a window")


@pytest.fixture
def platform(pytestconfig):
    if pytestconfig.getoption("--platform") == "sim":
        return SimPlatform()
    return PhysicalPlatform()


@pytest_asyncio.fixture
async def drone(platform):
    """The platform's drone, connected once PX4 has a position fix, a home position and telemetry."""
    await platform.drone.connect()
    yield platform.drone
    await platform.drone.disconnect()


@pytest_asyncio.fixture
async def camera(pytestconfig):
    """The platform's camera on its own, started, with frames stamped with a fixed pose instead of the
    Pixhawk's, so it runs without the drone connected.
    """
    stand_in = FakeDrone(Waypoint(0.0, 0.0, 0.0))
    camera = BevyCamera(stand_in) if pytestconfig.getoption("--platform") == "sim" else Rx0Camera(stand_in)
    await camera.start()
    yield camera
    await camera.stop()


@pytest_asyncio.fixture
async def detector():
    """The detector the mission uses, with its thresholds, loaded."""
    detector = LtdetrDetector(thresholds=DETECTION_THRESHOLDS)
    await detector.start()
    yield detector
    await detector.stop()


@pytest.fixture(scope="session")
def output_dir():
    directory = paths.new_run_dir("hardware_tests")
    directory.mkdir(parents=True)
    return directory


@pytest.fixture
def show(pytestconfig, output_dir):
    """Saves an image to this run's output folder, then shows it in a window until a key is pressed or the
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
