import time

import cv2
import numpy as np
import pytest

from bv_autonomy.types import Frame, Intrinsics, Pose, Waypoint

FRAME_TIMEOUT_S = 10.0
BOX_BGR = (84, 220, 64)


@pytest.mark.skipif("config.getoption('--image')", reason="--image given")
async def test_detects_camera_frame(detector, camera, show):
    """Runs the mission's detector on one frame from the platform's camera."""
    frame = await camera.latest(timeout=FRAME_TIMEOUT_S)
    assert frame is not None, f"no frame within {FRAME_TIMEOUT_S:.0f} s"
    await detect_and_show(detector, frame, show)


async def test_detects_image(detector, pytestconfig, show):
    """Runs the mission's detector on the image given with --image, with no drone or camera."""
    path = pytestconfig.getoption("--image")
    if path is None:
        pytest.skip("no --image given")
    image = cv2.imread(path)
    assert image is not None, f"can't read {path}"
    height, width = image.shape[:2]
    intrinsics = Intrinsics(width, height, fx=width, fy=width, cx=width / 2, cy=height / 2, dist=np.zeros(5))
    frame = Frame(image, time=0.0, pose=Pose(Waypoint(0, 0, 0), 0, 0, 0, 0.0), intrinsics=intrinsics, sequence=0)
    await detect_and_show(detector, frame, show)


async def detect_and_show(detector, frame, show):
    started = time.monotonic()
    detections = await detector.detect(frame)
    print(f"{len(detections)} detections in {time.monotonic() - started:.2f} s")
    image = frame.image.copy()
    for detection in detections:
        label = f"{detection.label} {detection.confidence:.2f}"
        print(f"{label} at {tuple(round(v) for v in detection.box)}")
        x1, y1, x2, y2 = (round(v) for v in detection.box)
        cv2.rectangle(image, (x1, y1), (x2, y2), BOX_BGR, 2)
        cv2.putText(image, label, (x1, max(y1 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, BOX_BGR, 2, cv2.LINE_AA)
    show("detections", image)
