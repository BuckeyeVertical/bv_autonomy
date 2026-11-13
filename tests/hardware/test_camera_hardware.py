import time

FRAME_TIMEOUT_S = 10.0
RATE_WINDOW_S = 2.0


async def test_camera_streams_frames(camera, show):
    """Takes one frame at the resolution the camera is calibrated for, measures the frame rate, and shows the
    frame.
    """
    frame = await camera.latest(timeout=FRAME_TIMEOUT_S)
    assert frame is not None, f"no frame within {FRAME_TIMEOUT_S:.0f} s"
    intrinsics = frame.intrinsics
    assert frame.image.shape[:2] == (intrinsics.height, intrinsics.width)

    count, started = 0, time.monotonic()
    while time.monotonic() - started < RATE_WINDOW_S:
        if await camera.latest(timeout=1.0) is not None:
            count += 1
    print(f"{frame.image.shape[1]}x{frame.image.shape[0]} at {count / RATE_WINDOW_S:.1f} frames/s")
    assert count > 0
    show("camera", frame.image)
