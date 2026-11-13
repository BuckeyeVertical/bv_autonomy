"""SIYI A8 mini over HDMI capture (V4L2) at its 1080p HDMI output, with a 1080p checkerboard calibration."""

import numpy as np

from bv_autonomy.components.camera.gstreamer_camera import GstreamerCamera
from bv_autonomy.components.drone import Drone
from bv_autonomy.types import Intrinsics

SIYI_A8_INTRINSICS = Intrinsics(
    width=1920,
    height=1080,
    fx=1069.6081019666549,
    fy=1067.909907222426,
    cx=1008.5225984934441,
    cy=573.0590297391683,
    dist=np.array([-0.12109121, 0.10084346, 0.00012351, 0.00106712, -0.0124725]),
)


def siyi_a8_pipeline(device: str = "/dev/video0", fps: int = 30) -> str:
    """Asks the capture card for the A8's own 1080p, since a larger size is only upscaled."""
    return (
        f"v4l2src device={device}"
        f" ! image/jpeg,width={SIYI_A8_INTRINSICS.width},height={SIYI_A8_INTRINSICS.height},framerate={fps}/1"
        " ! jpegdec ! videoconvert ! appsink drop=true sync=false"
    )


class SiyiA8Camera(GstreamerCamera):
    def __init__(self, drone: Drone, device: str = "/dev/video0", fps: int = 30):
        super().__init__(drone, siyi_a8_pipeline(device, fps), SIYI_A8_INTRINSICS)
