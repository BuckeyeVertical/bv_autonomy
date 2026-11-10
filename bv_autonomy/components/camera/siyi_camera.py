"""Sony RX0 II over HDMI capture (V4L2), with bv_core's calibration."""

import numpy as np

from bv_autonomy.components.camera.gstreamer_camera import GstreamerCamera
from bv_autonomy.components.drone import Drone
from bv_autonomy.types import Intrinsics

SIYI_INTRINSICS = Intrinsics(
    width=3840,
    height=2160,
    fx=701.8975,
    fy=702.2612,
    cx=666.7347,
    cy=380.5172,
    dist=np.array([0.04454957, -0.10911878, -0.00299501, 0.00620382, 0.16651256]),
)


def rx0_pipeline(device: str = "/dev/video0", fps: int = 30) -> str:
    return (
        f"v4l2src device={device}"
        f" ! image/jpeg,width={SIYI_INTRINSICS.width},height={SIYI_INTRINSICS.height},framerate={fps}/1"
        " ! jpegdec ! videoconvert ! appsink drop=true sync=false"
    )


class Rx0Camera(GstreamerCamera):
    def __init__(self, drone: Drone, device: str = "/dev/video0", fps: int = 30):
        super().__init__(drone, rx0_pipeline(device, fps), SIYI_INTRINSICS)
