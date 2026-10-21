"""Sony RX0 II over HDMI capture (V4L2), with bv_core's calibration."""

import numpy as np

from bv_autonomy.components.camera.gstreamer_camera import GstreamerCamera
from bv_autonomy.components.drone import Drone
from bv_autonomy.types import Intrinsics

# Calibrated at 3840x2160 (bv_core config/filtering_params.yaml). Changing the
# resolution below means recalibrating, or scaling fx, fy, cx and cy to match.
RX0_INTRINSICS = Intrinsics(
    fx=2296.4008369677636,
    fy=2312.535785524523,
    cx=1939.13003396931,
    cy=1056.8106645680252,
    dist=np.array([0.04454957, -0.10911878, -0.00299501, 0.00620382, 0.16651256]),
)


def rx0_pipeline(device: str = "/dev/video0", fps: int = 30) -> str:
    # Same flags as bv_core's working config (config/vision_params.yaml, gst_pipeline).
    return (
        f"v4l2src device={device}"
        f" ! image/jpeg,width=3840,height=2160,framerate={fps}/1"
        " ! jpegdec ! videoconvert ! appsink drop=true sync=false"
    )


class Rx0Camera(GstreamerCamera):
    def __init__(self, drone: Drone, device: str = "/dev/video0", fps: int = 30):
        super().__init__(drone, rx0_pipeline(device, fps), RX0_INTRINSICS)
