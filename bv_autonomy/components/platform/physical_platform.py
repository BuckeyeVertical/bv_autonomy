from bv_autonomy.components.camera import Rx0Camera
from bv_autonomy.components.camera.rx0_camera import RX0_INTRINSICS
from bv_autonomy.components.drone import MavsdkDrone
from bv_autonomy.components.payload import WinchPayload
from bv_autonomy.components.platform.base import Platform


class PhysicalPlatform(Platform):
    """The aircraft, with PX4 over serial, the RX0 and the plate and clamp payload."""

    def __init__(self, address: str = "serial:///dev/ttyTHS1:921600"):
        """Sets up the aircraft's drone, camera and payload."""
        self.drone = MavsdkDrone(address)
        self.camera = Rx0Camera(self.drone)
        self.payload = WinchPayload(self.drone)
        self.intrinsics = RX0_INTRINSICS
