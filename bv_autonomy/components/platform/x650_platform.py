from bv_autonomy.components.camera import SiyiA8Camera
from bv_autonomy.components.camera.siyi_a8_camera import SIYI_A8_INTRINSICS
from bv_autonomy.components.drone import MavsdkDrone
from bv_autonomy.components.payload import FakePayload
from bv_autonomy.components.platform.base import Platform


class X650Platform(Platform):
    """The X650, with PX4 over serial and the SIYI A8 mini. It carries no payload."""

    def __init__(self, address: str = "serial:///dev/ttyTHS1:921600"):
        """Sets up the aircraft's drone, camera and payload."""
        self.drone = MavsdkDrone(address)
        self.camera = SiyiA8Camera(self.drone)
        self.payload = FakePayload()
        self.intrinsics = SIYI_A8_INTRINSICS
