from bv_autonomy.components.camera import Rx0Camera
from bv_autonomy.components.camera.rx0_camera import RX0_INTRINSICS
from bv_autonomy.components.drone import MavsdkDrone
from bv_autonomy.components.payload import FakePayload
from bv_autonomy.components.platform.base import Platform


class X650Platform(Platform):
    """The X650, with PX4 over serial and the RX0. It carries no payload."""

    def __init__(self, address: str = "serial:///dev/ttyTHS1:921600"):
        """Sets up the aircraft's drone, camera and payload."""
        self.drone = MavsdkDrone(address)
        self.camera = Rx0Camera(self.drone)
        self.payload = FakePayload()
        self.intrinsics = RX0_INTRINSICS
