import logging

from bv_autonomy.components.camera import Rx0Camera
from bv_autonomy.components.camera.rx0_camera import RX0_INTRINSICS
from bv_autonomy.components.drone import MavsdkDrone
from bv_autonomy.components.payload import FakePayload
from bv_autonomy.components.platform.base import Platform

log = logging.getLogger(__name__)


class PhysicalPlatform(Platform):
    """The aircraft: PX4 over serial and the RX0."""

    def __init__(self, address: str = "serial:///dev/ttyTHS1:921600"):
        """
        Args:
            address: MAVSDK connection URL for the flight controller.
        """
        self.drone = MavsdkDrone(address)
        self.camera = Rx0Camera(self.drone)
        log.warning("PhysicalPlatform has no payload driver yet; deliveries won't drop anything")
        self.payload = FakePayload()
        self.intrinsics = RX0_INTRINSICS
