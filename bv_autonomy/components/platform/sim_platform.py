from bv_autonomy.components.camera import BevyCamera
from bv_autonomy.components.camera.fake_camera import BEVY_INTRINSICS
from bv_autonomy.components.drone import MavsdkDrone
from bv_autonomy.components.payload import FakePayload
from bv_autonomy.components.platform.base import Platform


class SimPlatform(Platform):
    """bv_bevy's simulator: PX4 SITL and Bevy's camera. The simulated aircraft has no payload to drop."""

    def __init__(self, address: str = "udpout://127.0.0.1:14580"):
        """
        Args:
            address: MAVSDK connection URL. The default talks to PX4's onboard link, which bv_bevy publishes.
        """
        self.drone = MavsdkDrone(address)
        self.camera = BevyCamera(self.drone)
        self.payload = FakePayload()
        self.intrinsics = BEVY_INTRINSICS
