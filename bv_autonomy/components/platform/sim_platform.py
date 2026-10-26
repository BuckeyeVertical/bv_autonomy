import sys

from bv_autonomy.components.camera import BevyCamera
from bv_autonomy.components.camera.fake_camera import BEVY_INTRINSICS
from bv_autonomy.components.drone import MavsdkDrone
from bv_autonomy.components.payload import FakePayload
from bv_autonomy.components.platform.base import Platform


def default_connection() -> str:
    """Returns the MAVSDK address that reaches bv_bevy's PX4 from this computer.

    On Windows PX4 sends to the host on 14540, so every new run can reconnect. On the Mac that link loses
    mission uploads through OrbStack, so it uses PX4's onboard link on 14580, which accepts one program per
    simulator start.
    """
    if sys.platform == "win32":
        return "udpin://0.0.0.0:14540"
    return "udpout://127.0.0.1:14580"


class SimPlatform(Platform):
    """bv_bevy's simulator: PX4 SITL and Bevy's camera. The simulated aircraft has no payload to drop."""

    def __init__(self, address: str | None = None):
        """Sets up the simulator's drone, camera and payload."""
        self.drone = MavsdkDrone(address or default_connection())
        self.camera = BevyCamera(self.drone)
        self.payload = FakePayload()
        self.intrinsics = BEVY_INTRINSICS
