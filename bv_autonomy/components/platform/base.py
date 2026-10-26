from abc import ABC

from bv_autonomy.components.camera import Camera
from bv_autonomy.components.drone import Drone
from bv_autonomy.components.payload import Payload
from bv_autonomy.types import Intrinsics


class Platform(ABC):
    """What a mission flies with: the parts that differ between the simulator and the aircraft.

    Implementations set these in their constructor.
    """

    drone: Drone
    camera: Camera
    payload: Payload
    intrinsics: Intrinsics
    async def stop(self) -> None:
        """Stops the camera, leaves the payload at rest, then disconnects the drone."""
        await self.camera.stop()
        await self.payload.stop()
        await self.drone.disconnect()
