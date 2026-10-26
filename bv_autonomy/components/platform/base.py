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

    async def start(self) -> None:
        """Connects the drone, then puts the payload at rest and starts the camera, which both need it."""
        await self.drone.connect()
        await self.payload.start()
        await self.camera.start()

    async def stop(self) -> None:
        """Stops the camera, leaves the payload at rest, then disconnects the drone."""
        await self.camera.stop()
        await self.payload.stop()
        await self.drone.disconnect()
