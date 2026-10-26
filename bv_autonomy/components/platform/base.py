import logging
from abc import ABC
from collections.abc import Awaitable, Callable

from bv_autonomy.components.camera import Camera
from bv_autonomy.components.drone import Drone
from bv_autonomy.components.payload import Payload
from bv_autonomy.types import Intrinsics

log = logging.getLogger(__name__)


async def stop_all(*stops: Callable[[], Awaitable[None]]) -> None:
    """Runs each stop in order, carrying on past any that fail, then raises the first failure after logging each."""
    errors = []
    for stop in stops:
        try:
            await stop()
        except Exception as error:
            log.exception("%s failed while stopping", getattr(stop, "__qualname__", stop))
            errors.append(error)
    if errors:
        raise errors[0]


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
        await stop_all(self.camera.stop, self.payload.stop, self.drone.disconnect)
