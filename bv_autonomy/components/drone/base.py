from abc import ABC, abstractmethod
from collections.abc import Callable

from bv_autonomy.types import Pose, Waypoint, WaypointSequence


class Drone(ABC):
    @abstractmethod
    async def connect(self) -> None: ...

    @abstractmethod
    async def arm(self) -> None: ...

    @abstractmethod
    async def disarm(self) -> None: ...

    @abstractmethod
    async def takeoff(self, alt: float) -> None:
        """Takes off and climbs to the given altitude above the ground."""

    @abstractmethod
    async def goto(self, point: Waypoint, speed: float | None = None, tolerance: float = 1.0) -> None:
        """Flies to a point and returns once the drone is within the tolerance of it."""

    @abstractmethod
    async def fly_mission(
        self,
        route: WaypointSequence,
        speed: float | None = None,
        tolerance: float = 1.0,
        on_progress: Callable[[int], None] | None = None,
    ) -> None:
        """Flies through every point of a route in order without stopping at each, reporting progress as
        points are reached.
        """

    @abstractmethod
    async def hold(self) -> None:
        """Stops the drone and hovers in place."""

    @abstractmethod
    async def land(self) -> None:
        """Lands where the drone is."""

    @abstractmethod
    async def return_home(self) -> None:
        """Flies back to the home position and lands there."""


    @abstractmethod
    def pose(self) -> Pose:
        """Returns the drone's latest known position and attitude."""

    @abstractmethod
    def position(self) -> Waypoint:
        """Returns the drone's latest known position."""

    @abstractmethod
    def heading(self) -> float:
        """Returns the direction the drone faces, in degrees clockwise from north."""

    @abstractmethod
    def home(self) -> Waypoint | None:
        """Returns the home position, or None if it isn't known yet."""

    @abstractmethod
    def battery(self) -> float:
        """Returns the remaining battery charge as a fraction from 0 to 1."""

    @abstractmethod
    def is_armed(self) -> bool: ...

    @abstractmethod
    def in_air(self) -> bool: ...
