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
        """Climb to alt meters above the ground. Finishes once at that altitude."""

    @abstractmethod
    async def goto(self, point: Waypoint, speed: float | None = None, tolerance: float = 1.0) -> None:
        """Fly to point. Finishes once the drone is within tolerance meters of it.

        A large tolerance finishes early, so the next goto redirects the drone before it slows
        down (bv_core flew laps with 40 m and scans with 1 m).
        """

    @abstractmethod
    async def fly_mission(
        self,
        route: WaypointSequence,
        speed: float | None = None,
        tolerance: float = 1.0,
        on_progress: Callable[[int], None] | None = None,
    ) -> None:
        """Fly through every point of route in order as one mission, without stopping at each.

        A point counts as reached within tolerance meters. Finishes once the last point is reached.
        on_progress(reached) is called with 0 at the start and again each time a point is reached.
        """

    @abstractmethod
    async def hold(self) -> None:
        """Stop and hover in place."""

    @abstractmethod
    async def land(self) -> None:
        """Land in place. Finishes once on the ground."""

    @abstractmethod
    async def return_home(self) -> None:
        """Fly back to the home position and land there."""

    # Getters read the latest telemetry, kept up to date in the background. None of them wait.

    @abstractmethod
    def pose(self) -> Pose:
        """Latest known position and attitude."""

    @abstractmethod
    def position(self) -> Waypoint:
        """Latest known position."""

    @abstractmethod
    def heading(self) -> float:
        """Degrees clockwise from north."""

    @abstractmethod
    def home(self) -> Waypoint | None:
        """Home position, or None before PX4 has set one."""

    @abstractmethod
    def battery(self) -> float:
        """Remaining charge, 0.0 to 1.0."""

    @abstractmethod
    def is_armed(self) -> bool: ...

    @abstractmethod
    def in_air(self) -> bool: ...
