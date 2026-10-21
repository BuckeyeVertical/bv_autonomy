from abc import ABC, abstractmethod

from bv_autonomy.types import GeoPoint, Pose


class Drone(ABC):
    @abstractmethod
    async def connect(self) -> None: ...

    @abstractmethod
    async def arm(self) -> None: ...

    @abstractmethod
    async def disarm(self) -> None: ...

    @abstractmethod
    async def takeoff(self, alt_agl: float) -> None:
        """Climb to alt_agl meters above the ground. Finishes once at that altitude."""

    @abstractmethod
    async def goto(self, point: GeoPoint, speed: float | None = None) -> None:
        """Fly to point. Finishes once the drone has arrived."""

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
    def position(self) -> GeoPoint:
        """Latest known position, altitude above sea level."""

    @abstractmethod
    def altitude_agl(self) -> float:
        """Meters above the ground."""

    @abstractmethod
    def heading(self) -> float:
        """Degrees clockwise from north."""

    @abstractmethod
    def home(self) -> GeoPoint | None:
        """Home position, or None before PX4 has set one."""

    @abstractmethod
    def battery(self) -> float:
        """Remaining charge, 0.0 to 1.0."""

    @abstractmethod
    def is_armed(self) -> bool: ...

    @abstractmethod
    def in_air(self) -> bool: ...
