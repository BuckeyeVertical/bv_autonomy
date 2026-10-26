from abc import ABC, abstractmethod

from bv_autonomy.types import Frame


class Camera(ABC):
    @abstractmethod
    async def start(self) -> None: ...

    @abstractmethod
    async def stop(self) -> None: ...

    @abstractmethod
    async def latest(self, timeout: float | None = None) -> Frame | None:
        """Returns the newest frame not returned before, or None if no new frame arrives within the timeout."""
