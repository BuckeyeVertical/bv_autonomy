from abc import ABC, abstractmethod

from bv_autonomy.types import Frame


class Camera(ABC):
    @abstractmethod
    async def start(self) -> None: ...

    @abstractmethod
    async def stop(self) -> None: ...

    @abstractmethod
    async def latest(self, timeout: float | None = None) -> Frame | None:
        """Newest frame, dropping older ones. Waits up to timeout for one; None if none arrives."""
