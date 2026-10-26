from abc import ABC, abstractmethod
from pathlib import Path

from bv_autonomy.types import Frame, StitchProgress


class Stitcher(ABC):
    @abstractmethod
    def add(self, frame: Frame) -> None:
        """Offers a frame for the map without waiting."""

    @abstractmethod
    async def stitch(self) -> Path:
        """Builds the map from the frames kept and returns the path to its image."""
    @abstractmethod
    def progress(self) -> StitchProgress | None:
        """Returns where the stitcher is in the scan, to save for a resume, or None if it has nothing to save."""

    @abstractmethod
    def resume(self, progress: StitchProgress) -> None:
        """Carries on from a saved place in the scan, keeping the frames already taken."""
