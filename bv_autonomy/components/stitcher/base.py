from abc import ABC, abstractmethod
from pathlib import Path

from bv_autonomy.types import Frame


class Stitcher(ABC):
    @abstractmethod
    def add(self, frame: Frame) -> None:
        """Offers a frame for the map without waiting."""

    @abstractmethod
    async def stitch(self) -> Path:
        """Builds the map from the frames kept and returns the path to its image."""
