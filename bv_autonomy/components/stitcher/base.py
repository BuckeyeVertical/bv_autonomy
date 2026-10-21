from abc import ABC, abstractmethod
from pathlib import Path

from bv_autonomy.types import Frame


class Stitcher(ABC):
    @abstractmethod
    def add(self, frame: Frame) -> None:
        """Offer a frame during the scan. The stitcher keeps only what it needs. Doesn't wait."""

    @abstractmethod
    async def stitch(self) -> Path:
        """Build the map from the kept frames and return the image path."""
