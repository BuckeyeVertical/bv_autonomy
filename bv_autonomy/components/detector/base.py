from abc import ABC, abstractmethod

from bv_autonomy.types import Detection, Frame


class Detector(ABC):
    @abstractmethod
    async def start(self) -> None:
        """Load the model."""

    @abstractmethod
    async def stop(self) -> None:
        """Free the model, and the GPU memory on the Orin, before stitching."""

    @abstractmethod
    async def detect(self, frame: Frame) -> list[Detection]:
        """Run detection on one frame. Thresholds come from the constructor."""
