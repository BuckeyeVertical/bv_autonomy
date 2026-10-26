from abc import ABC, abstractmethod

from bv_autonomy.types import Detection, Frame


class Detector(ABC):
    @abstractmethod
    async def start(self) -> None:
        """Loads the detection model."""

    @abstractmethod
    async def stop(self) -> None:
        """Frees the model and the memory it uses."""

    @abstractmethod
    async def detect(self, frame: Frame) -> list[Detection]:
        """Finds the objects in a frame and returns the ones above their confidence threshold."""
