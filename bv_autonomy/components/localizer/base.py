from abc import ABC, abstractmethod

from bv_autonomy.types import Detection, Waypoint


class Localizer(ABC):
    @abstractmethod
    def locate(self, detection: Detection) -> Waypoint | None:
        """Ground position of the detection, or None if its ray never reaches the ground."""
