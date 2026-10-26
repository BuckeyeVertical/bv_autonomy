from abc import ABC, abstractmethod

from bv_autonomy.types import Detection, Waypoint


class Localizer(ABC):
    @abstractmethod
    def locate(self, detection: Detection) -> Waypoint | None:
        """Returns where a detection is on the ground, or None if it can't be placed there."""
