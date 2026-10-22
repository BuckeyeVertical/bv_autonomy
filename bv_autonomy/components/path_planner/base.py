from abc import ABC, abstractmethod

from bv_autonomy.types import WaypointSequence


class PathPlanner(ABC):
    @abstractmethod
    def plan(self, boundary: WaypointSequence) -> WaypointSequence:
        """Waypoints covering boundary, ready to fly."""
