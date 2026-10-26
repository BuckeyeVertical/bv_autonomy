from abc import ABC, abstractmethod

from bv_autonomy.types import WaypointSequence


class PathPlanner(ABC):
    @abstractmethod
    def plan(self, boundary: WaypointSequence) -> WaypointSequence:
        """Returns the waypoints that cover the area inside a boundary."""
