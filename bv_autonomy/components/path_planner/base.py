from abc import ABC, abstractmethod

from bv_autonomy.types import GeoPoint


class PathPlanner(ABC):
    @abstractmethod
    def plan(self, boundary: list[GeoPoint]) -> list[GeoPoint]:
        """Waypoints covering boundary, ready to fly."""
