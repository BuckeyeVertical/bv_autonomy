from abc import ABC, abstractmethod

from bv_autonomy.types import GeoPoint


class PathPlanner(ABC):
    @abstractmethod
    def plan(self, boundary: list[GeoPoint]) -> list[GeoPoint]:
        """Waypoints covering boundary, with altitudes above sea level and ready to fly."""
