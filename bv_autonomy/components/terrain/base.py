from abc import ABC, abstractmethod

from bv_autonomy.types import Waypoint


class Terrain(ABC):
    """Ground height, for turning altitudes above the ground into altitudes above sea level."""

    @abstractmethod
    def ground_msl(self, lat: float, lon: float) -> float:
        """Returns the ground's height above sea level at a point, in meters."""

    def msl(self, point: Waypoint) -> float:
        """Returns a point's altitude above sea level, in meters."""
        return self.ground_msl(point.lat, point.lon) + point.alt
