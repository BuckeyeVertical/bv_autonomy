from .base import Terrain

__all__ = ["Terrain", "DemTerrain"]


def __getattr__(name: str):
    if name == "DemTerrain":
        from .dem_terrain import DemTerrain

        return DemTerrain
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
