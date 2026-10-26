from .base import Platform
from .fake_platform import FakePlatform

__all__ = ["Platform", "FakePlatform", "SimPlatform", "PhysicalPlatform"]


def __getattr__(name: str):
    if name == "SimPlatform":
        from .sim_platform import SimPlatform

        return SimPlatform
    if name == "PhysicalPlatform":
        from .physical_platform import PhysicalPlatform

        return PhysicalPlatform
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
