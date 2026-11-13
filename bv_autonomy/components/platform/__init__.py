from .base import Platform, stop_all

__all__ = ["Platform", "stop_all", "FakePlatform", "SimPlatform", "CompDronePlatform", "X650Platform"]


def __getattr__(name: str):
    if name == "FakePlatform":
        from .fake_platform import FakePlatform

        return FakePlatform
    if name == "SimPlatform":
        from .sim_platform import SimPlatform

        return SimPlatform
    if name == "CompDronePlatform":
        from .comp_drone_platform import CompDronePlatform

        return CompDronePlatform
    if name == "X650Platform":
        from .x650_platform import X650Platform

        return X650Platform
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
