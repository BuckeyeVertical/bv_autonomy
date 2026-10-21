from .base import Drone
from .fake_drone import FakeDrone

__all__ = ["Drone", "FakeDrone", "MavsdkDrone"]


def __getattr__(name: str):
    # Implementations load on first use, so importing Drone doesn't need MAVSDK.
    if name == "MavsdkDrone":
        from .mavsdk_drone import MavsdkDrone

        return MavsdkDrone
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
