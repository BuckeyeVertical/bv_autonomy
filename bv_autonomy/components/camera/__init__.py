from .base import Camera
from .fake_camera import FakeCamera

__all__ = ["Camera", "FakeCamera", "BevyCamera", "GstreamerCamera", "Rx0Camera"]


def __getattr__(name: str):
    if name == "BevyCamera":
        from .bevy_camera import BevyCamera

        return BevyCamera
    if name == "GstreamerCamera":
        from .gstreamer_camera import GstreamerCamera

        return GstreamerCamera
    if name == "Rx0Camera":
        from .rx0_camera import Rx0Camera

        return Rx0Camera
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
