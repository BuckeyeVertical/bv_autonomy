from .base import Camera

__all__ = ["Camera", "BevyCamera", "GstreamerCamera", "Rx0Camera"]


def __getattr__(name: str):
    # Implementations load on first use, so importing Camera doesn't need OpenCV.
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
