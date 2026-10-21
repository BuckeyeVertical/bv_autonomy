from .base import Camera

__all__ = ["Camera", "BevyCamera"]


def __getattr__(name: str):
    # Implementations load on first use, so importing Camera doesn't need OpenCV.
    if name == "BevyCamera":
        from .bevy_camera import BevyCamera

        return BevyCamera
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
