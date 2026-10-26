from .base import Localizer

__all__ = ["Localizer", "GimbalNadirLocalizer"]


def __getattr__(name: str):
    if name == "GimbalNadirLocalizer":
        from .gimbal_nadir_localizer import GimbalNadirLocalizer

        return GimbalNadirLocalizer
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
