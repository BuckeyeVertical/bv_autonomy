from .base import Detector

__all__ = ["Detector", "LtdetrDetector"]


def __getattr__(name: str):
    # Implementations load on first use, so importing Detector doesn't need torch.
    if name == "LtdetrDetector":
        from .ltdetr_detector import LtdetrDetector

        return LtdetrDetector
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
