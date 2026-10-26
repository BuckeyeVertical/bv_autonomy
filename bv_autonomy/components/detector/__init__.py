from .base import Detector
from .fake_detector import FakeDetector

__all__ = ["Detector", "FakeDetector", "LtdetrDetector"]


def __getattr__(name: str):
    if name == "LtdetrDetector":
        from .ltdetr_detector import LtdetrDetector

        return LtdetrDetector
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
