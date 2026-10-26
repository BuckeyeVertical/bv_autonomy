from .base import Stitcher
from .fake_stitcher import FakeStitcher

__all__ = ["Stitcher", "FakeStitcher", "OpencvStitcher", "OdmStitcher"]


def __getattr__(name: str):
    if name == "OpencvStitcher":
        from .opencv_stitcher import OpencvStitcher

        return OpencvStitcher
    if name == "OdmStitcher":
        from .odm_stitcher import OdmStitcher

        return OdmStitcher
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
