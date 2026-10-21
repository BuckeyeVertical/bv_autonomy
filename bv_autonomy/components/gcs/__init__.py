from .base import Gcs
from .null_gcs import NullGcs

__all__ = ["Gcs", "NullGcs", "BvGcs"]


def __getattr__(name: str):
    # Implementations load on first use, so importing Gcs doesn't need aiohttp.
    if name == "BvGcs":
        from .bv_gcs import BvGcs

        return BvGcs
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
