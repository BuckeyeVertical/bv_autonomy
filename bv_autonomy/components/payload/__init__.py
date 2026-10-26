from .base import Payload
from .fake_payload import FakePayload

__all__ = ["Payload", "FakePayload", "WinchPayload"]


def __getattr__(name: str):
    if name == "WinchPayload":
        from .winch_payload import WinchPayload

        return WinchPayload
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
