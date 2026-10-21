from abc import ABC, abstractmethod


class Payload(ABC):
    @abstractmethod
    async def release(self, slot: str) -> None:
        """Let go of the payload in slot, e.g. "bottle" or "beacon"."""

    @abstractmethod
    async def pick_up(self) -> None:
        """Grab a payload. Mechanisms that can't pick up raise NotImplementedError."""
