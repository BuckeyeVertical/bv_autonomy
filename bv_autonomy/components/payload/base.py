from abc import ABC, abstractmethod


class Payload(ABC):
    @abstractmethod
    async def release(self, slot: str) -> None:
        """Releases the payload held in the given slot."""

    @abstractmethod
    async def pick_up(self) -> None:
        """Picks up a payload."""
