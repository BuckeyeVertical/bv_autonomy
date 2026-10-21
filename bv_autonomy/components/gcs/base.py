import asyncio
from abc import ABC, abstractmethod

from bv_autonomy.types import ApprovalRequest, Frame


class Gcs(ABC):
    @abstractmethod
    async def start(self) -> None: ...

    @abstractmethod
    async def stop(self) -> None: ...

    @abstractmethod
    def report(self, event: str, data: dict) -> None:
        """Send status to the operator, e.g. state or progress. Dropped if no link. Doesn't wait."""

    @abstractmethod
    def preview(self, frame: Frame) -> None:
        """Offer a frame for the live view. Dropped if no link or busy. Doesn't wait."""

    @abstractmethod
    async def ask(self, request: ApprovalRequest, timeout: float) -> bool | None:
        """Operator's answer: True to approve, False to reject, None if no answer in time."""

    @abstractmethod
    def signal(self, name: str) -> asyncio.Event:
        """Event the operator can set, e.g. signal("end_laps"). Never set if there's no GCS."""
