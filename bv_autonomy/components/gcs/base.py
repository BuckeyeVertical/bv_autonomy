import asyncio
from abc import ABC, abstractmethod

from bv_autonomy.types import ApprovalRequest, Frame, WaypointSequence


class Gcs(ABC):
    @abstractmethod
    async def start(self) -> None: ...

    @abstractmethod
    async def stop(self) -> None: ...

    @abstractmethod
    def report(self, event: str, data: dict) -> None:
        """Sends a status update to the operator without waiting."""

    def report_progress(self, phase: str, route: WaypointSequence, reached: int) -> None:
        """Reports how far along a route the drone is."""
        self.report("path_progress", {
            "phase": phase,
            "completed": reached,
            "target": reached + 1 if reached < len(route) else None,
            "total": len(route),
            "segment_start": [route[reached - 1].lat, route[reached - 1].lon] if 0 < reached <= len(route) else None,
            "segment_end": [route[reached].lat, route[reached].lon] if reached < len(route) else None,
        })

    @abstractmethod
    def preview(self, frame: Frame) -> None:
        """Offers a frame to the operator's live view without waiting."""

    @abstractmethod
    async def ask(self, request: ApprovalRequest, timeout: float) -> bool | None:
        """Asks the operator to approve a target and returns their answer, or None if they don't answer in
        time.
        """

    @abstractmethod
    def signal(self, name: str) -> asyncio.Event:
        """Returns the event the operator sets with the named button."""
