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
        """Send status to the operator, e.g. state or progress. Dropped if no link. Doesn't wait."""

    def report_progress(self, phase: str, route: WaypointSequence, reached: int) -> None:
        """Report how far along route the drone is, e.g. report_progress("lap", route, 3) after 3 points."""
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
        """Offer a frame for the live view. Dropped if no link or busy. Doesn't wait."""

    @property
    def wants_preview(self) -> bool:
        """Whether the mission should supply frames for a live viewer."""
        return False

    @abstractmethod
    async def ask(self, request: ApprovalRequest, timeout: float) -> bool | None:
        """Operator's answer: True to approve, False to reject, None if no answer in time."""

    @abstractmethod
    def signal(self, name: str) -> asyncio.Event:
        """Event the operator can set, e.g. signal("end_laps"). Never set if there's no GCS."""
