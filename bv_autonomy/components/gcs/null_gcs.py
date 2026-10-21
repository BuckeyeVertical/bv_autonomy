"""GCS for missions flown without an operator: nobody answers and no button is ever pressed."""

import asyncio

from bv_autonomy.components.gcs.base import Gcs
from bv_autonomy.types import ApprovalRequest, Frame


class NullGcs(Gcs):
    def __init__(self):
        self._signals: dict[str, asyncio.Event] = {}

    async def start(self) -> None:
        pass

    async def stop(self) -> None:
        pass

    def report(self, event: str, data: dict) -> None:
        pass

    def preview(self, frame: Frame) -> None:
        pass

    async def ask(self, request: ApprovalRequest, timeout: float) -> bool | None:
        return None          # straight away; the mission falls back to its own rule

    def signal(self, name: str) -> asyncio.Event:
        return self._signals.setdefault(name, asyncio.Event())
