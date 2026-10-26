"""Payload that only counts what it holds and records what it did."""

import asyncio
import time

from bv_autonomy.components.payload.base import Payload


class FakePayload(Payload):
    """Starts holding one payload per slot (or the counts given) and logs each release and pickup."""

    def __init__(self, slots: dict[str, int] | None = None, can_pick_up: bool = False, action_s: float = 0.1):
        self.held: dict[str, int] = dict(slots if slots is not None else {"bottle": 1, "beacon": 1})
        self.log: list[tuple[float, str, str]] = []
        self._can_pick_up = can_pick_up
        self._action_s = action_s

    async def start(self) -> None:
        self.log.append((time.monotonic(), "start", ""))

    async def stop(self) -> None:
        self.log.append((time.monotonic(), "stop", ""))

    async def release(self, slot: str) -> None:
        if self.held.get(slot, 0) <= 0:
            raise RuntimeError(f"nothing left to release in {slot!r}")
        await asyncio.sleep(self._action_s)
        self.held[slot] -= 1
        self.log.append((time.monotonic(), "release", slot))

    async def pick_up(self) -> None:
        if not self._can_pick_up:
            raise NotImplementedError("this payload can't pick up")
        await asyncio.sleep(self._action_s)
        self.held["main"] = self.held.get("main", 0) + 1
        self.log.append((time.monotonic(), "pick_up", "main"))
