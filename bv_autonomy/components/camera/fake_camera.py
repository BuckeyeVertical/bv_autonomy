"""Camera that draws placed targets on a plain background, stamped with the drone's pose."""

import asyncio
import contextlib
import time

import numpy as np

from bv_autonomy.components.camera.base import Camera
from bv_autonomy.components.drone import Drone
from bv_autonomy.components.platform.fake_world import Target, project
from bv_autonomy.types import Frame, Intrinsics

BEVY_INTRINSICS = Intrinsics(width=1920, height=1080, fx=1148.2004184838818, fy=1148.2004184838818, cx=960.0, cy=540.0, dist=np.zeros(5))


class FakeCamera(Camera):
    """Makes frames at a fixed rate. Targets are drawn as filled squares where they'd be seen."""

    def __init__(self, drone: Drone, targets: list[Target] | None = None,
                 intrinsics: Intrinsics = BEVY_INTRINSICS, fps: float = 15.0):
        self._drone = drone
        self._targets = list(targets or [])
        self._intrinsics = intrinsics
        self._period = 1.0 / fps
        self._sequence = 0
        self._newest: Frame | None = None
        self._new_frame = asyncio.Event()
        self._task: asyncio.Task | None = None

    async def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._produce())

    async def stop(self) -> None:
        if self._task is None:
            return
        self._task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await self._task
        self._task = None
        self._newest = None
        self._new_frame.clear()

    async def latest(self, timeout: float | None = None) -> Frame | None:
        if not self._new_frame.is_set():
            waiting = asyncio.ensure_future(self._new_frame.wait())
            try:
                await asyncio.wait({waiting}, timeout=timeout)
            finally:
                waiting.cancel()
            if not self._new_frame.is_set():
                return None
        self._new_frame.clear()
        return self._newest

    async def _produce(self) -> None:
        while True:
            pose = self._drone.pose()
            image = np.full((self._intrinsics.height, self._intrinsics.width, 3), (60, 120, 80), dtype=np.uint8)
            for target in self._targets:
                box = project(target, pose, self._intrinsics, self._intrinsics.width, self._intrinsics.height)
                if box is not None:
                    x1, y1, x2, y2 = (int(round(v)) for v in box)
                    image[y1:y2, x1:x2] = target.color_bgr
            self._sequence += 1
            self._newest = Frame(image, time.monotonic(), pose, self._intrinsics, self._sequence)
            self._new_frame.set()
            await asyncio.sleep(self._period)
