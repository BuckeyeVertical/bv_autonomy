"""Camera that reads any GStreamer pipeline through OpenCV, e.g. a V4L2 capture device."""

import asyncio
import threading
import time

import cv2
import numpy as np

from bv_autonomy.components.camera.base import Camera
from bv_autonomy.components.drone import Drone
from bv_autonomy.types import Frame, Intrinsics

_REOPEN_DELAY_S = 1.0


class GstreamerCamera(Camera):
    """Reads frames from a GStreamer pipeline ending in appsink, reopening it if it fails.

    OpenCV's read() blocks, so it runs in a thread. Each frame is handed to the event
    loop, which stamps it with the drone's pose and keeps only the newest one.
    The pipeline needs OpenCV built with GStreamer, as on the Jetson; pip's OpenCV isn't.
    """

    def __init__(self, drone: Drone, pipeline: str, intrinsics: Intrinsics):
        self._drone = drone
        self._pipeline = pipeline
        self._intrinsics = intrinsics
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._stopping = threading.Event()
        self._sequence = 0
        self._newest: Frame | None = None
        self._new_frame = asyncio.Event()

    async def start(self) -> None:
        if self._thread is not None:
            return
        self._loop = asyncio.get_running_loop()
        self._stopping.clear()
        self._thread = threading.Thread(target=self._capture_forever, name="gstreamer-camera", daemon=True)
        self._thread.start()

    async def stop(self) -> None:
        if self._thread is None:
            return
        self._stopping.set()
        await asyncio.to_thread(self._thread.join, 2.0)
        self._thread = None
        self._newest = None
        self._new_frame.clear()

    async def latest(self, timeout: float | None = None) -> Frame | None:
        if not self._new_frame.is_set():
            try:
                await asyncio.wait_for(self._new_frame.wait(), timeout)
            except TimeoutError:
                return None
        self._new_frame.clear()
        return self._newest

    def _capture_forever(self) -> None:
        while not self._stopping.is_set():
            capture = cv2.VideoCapture(self._pipeline, cv2.CAP_GSTREAMER)
            try:
                while capture.isOpened() and not self._stopping.is_set():
                    ok, image = capture.read()
                    if not ok:
                        break
                    self._loop.call_soon_threadsafe(self._on_frame, image, time.monotonic())
            finally:
                capture.release()
            self._stopping.wait(_REOPEN_DELAY_S)

    def _on_frame(self, image: np.ndarray, captured: float) -> None:
        if self._stopping.is_set():
            return
        self._sequence += 1
        self._newest = Frame(
            image=image,
            time=captured,
            pose=self._drone.pose(),
            intrinsics=self._intrinsics,
            sequence=self._sequence,
        )
        self._new_frame.set()
