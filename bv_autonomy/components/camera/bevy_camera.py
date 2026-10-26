"""Camera that reads the bv_bevy frame stream (docs/camera_frame_v1.md in bv_bevy)."""

import asyncio
import contextlib
import json
import time

import cv2
import numpy as np

from bv_autonomy.components.camera.base import Camera
from bv_autonomy.components.drone import Drone
from bv_autonomy.types import Frame, Intrinsics

_MAX_HEADER_BYTES = 64 * 1024
_MAX_FRAME_BYTES = 100 * 1024 * 1024
_RECONNECT_DELAY_S = 0.5


class BevyCamera(Camera):
    """Connects to Bevy's TCP camera server, reconnecting whenever the stream drops.

    Frames are stamped with the drone's pose as they arrive. Only the newest one is
    kept, and it's decoded when asked for, so unread frames cost nothing.
    """

    def __init__(self, drone: Drone, host: str = "127.0.0.1", port: int = 7002):
        self._drone = drone
        self._host = host
        self._port = port
        self._task: asyncio.Task | None = None
        self._newest = None
        self._new_frame = asyncio.Event()

    async def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._receive_forever())

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
            try:
                await asyncio.wait_for(self._new_frame.wait(), timeout)
            except TimeoutError:
                return None
        self._new_frame.clear()
        jpeg, header, pose, received = self._newest
        image = await asyncio.to_thread(_decode, jpeg, header)
        return Frame(
            image=image,
            time=received,
            pose=pose,
            intrinsics=_intrinsics(header),
            sequence=header["sequence"],
        )

    async def _receive_forever(self) -> None:
        while True:
            try:
                reader, writer = await asyncio.open_connection(
                    self._host, self._port, limit=_MAX_HEADER_BYTES
                )
            except OSError:
                await asyncio.sleep(_RECONNECT_DELAY_S)
                continue
            try:
                while True:
                    header = json.loads(await reader.readline())
                    _check_header(header)
                    jpeg = await reader.readexactly(header["data_length"])
                    self._newest = (jpeg, header, self._drone.pose(), time.monotonic())
                    self._new_frame.set()
            except (OSError, ValueError, asyncio.IncompleteReadError, asyncio.LimitOverrunError):
                pass
            finally:
                writer.close()
            await asyncio.sleep(_RECONNECT_DELAY_S)


def _check_header(header: dict) -> None:
    if header.get("schema") != "bv.camera_frame" or header.get("version") != 1:
        raise ValueError("unsupported camera frame schema")
    if header.get("encoding") != "jpeg":
        raise ValueError("unsupported camera frame encoding")
    if header.get("camera_model") != "pinhole":
        raise ValueError("camera frame has no pinhole intrinsics")
    if not 0 < header.get("data_length", 0) <= _MAX_FRAME_BYTES:
        raise ValueError("invalid camera frame length")


def _decode(jpeg: bytes, header: dict) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (header["height"], header["width"]):
        raise ValueError("camera frame JPEG doesn't match its header")
    return image


def _intrinsics(header: dict) -> Intrinsics:
    return Intrinsics(
        width=header["width"], height=header["height"],
        fx=header["fx"], fy=header["fy"], cx=header["cx"], cy=header["cy"], dist=np.zeros(5),
    )
