"""Picks frames for stitching during a scan and writes them to disk, shared by the stitchers."""

import logging
import math
import re
from concurrent.futures import Future, ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

import cv2
import piexif

from bv_autonomy.components.stitcher.exif import CameraExif, FrameGeotag, build_exif_bytes
from bv_autonomy.components.terrain import Terrain
from bv_autonomy.types import Frame, StitchProgress, Waypoint

log = logging.getLogger(__name__)

_EARTH_R = 6378137.0
_NEW_ROW_TURN_DEG = 90.0
_FRAME_NAME = re.compile(r"row(\d+)_(\d+)\.jpg")


class FrameRecorder:
    """Keeps a frame each time the drone has moved one capture spacing, as bv_core's scan did.

    The spacing is how far the camera sees along the direction of flight, less the overlap:
    image_height * altitude / fy * (1 - overlap) (bv_core's compute_step_m). A frame whose
    heading has turned more than 90 degrees from the current row's starts a new row, so the
    mission should only add frames while flying rows, not on the legs between them.

    Frames are written as row{N}_{M}.jpg, like bv_core's raw_frames, on a background thread. A recorder
    started on a folder that already has frames keeps them and numbers new ones after them, and resume()
    carries on the row it was on before a crash instead of splitting it.
    With camera set, each is geotagged for ODM; its altitude is above sea level when a terrain
    is given, and otherwise the altitude above the ground.
    """

    def __init__(self, directory: Path, overlap: float, camera: CameraExif | None = None,
                 terrain: Terrain | None = None):
        if not 0.0 <= overlap < 1.0:
            raise ValueError("overlap must be in [0, 1)")
        self.directory = Path(directory)
        self._overlap = overlap
        self._camera = camera
        self._terrain = terrain
        self._writer = ThreadPoolExecutor(max_workers=1, thread_name_prefix="stitch-writer")
        self._writes: list[Future] = []
        self._rows: dict[int, list[Path]] = {}
        self._row = 0
        self._row_heading: float | None = None
        self._last: Waypoint | None = None
        self._pick_up_frames()

    def add(self, frame: Frame) -> bool:
        """Saves the frame if the drone has moved far enough since the last one, and returns whether it did."""
        position = frame.pose.position
        heading = math.degrees(frame.pose.yaw) % 360.0
        new_row = self._row_heading is None or _angle_between(heading, self._row_heading) > _NEW_ROW_TURN_DEG
        if not new_row and _distance_m(self._last, position) < self._spacing_m(frame):
            return False
        if new_row:
            self._row += 1
            self._row_heading = heading
            self._rows[self._row] = []
        column = len(self._rows[self._row]) + 1
        path = self.directory / f"row{self._row}_{column}.jpg"
        self._rows[self._row].append(path)
        self._last = position
        self._writes.append(self._writer.submit(self._write, path, frame, datetime.now()))
        return True

    def rows(self) -> dict[int, list[Path]]:
        """Returns the saved frame paths for each row in the order they were flown."""
        return {row: list(paths) for row, paths in self._rows.items()}

    def flush(self) -> None:
        """Waits until every saved frame has been written to disk."""
        for write in self._writes:
            write.result()
        self._writes.clear()

    def close(self) -> None:
        """Waits for the frames still being written, then ends the writer thread. Blocks, so run it off the
        event loop.
        """
        self._writer.shutdown(wait=True)

    def progress(self) -> StitchProgress:
        """Returns the current row, its heading and where the last frame was kept."""
        return StitchProgress(self._row, self._row_heading, self._last)

    def resume(self, progress: StitchProgress) -> None:
        """Carries on from a saved place, so the next frame continues that row unless the drone has turned. A
        """
        if progress.row < self._row:
            return
        self._row, self._row_heading, self._last = progress.row, progress.heading, progress.last
        self._rows.setdefault(self._row, [])

    def _pick_up_frames(self) -> None:
        """Adds the frames already in the folder to the rows, in order, and numbers new rows after them."""
        for path in self.directory.glob("row*_*.jpg"):
            if match := _FRAME_NAME.fullmatch(path.name):
                self._rows.setdefault(int(match[1]), []).append(path)
        for paths in self._rows.values():
            paths.sort(key=lambda path: int(_FRAME_NAME.fullmatch(path.name)[2]))
        self._rows = dict(sorted(self._rows.items()))
        self._row = max(self._rows, default=0)

    def _spacing_m(self, frame: Frame) -> float:
        height_px = frame.image.shape[0]
        footprint_m = height_px * frame.pose.position.alt / frame.intrinsics.fy
        return footprint_m * (1.0 - self._overlap)

    def _write(self, path: Path, frame: Frame, captured_at: datetime) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), frame.image):
            raise OSError(f"could not write {path}")
        if self._camera is None:
            return
        p = frame.pose.position
        altitude = self._terrain.msl(p) if self._terrain is not None else p.alt
        geotag = FrameGeotag(p.lat, p.lon, altitude, captured_at, math.degrees(frame.pose.yaw) % 360.0)
        height, width = frame.image.shape[:2]
        try:
            piexif.insert(build_exif_bytes(self._camera, geotag, width, height), str(path))
        except Exception as exc:
            log.error("could not geotag %s: %s", path, exc)


def _angle_between(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _distance_m(a: Waypoint, b: Waypoint) -> float:
    dx = _EARTH_R * math.radians(b.lon - a.lon) * math.cos(math.radians(a.lat))
    dy = _EARTH_R * math.radians(b.lat - a.lat)
    return math.hypot(dx, dy)
