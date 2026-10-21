"""Stitcher that pastes thumbnails of the frames it was given into a grid, without matching anything."""

import asyncio
from pathlib import Path

import numpy as np

from bv_autonomy import paths
from bv_autonomy.components.stitcher.base import Stitcher
from bv_autonomy.types import Frame


class FakeStitcher(Stitcher):
    """Keeps every n-th frame's thumbnail and writes them as one grid image (a PPM, so no image library is needed)."""

    def __init__(self, output_dir: str | Path | None = None, keep_every: int = 10, thumb_width: int = 160, columns: int = 8):
        self._output_dir = Path(output_dir) if output_dir is not None else paths.new_run_dir("fake_stitch")
        self._keep_every = keep_every
        self._thumb_width = thumb_width
        self._columns = columns
        self._offered = 0
        self.thumbnails: list[np.ndarray] = []

    def add(self, frame: Frame) -> None:
        self._offered += 1
        if (self._offered - 1) % self._keep_every:
            return
        step = max(1, frame.image.shape[1] // self._thumb_width)
        self.thumbnails.append(frame.image[::step, ::step].copy())

    async def stitch(self) -> Path:
        if not self.thumbnails:
            raise RuntimeError("no frames to stitch")
        return await asyncio.to_thread(self._write)

    def _write(self) -> Path:
        h = min(t.shape[0] for t in self.thumbnails)
        w = min(t.shape[1] for t in self.thumbnails)
        rows = -(-len(self.thumbnails) // self._columns)
        grid = np.zeros((rows * h, self._columns * w, 3), dtype=np.uint8)
        for i, thumb in enumerate(self.thumbnails):
            r, c = divmod(i, self._columns)
            grid[r * h:(r + 1) * h, c * w:(c + 1) * w] = thumb[:h, :w]
        self._output_dir.mkdir(parents=True, exist_ok=True)
        path = self._output_dir / "fake_mosaic.ppm"
        # PPM needs no image library: a short header, then raw RGB.
        with open(path, "wb") as f:
            f.write(f"P6 {grid.shape[1]} {grid.shape[0]} 255\n".encode())
            f.write(grid[:, :, ::-1].tobytes())
        return path
