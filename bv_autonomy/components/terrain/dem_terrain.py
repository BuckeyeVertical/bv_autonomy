"""Ground heights read from a digital elevation model, as bv_core's terrain following did."""

import math
from pathlib import Path

import numpy as np
import rasterio
from rasterio.warp import transform

from bv_autonomy import paths
from bv_autonomy.components.terrain.base import Terrain


class DemTerrain(Terrain):
    """Ground heights from a GeoTIFF elevation model, held in memory and interpolated between cells."""

    def __init__(self, path: str | Path = paths.SUAS2026_DEM):
        """Loads the elevation model from a GeoTIFF file."""
        self._path = Path(path)
        with rasterio.open(self._path) as source:
            self._elevations = source.read(1)
            self._to_pixel = ~source.transform
            self._crs = source.crs
            self._nodata = source.nodata

    def ground_msl(self, lat: float, lon: float) -> float:
        """Returns the ground's height above sea level at a point, raising if the map doesn't cover it."""
        if self._crs is None or self._crs.to_epsg() == 4326:
            x, y = lon, lat
        else:
            xs, ys = transform("EPSG:4326", self._crs, [lon], [lat])
            x, y = xs[0], ys[0]
        column, row = self._to_pixel @ (x, y)
        height, width = self._elevations.shape
        if not (0 <= column <= width and 0 <= row <= height):
            raise ValueError(f"{lat:.6f}, {lon:.6f} is outside {self._path.name}")

        column, row = column - 0.5, row - 0.5
        column_0 = min(max(math.floor(column), 0), max(width - 2, 0))
        row_0 = min(max(math.floor(row), 0), max(height - 2, 0))
        across = min(max(column - column_0, 0.0), 1.0)
        down = min(max(row - row_0, 0.0), 1.0)
        corners = [self._cell(row_0 + r, column_0 + c) for r, c in ((0, 0), (0, 1), (1, 0), (1, 1))]
        if None in corners:
            nearest = self._cell(min(max(round(row), 0), height - 1), min(max(round(column), 0), width - 1))
            if nearest is None:
                raise ValueError(f"{self._path.name} has no elevation at {lat:.6f}, {lon:.6f}")
            return nearest
        top = corners[0] + (corners[1] - corners[0]) * across
        bottom = corners[2] + (corners[3] - corners[2]) * across
        return top + (bottom - top) * down

    def _cell(self, row: int, column: int) -> float | None:
        """Returns one cell's elevation, or None if it's off the map or has no data."""
        height, width = self._elevations.shape
        if not (0 <= row < height and 0 <= column < width):
            return None
        value = float(self._elevations[row, column])
        if np.isnan(value) or value == self._nodata:
            return None
        return value
