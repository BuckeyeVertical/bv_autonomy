import math

import numpy as np
import pytest
import rasterio
from rasterio.transform import from_origin

from bv_autonomy import paths
from bv_autonomy.components.terrain import DemTerrain
from bv_autonomy.types import Waypoint

CELL = 0.001
WEST, NORTH = -96.0, 36.0


def write_dem(path, elevations, crs="EPSG:4326", origin=(WEST, NORTH), cell=CELL, nodata=None):
    """Writes a small GeoTIFF elevation model and returns its path."""
    elevations = np.asarray(elevations, dtype="float32")
    with rasterio.open(path, "w", driver="GTiff", height=elevations.shape[0], width=elevations.shape[1], count=1,
                       dtype="float32", crs=crs, transform=from_origin(*origin, cell, cell), nodata=nodata) as dem:
        dem.write(elevations, 1)
    return path


def center(row, column):
    """Returns the latitude and longitude of a cell's center."""
    return NORTH - (row + 0.5) * CELL, WEST + (column + 0.5) * CELL


@pytest.fixture
def ramp(tmp_path):
    """A map whose ground rises 10 m per cell to the east and 1 m per cell to the south."""
    return DemTerrain(write_dem(tmp_path / "ramp.tif", [[100, 110, 120], [101, 111, 121], [102, 112, 122]]))


def test_cell_centers_read_their_own_elevation(ramp):
    assert ramp.ground_msl(*center(0, 0)) == pytest.approx(100)
    assert ramp.ground_msl(*center(2, 1)) == pytest.approx(112)


def test_between_cells_is_interpolated(ramp):
    lat, lon = center(0, 0)
    assert ramp.ground_msl(lat - CELL / 2, lon + CELL / 4) == pytest.approx(103.0)


def test_the_outer_half_cell_reads_as_the_edge(ramp):
    lat, lon = center(0, 0)
    assert ramp.ground_msl(lat + CELL / 4, lon - CELL / 4) == pytest.approx(100)


def test_msl_adds_the_height_above_the_ground(ramp):
    lat, lon = center(1, 1)
    assert ramp.msl(Waypoint(lat, lon, 60.96)) == pytest.approx(111 + 60.96)


def test_a_point_off_the_map_raises(ramp):
    with pytest.raises(ValueError, match="outside ramp.tif"):
        ramp.ground_msl(NORTH + 0.01, WEST)


def test_a_hole_in_the_map_falls_back_to_the_nearest_cell(tmp_path):
    terrain = DemTerrain(write_dem(tmp_path / "hole.tif", [[100, 110], [np.nan, 130]]))
    lat, lon = center(0, 0)
    assert terrain.ground_msl(lat - CELL * 0.4, lon + CELL * 0.1) == pytest.approx(100)


def test_a_point_with_no_data_raises(tmp_path):
    terrain = DemTerrain(write_dem(tmp_path / "empty.tif", [[-9999, -9999], [-9999, -9999]], nodata=-9999))
    with pytest.raises(ValueError, match="no elevation"):
        terrain.ground_msl(*center(0, 0))


def test_a_projected_map_is_read_at_the_right_place(tmp_path):
    x, y = rasterio.warp.transform("EPSG:4326", "EPSG:5070", [WEST], [NORTH])
    terrain = DemTerrain(write_dem(tmp_path / "albers.tif", [[200, 210], [220, 230]], crs="EPSG:5070",
                                   origin=(x[0] - 1.0, y[0] + 1.0), cell=1.0))
    assert terrain.ground_msl(NORTH, WEST) == pytest.approx(215, abs=0.5)


@pytest.mark.skipif(not paths.SUAS2026_DEM.exists(), reason="terrain_maps/suas2026.tif isn't downloaded")
def test_the_suas_field_map_covers_the_sim_site():
    from bv_autonomy.missions.suas2026.sites import SIM

    terrain = DemTerrain()
    heights = [terrain.ground_msl(p.lat, p.lon) for p in [*SIM.laps, *SIM.scan_area]]
    assert all(190 < h < 235 and not math.isnan(h) for h in heights)
