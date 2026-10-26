import asyncio
import logging

import pytest

from bv_autonomy.components.camera.fake_camera import BEVY_INTRINSICS
from bv_autonomy.missions.suas2026 import MissionState
from bv_autonomy.types import WaypointSequence


def scan_only(site):
    """Returns a new state with laps already done."""
    return MissionState(lap_route=WaypointSequence(), scan_route=MissionState.new(site, BEVY_INTRINSICS).scan_route)


async def test_map_is_built_from_the_scan_frames_and_the_detector_is_stopped(make_mission, drone, site):
    drone._speedup = 100
    mission = make_mission(scan_only(site))
    await mission.camera.start()
    await mission.detector.start()
    await asyncio.wait_for(mission.start(), 10)
    assert mission.stitcher.thumbnails
    assert list((mission.stitcher._output_dir).iterdir())
    assert not mission.detector._started
    assert not drone.in_air()


async def test_map_is_built_while_flying_home(make_mission, drone, site, monkeypatch, tmp_path):
    mission = make_mission(scan_only(site))
    in_air_when_stitching = []

    async def stitch():
        in_air_when_stitching.append(drone.in_air())
        return tmp_path / "map.png"

    monkeypatch.setattr(mission.stitcher, "stitch", stitch)
    await asyncio.wait_for(mission.start(), 2)
    assert in_air_when_stitching == [True]


async def test_a_failed_map_still_lands(make_mission, drone, site, caplog):
    mission = make_mission(scan_only(site))
    with caplog.at_level(logging.ERROR):
        await asyncio.wait_for(mission.start(), 2)
    assert any(r.levelno == logging.ERROR and r.exc_info for r in caplog.records)
    assert not drone.in_air()


async def test_a_failed_return_stops_the_map_and_raises(make_mission, drone, site, monkeypatch):
    mission = make_mission(scan_only(site))
    stitching = []

    async def stitch():
        stitching.append(asyncio.current_task())
        await asyncio.Event().wait()

    async def broken_return():
        await asyncio.sleep(0.01)
        raise RuntimeError("PX4 switched to HOLD while returning home")

    monkeypatch.setattr(mission.stitcher, "stitch", stitch)
    monkeypatch.setattr(drone, "return_home", broken_return)
    with pytest.raises(RuntimeError, match="returning home"):
        await asyncio.wait_for(mission.start(), 2)
    await asyncio.sleep(0)
    assert stitching[0].cancelled()
