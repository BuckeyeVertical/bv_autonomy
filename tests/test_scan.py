import asyncio
import math

import pytest

from bv_autonomy.components.camera.rx0_camera import RX0_INTRINSICS
from bv_autonomy.components.path_planner import SPathPlanner
from bv_autonomy.missions.suas2026 import MissionState
from bv_autonomy.missions.suas2026.suas2026 import (
    SCAN_ALT, SCAN_OVERLAP, SCAN_SPEED, SCAN_TOLERANCE, TRANSIT_SPEED,
)
from bv_autonomy.fake_world import Target
from bv_autonomy.types import Waypoint, WaypointSequence


def scan_only(site):
    """A fresh state with laps already done, for tests about the scan alone."""
    return MissionState(lap_route=WaypointSequence(), scan_route=MissionState.new(site).scan_route)


def test_new_state_plans_scan_once_at_scan_altitude(site):
    route = MissionState.new(site).scan_route
    assert route == SPathPlanner(RX0_INTRINSICS, SCAN_ALT, SCAN_OVERLAP).plan(site.scan_area)
    assert len(route) >= 4
    assert all(point.alt == SCAN_ALT for point in route)


async def test_scan_transits_then_flies_rows_at_scan_speed(make_mission, drone, gcs, site):
    mission = make_mission(scan_only(site))
    route = list(mission.state.scan_route)
    await asyncio.wait_for(mission.start(), 2)
    assert drone.gotos == [(route[0], TRANSIT_SPEED, SCAN_TOLERANCE)]
    assert drone.flights == [(SCAN_SPEED, SCAN_TOLERANCE)]
    assert drone.reached == route
    assert ("mission_state", {"state": "scan"}) in gcs.reports
    assert [p["completed"] for p in gcs.progress if p["phase"] == "scan"] == list(range(len(route) + 1))
    assert MissionState.load().scan_route == []


async def test_stop_mid_scan_and_resume_transits_to_next_point(make_mission, drone, gcs, site):
    mission = make_mission(scan_only(site))
    route = list(mission.state.scan_route)
    task = asyncio.create_task(mission.start())
    await gcs.wait_reached(2)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert drone.holds == 1
    saved = MissionState.load()
    assert saved.scan_route == route[len(drone.reached):]
    next_point = saved.scan_route[0]

    await asyncio.wait_for(make_mission(saved).start(), 2)
    assert drone.gotos[-1] == (next_point, TRANSIT_SPEED, SCAN_TOLERANCE)
    assert drone.reached == route
    assert MissionState.load().scan_route == []


async def test_empty_scan_route_skips_transit_and_reports(make_mission, drone, gcs):
    mission = make_mission(MissionState(lap_route=WaypointSequence(), scan_route=WaypointSequence()))
    await mission.scan(mission.state.scan_route, TRANSIT_SPEED, SCAN_SPEED, SCAN_TOLERANCE)
    assert drone.gotos == []
    assert gcs.reports == []


async def test_watch_shows_maps_and_confirms_targets(make_mission, drone, gcs, targets, site):
    target = Waypoint(site.laps[0].lat + 0.00003, site.laps[0].lon, 0.0)   # ~3 m north of the drone
    targets.append(Target("person", target))
    mission = make_mission()
    await drone.arm()
    await drone.takeoff(20)
    await mission.camera.start()
    await mission.detector.start()
    confirmed = await asyncio.wait_for(mission.watch(), 2)
    assert gcs.previews
    assert mission.stitcher.thumbnails
    assert confirmed.label == "person"
    assert math.dist((confirmed.position.lat, confirmed.position.lon), (target.lat, target.lon)) * 111_000 < 1.0
    window = [data for event, data in gcs.reports if event == "confirm_window"][-1]
    assert [c["hits"].count(True) for c in window["classes"]] == [2, 0]


async def test_scan_watches_only_while_flying_rows(make_mission, drone, site, monkeypatch):
    mission = make_mission(scan_only(site))
    route = list(mission.state.scan_route)
    started, stopped = [], []

    async def watch():
        started.append(len(drone.gotos))
        try:
            await asyncio.Event().wait()
        finally:
            stopped.append(len(drone.reached))

    monkeypatch.setattr(mission, "watch", watch)
    await asyncio.wait_for(mission.start(), 2)
    assert started == [1]                 # once, after the transit goto
    assert stopped == [len(route)]        # stopped once every row point was reached


async def test_watch_failure_stops_scan_and_holds(make_mission, drone, site, monkeypatch):
    mission = make_mission(scan_only(site))
    route = list(mission.state.scan_route)

    async def watch():
        raise RuntimeError("camera broke")

    monkeypatch.setattr(mission, "watch", watch)
    with pytest.raises(RuntimeError, match="camera broke"):
        await asyncio.wait_for(mission.start(), 2)
    assert drone.holds == 1
    assert MissionState.load().scan_route == route[len(drone.reached):]
