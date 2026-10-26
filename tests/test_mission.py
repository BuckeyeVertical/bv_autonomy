import asyncio
from unittest.mock import AsyncMock

import pytest

from bv_autonomy.components.camera.fake_camera import BEVY_INTRINSICS
from bv_autonomy.components.platform import FakePlatform
from bv_autonomy.missions.suas2026 import MissionState
from bv_autonomy.missions.suas2026.suas2026 import MIN_BATTERY
from bv_autonomy.types import WaypointSequence


def laps_only(site):
    """A fresh state with no scan, for tests about laps alone."""
    return MissionState(lap_route=MissionState.new(site, BEVY_INTRINSICS).lap_route, scan_route=WaypointSequence())


async def test_fresh_run_flies_full_route_and_reports_progress(make_mission, drone, gcs):
    mission = make_mission()
    expected = list(mission.state.lap_route)
    scan = list(mission.state.scan_route)
    await asyncio.wait_for(mission.start(), 2)
    assert drone.takeoffs == [expected[0].alt]
    assert drone.reached == expected + scan
    assert mission.state == MissionState.load() == MissionState(lap_route=[], scan_route=[])
    expected_progress = []
    for i in range(len(expected) + 1):
        expected_progress.append({
            "phase": "lap", "completed": i, "total": len(expected),
            "target": i + 1 if i < len(expected) else None,
            "segment_start": [expected[i - 1].lat, expected[i - 1].lon] if i else None,
            "segment_end": [expected[i].lat, expected[i].lon] if i < len(expected) else None,
        })
    assert [p for p in gcs.progress if p["phase"] == "lap"] == expected_progress


async def test_cancel_and_resume_in_air_has_no_repeats_or_gaps(make_mission, drone, gcs):
    mission = make_mission()
    laps, scan = list(mission.state.lap_route), list(mission.state.scan_route)
    task = asyncio.create_task(mission.start())
    await gcs.wait_reached(2)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert drone.holds == 1
    assert drone._target is None
    assert gcs.reports[-1] == ("mission_state", {"state": "aborted"})
    saved = MissionState.load()
    assert saved.lap_route == laps[len(drone.reached):]
    assert saved.scan_route == scan
    assert drone.in_air()
    resumed = make_mission(saved)
    resumed.preflight_checks(resuming=True)
    await asyncio.wait_for(resumed.start(), 2)
    assert drone.takeoffs == [laps[0].alt]
    assert drone.reached == laps + scan
    assert MissionState.load() == MissionState(lap_route=[], scan_route=[])


async def test_ground_resume_takes_off_and_flies_only_remaining_points(make_mission, drone, site):
    remaining = WaypointSequence(MissionState.new(site, BEVY_INTRINSICS).lap_route[3:])
    MissionState(lap_route=remaining, scan_route=WaypointSequence()).save()
    mission = make_mission(MissionState.load())
    await asyncio.wait_for(mission.start(), 2)
    assert drone.takeoffs == [site.laps[0].alt]
    assert drone.reached == remaining
    assert MissionState.load().lap_route == []


async def test_empty_laps_skip_flight_and_reports(make_mission, gcs, drone):
    mission = make_mission(MissionState(lap_route=WaypointSequence(), scan_route=WaypointSequence()))
    await mission.laps(mission.state.lap_route, 20, 1)
    assert gcs.reports == []
    assert drone.flight_tasks == []


async def test_state_is_saved_after_each_reached_point(make_mission, drone, gcs, site, monkeypatch):
    mission = make_mission(laps_only(site))
    full = list(mission.state.lap_route)
    save = MissionState.save
    snapshots = []

    def save_and_check(state):
        save(state)
        snapshot = MissionState.load().lap_route
        assert snapshot == full[len(drone.reached):]
        snapshots.append(snapshot)

    monkeypatch.setattr(MissionState, "save", save_and_check)
    task = asyncio.create_task(mission.start())
    await gcs.wait_reached(2)
    assert not task.done()
    assert MissionState.load().lap_route == full[2:]
    await asyncio.wait_for(task, 2)
    assert snapshots == [full[i:] for i in range(len(full) + 1)]


async def test_end_laps_holds_and_saves_empty_route(make_mission, drone, gcs, site):
    mission = make_mission(laps_only(site))
    full = list(mission.state.lap_route)
    task = asyncio.create_task(mission.start())
    await gcs.wait_reached(2)
    gcs.signal("end_laps").set()
    await asyncio.wait_for(task, 2)
    assert 0 < len(drone.reached) < len(full)
    assert drone.holds == 1
    assert drone._target is None
    assert MissionState.load().lap_route == []


async def test_stale_end_laps_signal_is_cleared(make_mission, drone, gcs, site):
    mission = make_mission(laps_only(site))
    full = list(mission.state.lap_route)
    gcs.signal("end_laps").set()
    await asyncio.wait_for(mission.start(), 2)
    assert not gcs.signal("end_laps").is_set()
    assert drone.reached == full
    assert drone.holds == 0


async def test_error_mid_lap_holds_reports_abort_and_reraises(make_mission, drone, gcs, monkeypatch):
    mission = make_mission()
    error = RuntimeError("flight failed")

    async def fail(route, speed, tolerance, on_progress):
        on_progress(0)
        await drone.goto(route[0])
        on_progress(1)
        raise error

    monkeypatch.setattr(drone, "fly_mission", fail)
    with pytest.raises(RuntimeError) as raised:
        await mission.start()
    assert raised.value is error
    assert drone.holds == 1
    assert gcs.reports[-1] == ("mission_state", {"state": "aborted"})
    assert MissionState.load().lap_route == MissionState.new(mission.site, BEVY_INTRINSICS).lap_route[1:]


async def test_cancellation_awaits_lap_child_tasks(make_mission, drone, gcs):
    mission = make_mission()
    before = asyncio.all_tasks()
    task = asyncio.create_task(mission.start())
    await gcs.wait_reached(2)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    children = asyncio.all_tasks() - before - {task}
    assert not children


async def test_ground_failure_does_not_hold_and_reraises(make_mission, drone, gcs, monkeypatch):
    mission = make_mission()
    error = RuntimeError("takeoff rejected")
    monkeypatch.setattr(drone, "takeoff", AsyncMock(side_effect=error))
    with pytest.raises(RuntimeError) as raised:
        await mission.start()
    assert raised.value is error
    assert drone.holds == 0
    assert gcs.reports[-1] == ("mission_state", {"state": "aborted"})


@pytest.mark.parametrize("air,armed,battery,message", [
    (True, True, 1.0, "already flying"),
    (False, True, 1.0, "already armed"),
    (False, False, MIN_BATTERY - 0.01, "battery"),
])
def test_preflight_refuses_unsafe_start(make_mission, drone, monkeypatch, air, armed, battery, message):
    monkeypatch.setattr(drone, "in_air", lambda: air)
    monkeypatch.setattr(drone, "is_armed", lambda: armed)
    monkeypatch.setattr(drone, "battery", lambda: battery)
    with pytest.raises(RuntimeError, match=message):
        make_mission().preflight_checks()


def test_resuming_allows_armed_airborne_drone(make_mission, drone, monkeypatch):
    monkeypatch.setattr(drone, "in_air", lambda: True)
    monkeypatch.setattr(drone, "is_armed", lambda: True)
    make_mission().preflight_checks(resuming=True)


def test_resuming_still_refuses_low_battery(make_mission, drone, monkeypatch):
    monkeypatch.setattr(drone, "battery", lambda: MIN_BATTERY - 0.01)
    with pytest.raises(RuntimeError, match="battery"):
        make_mission().preflight_checks(resuming=True)


def test_preflight_accepts_exact_minimum_battery(make_mission, drone, monkeypatch):
    monkeypatch.setattr(drone, "battery", lambda: MIN_BATTERY)
    make_mission().preflight_checks()


async def test_connect_checks_preflight_after_drone_and_gcs_start(make_mission, drone, gcs, monkeypatch):
    mission = make_mission()
    calls = []

    async def connect():
        calls.append("drone")

    async def start():
        calls.append("gcs")

    monkeypatch.setattr(drone, "connect", connect)
    monkeypatch.setattr(gcs, "start", start)
    monkeypatch.setattr(mission, "preflight_checks", lambda resuming: calls.append(resuming))
    await mission.connect(resuming=True)
    assert calls == ["drone", "gcs", True]


@pytest.mark.parametrize("resume", [False, True])
async def test_main_saves_selected_state_before_connect(site, drone, monkeypatch, resume):
    from bv_autonomy.missions.suas2026 import __main__ as entry

    old = MissionState(lap_route=WaypointSequence(site.laps[1:]), scan_route=WaypointSequence())
    old.save()
    expected = old if resume else MissionState.new(site, BEVY_INTRINSICS)
    calls = []

    class StandInMission:
        def __init__(self, selected_site, state, platform):
            assert platform.drone is drone
            assert selected_site is site
            assert state == expected
            assert MissionState.load() == expected
            calls.append("constructed")

        async def connect(self, resuming):
            assert resuming is resume
            assert MissionState.load() == expected
            calls.append("connected")

        async def start(self):
            calls.append("started")

    monkeypatch.setattr(entry, "SIM", site)
    monkeypatch.setattr(entry, "SimPlatform", lambda: FakePlatform(drone))
    monkeypatch.setattr(entry, "Suas2026", StandInMission)
    await entry.main(resume=resume)
    assert calls == ["constructed", "connected", "started"]
