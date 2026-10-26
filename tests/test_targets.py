import asyncio
from unittest.mock import AsyncMock

import pytest

from bv_autonomy.components.camera.fake_camera import BEVY_INTRINSICS
from bv_autonomy.fake_world import Target
from bv_autonomy.missions.suas2026 import MissionState, suas2026
from bv_autonomy.missions.suas2026.confirmation import Sighting, distance_m
from bv_autonomy.types import Waypoint, WaypointSequence


def scan_only(site):
    return MissionState(lap_route=WaypointSequence(), scan_route=MissionState.new(site, BEVY_INTRINSICS).scan_route)


@pytest.fixture
def on_first_row(site, targets):
    """Places a person halfway along the first scan row."""
    a, b = MissionState.new(site, BEVY_INTRINSICS).scan_route[:2]
    position = Waypoint((a.lat + b.lat) / 2, (a.lon + b.lon) / 2, 0.0)
    targets.append(Target("person", position))
    return position


@pytest.fixture
def slow_drone(drone):
    drone._speedup = 100
    return drone


async def fly(mission):
    """Runs the mission up to the end of the scan."""
    mission.return_home = AsyncMock()
    await mission.camera.start()
    await mission.detector.start()
    await asyncio.wait_for(mission.start(), 10)


async def test_approved_target_is_delivered_then_scan_carries_on_from_where_it_stopped(
        make_mission, slow_drone, gcs, site, on_first_row):
    gcs.answer = True
    mission = make_mission(scan_only(site))
    route = list(mission.state.scan_route)
    await fly(mission)

    assert len(gcs.requests) == 1
    request, timeout = gcs.requests[0]
    assert timeout == suas2026.APPROVAL_TIMEOUT_S
    assert request.label == "person"
    assert distance_m(request.position, on_first_row) < 2.0
    assert request.crop.startswith(b"\xff\xd8")
    assert slow_drone.holds == 2
    assert ("mission_state", {"state": "localize"}) in gcs.reports
    assert ("mission_state", {"state": "deliver"}) in gcs.reports

    drop, back = slow_drone.gotos[1:3]
    assert drop[1:] == (suas2026.DELIVER_SPEED, suas2026.DELIVER_TOLERANCE)
    assert distance_m(drop[0], request.position) < 0.01 and drop[0].alt == suas2026.SCAN_ALT
    assert back[1] == suas2026.TRANSIT_SPEED
    assert distance_m(back[0], request.drone_position) < 0.5
    assert [action for _, action, _ in mission.payload.log] == ["release"]
    assert mission.payload.log[0][2] == "bottle"

    saved = MissionState.load()
    assert saved.targets == []
    assert [t.label for t in saved.delivered] == ["person"]
    assert saved.rejected == []
    assert saved.scan_route == []
    assert [p for p in slow_drone.reached if p in route] == route


async def test_no_answer_counts_as_approved(make_mission, slow_drone, gcs, site, on_first_row):
    gcs.answer = None
    mission = make_mission(scan_only(site))
    await fly(mission)
    assert [t.label for t in MissionState.load().delivered] == ["person"]


async def test_rejected_spot_is_saved_and_not_asked_about_again(make_mission, slow_drone, gcs, site, on_first_row):
    gcs.answer = False
    mission = make_mission(scan_only(site))
    await fly(mission)
    saved = MissionState.load()
    assert len(gcs.requests) == 1
    assert saved.targets == saved.delivered == []
    assert mission.payload.log == []
    assert len(saved.rejected) == 2
    assert all(distance_m(r.position, on_first_row) < 2.0 for r in saved.rejected)
    assert saved.scan_route == []


async def test_target_lost_after_holding_resumes_scan_without_asking(
        make_mission, slow_drone, gcs, site, on_first_row, monkeypatch):
    monkeypatch.setattr(suas2026, "LOCATE_RETRY_S", 0.01)
    mission = make_mission(scan_only(site))
    hold = slow_drone.hold

    async def hold_and_lose_sight():
        mission.detector._targets.clear()
        await hold()

    monkeypatch.setattr(slow_drone, "hold", hold_and_lose_sight)
    await fly(mission)
    assert slow_drone.holds == 1
    assert gcs.requests == []
    assert MissionState.load().targets == []
    assert MissionState.load().scan_route == []


async def test_crop_failure_still_asks_without_an_image(make_mission, slow_drone, gcs, site, on_first_row, monkeypatch):
    def broken_crop(detection):
        raise RuntimeError("encoder broke")

    monkeypatch.setattr(suas2026, "annotated_crop", broken_crop)
    mission = make_mission(scan_only(site))
    await fly(mission)
    assert gcs.requests[0][0].crop == b""


async def test_resume_delivers_an_approved_target_before_scanning(make_mission, drone, gcs, site):
    tent = Waypoint(site.laps[0].lat + 0.0003, site.laps[0].lon, 0.0)
    state = scan_only(site)
    state.targets.append(Sighting("tent", tent))
    state.save()
    mission = make_mission(MissionState.load())
    route = list(mission.state.scan_route)
    await fly(mission)
    assert distance_m(drone.gotos[0][0], tent) < 0.01
    assert mission.payload.log[0][2] == "beacon"
    assert [t.label for t in MissionState.load().delivered] == ["tent"]
    assert drone.reached == route


async def test_a_delivered_label_is_not_looked_for_again(make_mission, slow_drone, gcs, site, on_first_row):
    state = scan_only(site)
    state.delivered.append(Sighting("person", Waypoint(36.5, -96.5, 0.0)))
    mission = make_mission(state)
    await fly(mission)
    assert gcs.requests == []
