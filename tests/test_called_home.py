import asyncio

from bv_autonomy.components.camera.fake_camera import BEVY_INTRINSICS
from bv_autonomy.components.platform.fake_world import Target
from bv_autonomy.missions.suas2026 import MissionState
from bv_autonomy.missions.suas2026.confirmation import Sighting
from bv_autonomy.types import Waypoint, WaypointSequence


async def wait_for(condition):
    """Waits until the condition holds."""
    async with asyncio.timeout(5):
        while not condition():
            await asyncio.sleep(0.005)


async def test_return_home_during_laps_skips_the_rest_and_lands(make_mission, drone, gcs):
    mission = make_mission()
    laps = list(mission.state.lap_route)
    task = asyncio.create_task(mission.start())
    await gcs.wait_reached(2)
    gcs.signal("return_home").set()
    await asyncio.wait_for(task, 2)
    saved = MissionState.load()
    assert 0 < len(drone.reached) < len(laps)
    assert saved.returning
    assert saved.scan_route
    assert gcs.reports[-1] == ("mission_state", {"state": "return"})
    assert not drone.in_air()


async def test_return_home_while_waiting_for_approval_withdraws_it(make_mission, drone, gcs, targets, site,
                                                                     monkeypatch):
    route = MissionState.new(site, BEVY_INTRINSICS).scan_route
    middle = Waypoint((route[0].lat + route[1].lat) / 2, (route[0].lon + route[1].lon) / 2, 0.0)
    targets.append(Target("person", middle))
    drone._speedup = 100
    asked = asyncio.Event()

    async def wait_for_operator(request, timeout):
        asked.set()
        await asyncio.Event().wait()

    monkeypatch.setattr(gcs, "ask", wait_for_operator)
    mission = make_mission(MissionState(lap_route=WaypointSequence(), scan_route=route))
    await mission.camera.start()
    await mission.detector.start()
    task = asyncio.create_task(mission.start())
    await asyncio.wait_for(asked.wait(), 5)
    gcs.signal("return_home").set()
    await asyncio.wait_for(task, 5)
    saved = MissionState.load()
    assert saved.targets == saved.delivered == saved.rejected == []
    assert saved.returning
    assert not drone.in_air()


async def test_return_home_during_a_drop_stops_it(make_mission, drone, gcs, site):
    tent = Waypoint(site.laps[0].lat + 0.0003, site.laps[0].lon, 0.0)
    state = MissionState(lap_route=WaypointSequence(), scan_route=WaypointSequence(), targets=[Sighting("tent", tent)])
    mission = make_mission(state)
    mission.payload._action_s = 5.0
    task = asyncio.create_task(mission.start())
    await wait_for(lambda: ("mission_state", {"state": "deliver"}) in gcs.reports and drone.holds)
    gcs.signal("return_home").set()
    await asyncio.wait_for(task, 2)
    assert mission.payload.log == []
    assert [t.label for t in MissionState.load().targets] == ["tent"]
    assert not drone.in_air()


async def test_a_press_before_the_mission_starts_is_ignored(make_mission, drone, gcs):
    gcs.signal("return_home").set()
    mission = make_mission()
    await asyncio.wait_for(mission.start(), 2)
    saved = MissionState.load()
    assert saved.lap_route == saved.scan_route == []


async def test_resume_after_return_home_was_pressed_goes_straight_home(make_mission, drone, site):
    await drone.arm()
    await drone.takeoff(20)
    state = MissionState.new(site, BEVY_INTRINSICS)
    state.returning = True
    mission = make_mission(state)
    await asyncio.wait_for(mission.start(), 2)
    assert drone.reached == []
    assert drone.gotos == []
    assert not drone.in_air()


async def test_resume_of_a_finished_mission_stays_on_the_ground(make_mission, drone, site):
    state = MissionState(lap_route=WaypointSequence(), scan_route=WaypointSequence(), returning=True)
    mission = make_mission(state)
    await asyncio.wait_for(mission.start(), 2)
    assert drone.takeoffs == []
    assert not drone.is_armed()
