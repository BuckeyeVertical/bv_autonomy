import asyncio

import pytest
import pytest_asyncio

from bv_autonomy.components.drone.fake_drone import _offset_m
from bv_autonomy.types import Waypoint, WaypointSequence


@pytest_asyncio.fixture
async def airborne(drone):
    await drone.arm()
    await asyncio.wait_for(drone.takeoff(10), 2)
    return drone


async def test_takeoff_requires_arming_and_reaches_altitude(drone):
    with pytest.raises(RuntimeError, match="disarmed"):
        await drone.takeoff(10)
    await drone.arm()
    await asyncio.wait_for(drone.takeoff(10), 2)
    assert drone.in_air()
    assert drone.position().alt == pytest.approx(10, abs=0.1)


async def test_goto_refuses_ground(drone, site):
    with pytest.raises(RuntimeError, match="ground"):
        await drone.goto(site.laps[1])


async def test_fly_mission_refuses_ground(drone, site):
    with pytest.raises(RuntimeError, match="ground"):
        await drone.fly_mission(site.laps)


async def test_goto_reaches_point_within_tolerance(airborne, site):
    target = site.laps[1]
    await asyncio.wait_for(airborne.goto(target, tolerance=0.5), 2)
    assert _offset_m(airborne.position(), target)[3] <= 0.5


async def test_fly_mission_reports_each_point_in_order(airborne, site):
    reached = []
    await asyncio.wait_for(airborne.fly_mission(site.laps, on_progress=reached.append), 2)
    assert reached == list(range(len(site.laps) + 1))
    assert _offset_m(airborne.position(), site.laps[-1])[3] <= 1


async def test_hold_stops_partway_and_fly_mission_returns(airborne):
    start = airborne.position()
    far = Waypoint(start.lat + 1, start.lon, start.alt)
    progress = []
    task = asyncio.create_task(airborne.fly_mission(WaypointSequence([far]), on_progress=progress.append))
    async with asyncio.timeout(2):
        while airborne.position() == start:
            await asyncio.sleep(0.005)
    await airborne.hold()
    stopped = airborne.position()
    await asyncio.wait_for(task, 2)
    await asyncio.sleep(0.03)
    assert start.lat < stopped.lat < far.lat
    assert airborne.position() == stopped
    assert progress == [0]


async def test_land_reaches_ground_and_disarms(airborne):
    await asyncio.wait_for(airborne.land(), 2)
    assert not airborne.in_air()
    assert not airborne.is_armed()
    assert airborne.position().alt == 0


async def test_return_home_lands_at_home(airborne, site):
    await asyncio.wait_for(airborne.goto(site.laps[1]), 2)
    await asyncio.wait_for(airborne.return_home(), 2)
    assert _offset_m(airborne.position(), airborne.home())[3] <= 0.1
    assert not airborne.in_air()
    assert not airborne.is_armed()


async def test_cannot_disarm_in_air(airborne):
    with pytest.raises(RuntimeError, match="air"):
        await airborne.disarm()


async def test_battery_drains_only_while_armed(drone, monkeypatch):
    monkeypatch.setattr(drone, "_drain", 0.001)
    initial = drone.battery()
    await asyncio.sleep(0.03)
    assert drone.battery() == initial
    await drone.arm()
    await asyncio.sleep(0.03)
    assert 0 <= drone.battery() < initial
    await drone.disarm()
    remaining = drone.battery()
    await asyncio.sleep(0.03)
    assert drone.battery() == remaining


async def test_battery_never_goes_negative(drone, monkeypatch):
    monkeypatch.setattr(drone, "_drain", 1)
    await drone.arm()
    await asyncio.sleep(0.03)
    assert drone.battery() == 0


