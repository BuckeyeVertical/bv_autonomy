import asyncio
import math
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
from mavsdk.asyncio.plugins.action import ActionError
from mavsdk.asyncio.plugins.mission import MissionError
from mavsdk.plugins.telemetry import FlightMode

from bv_autonomy.components.drone import mavsdk_drone as md
from bv_autonomy.types import Waypoint, WaypointSequence


class TelemetryClock:
    def __init__(self):
        self.now = 0.0
        self.step = lambda: None

    async def sleep(self, seconds):
        self.now += seconds
        assert self.now < 100, "telemetry scenario did not terminate"
        self.step()
        await asyncio.sleep(0)


@pytest.fixture
def clock(monkeypatch):
    clock = TelemetryClock()
    monkeypatch.setattr(md, "time", SimpleNamespace(monotonic=lambda: clock.now))
    monkeypatch.setattr(md, "asyncio", SimpleNamespace(sleep=clock.sleep, create_task=asyncio.create_task))
    monkeypatch.setattr(md, "_STALL_S", 1.0)
    monkeypatch.setattr(md, "_MODE_GRACE_S", 0.3)
    monkeypatch.setattr(md, "_ACCEPT_TIMEOUT_S", 0.2)
    return clock


@pytest.fixture
def mavdrone():
    drone = md.MavsdkDrone()
    drone._position = SimpleNamespace(latitude_deg=0.0, longitude_deg=0.0, relative_altitude_m=20.0)
    drone._home = SimpleNamespace(latitude_deg=0.0, longitude_deg=0.0)
    drone._flight_mode = FlightMode.MISSION
    drone._mission_progress = SimpleNamespace(current=0, total=3)
    drone._in_air = True
    return drone


async def test_wait_finishes_on_long_slow_approach(mavdrone, clock):
    remaining = lambda: 100 - clock.now * 10
    await mavdrone._wait_for_progress(
        lambda: remaining() <= 0, remaining, {FlightMode.MISSION}, "slow approach"
    )
    assert clock.now > md._STALL_S * 5


async def test_wait_times_out_despite_position_jitter(mavdrone, clock):
    with pytest.raises(TimeoutError, match="no progress"):
        await mavdrone._wait_for_progress(
            lambda: False, lambda: 20 + 2 * math.sin(clock.now * 20),
            {FlightMode.MISSION}, "jittering position",
        )
    assert md._STALL_S < clock.now < md._STALL_S + 0.3


async def test_wait_raises_after_mode_grace(mavdrone, clock):
    mavdrone._flight_mode = FlightMode.HOLD
    with pytest.raises(RuntimeError, match="switched to HOLD"):
        await mavdrone._wait_for_progress(lambda: False, lambda: 100, {FlightMode.MISSION}, "flight")
    assert clock.now >= md._MODE_GRACE_S
    assert clock.now < md._STALL_S


async def test_wait_tolerates_transient_mode_change(mavdrone, clock):
    def telemetry():
        mavdrone._flight_mode = FlightMode.HOLD if 0.2 <= clock.now < 0.4 else FlightMode.MISSION

    clock.step = telemetry
    await mavdrone._wait_for_progress(
        lambda: clock.now >= 0.6, lambda: 100, {FlightMode.MISSION}, "brief mode change"
    )


async def test_new_mission_item_counts_as_progress(mavdrone, clock):
    clock.step = lambda: setattr(mavdrone._mission_progress, "current", int(clock.now / 0.7))
    await mavdrone._wait_for_progress(
        lambda: clock.now >= 2.5, lambda: 100, {FlightMode.MISSION}, "new mission items"
    )
    assert mavdrone._mission_progress.current >= 3


@pytest.mark.parametrize("error_type", [ActionError, MissionError])
async def test_command_retries_sdk_errors_until_telemetry_confirms(mavdrone, clock, error_type):
    attempts = 0
    accepted = False

    async def send():
        nonlocal attempts, accepted
        attempts += 1
        if attempts < md._RETRIES:
            raise error_type(SimpleNamespace(name="TIMEOUT"), "test")
        accepted = True

    await mavdrone._command(send, lambda: accepted, "test command")
    assert attempts == md._RETRIES


@pytest.mark.parametrize("error_type", [ActionError, MissionError])
async def test_command_stops_at_retry_limit(mavdrone, clock, error_type):
    send = AsyncMock(side_effect=error_type(SimpleNamespace(name="TIMEOUT"), "test"))
    with pytest.raises(RuntimeError, match=f"after {md._RETRIES} tries"):
        await mavdrone._command(send, lambda: False, "test command")
    assert send.await_count == md._RETRIES


async def test_command_requires_telemetry_even_after_successful_ack(mavdrone, clock):
    send = AsyncMock()
    with pytest.raises(RuntimeError, match=f"after {md._RETRIES} tries"):
        await mavdrone._command(send, lambda: False, "test command")
    assert send.await_count == md._RETRIES


async def test_command_accepts_telemetry_after_lost_ack(mavdrone, clock):
    accepted = False

    async def send():
        nonlocal accepted
        accepted = True
        raise ActionError(SimpleNamespace(name="TIMEOUT"), "test")

    await mavdrone._command(send, lambda: accepted, "test command")
    assert clock.now == 0


async def test_command_waits_for_delayed_telemetry(mavdrone, clock):
    send = AsyncMock()
    await mavdrone._command(send, lambda: clock.now >= 0.1, "test command")
    send.assert_awaited_once()
    assert clock.now >= 0.1


@pytest.mark.parametrize("always_send,expected_sends", [(False, 0), (True, 1)])
async def test_command_already_in_target_state(mavdrone, clock, always_send, expected_sends):
    send = AsyncMock()
    await mavdrone._command(send, lambda: True, "test command", always_send=always_send)
    assert send.await_count == expected_sends


async def test_command_does_not_swallow_unexpected_error(mavdrone, clock):
    error = ValueError("bad command")
    with pytest.raises(ValueError) as raised:
        await mavdrone._command(AsyncMock(side_effect=error), lambda: False, "test command")
    assert raised.value is error


@pytest.mark.parametrize("failure", [asyncio.CancelledError, RuntimeError])
async def test_failed_flight_never_reports_final_progress(mavdrone, monkeypatch, failure):
    route = WaypointSequence([Waypoint(0, 0, 20), Waypoint(0.001, 0, 20)])
    progress = []
    entered = asyncio.Event()

    async def wait(*args):
        await asyncio.sleep(0)
        entered.set()
        if failure is RuntimeError:
            raise RuntimeError("flight failed")
        await asyncio.Event().wait()

    monkeypatch.setattr(mavdrone, "_start_mission", AsyncMock())
    monkeypatch.setattr(mavdrone, "_wait_for_progress", wait)
    mavdrone._mission_progress = SimpleNamespace(current=1, total=len(route))
    task = asyncio.create_task(mavdrone.fly_mission(route, on_progress=progress.append))
    await asyncio.wait_for(entered.wait(), 2)
    if failure is asyncio.CancelledError:
        task.cancel()
    with pytest.raises(failure):
        await task
    await asyncio.sleep(0)
    assert progress == [1]


async def test_flight_waits_for_matching_route_and_reports_real_progress(mavdrone, clock, monkeypatch):
    route = WaypointSequence([Waypoint(0, 0, 20), Waypoint(0.001, 0, 20), Waypoint(0.002, 0, 20)])
    progress = []
    monkeypatch.setattr(mavdrone, "_start_mission", AsyncMock())
    mavdrone._mission_progress = SimpleNamespace(current=99, total=99)

    def telemetry():
        if clock.now >= 0.2:
            current = min(3, int((clock.now - 0.2) / 0.4))
            mavdrone._mission_progress = SimpleNamespace(current=current, total=3)

    clock.step = telemetry
    await mavdrone.fly_mission(route, on_progress=progress.append)
    assert clock.now >= 1.4
    assert progress[0] == 0
    assert 1 in progress and 2 in progress
    assert progress[-1] == len(route)
    assert 99 not in progress


async def test_report_progress_ignores_mismatched_total_and_duplicate_updates(mavdrone, clock):
    progress = []
    mavdrone._mission_progress = SimpleNamespace(current=2, total=99)

    def telemetry():
        if clock.now >= 0.2:
            mavdrone._mission_progress = SimpleNamespace(current=1, total=3)

    clock.step = telemetry
    reporter = asyncio.create_task(mavdrone._report_progress(3, progress.append))
    async with asyncio.timeout(2):
        while clock.now < 0.5:
            await asyncio.sleep(0)
    reporter.cancel()
    with pytest.raises(asyncio.CancelledError):
        await reporter
    assert progress == [1]


async def test_start_mission_uploads_even_if_previous_mission_mode_is_active(mavdrone, clock):
    mavdrone._mission = SimpleNamespace(upload_mission=AsyncMock(), start_mission=AsyncMock())
    point = Waypoint(0.001, 0, 20)
    mavdrone._mission_progress = SimpleNamespace(current=3, total=3)

    def telemetry():
        mavdrone._mission_progress = SimpleNamespace(current=0, total=1)

    clock.step = telemetry
    item = md._mission_item(point, 12, 4, True)
    await mavdrone._start_mission([item], point, 4)
    mavdrone._mission.upload_mission.assert_awaited_once()
    mavdrone._mission.start_mission.assert_awaited_once()
    assert clock.now > 0


async def test_start_mission_rejects_old_completed_telemetry_far_from_destination(mavdrone, clock):
    mavdrone._mission = SimpleNamespace(upload_mission=AsyncMock(), start_mission=AsyncMock())
    mavdrone._mission_progress = SimpleNamespace(current=3, total=3)
    point = Waypoint(0.001, 0, 20)
    with pytest.raises(RuntimeError, match="start the mission"):
        await mavdrone._start_mission([md._mission_item(point, 12, 4, True)], point, 4)
    assert mavdrone._mission.upload_mission.await_count == md._RETRIES


async def test_return_home_finishes_long_return_and_descent(mavdrone, clock):
    async def enter_rtl():
        mavdrone._flight_mode = FlightMode.RETURN_TO_LAUNCH

    mavdrone._action = SimpleNamespace(return_to_launch=AsyncMock(side_effect=enter_rtl))
    mavdrone._position.latitude_deg = math.degrees(100 / md._EARTH_R)

    def telemetry():
        mavdrone._position.latitude_deg = math.degrees(max(0, 100 - clock.now * 50) / md._EARTH_R)
        mavdrone._position.relative_altitude_m = max(0, 20 - max(0, clock.now - 2) * 5)
        mavdrone._in_air = clock.now < 6

    clock.step = telemetry
    await mavdrone.return_home()
    assert not mavdrone.in_air()
    assert clock.now > md._STALL_S
    assert mavdrone.position() == Waypoint(0, 0, 0)
    mavdrone._action.return_to_launch.assert_awaited_once()


def test_mission_item_preserves_flight_parameters():
    item = md._mission_item(Waypoint(36, -96, 60), 20, 40, True)
    assert (item.latitude_deg, item.longitude_deg, item.relative_altitude_m) == (36, -96, 60)
    assert item.speed_m_s == 20
    assert item.acceptance_radius_m == 40
    assert item.is_fly_through


def test_mission_item_default_speed_leaves_px4_setting():
    item = md._mission_item(Waypoint(36, -96, 60), None, 1, False)
    assert math.isnan(item.speed_m_s)
    assert not item.is_fly_through


def test_pose_converts_degrees_and_keeps_position_timestamp(mavdrone):
    mavdrone._attitude = SimpleNamespace(roll_deg=30, pitch_deg=-45, yaw_deg=-90)
    mavdrone._position_time = 12.5
    pose = mavdrone.pose()
    assert (pose.roll, pose.pitch, pose.yaw) == pytest.approx((math.pi / 6, -math.pi / 4, -math.pi / 2))
    assert pose.position == Waypoint(0, 0, 20)
    assert pose.time == 12.5
    assert mavdrone.heading() == 270


async def test_hold_waits_for_braking_after_mode_switch(mavdrone, clock):
    async def enter_hold():
        mavdrone._flight_mode = FlightMode.HOLD

    mavdrone._action = SimpleNamespace(hold=AsyncMock(side_effect=enter_hold))
    mavdrone._velocity = SimpleNamespace(north_m_s=3, east_m_s=4)

    def telemetry():
        if clock.now >= 0.2:
            mavdrone._velocity = SimpleNamespace(north_m_s=0.1, east_m_s=0.1)

    clock.step = telemetry
    await mavdrone.hold()
    assert clock.now >= 0.2
    mavdrone._action.hold.assert_awaited_once()


async def test_land_waits_for_touchdown_after_entering_land_mode(mavdrone, clock):
    async def enter_land():
        mavdrone._flight_mode = FlightMode.LAND

    mavdrone._action = SimpleNamespace(land=AsyncMock(side_effect=enter_land))
    clock.step = lambda: setattr(mavdrone, "_in_air", clock.now < 0.3)
    await mavdrone.land()
    assert clock.now >= 0.3
    assert not mavdrone.in_air()


async def test_takeoff_waits_for_climb_after_leaving_ground(mavdrone, clock):
    async def leave_ground():
        mavdrone._in_air = True

    mavdrone._in_air = False
    mavdrone._position.relative_altitude_m = 0
    mavdrone._action = SimpleNamespace(
        set_takeoff_altitude=AsyncMock(), takeoff=AsyncMock(side_effect=leave_ground),
    )
    clock.step = lambda: setattr(mavdrone._position, "relative_altitude_m", clock.now * 10)
    await mavdrone.takeoff(10)
    assert mavdrone.position().alt >= 9.5
    mavdrone._action.set_takeoff_altitude.assert_awaited_once_with(10)


async def test_goto_to_where_the_drone_already_is_flies_nothing(mavdrone, monkeypatch):
    start = AsyncMock()
    monkeypatch.setattr(mavdrone, "_start_mission", start)
    mavdrone._flight_mode = FlightMode.HOLD
    here = mavdrone.position()
    await mavdrone.goto(Waypoint(here.lat, here.lon, here.alt), 3.0, 1.0)
    start.assert_not_awaited()


