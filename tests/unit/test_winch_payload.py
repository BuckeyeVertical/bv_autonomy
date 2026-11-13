import asyncio
import logging
import time
from types import SimpleNamespace

import pytest
from mavsdk.asyncio.plugins.action import ActionError

from bv_autonomy.components.payload import winch_payload as ap
from bv_autonomy.components.payload.winch_payload import (
    AIRCRAFT_PAYLOAD, WinchPayload, BrakePhase, Drop, DropSequence, PayloadConfig, Servo,
)

PLATE, CLAMP = Servo(2, 800, 2200), Servo(1, 800, 2200)
HOLD, OPEN = 1685, 1900


def config(**changes):
    settings = dict(plate=PLATE, clamp=CLAMP, plate_hold_us=HOLD, unclamped_us=OPEN, pre_drop_s=0.05,
                    drops={"bottle": Drop(2050, 2020, (BrakePhase(0.0, 0.2), BrakePhase(0.11, 0.03),
                                                       BrakePhase(0.07, 0.02)))})
    settings.update(changes)
    return PayloadConfig(**settings)


class ActuatorDrone:
    """Records each actuator command PX4 would get, and fails the ones a test asks it to."""

    def __init__(self, failures=0):
        self.commands = []
        self.failures = failures

    async def set_actuator(self, actuator_set, value):
        await asyncio.sleep(0)
        if self.failures:
            self.failures -= 1
            raise ActionError(SimpleNamespace(name="TIMEOUT"), "set_actuator")
        self.commands.append((time.monotonic(), actuator_set, round(value, 4)))

    def last(self, actuator_set):
        return next(value for _, s, value in reversed(self.commands) if s == actuator_set)


def test_servo_maps_its_pulse_range_onto_minus_one_to_one():
    assert (PLATE.value(800), PLATE.value(1500), PLATE.value(2200)) == (-1.0, 0.0, 1.0)


@pytest.mark.parametrize("changes,message", [
    ({"clamp": Servo(2, 800, 2200)}, "different actuator sets"),
    ({"plate_hold_us": 2300}, "plate hold 2300"),
    ({"drops": {"bottle": Drop(2050, 2020, (BrakePhase(0.0, 0.1),))}}, "more than 0 s"),
    ({"drops": {"bottle": Drop(2050, 2020, (BrakePhase(1.0, 0.0),))}}, "toggle"),
    ({"pre_drop_s": -1}, "negative"),
])
def test_config_refuses_what_would_never_release(changes, message):
    with pytest.raises(ValueError, match=message):
        config(**changes)


def test_aircraft_config_is_bv_cores():
    assert DropSequence(AIRCRAFT_PAYLOAD, "bottle").total_s == pytest.approx(20.0)
    assert DropSequence(AIRCRAFT_PAYLOAD, "beacon").total_s == pytest.approx(29.0)


def test_drop_brakes_before_the_plate_moves_then_rests_at_the_end():
    sequence = DropSequence(config(), "bottle")
    assert sequence.total_s == pytest.approx(0.23)
    assert sequence.positions_at(0.0) == (HOLD, 2020, False)
    assert sequence.positions_at(0.04)[0] == HOLD
    assert sequence.positions_at(0.06)[0] == 2050
    assert sequence.positions_at(0.24) == (HOLD, OPEN, True)


def test_clamp_toggles_in_one_rhythm_using_each_phases_interval():
    sequence = DropSequence(config(), "bottle")
    assert sequence._flips == pytest.approx([0.03, 0.06, 0.09, 0.12, 0.15, 0.18, 0.20, 0.22])


async def test_start_puts_the_plate_at_hold_and_the_clamp_open():
    drone = ActuatorDrone()
    await WinchPayload(drone, config()).start()
    assert drone.last(2) == round(PLATE.value(HOLD), 4) and drone.last(1) == round(CLAMP.value(OPEN), 4)


async def test_release_drops_on_schedule_and_ends_at_rest():
    drone = ActuatorDrone()
    payload = WinchPayload(drone, config())
    started = time.monotonic()
    await payload.release("bottle")
    assert time.monotonic() - started == pytest.approx(0.23, abs=0.05)
    plates = {value for _, s, value in drone.commands if s == 2}
    assert round(PLATE.value(2050), 4) in plates
    assert drone.last(2) == round(PLATE.value(HOLD), 4) and drone.last(1) == round(CLAMP.value(OPEN), 4)
    sets = [s for _, s, _ in drone.commands]
    assert sets == [2, 1] * (len(sets) // 2)


async def test_unconfirmed_commands_are_retried_and_the_drop_still_ends_on_time(monkeypatch):
    monkeypatch.setattr(ap, "_RETRY_S", 0.01)
    drone = ActuatorDrone(failures=3)
    started = time.monotonic()
    await WinchPayload(drone, config()).release("bottle")
    assert time.monotonic() - started == pytest.approx(0.23, abs=0.06)
    assert drone.last(2) == round(PLATE.value(HOLD), 4)


async def test_a_stopped_drop_returns_to_rest():
    drone = ActuatorDrone()
    task = asyncio.create_task(WinchPayload(drone, config()).release("bottle"))
    await asyncio.sleep(0.08)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert drone.last(2) == round(PLATE.value(HOLD), 4) and drone.last(1) == round(CLAMP.value(OPEN), 4)


async def test_start_raises_if_px4_never_confirms(monkeypatch):
    monkeypatch.setattr(ap, "_RETRY_S", 0.001)
    with pytest.raises(RuntimeError, match="didn't confirm"):
        await WinchPayload(ActuatorDrone(failures=1000), config()).start()


async def test_unknown_slot_and_pick_up_are_refused():
    payload = WinchPayload(ActuatorDrone(), config())
    with pytest.raises(ValueError, match="no payload slot"):
        await payload.release("beacon")
    with pytest.raises(NotImplementedError):
        await payload.pick_up()


async def test_each_stage_of_the_drop_is_logged_once(caplog):
    with caplog.at_level(logging.INFO, logger=ap.__name__):
        await WinchPayload(ActuatorDrone(), config()).release("bottle")
    stages = len(DropSequence(config(), "bottle").stages)
    lines = [r.getMessage() for r in caplog.records if r.levelno == logging.INFO]
    assert len(lines) == stages + 1
    assert len(set(lines)) == len(lines)


async def test_unconfirmed_commands_are_warned_about_once_until_they_recover(caplog, monkeypatch):
    monkeypatch.setattr(ap, "_RETRY_S", 0.001)
    with caplog.at_level(logging.INFO, logger=ap.__name__):
        await WinchPayload(ActuatorDrone(failures=4), config()).start()
    levels = [r.levelno for r in caplog.records]
    assert levels.count(logging.WARNING) == 1
    assert levels[-1] == logging.INFO


async def test_stopping_rests_the_payload_and_never_raises(caplog):
    drone = ActuatorDrone()
    await WinchPayload(drone, config()).stop()
    assert drone.last(2) == round(PLATE.value(HOLD), 4) and drone.last(1) == round(CLAMP.value(OPEN), 4)
    with caplog.at_level(logging.WARNING):
        await WinchPayload(ActuatorDrone(failures=100), config()).stop()
    assert any(r.levelno == logging.WARNING for r in caplog.records)
