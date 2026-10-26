"""The aircraft's plate and clamp payload release, driven through PX4's actuator outputs, as in bv_core."""

import asyncio
import bisect
import logging
import time
from dataclasses import dataclass

from mavsdk.asyncio.plugins.action import ActionError

from bv_autonomy.components.drone import MavsdkDrone
from bv_autonomy.components.payload.base import Payload

log = logging.getLogger(__name__)

_TICK_S = 0.02
_RETRY_S = 0.2
_SETTLE_TRIES = 10


@dataclass(frozen=True)
class Servo:
    """A servo on one of PX4's actuator sets, which maps commands onto the set's pulse range."""

    actuator_set: int
    min_us: float
    max_us: float

    def value(self, pulse_us: float) -> float:
        """Returns the actuator command that makes the servo emit this pulse width."""
        return 2.0 * (pulse_us - self.min_us) / (self.max_us - self.min_us) - 1.0


@dataclass(frozen=True)
class BrakePhase:
    """A stretch of the drop during which the clamp toggles at a steady rate."""

    duration_s: float
    toggle_s: float


@dataclass(frozen=True)
class Drop:
    """How one payload is dropped, with the plate position that releases it and how the clamp brakes it."""

    plate_us: float
    clamped_us: float
    phases: tuple[BrakePhase, ...]


@dataclass(frozen=True)
class PayloadConfig:
    """The plate and clamp servos, their resting positions, and how each payload is dropped."""

    plate: Servo
    clamp: Servo
    plate_hold_us: float
    unclamped_us: float
    pre_drop_s: float
    drops: dict[str, Drop]

    def __post_init__(self):
        if self.plate.actuator_set == self.clamp.actuator_set:
            raise ValueError("the plate and clamp need different actuator sets")
        if self.pre_drop_s < 0:
            raise ValueError("pre_drop_s can't be negative")
        positions = [(self.plate, "plate hold", self.plate_hold_us), (self.clamp, "unclamped", self.unclamped_us)]
        for slot, drop in self.drops.items():
            positions += [(self.plate, f"plate {slot} drop", drop.plate_us),
                          (self.clamp, f"{slot} clamped", drop.clamped_us)]
            if any(phase.duration_s < 0 or phase.toggle_s <= 0 for phase in drop.phases):
                raise ValueError(f"{slot} has a phase with a negative duration or a toggle that isn't positive")
            if sum(phase.duration_s for phase in drop.phases) <= 0:
                raise ValueError(f"{slot}'s phases must last more than 0 s, or the plate moves out and straight back")
        unreachable = [f"{name} {pulse:g} us" for servo, name, pulse in positions
                       if not servo.min_us <= pulse <= servo.max_us]
        if unreachable:
            raise ValueError(f"outside the servo's pulse range, so PX4 would never reach them: {', '.join(unreachable)}")


AIRCRAFT_PAYLOAD = PayloadConfig(
    plate=Servo(actuator_set=2, min_us=800, max_us=2200),
    clamp=Servo(actuator_set=1, min_us=800, max_us=2200),
    plate_hold_us=1685,
    unclamped_us=1900,
    pre_drop_s=1.0,
    drops={
        "bottle": Drop(plate_us=2050, clamped_us=2020,
                       phases=(BrakePhase(0.0, 0.2), BrakePhase(9.0, 0.15), BrakePhase(10.0, 0.1))),
        "beacon": Drop(plate_us=1360, clamped_us=2020,
                       phases=(BrakePhase(0.0, 0.2), BrakePhase(8.0, 0.15), BrakePhase(20.0, 0.1))),
    },
)


class DropSequence:
    """Where the plate and clamp should be at each moment of one drop.

    The clamp brakes for pre_drop_s before the plate moves, then the plate stays at the drop position
    through the braking phases while the clamp toggles in one continuous rhythm. At the end the plate
    returns to hold and the clamp opens, so the other payload is held again.
    """

    def __init__(self, config: PayloadConfig, slot: str):
        self._config = config
        self._drop = config.drops[slot]
        phases = [phase for phase in self._drop.phases if phase.duration_s > 0]
        self.total_s = config.pre_drop_s + sum(phase.duration_s for phase in phases)
        self._flips = self._flip_times(phases)
        self.stages: list[tuple[float, str]] = []
        if config.pre_drop_s > 0:
            self.stages.append((0.0, "braking before the plate opens"))
        start = config.pre_drop_s
        for phase in phases:
            self.stages.append((start, f"plate open, clamp toggling every {phase.toggle_s * 1000:.0f} ms "
                                       f"for {phase.duration_s:g} s"))
            start += phase.duration_s

    def stage_at(self, elapsed_s: float) -> int:
        """Returns which of the drop's stages this time into the drop falls in."""
        return max(bisect.bisect_right([start for start, _ in self.stages], elapsed_s) - 1, 0)

    def _flip_times(self, phases: list[BrakePhase]) -> list[float]:
        """Returns the times at which the clamp changes position, each one toggle after the last."""
        segments, start = [], 0.0
        if self._config.pre_drop_s > 0:
            segments.append((0.0, phases[0].toggle_s))
            start = self._config.pre_drop_s
        for phase in phases:
            segments.append((start, phase.toggle_s))
            start += phase.duration_s
        flips, t = [], 0.0
        while True:
            t += next(toggle for segment_start, toggle in reversed(segments) if segment_start <= t)
            if t >= self.total_s:
                return flips
            flips.append(t)

    def positions_at(self, elapsed_s: float) -> tuple[float, float, bool]:
        """Returns the plate and clamp pulse widths at this time into the drop, and whether it's over."""
        if elapsed_s >= self.total_s:
            return self._config.plate_hold_us, self._config.unclamped_us, True
        plate = self._config.plate_hold_us if elapsed_s < self._config.pre_drop_s else self._drop.plate_us
        clamped = bisect.bisect_right(self._flips, elapsed_s) % 2 == 0
        return plate, self._drop.clamped_us if clamped else self._config.unclamped_us, False


class WinchPayload(Payload):
    """The plate that holds both payloads and drops one at a time, with a clamp that brakes each fall.

    Every change commands both servos, so a lost command can't leave the plate where it was. Timing
    follows the clock, not PX4's replies, so a drop always takes the same time and a command PX4 doesn't
    confirm is sent again shortly after.
    """

    def __init__(self, drone: MavsdkDrone, config: PayloadConfig = AIRCRAFT_PAYLOAD):
        self._drone = drone
        self._config = config
        self._confirmed: tuple[float, float] | None = None
        self._retry_at = 0.0
        self._failing = False
        self._sent = 0
        self._unconfirmed = 0

    async def start(self) -> None:
        """Puts the plate at hold and opens the clamp, which PX4 keeps once the drone arms."""
        await self._settle(self._config.plate_hold_us, self._config.unclamped_us)

    async def stop(self) -> None:
        """Puts the plate at hold and opens the clamp, warning instead of raising if PX4 doesn't confirm it."""
        if not await self._set(self._config.plate_hold_us, self._config.unclamped_us):
            log.warning("payload rest position not confirmed while stopping")

    async def release(self, slot: str) -> None:
        """Drops the payload in the given slot and returns the plate and clamp to rest."""
        if slot not in self._config.drops:
            raise ValueError(f"no payload slot {slot!r}")
        sequence = DropSequence(self._config, slot)
        self._sent = self._unconfirmed = 0
        stage = None
        started = time.monotonic()
        try:
            while True:
                elapsed = time.monotonic() - started
                plate, clamp, done = sequence.positions_at(elapsed)
                if done:
                    break
                if sequence.stage_at(elapsed) != stage:
                    stage = sequence.stage_at(elapsed)
                    log.info("%s: %s", slot, sequence.stages[stage][1])
                await self._set(plate, clamp)
                await asyncio.sleep(_TICK_S)
        except BaseException:
            log.warning("%s: drop stopped, putting the plate back at hold", slot)
            await self._set(self._config.plate_hold_us, self._config.unclamped_us)
            raise
        await self._settle(self._config.plate_hold_us, self._config.unclamped_us)
        log.info("%s: dropped, plate back at hold and clamp open", slot)
        log.debug("%s drop sent %d servo commands, %d not confirmed", slot, self._sent, self._unconfirmed)

    async def pick_up(self) -> None:
        """Picks up a payload, which this mechanism can't do."""
        raise NotImplementedError("the plate and clamp can't pick up a payload")

    async def _settle(self, plate_us: float, clamp_us: float) -> None:
        """Commands these positions until PX4 confirms them, raising if it never does."""
        for _ in range(_SETTLE_TRIES):
            if await self._set(plate_us, clamp_us):
                return
            await asyncio.sleep(_RETRY_S)
        raise RuntimeError("PX4 didn't confirm the payload servo positions")

    async def _set(self, plate_us: float, clamp_us: float) -> bool:
        """Commands both servos unless PX4 already confirmed these positions, and returns whether it has."""
        wanted = (plate_us, clamp_us)
        if wanted == self._confirmed:
            return True
        if time.monotonic() < self._retry_at:
            return False
        self._sent += 1
        try:
            await self._drone.set_actuator(self._config.plate.actuator_set, self._config.plate.value(plate_us))
            await self._drone.set_actuator(self._config.clamp.actuator_set, self._config.clamp.value(clamp_us))
        except ActionError as error:
            self._unconfirmed += 1
            if not self._failing:
                log.warning("payload servo commands aren't being confirmed (%s); retrying", error)
                self._failing = True
            self._confirmed = None
            self._retry_at = time.monotonic() + _RETRY_S
            return False
        if self._failing:
            log.info("payload servo commands are confirmed again")
            self._failing = False
        self._confirmed = wanted
        return True
