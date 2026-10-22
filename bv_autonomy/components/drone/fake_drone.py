"""Drone that only exists in memory, so missions run on a laptop in seconds."""

import asyncio
import math
import time
from collections.abc import Callable

from bv_autonomy.components.drone.base import Drone
from bv_autonomy.types import Pose, Waypoint, WaypointSequence

_EARTH_R = 6378137.0
_TICK_S = 0.02


class FakeDrone(Drone):
    """Moves in straight lines toward whatever it was last told, at a fixed speed.

    Physics is skipped: no wind, no acceleration, attitude stays level. speedup makes
    simulated time run faster than real time, so a long mission finishes quickly.
    """

    def __init__(self, home: Waypoint, speed: float = 5.0, climb_rate: float = 2.0,
                 speedup: float = 1.0, battery_drain_per_s: float = 0.0005):
        self._home = Waypoint(home.lat, home.lon, 0.0)
        self._position = Waypoint(home.lat, home.lon, 0.0)
        self._yaw = 0.0
        self._default_speed = speed
        self._climb_rate = climb_rate
        self._speedup = speedup
        self._drain = battery_drain_per_s
        self._battery = 1.0
        self._armed = False
        self._target: Waypoint | None = None
        self._speed = speed
        self._task: asyncio.Task | None = None

    async def connect(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._simulate())

    async def arm(self) -> None:
        self._armed = True

    async def disarm(self) -> None:
        if self.in_air():
            raise RuntimeError("can't disarm in the air")
        self._armed = False

    async def takeoff(self, alt: float) -> None:
        if not self._armed:
            raise RuntimeError("can't take off while disarmed")
        p = self._position
        await self._fly_to(Waypoint(p.lat, p.lon, alt), self._default_speed, 0.1)

    async def goto(self, point: Waypoint, speed: float | None = None, tolerance: float = 1.0) -> None:
        if not self.in_air():
            raise RuntimeError("can't goto from the ground")
        await self._fly_to(point, speed or self._default_speed, tolerance)

    async def fly_mission(
        self,
        route: WaypointSequence,
        speed: float | None = None,
        tolerance: float = 1.0,
        on_progress: Callable[[int], None] | None = None,
    ) -> None:
        if not self.in_air():
            raise RuntimeError("can't fly a mission from the ground")
        if on_progress:
            on_progress(0)
        for reached, point in enumerate(route, start=1):
            await self._fly_to(point, speed or self._default_speed, tolerance)
            if self._target is not point:
                return               # told to hold partway through
            if on_progress:
                on_progress(reached)

    async def hold(self) -> None:
        self._target = None

    async def land(self) -> None:
        p = self._position
        await self._fly_to(Waypoint(p.lat, p.lon, 0.0), self._default_speed, 0.0)
        self._armed = False                 # PX4 disarms itself after landing

    async def return_home(self) -> None:
        cruise = max(self._position.alt, 1.0)
        await self._fly_to(Waypoint(self._home.lat, self._home.lon, cruise), self._default_speed, 0.1)
        await self.land()

    def pose(self) -> Pose:
        return Pose(self.position(), 0.0, 0.0, self._yaw, time.monotonic())

    def position(self) -> Waypoint:
        p = self._position
        return Waypoint(p.lat, p.lon, p.alt)

    def heading(self) -> float:
        return math.degrees(self._yaw) % 360.0

    def home(self) -> Waypoint | None:
        return Waypoint(self._home.lat, self._home.lon, 0.0)

    def battery(self) -> float:
        return self._battery

    def is_armed(self) -> bool:
        return self._armed

    def in_air(self) -> bool:
        return self._position.alt > 0.05

    async def _fly_to(self, point: Waypoint, speed: float, tolerance: float) -> None:
        self._target, self._speed = point, speed
        while _offset_m(self._position, point)[3] > tolerance:
            if self._target is not point:
                return                       # redirected or told to hold
            await asyncio.sleep(_TICK_S)

    async def _simulate(self) -> None:
        last = time.monotonic()
        while True:
            await asyncio.sleep(_TICK_S)
            now = time.monotonic()
            dt, last = (now - last) * self._speedup, now
            if self._armed:
                self._battery = max(0.0, self._battery - self._drain * dt)
            target = self._target
            if target is None or not self._armed:
                continue
            east, north, up, _ = _offset_m(self._position, target)
            horizontal = math.hypot(east, north)
            step = min(1.0, self._speed * dt / horizontal) if horizontal > 1e-6 else 1.0
            climb = max(-self._climb_rate * dt, min(self._climb_rate * dt, up))
            if horizontal > 0.5:
                self._yaw = math.atan2(east, north)  # faces the direction of travel
            p = self._position
            self._position = Waypoint(
                p.lat + math.degrees(north * step / _EARTH_R),
                p.lon + math.degrees(east * step / (_EARTH_R * math.cos(math.radians(p.lat)))),
                max(0.0, p.alt + climb),
            )


def _offset_m(a: Waypoint, b: Waypoint) -> tuple[float, float, float, float]:
    """East, north and up from a to b, and the straight-line distance, in meters."""
    east = _EARTH_R * math.radians(b.lon - a.lon) * math.cos(math.radians(a.lat))
    north = _EARTH_R * math.radians(b.lat - a.lat)
    up = b.alt - a.alt
    return east, north, up, math.sqrt(east * east + north * north + up * up)
