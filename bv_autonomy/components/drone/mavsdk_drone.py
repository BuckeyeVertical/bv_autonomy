"""Drone flown through MAVSDK (the native v4 binding), for PX4 in SITL or on the real aircraft."""

import asyncio
import contextlib
import math
import time

from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync, ActionError
from mavsdk.asyncio.plugins.mission import MissionAsync, MissionItem, MissionPlan
from mavsdk.asyncio.plugins.telemetry import TelemetryAsync
from mavsdk.plugins.telemetry import FlightMode

from bv_autonomy.components.drone.base import Drone
from bv_autonomy.types import GeoPoint, Pose

_EARTH_R = 6378137.0                # WGS-84 semi-major axis (meters)
_ACCEPT_TIMEOUT_S = 5.0             # how long one attempt waits for PX4 to show a command took effect
_RETRIES = 3


class MavsdkDrone(Drone):
    """PX4 over MAVSDK. Altitudes are above home, which is above the ground on a flat field.

    goto flies a one-waypoint PX4 mission with relative altitude, as bv_core flew its routes.
    Every command is confirmed by telemetry, not by its acknowledgment alone, and is resent if
    PX4 doesn't show it took effect: acknowledgments get lost, which stalled bv_core's arming.

    Cancelling a command (e.g. cancelling run() mid-goto) does not stop the aircraft; PX4 keeps
    flying what it was told. Call hold(), land() or return_home() to stop it.
    """

    def __init__(self, address: str = "udpin://0.0.0.0:14540"):
        """
        Args:
            address: MAVSDK connection URL. The default listens where PX4 sends to API programs,
                which bv_bevy's SITL does; on the aircraft it's e.g. "serial:///dev/ttyTHS1:921600".
        """
        self._address = address
        self._mavsdk: Mavsdk | None = None
        self._action: ActionAsync | None = None
        self._mission: MissionAsync | None = None
        self._telemetry: TelemetryAsync | None = None
        self._tasks: list[asyncio.Task] = []

        # Latest telemetry, filled in by the background tasks.
        self._position = None
        self._position_time = 0.0
        self._attitude = None
        self._home = None
        self._battery = None
        self._armed = False
        self._in_air = False
        self._flight_mode = FlightMode.UNKNOWN
        self._health = None
        self._mission_progress = None
        self._velocity = None

    async def connect(self, timeout: float = 30.0) -> None:
        """Connect, then wait until PX4 has a good position fix, a home position and telemetry."""
        self._mavsdk = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
        await self._mavsdk.add_any_connection(self._address)
        system = await self._mavsdk.first_autopilot(timeout_s=timeout)
        if system is None:
            raise RuntimeError(f"no autopilot found at {self._address} within {timeout:.0f} s")
        self._action = ActionAsync(system)
        self._mission = MissionAsync(system)
        self._telemetry = TelemetryAsync(system)
        t = self._telemetry
        self._tasks = [
            asyncio.create_task(self._watch(t.subscribe_position(), self._on_position)),
            asyncio.create_task(self._watch(t.subscribe_attitude_euler(), self._set("_attitude"))),
            asyncio.create_task(self._watch(t.subscribe_velocity_ned(), self._set("_velocity"))),
            asyncio.create_task(self._watch(t.subscribe_home(), self._set("_home"))),
            asyncio.create_task(self._watch(t.subscribe_battery(), self._set("_battery"))),
            asyncio.create_task(self._watch(t.subscribe_armed(), self._set("_armed"))),
            asyncio.create_task(self._watch(t.subscribe_in_air(), self._set("_in_air"))),
            asyncio.create_task(self._watch(t.subscribe_flight_mode(), self._set("_flight_mode"))),
            asyncio.create_task(self._watch(t.subscribe_health(), self._set("_health"))),
            asyncio.create_task(self._watch(self._mission.subscribe_mission_progress(), self._set("_mission_progress"))),
        ]
        await self._wait(
            lambda: self._position is not None and self._attitude is not None and self._home is not None
            and self._battery is not None and self._velocity is not None and self._health is not None
            and self._health.is_global_position_ok and self._health.is_home_position_ok,
            timeout, "a position fix, home position and telemetry",
        )
        # After a goto's one-waypoint mission, stay there rather than flying home.
        await self._mission.set_return_to_launch_after_mission(False)

    async def arm(self) -> None:
        await self._command(self._action.arm, lambda: self._armed, "arm")

    async def disarm(self) -> None:
        await self._command(self._action.disarm, lambda: not self._armed, "disarm")

    async def takeoff(self, alt: float) -> None:
        await self._action.set_takeoff_altitude(alt)
        await self._command(
            self._action.takeoff, lambda: self._in_air or self._flight_mode == FlightMode.TAKEOFF, "take off"
        )
        # PX4 levels off just under the takeoff altitude.
        await self._wait(lambda: self._position.relative_altitude_m >= alt - 0.5, alt / 0.5 + 30, f"reaching {alt} m")

    async def goto(self, point: GeoPoint, speed: float | None = None, tolerance: float = 1.0) -> None:
        item = MissionItem(
            latitude_deg=point.lat,
            longitude_deg=point.lon,
            relative_altitude_m=point.alt,
            speed_m_s=float("nan") if speed is None else speed,
            is_fly_through=False,
            gimbal_pitch_deg=float("nan"),
            gimbal_yaw_deg=float("nan"),
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=float("nan"),
            camera_photo_interval_s=float("nan"),
            acceptance_radius_m=tolerance,
            yaw_deg=float("nan"),                # PX4 faces the direction of travel
            camera_photo_distance_m=float("nan"),
            vehicle_action=MissionItem.VehicleAction.NONE,
        )

        async def start():
            await self._mission.upload_mission(MissionPlan([item]))
            await self._mission.start_mission()

        def started():
            # Mission mode alone isn't enough: PX4 stays in it after the previous goto finishes.
            # The new waypoint is active until reached, unless the drone was already there.
            progress = self._mission_progress
            return self._flight_mode == FlightMode.MISSION and (
                (progress is not None and progress.current < progress.total)
                or _distance_m(self.position(), point) <= tolerance
            )

        await self._command(start, started, "start the goto mission", always_send=True)
        distance = _distance_m(self.position(), point)
        await self._wait(
            lambda: _distance_m(self.position(), point) <= tolerance,
            distance / (speed or 1.0) + 60, f"reaching {point}",
        )

    async def hold(self) -> None:
        """Switch to Hold and finish once the drone has braked to a stop."""
        # Holding also ends a goto's mission. (PX4 keeps flying a DO_REPOSITION after a switch to
        # Hold, which is one reason goto uses missions.)
        await self._command(self._action.hold, lambda: self._flight_mode == FlightMode.HOLD, "hold")
        await self._wait(lambda: self._ground_speed() < 0.3, 15, "the drone to stop")

    async def land(self) -> None:
        await self._command(
            self._action.land, lambda: self._flight_mode == FlightMode.LAND or not self._in_air, "land"
        )
        await self._wait(lambda: not self._in_air, self.position().alt / 0.5 + 30, "landing")

    async def return_home(self) -> None:
        await self._command(
            self._action.return_to_launch,
            lambda: self._flight_mode == FlightMode.RETURN_TO_LAUNCH or not self._in_air,
            "return home",
        )
        await self._wait(lambda: not self._in_air, 600, "returning home and landing")

    def pose(self) -> Pose:
        a = self._attitude
        return Pose(
            position=self.position(),
            roll=math.radians(a.roll_deg),
            pitch=math.radians(a.pitch_deg),
            yaw=math.radians(a.yaw_deg),
            time=self._position_time,
        )

    def position(self) -> GeoPoint:
        p = self._position
        return GeoPoint(p.latitude_deg, p.longitude_deg, p.relative_altitude_m)

    def heading(self) -> float:
        return self._attitude.yaw_deg % 360.0

    def home(self) -> GeoPoint | None:
        if self._home is None:
            return None
        return GeoPoint(self._home.latitude_deg, self._home.longitude_deg, 0.0)

    def battery(self) -> float:
        return self._battery.remaining_percent / 100.0

    def is_armed(self) -> bool:
        return self._armed

    def in_air(self) -> bool:
        return self._in_air

    async def _command(self, send, accepted, name: str, always_send: bool = False) -> None:
        """Send until telemetry shows it took effect, whether or not PX4's acknowledgment arrives.

        A command whose effect is already showing isn't sent, unless always_send (the first time).
        """
        error = None
        for attempt in range(_RETRIES):
            if accepted() and not (always_send and attempt == 0):
                return
            try:
                await send()
            except ActionError as exc:      # a lost acknowledgment shows up as a timeout here
                error = exc
            with contextlib.suppress(TimeoutError):
                await self._wait(accepted, _ACCEPT_TIMEOUT_S, name)
                return
        raise RuntimeError(f"PX4 didn't {name} after {_RETRIES} tries (last error: {error})")

    async def _wait(self, condition, timeout: float, what: str) -> None:
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                raise TimeoutError(f"timed out after {timeout:.0f} s waiting for {what}")
            await asyncio.sleep(0.05)

    async def _watch(self, stream, handle) -> None:
        async for value in stream:
            handle(value)

    def _ground_speed(self) -> float:
        return math.hypot(self._velocity.north_m_s, self._velocity.east_m_s)

    def _set(self, name: str):
        return lambda value: setattr(self, name, value)

    def _on_position(self, position) -> None:
        self._position = position
        self._position_time = time.monotonic()


def _distance_m(a: GeoPoint, b: GeoPoint) -> float:
    # Flat-earth approximation, accurate over a competition field.
    dx = _EARTH_R * math.radians(b.lon - a.lon) * math.cos(math.radians(a.lat))
    dy = _EARTH_R * math.radians(b.lat - a.lat)
    return math.sqrt(dx * dx + dy * dy + (b.alt - a.alt) ** 2)
