import asyncio
from dataclasses import dataclass

from bv_autonomy import paths
from bv_autonomy.components.drone import MavsdkDrone
from bv_autonomy.components.gcs import BvGcs
from bv_autonomy.missions import base
from bv_autonomy.missions.base import Mission
from bv_autonomy.types import Waypoint, WaypointSequence

MIN_BATTERY = 0.5

# Speeds, m/s
LAP_SPEED = 20.0
TRANSIT_SPEED = 20.0
SCAN_SPEED = 3.0
DELIVER_SPEED = 3.0
RETURN_SPEED = 20.0

LAP_COUNT = 2
LAP_TOLERANCE = 40.0


@dataclass(frozen=True)
class Site:
    """Where the mission flies: the lap route and the four corners of the search area."""

    laps: WaypointSequence
    scan_area: WaypointSequence


@dataclass
class MissionState(base.MissionState):
    """What's left of the mission. A new flight starts from new(), a resumed one from load()."""

    path = paths.SUAS2026_STATE

    lap_route: WaypointSequence

    @classmethod
    def new(cls, site: Site) -> "MissionState":
        return cls(lap_route=WaypointSequence(site.laps * LAP_COUNT + [site.laps[0]]))

    @classmethod
    def from_dict(cls, data: dict) -> "MissionState":
        return cls(lap_route=WaypointSequence(Waypoint(**p) for p in data["lap_route"]))


class Suas2026(Mission):
    """SUAS 2026: fly laps, then scan the search area, delivering a payload to each target found."""

    def __init__(self, site: Site, state: MissionState):
        self.site = site
        self.state = state
        self.drone = MavsdkDrone()
        self.gcs = BvGcs(drone=self.drone, class_ids={"person": 0, "tent": 1})

    async def run(self):
        await self.takeoff(self.site.laps[0].alt)
        await self.laps(self.state.lap_route, LAP_SPEED, LAP_TOLERANCE)
        await self.scan(self.site.scan_area, TRANSIT_SPEED, SCAN_SPEED)

    async def on_abort(self):
        """Stop where the drone is, so the operator can take over or the mission can resume from here."""
        self.gcs.report("mission_state", {"state": "aborted"})
        if self.drone.in_air():
            await self.drone.hold()

    async def connect(self, resuming: bool = False):
        """Connect to the drone and start the GCS, then refuse to start unless it's safe to."""
        await self.drone.connect()
        await self.gcs.start()
        self.preflight_checks(resuming)

    def preflight_checks(self, resuming: bool = False):
        """Raise if the drone isn't ready to start the mission. A resumed mission may start in the air."""
        if not resuming:
            if self.drone.in_air():
                raise RuntimeError("drone is already flying; not starting the mission")
            if self.drone.is_armed():
                raise RuntimeError("drone is already armed; disarm before starting")
        if self.drone.battery() < MIN_BATTERY:
            raise RuntimeError(f"battery at {self.drone.battery():.0%}; need at least {MIN_BATTERY:.0%}")

    async def takeoff(self, alt: float):
        """Arm and climb to alt meters above the ground, unless already flying."""
        if self.drone.in_air():
            return
        self.gcs.report("mission_state", {"state": "takeoff"})
        await self.drone.arm()
        await self.drone.takeoff(alt)

    async def laps(self, route: WaypointSequence, speed: float, tolerance: float):
        """Fly what's left of the lap route as one mission, or stop where it is if the operator ends laps."""
        if not route:
            return
        self.gcs.report("mission_state", {"state": "lap"})
        end_laps = self.gcs.signal("end_laps")
        end_laps.clear()

        def record_progress(reached: int):
            self.gcs.report_progress("lap", route, reached)
            self.state.lap_route = WaypointSequence(route[reached:])
            self.state.save()

        flight = asyncio.create_task(self.drone.fly_mission(route, speed, tolerance, on_progress=record_progress))
        end_pushed = asyncio.create_task(end_laps.wait())
        try:
            done, _ = await asyncio.wait({flight, end_pushed}, return_when=asyncio.FIRST_COMPLETED)
        finally:
            flight.cancel()
            end_pushed.cancel()
            await asyncio.wait({flight, end_pushed})
        if flight in done:
            flight.result()
        else:
            self.state.lap_route = WaypointSequence()
            self.state.save()
            await self.drone.hold()

    async def scan(self, area: WaypointSequence, transit_speed: float, speed: float):
        """Fly the search area in an S pattern, handling each target found along the way."""
