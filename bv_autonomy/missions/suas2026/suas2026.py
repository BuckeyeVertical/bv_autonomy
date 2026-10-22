import asyncio
from dataclasses import dataclass

from bv_autonomy.components.drone import MavsdkDrone
from bv_autonomy.components.gcs import BvGcs
from bv_autonomy.missions.base import Mission
from bv_autonomy.types import WaypointSequence

MIN_BATTERY = 0.9

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


class Suas2026(Mission):
    """SUAS 2026: fly laps, then scan the search area, delivering a payload to each target found."""

    def __init__(self, site: Site):
        self.site = site
        self.drone = MavsdkDrone()
        self.gcs = BvGcs(drone=self.drone, class_ids={"person": 0, "tent": 1})

    async def run(self):
        await self.takeoff(self.site.laps[0].alt)
        await self.laps(self.site.laps, LAP_SPEED, LAP_COUNT, LAP_TOLERANCE)
        await self.scan(self.site.scan_area, TRANSIT_SPEED, SCAN_SPEED)

    async def connect(self):
        """Connect to the drone and start the GCS, then refuse to start unless it's safe to."""
        await self.drone.connect()
        await self.gcs.start()
        self.preflight_checks()

    def preflight_checks(self):
        """Raise if the drone isn't ready to start the mission from the ground."""
        if self.drone.in_air():
            raise RuntimeError("drone is already flying; not starting the mission")
        if self.drone.is_armed():
            raise RuntimeError("drone is already armed; disarm before starting")
        if self.drone.battery() < MIN_BATTERY:
            raise RuntimeError(f"battery at {self.drone.battery():.0%}; need at least {MIN_BATTERY:.0%}")

    async def takeoff(self, alt: float):
        """Arm and climb to alt meters above the ground."""
        self.gcs.report("mission_state", {"state": "takeoff"})
        await self.drone.arm()
        await self.drone.takeoff(alt)

    async def laps(self, waypoints: WaypointSequence, speed: float, count: int, tolerance: float):
        """Fly the lap route count times as one mission, or stop where it is if the operator ends laps."""
        self.gcs.report("mission_state", {"state": "lap"})
        end_laps = self.gcs.signal("end_laps")
        end_laps.clear()
        route = WaypointSequence(waypoints * count + [waypoints[0]])
        flight = asyncio.create_task(self.drone.fly_mission(
            route, speed, tolerance, on_progress=lambda reached: self.gcs.report_progress("lap", route, reached)))
        end_pushed = asyncio.create_task(end_laps.wait())
        done, _ = await asyncio.wait({flight, end_pushed}, return_when=asyncio.FIRST_COMPLETED)
        if flight in done:
            end_pushed.cancel()
            flight.result()
        else:
            flight.cancel()
            await self.drone.hold()

    async def scan(self, area: WaypointSequence, transit_speed: float, speed: float):
        """Fly the search area in an S pattern, handling each target found along the way."""
