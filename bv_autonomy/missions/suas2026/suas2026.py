import asyncio
import logging
from dataclasses import dataclass, field

from bv_autonomy import paths
from bv_autonomy.crop import annotated_crop
from bv_autonomy.components.detector import LtdetrDetector
from bv_autonomy.components.gcs import BvGcs
from bv_autonomy.components.localizer import GimbalNadirLocalizer
from bv_autonomy.components.path_planner import SPathPlanner
from bv_autonomy.components.platform import Platform
from bv_autonomy.components.stitcher import OpencvStitcher
from bv_autonomy.missions import base
from bv_autonomy.missions.base import Mission
from bv_autonomy.missions.suas2026.confirmation import ConfirmationWindow, Sighting, distance_m, wanted
from bv_autonomy.types import ApprovalRequest, Detection, Intrinsics, Waypoint, WaypointSequence

log = logging.getLogger(__name__)

MIN_BATTERY = 0.5

LAP_ALT = 60.96
SCAN_ALT = 60.96

SCAN_OVERLAP = 0.35
SCAN_TOLERANCE = 1.0
DELIVER_TOLERANCE = 1.0

LAP_SPEED = 20.0
TRANSIT_SPEED = 20.0
SCAN_SPEED = 3.0
DELIVER_SPEED = 3.0
RETURN_SPEED = 20.0

CLASS_IDS = {"person": 0, "tent": 1}
PAYLOAD_SLOTS = {"person": "bottle", "tent": "beacon"}
DETECTION_THRESHOLDS = {"person": 0.5, "tent": 0.7}

CONFIRM_HITS = 2
CONFIRM_FRAMES = 5
CONFIRM_RADIUS_M = 11.0

LOCATE_TRIES = 5
LOCATE_RETRY_S = 1.0
APPROVAL_TIMEOUT_S = 60.0

LAP_COUNT = 2
LAP_TOLERANCE = 40.0


@dataclass(frozen=True)
class Site:
    """Where the mission flies: the lap route and the four corners of the search area."""

    laps: WaypointSequence
    scan_area: WaypointSequence


@dataclass
class MissionState(base.MissionState):
    """State of the mission at any time"""

    path = paths.SUAS2026_STATE

    lap_route: WaypointSequence
    scan_route: WaypointSequence
    targets: list[Sighting] = field(default_factory=list)
    delivered: list[Sighting] = field(default_factory=list)
    rejected: list[Sighting] = field(default_factory=list)

    @classmethod
    def new(cls, site: Site, intrinsics: Intrinsics) -> "MissionState":
        """Creates the state for a new flight, with the full lap route and a scan planned for the given
        camera.
        """

        planner = SPathPlanner(intrinsics, SCAN_ALT, SCAN_OVERLAP)
        return cls(
            lap_route=WaypointSequence(site.laps * LAP_COUNT + [site.laps[0]]),
            scan_route=planner.plan(site.scan_area),
        )

    @classmethod
    def from_dict(cls, data: dict) -> "MissionState":
        return cls(
            lap_route=WaypointSequence(Waypoint(**p) for p in data["lap_route"]),
            scan_route=WaypointSequence(Waypoint(**p) for p in data["scan_route"]),
            targets=[Sighting(s["label"], Waypoint(**s["position"])) for s in data["targets"]],
            delivered=[Sighting(s["label"], Waypoint(**s["position"])) for s in data["delivered"]],
            rejected=[Sighting(s["label"], Waypoint(**s["position"])) for s in data["rejected"]],
        )


class Suas2026(Mission):
    """SUAS 2026: fly laps, then scan the search area, delivering a payload to each target found."""

    def __init__(self, site: Site, state: MissionState, platform: Platform):
        self.site = site
        self.state = state
        self.drone = platform.drone
        self.camera = platform.camera
        self.payload = platform.payload
        self.detector = LtdetrDetector(thresholds=DETECTION_THRESHOLDS)
        self.localizer = GimbalNadirLocalizer()
        self.stitcher = OpencvStitcher(overlap=SCAN_OVERLAP)
        self.gcs = BvGcs(drone=self.drone, class_ids=CLASS_IDS)
        self.window = ConfirmationWindow(CLASS_IDS, CONFIRM_HITS, CONFIRM_FRAMES, CONFIRM_RADIUS_M)

    async def run(self):
        await self.takeoff(self.site.laps[0].alt)
        await self.laps(self.state.lap_route, LAP_SPEED, LAP_TOLERANCE)
        await self.scan(self.state.scan_route, TRANSIT_SPEED, SCAN_SPEED, SCAN_TOLERANCE)
        await self.return_home()

    async def on_abort(self):
        """Holds the drone where it is so the operator can take over or the mission can resume."""
        self.gcs.report("mission_state", {"state": "aborted"})
        if self.drone.in_air():
            await self.drone.hold()

    async def connect(self, resuming: bool = False):
        """Connects to the drone, starts the camera, detector and GCS, and checks that it's safe to start."""
        await self.drone.connect()
        await self.camera.start()
        await self.detector.start()
        await self.gcs.start()
        self.preflight_checks(resuming)

    def preflight_checks(self, resuming: bool = False):
        """Raises if the drone isn't ready to start the mission."""
        if not resuming:
            if self.drone.in_air():
                raise RuntimeError("drone is already flying; not starting the mission")
            if self.drone.is_armed():
                raise RuntimeError("drone is already armed; disarm before starting")
        if self.drone.battery() < MIN_BATTERY:
            raise RuntimeError(f"battery at {self.drone.battery():.0%}; need at least {MIN_BATTERY:.0%}")

    async def takeoff(self, alt: float):
        """Arms and takes off to the given altitude."""
        if self.drone.in_air():
            return
        self.gcs.report("mission_state", {"state": "takeoff"})
        await self.drone.arm()
        await self.drone.takeoff(alt)

    async def laps(self, route: WaypointSequence, speed: float, tolerance: float):
        """Flies the lap route, stopping early if the operator ends laps."""
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

    async def scan(self, route: WaypointSequence, transit_speed: float, speed: float, tolerance: float):
        """Flies the scan route, checking each target found and delivering to the approved ones."""
        await self.deliver_approved()
        while route:
            target = await self.scan_rows(route, transit_speed, speed, tolerance)
            if target is None:
                return
            await self.check_target(target)
            await self.deliver_approved()
            route = self.state.scan_route

    async def scan_rows(self, route: WaypointSequence, transit_speed: float, speed: float,
                        tolerance: float) -> Sighting | None:
        """Flies the scan rows while watching the camera, and returns a target if one is found."""
        self.gcs.report("mission_state", {"state": "scan"})
        await self.drone.goto(route[0], transit_speed, tolerance)

        def record_progress(reached: int):
            self.gcs.report_progress("scan", route, reached)
            self.state.scan_route = WaypointSequence(route[reached:])
            self.state.save()

        flight = asyncio.create_task(self.drone.fly_mission(route, speed, tolerance, on_progress=record_progress))
        watching = asyncio.create_task(self.watch())
        try:
            done, _ = await asyncio.wait({flight, watching}, return_when=asyncio.FIRST_COMPLETED)
        finally:
            flight.cancel()
            watching.cancel()
            await asyncio.wait({flight, watching})
        if watching in done:
            return watching.result()
        flight.result()
        return None

    async def watch(self) -> Sighting:
        """Sends each camera frame to the dashboard and the map, and returns the first target confirmed."""
        self.window.clear()
        while True:
            frame = await self.camera.latest(timeout=1.0)
            if frame is None:
                log.warning("no camera frame for 1 s")
                continue
            self.gcs.preview(frame)
            self.stitcher.add(frame)
            sightings = [Sighting(d.label, p) for d in await self.detector.detect(frame)
                         if (p := self.localizer.locate(d)) is not None]
            found = self.state.targets + self.state.delivered
            wanted_now = [s for s in sightings if wanted(s, found, self.state.rejected, CONFIRM_RADIUS_M)]
            confirmed = self.window.add(wanted_now)
            self.gcs.report("confirm_window", self.window.summary({t.label for t in found}))
            if confirmed:
                log.info("confirmed %s at %.6f, %.6f", confirmed.label, confirmed.position.lat, confirmed.position.lon)
                return confirmed

    async def check_target(self, target: Sighting):
        """Stops over a target, finds it again, and asks the operator whether it's real."""
        self.gcs.report("mission_state", {"state": "localize"})
        await self.drone.hold()
        found = await self.locate_again(target)
        if found is None:
            log.warning("lost the %s after holding; resuming the scan", target.label)
            return
        detection, position = found
        try:
            crop = annotated_crop(detection)
        except Exception:
            log.exception("could not crop the %s; asking without an image", target.label)
            crop = b""
        request = ApprovalRequest(target.label, position, detection.confidence, crop, self.drone.position())
        answer = await self.gcs.ask(request, APPROVAL_TIMEOUT_S)
        if answer is False:
            log.info("operator rejected the %s at %.6f, %.6f", target.label, position.lat, position.lon)
            self.state.rejected += [target, Sighting(target.label, position)]
        else:
            log.info("%s the %s at %.6f, %.6f", "operator approved" if answer else "no answer; approving",
                     target.label, position.lat, position.lon)
            self.state.targets.append(Sighting(target.label, position))
            if self.state.scan_route:
                here = self.drone.position()
                self.state.scan_route = WaypointSequence([Waypoint(here.lat, here.lon, SCAN_ALT), *self.state.scan_route])
        self.state.save()

    async def deliver_approved(self):
        """Delivers a payload to each approved target."""
        for target in list(self.state.targets):
            self.gcs.report("mission_state", {"state": "deliver"})
            above = Waypoint(target.position.lat, target.position.lon, SCAN_ALT)
            await self.drone.goto(above, DELIVER_SPEED, DELIVER_TOLERANCE)
            await self.drone.hold()
            await self.payload.release(PAYLOAD_SLOTS[target.label])
            log.info("delivered the %s's %s", target.label, PAYLOAD_SLOTS[target.label])
            self.state.targets.remove(target)
            self.state.delivered.append(target)
            self.state.save()

    async def return_home(self):
        """Flies back to the home position and lands there, building the map from the scan on the way."""
        self.gcs.report("mission_state", {"state": "return"})
        await self.detector.stop()
        stitching = asyncio.create_task(self.stitcher.stitch())
        try:
            await self.drone.return_home()
        except BaseException:
            stitching.cancel()
            raise
        try:
            log.info("map saved to %s", await stitching)
        except Exception:
            log.exception("could not build the map")

    async def locate_again(self, target: Sighting) -> tuple[Detection, Waypoint] | None:
        """Finds the target again in fresh frames and returns its detection and position, or None if it isn't
        seen.
        """
        for _ in range(LOCATE_TRIES):
            frame = await self.camera.latest(timeout=1.0)
            if frame is not None:
                matches = [(d, p) for d in await self.detector.detect(frame)
                           if d.label == target.label and (p := self.localizer.locate(d)) is not None
                           and distance_m(p, target.position) <= CONFIRM_RADIUS_M]
                if matches:
                    return min(matches, key=lambda match: distance_m(match[1], target.position))
            await asyncio.sleep(LOCATE_RETRY_S)
        return None
