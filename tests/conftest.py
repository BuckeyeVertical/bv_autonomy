import asyncio
from contextlib import suppress

import numpy as np
import pytest
import pytest_asyncio

from bv_autonomy import paths
from bv_autonomy.components.camera import FakeCamera
from bv_autonomy.components.detector import FakeDetector
from bv_autonomy.components.drone import FakeDrone
from bv_autonomy.components.gcs import NullGcs
from bv_autonomy.components.stitcher import FakeStitcher
from bv_autonomy.missions import base
from bv_autonomy.missions.suas2026 import MissionState, Site, Suas2026
from bv_autonomy.types import Frame, Intrinsics, Pose, Waypoint, WaypointSequence


class RecordingGcs(NullGcs):
    def __init__(self):
        super().__init__()
        self.reports = []
        self.previews = []
        self.requests = []
        self.answer = None          # what ask() answers: True, False, or None for no answer
        self.changed = asyncio.Event()

    def report(self, event, data):
        self.reports.append((event, data.copy()))
        self.changed.set()

    def preview(self, frame):
        self.previews.append(frame)

    async def ask(self, request, timeout):
        self.requests.append((request, timeout))
        return self.answer

    @property
    def progress(self):
        return [data for event, data in self.reports if event == "path_progress"]

    async def wait_reached(self, reached):
        async with asyncio.timeout(2):
            while not any(p["completed"] >= reached for p in self.progress):
                self.changed.clear()
                await self.changed.wait()


class RecordingDrone(FakeDrone):
    def __init__(self, home):
        super().__init__(home, speedup=1000, battery_drain_per_s=0)
        self.takeoffs = []
        self.holds = 0
        self.reached = []
        self.flight_tasks = []
        self.gotos = []
        self.flights = []

    async def takeoff(self, alt):
        self.takeoffs.append(alt)
        await super().takeoff(alt)

    async def hold(self):
        self.holds += 1
        await super().hold()

    async def goto(self, point, speed=None, tolerance=1.0):
        self.gotos.append((point, speed, tolerance))
        await super().goto(point, speed, tolerance)

    async def fly_mission(self, route, speed=None, tolerance=1.0, on_progress=None):
        self.flight_tasks.append(asyncio.current_task())
        self.flights.append((speed, tolerance))

        def record(reached):
            if reached:
                self.reached.append(route[reached - 1])
            if on_progress:
                on_progress(reached)

        await super().fly_mission(route, speed, tolerance, record)


@pytest.fixture(autouse=True)
def isolate_paths(tmp_path, monkeypatch):
    state_path = tmp_path / "state" / "mission_state.json"
    monkeypatch.setattr(paths, "SUAS2026_STATE", state_path)
    monkeypatch.setattr(paths, "OUTPUT_DIR", tmp_path / "output")
    monkeypatch.setattr(base.MissionState, "path", state_path, raising=False)
    monkeypatch.setattr(MissionState, "path", state_path)


@pytest_asyncio.fixture(autouse=True)
async def clean_background_tasks():
    before = asyncio.all_tasks()
    yield
    # The fakes have no disconnect API; also clean up children left by cancelled laps.
    pending = [task for task in asyncio.all_tasks() - before
               if task is not asyncio.current_task() and not task.done()]
    for task in pending:
        task.cancel()
    for task in pending:
        with suppress(asyncio.CancelledError):
            await task


@pytest.fixture
def site():
    return Site(
        laps=WaypointSequence([
            Waypoint(36.0, -96.0, 10),
            Waypoint(36.001, -96.0, 10),
            Waypoint(36.001, -95.999, 10),
        ]),
        scan_area=WaypointSequence.at_alt(0.0, [
            (36.0002, -96.0020),
            (36.0002, -96.0002),
            (36.0012, -96.0002),
            (36.0012, -96.0020),
        ]),
    )


@pytest.fixture
def gcs():
    return RecordingGcs()


@pytest_asyncio.fixture
async def drone(site):
    drone = RecordingDrone(site.laps[0])
    await drone.connect()
    return drone


@pytest.fixture
def targets():
    """What the fake camera and detector see; empty unless a test adds some."""
    return []


@pytest.fixture
def make_mission(site, drone, gcs, targets, tmp_path, monkeypatch):
    # Replace constructors before __init__, so no real GCS, MAVSDK, camera or model is used.
    from bv_autonomy.missions.suas2026 import suas2026

    small = Intrinsics(width=100, height=100, fx=100, fy=100, cx=50, cy=50, dist=np.zeros(5))
    monkeypatch.setattr(suas2026, "MavsdkDrone", lambda: drone)
    monkeypatch.setattr(suas2026, "BvGcs", lambda **kwargs: gcs)
    monkeypatch.setattr(suas2026, "BevyCamera", lambda drone: FakeCamera(drone, targets, small, fps=50))
    monkeypatch.setattr(suas2026, "LtdetrDetector", lambda **kwargs: FakeDetector(targets))
    monkeypatch.setattr(suas2026, "OpencvStitcher", lambda **kwargs: FakeStitcher(tmp_path / "map", keep_every=1))

    def make(state=None):
        return Suas2026(site, state if state is not None else MissionState.new(site))

    return make


@pytest.fixture
def frame():
    return Frame(
        image=np.zeros((100, 100, 3), dtype=np.uint8),
        time=12.0,
        pose=Pose(Waypoint(0, 0, 20), 0, 0, 0, 12.0),
        intrinsics=Intrinsics(width=100, height=100, fx=100, fy=100, cx=50, cy=50, dist=np.zeros(5)),
        sequence=1,
    )
