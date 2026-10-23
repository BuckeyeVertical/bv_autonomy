import asyncio
import math

import numpy as np
import pytest

from bv_autonomy import paths
from bv_autonomy.components.camera.fake_camera import FakeCamera
from bv_autonomy.components.detector.fake_detector import FakeDetector
from bv_autonomy.components.payload.fake_payload import FakePayload
from bv_autonomy.components.stitcher.fake_stitcher import FakeStitcher
from bv_autonomy.components.terrain.base import Terrain
from bv_autonomy.fake_world import Target, project
from bv_autonomy.types import Waypoint


def test_terrain_adds_local_ground_elevation():
    class SlopedTerrain(Terrain):
        def ground_msl(self, lat, lon):
            return 200 + lat + 2 * lon

    assert SlopedTerrain().msl(Waypoint(3, 4, 15)) == 226


def test_project_center_target_has_expected_box(frame):
    target = Target("person", Waypoint(0, 0, 0), size_m=2)
    assert project(target, frame.pose, frame.intrinsics, 100, 100) == pytest.approx((45, 45, 55, 55))


@pytest.mark.parametrize("yaw,expected", [(0, (45, 20, 55, 30)), (math.pi / 2, (20, 45, 30, 55))])
def test_project_rotates_ground_offsets_with_heading(frame, yaw, expected):
    frame.pose.yaw = yaw
    target = Target("person", Waypoint(math.degrees(5 / 6378137), 0, 0), size_m=2)
    assert project(target, frame.pose, frame.intrinsics, 100, 100) == pytest.approx(expected)


def test_project_clips_to_image_edges(frame):
    target = Target("person", Waypoint(0, math.degrees(10 / 6378137), 0), size_m=2)
    assert project(target, frame.pose, frame.intrinsics, 100, 100) == pytest.approx((95, 45, 100, 55))


def test_project_rejects_target_outside_image(frame):
    target = Target("person", Waypoint(1, 1, 0))
    assert project(target, frame.pose, frame.intrinsics, 100, 100) is None


def test_project_rejects_camera_on_ground(frame):
    frame.pose.position.alt = 0
    assert project(Target("person", Waypoint(0, 0, 0)), frame.pose, frame.intrinsics, 100, 100) is None


async def test_fake_detector_requires_start_and_stop(frame):
    detector = FakeDetector([])
    with pytest.raises(RuntimeError, match="before start"):
        await detector.detect(frame)
    await detector.start()
    assert await detector.detect(frame) == []
    await detector.stop()
    with pytest.raises(RuntimeError, match="before start"):
        await detector.detect(frame)


async def test_fake_detector_returns_visible_target_and_source_frame(frame):
    detector = FakeDetector([Target("person", Waypoint(0, 0, 0)), Target("tent", Waypoint(1, 1, 0))])
    await detector.start()
    found = await detector.detect(frame)
    assert len(found) == 1
    assert found[0].label == "person"
    assert found[0].confidence == 0.9
    assert found[0].frame is frame
    assert found[0].box == pytest.approx((46.25, 46.25, 53.75, 53.75))
    await detector.stop()


async def test_fake_detector_seed_repeats_misses_and_noise(frame):
    target = Target("person", Waypoint(0, 0, 0))
    detectors = [FakeDetector([target], miss_rate=0.5, box_noise_px=2, seed=17) for _ in range(2)]
    results = []
    for detector in detectors:
        await detector.start()
        results.append([[d.box for d in await detector.detect(frame)] for _ in range(20)])
        await detector.stop()
    assert results[0] == results[1]
    assert any(results[0]) and not all(results[0])


async def test_fake_camera_produces_fresh_frames_and_stamps_pose(drone, frame):
    await drone.arm()
    await asyncio.wait_for(drone.takeoff(20), 2)
    camera = FakeCamera(drone, [Target("person", drone.home(), color_bgr=(1, 2, 3))],
                        width=100, height=100, intrinsics=frame.intrinsics, fps=200)
    await camera.start()
    try:
        first = await camera.latest(timeout=1)
        second = await camera.latest(timeout=1)
        assert second.sequence > first.sequence
        assert first.pose.position == drone.position()
        assert first.intrinsics is frame.intrinsics
        assert first.time >= first.pose.time
        assert tuple(first.image[50, 50]) == (1, 2, 3)
    finally:
        await camera.stop()


async def test_fake_camera_timeout_and_stop(frame, drone):
    camera = FakeCamera(drone, width=100, height=100, intrinsics=frame.intrinsics)
    assert await camera.latest(timeout=0.001) is None
    await camera.start()
    await camera.latest(timeout=1)
    await camera.stop()
    await camera.stop()
    assert await camera.latest(timeout=0.001) is None


async def test_fake_payload_release_counts_and_logs():
    payload = FakePayload({"bottle": 1}, action_s=0)
    await payload.release("bottle")
    assert payload.held == {"bottle": 0}
    assert [entry[1:] for entry in payload.log] == [("release", "bottle")]
    for slot in ("bottle", "missing"):
        with pytest.raises(RuntimeError, match="nothing left"):
            await payload.release(slot)


async def test_fake_payload_pickup_requires_capability():
    with pytest.raises(NotImplementedError):
        await FakePayload(action_s=0).pick_up()


async def test_fake_payload_pickup_adds_main_slot_and_logs():
    payload = FakePayload(can_pick_up=True, action_s=0)
    await payload.pick_up()
    assert payload.held["main"] == 1
    assert payload.log[0][1:] == ("pick_up", "main")


async def test_fake_stitcher_rejects_no_frames(tmp_path):
    with pytest.raises(RuntimeError, match="no frames"):
        await FakeStitcher(output_dir=tmp_path).stitch()


async def test_fake_stitcher_samples_copies_and_writes_rgb_grid(tmp_path, frame):
    stitcher = FakeStitcher(output_dir=tmp_path, keep_every=2, thumb_width=50, columns=2)
    frame.image[:] = (10, 20, 30)
    stitcher.add(frame)
    frame.image[:] = (40, 50, 60)
    stitcher.add(frame)
    frame.image[:] = (70, 80, 90)
    stitcher.add(frame)
    frame.image[:] = 0
    output = await stitcher.stitch()
    header, pixels = output.read_bytes().split(b"\n", 1)
    assert output.parent == tmp_path
    assert header == b"P6 100 50 255"
    image = np.frombuffer(pixels, dtype=np.uint8).reshape(50, 100, 3)
    assert tuple(image[0, 0]) == (30, 20, 10)
    assert tuple(image[0, 50]) == (90, 80, 70)


def test_new_run_dir_stays_under_configured_output(tmp_path):
    path = paths.new_run_dir("test")
    assert path.parent == tmp_path / "output"
    assert path.name.endswith("_test")
    assert not path.exists()
