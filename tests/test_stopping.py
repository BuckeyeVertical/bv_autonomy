import asyncio
import logging
import math
from dataclasses import replace

import pytest

from bv_autonomy.components.platform import stop_all
from bv_autonomy.components.stitcher import odm_stitcher
from bv_autonomy.components.stitcher.frame_recorder import FrameRecorder
from bv_autonomy.components.stitcher.opencv_stitcher import OpencvStitcher
from bv_autonomy.types import Waypoint


def moved(frame, east):
    return replace(frame, pose=replace(frame.pose, position=Waypoint(0, math.degrees(east / 6378137), 20)))


def test_closing_a_recorder_finishes_its_writes_and_ends_its_thread(tmp_path, frame):
    recorder = FrameRecorder(tmp_path / "frames", overlap=0.5)
    recorder.add(moved(frame, 0))
    recorder.add(moved(frame, 11))
    recorder.close()
    assert all(path.exists() for path in recorder.rows()[1])
    with pytest.raises(RuntimeError):
        recorder._writer.submit(print)


async def test_stopping_the_opencv_stitcher_writes_the_frames_it_kept(tmp_path, frame):
    stitcher = OpencvStitcher(output_dir=tmp_path)
    stitcher.add(moved(frame, 0))
    await stitcher.stop()
    assert list((tmp_path / "frames").glob("row1_1.jpg"))


class Docker:
    """Records docker commands, with docker run printing nothing until it's stopped."""

    def __init__(self):
        self.commands = []
        self.stopped = asyncio.Event()

    async def __call__(self, *command, **kwargs):
        self.commands.append(command)
        docker = self

        class Process:
            returncode = None

            def __init__(self):
                self.stdout = self.lines() if command[1] == "run" else None

            async def lines(self):
                await docker.stopped.wait()
                return
                yield

            async def wait(self):
                if command[1] == "stop":
                    docker.stopped.set()
                    self.returncode = 0
                return self.returncode

        return Process()

    def stops(self):
        return [command[2] for command in self.commands if command[1] == "stop"]


@pytest.fixture
def docker(monkeypatch):
    fake = Docker()
    monkeypatch.setattr(odm_stitcher.shutil, "which", lambda name: "/usr/bin/docker")
    monkeypatch.setattr(odm_stitcher.asyncio, "create_subprocess_exec", fake)
    return fake


async def test_a_cancelled_odm_run_stops_its_container(docker, tmp_path):
    run = asyncio.create_task(odm_stitcher.run_odm(tmp_path, container="bv-odm-test"))
    await asyncio.sleep(0.01)
    run.cancel()
    with pytest.raises(asyncio.CancelledError):
        await run
    assert docker.commands[0][:5] == ("docker", "run", "--rm", "--name", "bv-odm-test")
    assert docker.stops() == ["bv-odm-test"]


async def test_stopping_the_odm_stitcher_stops_its_named_container(docker, tmp_path):
    from bv_autonomy.components.stitcher.exif import CameraExif

    stitcher = odm_stitcher.OdmStitcher(CameraExif.from_calibration("Sony", "RX0", 800, 1000, 1600, 900, 12),
                                        output_dir=tmp_path)
    await stitcher.stop()
    assert docker.stops() == [stitcher._container]


async def test_stop_all_runs_every_stop_in_order_and_raises_the_first_failure(caplog):
    ran = []
    first, second = RuntimeError("first"), RuntimeError("second")

    def step(name, error=None):
        async def stop():
            ran.append(name)
            if error:
                raise error
        return stop

    with pytest.raises(RuntimeError) as raised:
        await stop_all(step("a"), step("b", first), step("c", second), step("d"))
    assert ran == ["a", "b", "c", "d"]
    assert raised.value is first
    assert sum(r.levelno == logging.ERROR and r.exc_info is not None for r in caplog.records) == 2
