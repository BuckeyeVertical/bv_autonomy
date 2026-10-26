"""Stitcher that builds an orthophoto with OpenDroneMap in Docker, as bv_core's scripts/run_odm.sh did."""

import asyncio
import logging
import shutil
import uuid
from pathlib import Path

from bv_autonomy import paths
from bv_autonomy.components.stitcher.base import Stitcher
from bv_autonomy.components.stitcher.exif import CameraExif
from bv_autonomy.components.stitcher.frame_recorder import FrameRecorder
from bv_autonomy.components.terrain import Terrain
from bv_autonomy.types import Frame, StitchProgress

log = logging.getLogger(__name__)


class OdmStitcher(Stitcher):
    """Saves geotagged frames during the scan; stitch() runs ODM on them.

    ODM is slow and heavy, so bv_core ran it off the aircraft. stitch() runs Docker wherever
    it's called, so on the Jetson it would run there; to do it on a laptop instead, copy
    output_dir/project/images there and call run_odm() on the copy.
    """

    def __init__(
        self,
        camera: CameraExif,
        output_dir: str | Path | None = None,
        overlap: float = 0.35,
        terrain: Terrain | None = None,
        orthophoto_resolution_cm: float = 2.2,
        image: str = "opendronemap/odm",
    ):
        """Sets up a stitcher that saves geotagged frames for ODM to build the map from."""
        output_dir = Path(output_dir) if output_dir is not None else paths.new_run_dir("odm")
        self._project = output_dir / "project"
        self._recorder = FrameRecorder(self._project / "images", overlap, camera, terrain)
        self._resolution = orthophoto_resolution_cm
        self._image = image
        self._container = f"bv-odm-{uuid.uuid4().hex[:8]}"

    def add(self, frame: Frame) -> None:
        self._recorder.add(frame)

    async def stop(self) -> None:
        await asyncio.to_thread(self._recorder.close)
        await stop_container(self._container)

    def progress(self) -> StitchProgress | None:
        return self._recorder.progress()

    def resume(self, progress: StitchProgress) -> None:
        self._recorder.resume(progress)

    async def stitch(self) -> Path:
        await asyncio.to_thread(self._recorder.flush)
        if not self._recorder.rows():
            raise RuntimeError("no frames to stitch")
        return await run_odm(self._project, self._resolution, self._image, self._container)


async def run_odm(project: Path, orthophoto_resolution_cm: float = 2.2, image: str = "opendronemap/odm",
                  container: str | None = None) -> Path:
    """Runs ODM on a project's images and returns the path to the map it makes, stopping ODM if cancelled."""
    project = Path(project).resolve()
    if shutil.which("docker") is None:
        raise RuntimeError("Docker is needed to run ODM but wasn't found")
    container = container or f"bv-odm-{uuid.uuid4().hex[:8]}"
    process = await asyncio.create_subprocess_exec(
        "docker", "run", "--rm", "--name", container,
        "-v", f"{project}:/datasets/code",
        image,
        "--project-path", "/datasets",
        "--orthophoto-resolution", str(orthophoto_resolution_cm),
        "--orthophoto-png",
        "--rerun-all",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )
    tail: list[str] = []
    try:
        async for line in process.stdout:
            text = line.decode(errors="replace").rstrip()
            log.debug("odm: %s", text)
            tail = (tail + [text])[-20:]
        await process.wait()
    except BaseException:
        await stop_container(container)
        raise
    if process.returncode != 0:
        raise RuntimeError(f"ODM failed (exit {process.returncode}):\n" + "\n".join(tail))

    for candidate in (project / "odm_orthophoto" / "odm_orthophoto.png", project / "opensfm" / "stats" / "ortho.png"):
        if candidate.is_file():
            return candidate
    raise RuntimeError("ODM finished but wrote no orthophoto PNG")


async def stop_container(name: str) -> None:
    """Stops the named Docker container if it's running, which also ends the docker run that started it."""
    if shutil.which("docker") is None:
        return
    process = await asyncio.create_subprocess_exec(
        "docker", "stop", name, stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL
    )
    await process.wait()
