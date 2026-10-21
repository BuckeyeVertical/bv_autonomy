"""Stitcher that builds an orthophoto with OpenDroneMap in Docker, as bv_core's scripts/run_odm.sh did."""

import asyncio
import logging
import shutil
from pathlib import Path

from bv_autonomy import paths
from bv_autonomy.components.stitcher.base import Stitcher
from bv_autonomy.components.stitcher.exif import CameraExif
from bv_autonomy.components.stitcher.frame_recorder import FrameRecorder
from bv_autonomy.components.terrain import Terrain
from bv_autonomy.types import Frame

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
        """
        Args:
            camera: Written into each frame's EXIF; ODM uses it to model the lens.
            output_dir: ODM's project directory goes in output_dir/project. Defaults to a new run folder in output/.
            overlap: Fraction of each frame shared with the next one along a row.
            terrain: Makes frame altitudes above sea level. Without it they're above the ground,
                which ODM accepts; only the orthophoto's elevations are then relative to the ground.
            orthophoto_resolution_cm: Ground size of one orthophoto pixel (bv_core: 2.2).
        """
        output_dir = Path(output_dir) if output_dir is not None else paths.new_run_dir("odm")
        self._project = output_dir / "project"
        self._recorder = FrameRecorder(self._project / "images", overlap, camera, terrain)
        self._resolution = orthophoto_resolution_cm
        self._image = image

    def add(self, frame: Frame) -> None:
        self._recorder.add(frame)

    async def stitch(self) -> Path:
        await asyncio.to_thread(self._recorder.flush)
        if not self._recorder.rows():
            raise RuntimeError("no frames to stitch")
        return await run_odm(self._project, self._resolution, self._image)


async def run_odm(project: Path, orthophoto_resolution_cm: float = 2.2, image: str = "opendronemap/odm") -> Path:
    """Run ODM on project/images and return the orthophoto PNG. Flags match bv_core's run_odm.sh."""
    project = Path(project).resolve()
    if shutil.which("docker") is None:
        raise RuntimeError("Docker is needed to run ODM but wasn't found")
    # ODM treats /datasets/<name> as the project, so the project is mounted as "code".
    process = await asyncio.create_subprocess_exec(
        "docker", "run", "--rm",
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
    async for line in process.stdout:
        text = line.decode(errors="replace").rstrip()
        log.debug("odm: %s", text)
        tail = (tail + [text])[-20:]
    if await process.wait() != 0:
        raise RuntimeError(f"ODM failed (exit {process.returncode}):\n" + "\n".join(tail))

    for candidate in (project / "odm_orthophoto" / "odm_orthophoto.png", project / "opensfm" / "stats" / "ortho.png"):
        if candidate.is_file():
            return candidate
    raise RuntimeError("ODM finished but wrote no orthophoto PNG")
