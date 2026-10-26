"""Where things live, worked out from this repo's location so nobody edits paths in code.

Assumes the team layout from the README: bv_autonomy, bv_bevy and bv_gcs cloned side by
side in one BuckeyeVertical folder, with bv_autonomy installed editable (pip install -e).
"""

import shutil
from datetime import datetime
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parent.parent
WORKSPACE_DIR = REPO_DIR.parent

GCS_WEB_DIST = WORKSPACE_DIR / "bv_gcs" / "web" / "dist"

MODELS_DIR = REPO_DIR / "models"
LTDETR_WEIGHTS = MODELS_DIR / "ltdetr.pt"

TERRAIN_MAPS_DIR = REPO_DIR / "terrain_maps"
SUAS2026_DEM = TERRAIN_MAPS_DIR / "suas2026.tif"

OUTPUT_DIR = REPO_DIR / "output"
SUAS2026_OUTPUT = OUTPUT_DIR / "suas2026"

SUAS2026_STATE = REPO_DIR / "bv_autonomy" / "missions" / "suas2026" / "state" / "mission_state.json"


def new_run_dir(name: str) -> Path:
    """Returns a new folder under output for one run."""
    return OUTPUT_DIR / f"{datetime.now():%Y%m%d_%H%M%S}_{name}"


def archive_outputs(directory: Path) -> None:
    """Moves a mission's outputs from its last run into its old folder, under the time they were moved."""
    last_run = [path for path in directory.iterdir() if path.name != "old"] if directory.exists() else []
    if not last_run:
        return
    archive = directory / "old" / f"{datetime.now():%Y%m%d_%H%M%S}"
    archive.mkdir(parents=True)
    for path in last_run:
        shutil.move(path, archive / path.name)
