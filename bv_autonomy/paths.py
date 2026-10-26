"""Where things live, worked out from this repo's location so nobody edits paths in code.

Assumes the team layout from the README: bv_autonomy, bv_bevy and bv_gcs cloned side by
side in one BuckeyeVertical folder, with bv_autonomy installed editable (pip install -e).
"""

from datetime import datetime
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parent.parent
WORKSPACE_DIR = REPO_DIR.parent

GCS_WEB_DIST = WORKSPACE_DIR / "bv_gcs" / "web" / "dist"

MODELS_DIR = REPO_DIR / "models"
LTDETR_WEIGHTS = MODELS_DIR / "ltdetr.pt"

OUTPUT_DIR = REPO_DIR / "output"

SUAS2026_STATE = REPO_DIR / "bv_autonomy" / "missions" / "suas2026" / "state" / "mission_state.json"


def new_run_dir(name: str) -> Path:
    """Returns a new folder under output for one run."""
    return OUTPUT_DIR / f"{datetime.now():%Y%m%d_%H%M%S}_{name}"
