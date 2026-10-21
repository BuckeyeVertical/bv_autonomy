"""Where things live, worked out from this repo's location so nobody edits paths in code.

Assumes the team layout from the README: bv_autonomy, bv_bevy and bv_gcs cloned side by
side in one BuckeyeVertical folder, with bv_autonomy installed editable (pip install -e).
"""

from datetime import datetime
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parent.parent       # BuckeyeVertical/bv_autonomy
WORKSPACE_DIR = REPO_DIR.parent                          # BuckeyeVertical

GCS_WEB_DIST = WORKSPACE_DIR / "bv_gcs" / "web" / "dist"  # built with npm run build in bv_gcs/web

MODELS_DIR = REPO_DIR / "models"                         # not in git; see the README
LTDETR_WEIGHTS = MODELS_DIR / "ltdetr.pt"

OUTPUT_DIR = REPO_DIR / "output"                         # not in git


def new_run_dir(name: str) -> Path:
    """A fresh folder under output/ for one run, e.g. output/20261021_142530_odm."""
    return OUTPUT_DIR / f"{datetime.now():%Y%m%d_%H%M%S}_{name}"
