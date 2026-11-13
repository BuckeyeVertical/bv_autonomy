#!/usr/bin/env bash
set -euo pipefail

TORCH_INDEX="https://pypi.jetson-ai-lab.io/jp6/cu126"
EXTRAS="dev,ltdetr,localizer,mavsdk,stitcher,gcs,terrain"
SYSTEM_CV2="/usr/lib/python3.10/dist-packages/cv2"

cd "$(dirname "$0")/.."
UV="$(command -v uv || echo "$HOME/.local/bin/uv")"
PY=".venv/bin/python"
SITE=".venv/lib/python3.10/site-packages"

step() { printf '\n== %s\n' "$1"; }
fail() { printf 'ERROR: %s\n' "$1" >&2; exit 1; }

[ -x "$UV" ] || fail "uv not found; install it with: curl -LsSf https://astral.sh/uv/install.sh | sh"
grep -q "^# R36" /etc/nv_tegra_release 2>/dev/null || fail "this is for JetPack 6 (L4T R36); check /etc/nv_tegra_release and the torch index"
[ -d "$SYSTEM_CV2" ] || fail "NVIDIA's OpenCV is missing at $SYSTEM_CV2; install JetPack's nvidia-opencv"

step "venv on the system Python, with its packages"
if [ -d .venv ]; then
    grep -q "include-system-site-packages = true" .venv/pyvenv.cfg || fail ".venv exists without system site packages; remove it and rerun"
else
    "$UV" venv --python /usr/bin/python3 --system-site-packages
fi

step "NVIDIA's torch"
"$UV" pip install --python "$PY" torch torchvision --index-url "$TORCH_INDEX"
"$UV" pip freeze --python "$PY" | grep -E '^(torch|torchvision)==' > .venv/constraints.txt
echo "numpy<2" >> .venv/constraints.txt

step "bv_autonomy and its extras"
[ ! -L "$SITE/cv2" ] || rm "$SITE/cv2"
"$UV" pip install --python "$PY" -e ".[$EXTRAS]" -c .venv/constraints.txt

step "NVIDIA's OpenCV in place of pip's"
for package in opencv-python-headless opencv-python opencv-contrib-python; do
    if "$UV" pip show --python "$PY" "$package" >/dev/null 2>&1; then
        "$UV" pip uninstall --python "$PY" "$package"
    fi
done
[ ! -e "$SITE/cv2" ] || [ -L "$SITE/cv2" ] || fail "$SITE/cv2 is still a real folder after removing pip's OpenCV"
ln -sfn "$SYSTEM_CV2" "$SITE/cv2"

step "cuDSS for torch"
"$UV" pip install --python "$PY" --no-deps "nvidia-cudss-cu12<1"
CUDSS="$(find "$SITE/nvidia" -name 'libcudss.so.0' | head -1)"
[ -n "$CUDSS" ] || fail "libcudss.so.0 not found after installing nvidia-cudss-cu12"
ln -sf "$(realpath "$CUDSS")" "$SITE/torch/lib/libcudss.so.0"

step "uv run without syncing"
grep -q "UV_NO_SYNC=1" ~/.bashrc 2>/dev/null || echo "export UV_NO_SYNC=1" >> ~/.bashrc

step "checks"
"$PY" -c "import torch; assert torch.cuda.is_available(), 'torch has no CUDA'; print('torch', torch.__version__, 'on', torch.cuda.get_device_name(0))"
"$PY" -c "import cv2; assert 'GStreamer: YES' in ' '.join(cv2.getBuildInformation().split()), 'OpenCV has no GStreamer'; print('OpenCV', cv2.__version__, 'with GStreamer')"
"$PY" -c "import numpy; assert numpy.__version__ < '2', 'numpy 2 breaks the Jetson builds'; print('numpy', numpy.__version__)"
"$PY" -c "import bv_autonomy, lightly_train, mavsdk, aiohttp, piexif, rasterio, geographiclib, scipy; print('extras import')"

step "still to do by hand"
id -nG | grep -qw dialout || echo "- add yourself to dialout for the Pixhawk: sudo usermod -aG dialout \$USER, then log out and in"
"$PY" - <<'EOF'
from bv_autonomy import paths
for path in (paths.LTDETR_WEIGHTS, paths.SUAS2026_DEM, paths.GCS_WEB_DIST):
    if not path.exists():
        print(f"- copy {path} from a laptop (see docs/setup/jetson.md)")
EOF
echo "Done."
