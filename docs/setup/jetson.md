# Jetson setup

For a Jetson Orin Nano on JetPack 6. The venv uses the system Python 3.10 so it keeps
JetPack's OpenCV, which can read the camera through GStreamer.

Don't run `uv sync` here. It makes the venv match a `uv.lock`, which would have PyPI's
torch, OpenCV and numpy 2 instead of the Jetson builds, and removes every extra. `uv run`
would sync too, upgrading numpy to 2, so the setup script sets `UV_NO_SYNC=1` in
`~/.bashrc` to stop it and makes `uv sync` and `uv lock` refuse to run. Change packages with
`uv pip install`.

## 1. Install uv and clone

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
mkdir ~/BuckeyeVertical
cd ~/BuckeyeVertical
git clone https://github.com/BuckeyeVertical/bv_autonomy.git
cd bv_autonomy
```

## 2. Run the setup script

```bash
./scripts/setup_jetson.sh
```

It makes the venv, installs NVIDIA's torch and bv_autonomy with its extras, swaps pip's
OpenCV for NVIDIA's (pip's has no GStreamer), adds the cuDSS library NVIDIA's torch needs,
and checks that torch has CUDA and OpenCV has GStreamer. It ends by listing anything left
to do by hand.

Rerun it after any `uv pip install` into this venv. Installing the `ltdetr` extra brings
back pip's OpenCV, and reinstalling torch drops cuDSS. Rerunning is safe.

The torch index in the script is for JetPack 6.1 and 6.2 (`cat /etc/nv_tegra_release`).

## 3. Serial ports

```bash
sudo usermod -aG dialout $USER
```

Log out and back in.

## 4. Copy files from a laptop

The weights, terrain maps and dashboard aren't in git. From the laptop's `BuckeyeVertical` folder:

```bash
scp bv_autonomy/models/ltdetr.pt jetson:~/BuckeyeVertical/bv_autonomy/models/
scp bv_autonomy/terrain_maps/*.tif jetson:~/BuckeyeVertical/bv_autonomy/terrain_maps/
ssh jetson mkdir -p BuckeyeVertical/bv_gcs/web
scp -r bv_gcs/web/dist jetson:~/BuckeyeVertical/bv_gcs/web/
```

Build the dashboard on the laptop first (see the README).
