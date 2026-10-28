# Jetson setup

For a Jetson Orin Nano on JetPack 6. The venv uses the system Python 3.10 so it keeps
JetPack's OpenCV, which can read the camera through GStreamer.

## 1. Install uv and clone

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
mkdir ~/BuckeyeVertical
cd ~/BuckeyeVertical
git clone https://github.com/BuckeyeVertical/bv_autonomy.git
cd bv_autonomy
```

## 2. Make the venv

```bash
uv venv --python /usr/bin/python3 --system-site-packages
source .venv/bin/activate
```

## 3. Install NVIDIA's torch

Use the index for your JetPack version (`cat /etc/nv_tegra_release`). This one is JetPack 6.1 and 6.2:

```bash
uv pip install torch torchvision --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
uv pip freeze | grep -E '^(torch|torchvision)==' > .venv/constraints.txt
echo "numpy<2" >> .venv/constraints.txt
```

## 4. Install bv_autonomy

Leave out the `opencv` extra:

```bash
uv pip install -e ".[ltdetr,localizer,mavsdk,stitcher,gcs,terrain]" -c .venv/constraints.txt
```

## 5. Check

```bash
python -c "import torch; print(torch.cuda.is_available())"
python -c "import cv2; print('GStreamer: YES' in cv2.getBuildInformation())"
```

Both should print True.

## 6. Serial ports

```bash
sudo usermod -aG dialout $USER
```

Log out and back in.

## 7. Copy files from a laptop

The weights, terrain maps and dashboard aren't in git. From the laptop's `BuckeyeVertical` folder:

```bash
scp bv_autonomy/models/ltdetr.pt jetson:~/BuckeyeVertical/bv_autonomy/models/
scp bv_autonomy/terrain_maps/*.tif jetson:~/BuckeyeVertical/bv_autonomy/terrain_maps/
ssh jetson mkdir -p BuckeyeVertical/bv_gcs/web
scp -r bv_gcs/web/dist jetson:~/BuckeyeVertical/bv_gcs/web/
```

Build the dashboard on the laptop first (see the README).
