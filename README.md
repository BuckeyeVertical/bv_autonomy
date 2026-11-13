# bv_autonomy

## Setup

Make a folder for the team's repos and clone all three into it:

```bash
mkdir BuckeyeVertical
cd BuckeyeVertical
git clone https://github.com/BuckeyeVertical/bv_autonomy.git
git clone https://github.com/BuckeyeVertical/bv_bevy.git
git clone https://github.com/BuckeyeVertical/bv_gcs.git
```

### bv_autonomy

```bash
cd bv_autonomy
uv venv --python 3.10
uv pip install -e ".[opencv,ltdetr,localizer,mavsdk,stitcher,gcs,terrain]"
```

### bv_bevy

The simulator: PX4 and the 3D world the drone's camera sees. Follow the setup in
[bv_bevy's README](https://github.com/BuckeyeVertical/bv_bevy), then run the forest world
from the `bv_bevy` folder, in two terminals, PX4 first:

```bash
./px4.sh --prebuilt forest
```

```bash
cargo run -- forest
```

### bv_gcs

The operator's dashboard. See [bv_gcs's README](https://github.com/BuckeyeVertical/bv_gcs),
then build its frontend once:

```bash
cd bv_gcs/web
npm ci
npm run build
```

## Hardware checks

Before a flight, check the Pixhawk, camera and detector on the competition drone, or pass
`--platform x650` or `--platform sim` for the simulator. Each check prints what it found and
shows the camera frame or detections in a window, also saved under `output/`:

```bash
pytest hardware_tests -s
pytest hardware_tests/test_camera.py -s
pytest hardware_tests/test_detector.py --image photo.jpg -s
```
