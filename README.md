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
uv venv --python 3.12
uv pip install -e ".[opencv,ltdetr,localizer,mavsdk,stitcher,gcs]"
```

Put the LT-DETR weights (`ltdetr.pt`) in `bv_autonomy/models/`. The file is too big for
git, so get it from the team drive.

Keep this folder layout. The code finds the dashboard (`bv_gcs/web/dist`) and the weights
from where `bv_autonomy` is, so no paths need changing, and run output goes in
`bv_autonomy/output/`.

### bv_bevy

The simulator: PX4 and the 3D world the drone's camera sees. Follow the setup in
[bv_bevy's README](https://github.com/BuckeyeVertical/bv_bevy), then run the forest world
from the `bv_bevy` folder, in two terminals, PX4 first:

```bash
./px4.sh forest
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