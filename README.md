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

### bv_sim
macOS (Apple Silicon) or Linux (x86_64, Ubuntu 24.04 or compatible), in Terminal:
```bash
curl -fsSL https://raw.githubusercontent.com/BuckeyeVertical/bv_sim/main/install.sh | bash
```
Windows (x86_64), in PowerShell:
```bash
irm https://raw.githubusercontent.com/BuckeyeVertical/bv_sim/main/install.ps1 | iex
```
Open a new terminal and run:
```bash
bv-sim run suas_2026
```
### bv_gcs

The operator's dashboard. See [bv_gcs's README](https://github.com/BuckeyeVertical/bv_gcs),
then build its frontend once:

```bash
cd bv_gcs/web
npm ci
npm run build
```

With `bv-sim run suas_2026` running, start the mission from `bv_autonomy`:

```bash
.venv/bin/python -m bv_autonomy.missions.suas2026
```

Open `http://localhost:8765` and click **Start stream** for the onboard camera.
The `gcs` extra includes PyAV for the dashboard's H.264/MP4 preview and recording;
existing environments should run `uv pip install -e ".[gcs]"` to add it.
Preview encoding runs only while requested and viewed, at up to 15 fps and
640×480 (preserving aspect ratio). **Stop stream** or closing the requesting
dashboard releases the encoder. The scan phase is still unimplemented.
