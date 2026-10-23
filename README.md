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
