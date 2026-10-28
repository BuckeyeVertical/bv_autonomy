# bv_autonomy

## Setup

Make a folder for the team's repos and clone all three into it:

```bash
mkdir BuckeyeVertical
cd BuckeyeVertical
git clone https://github.com/BuckeyeVertical/bv_autonomy.git
git clone https://github.com/BuckeyeVertical/bv_gcs.git
```

### bv_autonomy

```bash
cd bv_autonomy
uv venv --python 3.10
uv pip install -e ".[opencv,ltdetr,localizer,mavsdk,stitcher,gcs,terrain]"
```

### bv_sim

##### download sim executable depending on your os:
#### mac/linux:
```bash
curl -fsSL https://raw.githubusercontent.com/BuckeyeVertical/bv_sim/main/install.sh | bash
```
#### windows:
```bash
irm https://raw.githubusercontent.com/BuckeyeVertical/bv_sim/main/install.ps1 | iex
```

run sim from a new terminal with
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
