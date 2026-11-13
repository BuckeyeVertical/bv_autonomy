# Running a mission on the aircraft

## 1. Set up the Jetson

Once, following [Jetson setup](../setup/jetson.md).

## 2. Check the hardware

On the Jetson, with the Pixhawk and camera plugged in:

```bash
.venv/bin/python -m pytest tests/hardware -s
```

For the X650:

```bash
.venv/bin/python -m pytest tests/hardware -s --platform x650
```

See [hardware tests](../tests/hardware.md) and [telemetry echo](../tests/echo.md).

## 3. Switch main to the aircraft

In `bv_autonomy/missions/suas2026/__main__.py`, change the platform
```python
platform = CompDronePlatform()
```

or

```python
platform = X650Platform()
```

## 4. Change the site

In the same file, change the site to one from `sites.py`
```python
site = sites.GLACIER
```

## 5. Run the mission

On the Jetson:

```bash
.venv/bin/python -m bv_autonomy.missions.suas2026
```

Once the preflight checks pass, it asks `Start the mission? [y/N]`. Type `y` to arm and take off;
anything else exits without arming.

Open the dashboard at `http://<jetson-ip>:8765` to follow the flight and approve targets.

## Stopping and resuming

Press Ctrl-C to stop. The drone holds where it is.

Pick up where it stopped:

```bash
.venv/bin/python -m bv_autonomy.missions.suas2026 --resume
```

Without `--resume`, the mission starts over and moves the last run's outputs to
`output/suas2026/old/`.

## Outputs

Logs, frames and the stitched map are saved in `output/suas2026/` on the Jetson.
