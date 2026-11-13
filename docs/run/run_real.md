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

In `bv_autonomy/missions/suas2026/__main__.py`, change the platform line from
`SimPlatform()` to the aircraft you're flying, and import it from
`bv_autonomy.components.platform`:

```python
platform = CompDronePlatform()
```

or

```python
platform = X650Platform()
```

The mission also flies `SIM` from `sites.py`. Make sure that site is the field you're
flying at.

## 4. Run the mission

On the Jetson:

```bash
.venv/bin/python -m bv_autonomy.missions.suas2026
```

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
