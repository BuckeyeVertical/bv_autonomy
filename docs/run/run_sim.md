# Running a mission in the simulator

The SUAS 2026 mission flies the laps, scans the search area, delivers to each target the
operator approves, then returns home and builds the map.

## 1. Start the simulator

From `bv_bevy`, in two terminals, PX4 first:

```bash
./px4.sh --prebuilt suas_2026
```

```bash
cargo run --release -- suas_2026
```

Wait for PX4 to print `Ready for takeoff!`.

## 2. Run the mission

From `bv_autonomy`:

```bash
uv run python -m bv_autonomy.missions.suas2026
```

Type `y` when it asks `Start the mission? [y/N]`.

Open the dashboard at http://localhost:8765 to follow the flight and approve targets.

## Stopping and resuming

Press Ctrl-C to stop. The drone holds where it is.

Pick up where it stopped:

```bash
uv run python -m bv_autonomy.missions.suas2026 --resume
```

Without `--resume`, the mission starts over and moves the last run's outputs to
`output/suas2026/old/`.

## Outputs

Logs, frames and the stitched map are saved in `output/suas2026/`.

On a Mac, PX4 accepts one connection per simulator start. Restart `./px4.sh` before
running the mission again.
