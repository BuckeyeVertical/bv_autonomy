Keep this file general and useful: only what agents need to write, test and debug code in this repo, not one-off details, history or anything already clear from the code.

# Layout

- `components/<name>/`: a component's abstract in `base.py`, with its implementations and
  fake next to it.
- `missions/<competition>/`: a competition's mission. Competition logic lives here.
- `types.py`: data shared between components and missions.
- `paths.py`: every file location (dashboard, model weights, output). Use it; never
  hard-code a path.

# Testing

Run Python with the repo's venv (`.venv/bin/python`, or `uv run`).

Tests: install `.[dev,gcs,mavsdk,localizer,opencv,stitcher]`, then run `.venv/bin/python -m pytest`.

Every component has a fake next to its real implementations (`FakeDrone`, `FakeCamera`,
`FakeDetector`, `FakePayload`, `FakeStitcher`, `NullGcs`). Use them to run mission logic on
a laptop in seconds before going to the simulator.

## Simulator

The simulator is bv_bevy, cloned next to this repo in `../bv_bevy`. Run its commands from
that folder:

@../bv_bevy/AGENTS.md

# Adding an implementation

Only when a component has to behave differently: new hardware, sim instead of real, or a
different method. Different values (weights, thresholds, a camera's pipeline and
calibration) are constructor arguments or a thin subclass like `Rx0Camera`. Competition
logic goes in the mission, not a component.

Put it next to `base.py` named after what it is (`bevy_camera.py`). Make heavy
dependencies an optional extra in `pyproject.toml`, imported lazily in the folder's
`__init__.py`. If the abstract changes, update the fake.
