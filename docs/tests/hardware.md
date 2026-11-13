# Hardware tests

Checks of the real components, in `tests/hardware/`. Run them by hand before a flight; CI
and plain `pytest` never run them.

```bash
.venv/bin/python -m pytest tests/hardware -s
```

Run one check by naming its file:

| File | Checks | Needs |
| --- | --- | --- |
| `test_pixhawk_hardware.py` | Connects and prints battery, attitude and flight mode, then waits up to 60 s for a 3D GPS fix. Never arms. | Pixhawk, GPS outdoors |
| `test_camera_hardware.py` | Grabs a frame, checks its size and frame rate, and shows it. | Camera |
| `test_detector_hardware.py` | Runs the mission's detector on one camera frame and shows the detections. | Camera, model weights |

## Options

- `--platform comp_drone|x650|sim`: which aircraft, or bv_bevy. Defaults to `comp_drone`.
- `--image photo.jpg`: run the detector on an image instead of the camera.
- `--no-show`: don't open a window.
- `-s`: print what each check found, even when it passes.

Images are saved to `output/hardware_tests/`, and the previous run's are moved to
`output/hardware_tests/old/`. Without a display, such as over SSH, they're only saved.
