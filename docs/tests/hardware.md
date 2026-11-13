# Hardware tests

Checks of the real components, in `hardware_tests/`. Run them by hand before a flight; CI
and plain `pytest` never run them.

```bash
uv run pytest hardware_tests -s
```

Run one check by naming its file:

| File | Checks | Needs |
| --- | --- | --- |
| `test_pixhawk.py` | Connects and prints position, home, heading, battery and armed state. Never arms. | Pixhawk |
| `test_camera.py` | Grabs a frame, checks its size and frame rate, and shows it. | Camera |
| `test_detector.py` | Runs the mission's detector on one camera frame and shows the detections. | Camera, model weights |

## Options

- `--platform comp_drone|x650|sim`: which aircraft, or bv_bevy. Defaults to `comp_drone`.
- `--image photo.jpg`: run the detector on an image instead of the camera.
- `--no-show`: don't open a window.
- `-s`: print what each check found, even when it passes.

Images are saved to `output/<time>_hardware_tests/`. Without a display, such as over SSH,
they're only saved.
