# Telemetry echo

Prints PX4's telemetry topics as they arrive, like `ros2 topic echo`. Stop it with Ctrl-C.

PX4 has to be running: the Pixhawk plugged in on the Jetson, or bv_bevy with `--platform sim`.

List the topics:

```bash
.venv/bin/python tests/echo/telemetry.py --list
```

Echo one topic:

```bash
.venv/bin/python tests/echo/telemetry.py gps_info
```

Echo several:

```bash
.venv/bin/python tests/echo/telemetry.py position battery
```

Echo the simulator:

```bash
.venv/bin/python tests/echo/telemetry.py gps_info --platform sim
```

- `--platform comp_drone|x650|sim`: which aircraft, or bv_bevy. Defaults to `comp_drone`.
- `--timeout 30`: seconds to wait for PX4.

## Common topics

- `gps_info`: satellites and fix type (3 is a 3D fix, 6 is RTK fixed)
- `position`: latitude, longitude and altitude
- `battery`: voltage and remaining charge
- `health`: whether GPS, home and sensors are ready to arm
- `flight_mode`: what mode PX4 is in
- `armed`: whether the motors are armed
- `status_text`: PX4's messages, such as why it won't arm
