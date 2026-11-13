import asyncio
import time

import pytest

GPS_TIMEOUT = 60.0
FIX_3D = 3


async def first(stream):
    """The next message on a telemetry stream."""
    async for value in stream:
        return value


async def test_pixhawk_connects(telemetry):
    """Reads PX4's battery, attitude and state, with or without a GPS fix. Never arms."""
    battery = await asyncio.wait_for(first(telemetry.subscribe_battery()), 10.0)
    attitude = await asyncio.wait_for(first(telemetry.subscribe_attitude_euler()), 10.0)
    flight_mode = await asyncio.wait_for(first(telemetry.subscribe_flight_mode()), 10.0)
    armed = await asyncio.wait_for(first(telemetry.subscribe_armed()), 10.0)
    charge = f"{battery.remaining_percent:.0f}%" if battery.remaining_percent >= 0 else "unknown charge"
    print(f"battery {charge} at {battery.voltage_v:.1f} V")
    print(f"roll {attitude.roll_deg:.1f}, pitch {attitude.pitch_deg:.1f}, yaw {attitude.yaw_deg:.0f} deg")
    print(f"flight mode {flight_mode.name}, armed {armed}")


async def test_gps_fix(telemetry):
    """Waits for a 3D GPS fix and prints the satellite count as it searches."""
    deadline = time.monotonic() + GPS_TIMEOUT
    last = None
    async for info in telemetry.subscribe_gps_info():
        if (info.num_satellites, info.fix_type) != last:
            print(f"{info.num_satellites} satellites, fix {info.fix_type.name}")
            last = (info.num_satellites, info.fix_type)
        if info.fix_type.value >= FIX_3D:
            return
        if time.monotonic() > deadline:
            pytest.fail(f"no 3D fix within {GPS_TIMEOUT:.0f} s; 0 satellites means PX4 sees no GPS")
