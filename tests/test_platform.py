import pytest

from bv_autonomy.components.camera import BevyCamera, FakeCamera, Rx0Camera
from bv_autonomy.components.camera.fake_camera import BEVY_INTRINSICS
from bv_autonomy.components.camera.rx0_camera import RX0_INTRINSICS
from bv_autonomy.components.drone import MavsdkDrone
from bv_autonomy.components.payload import WinchPayload
from bv_autonomy.components.platform import FakePlatform, PhysicalPlatform, SimPlatform


def test_sim_platform_flies_px4_sitl_with_bevys_camera():
    platform = SimPlatform()
    assert isinstance(platform.drone, MavsdkDrone)
    assert platform.drone._address == "udpin://0.0.0.0:14540"
    assert isinstance(platform.camera, BevyCamera) and platform.camera._drone is platform.drone
    assert platform.intrinsics is BEVY_INTRINSICS


def test_physical_platform_uses_the_rx0_and_the_plate_and_clamp_payload():
    platform = PhysicalPlatform("serial:///dev/ttyUSB0:57600")
    assert platform.drone._address == "serial:///dev/ttyUSB0:57600"
    assert isinstance(platform.camera, Rx0Camera) and platform.camera._drone is platform.drone
    assert platform.intrinsics is RX0_INTRINSICS
    assert isinstance(platform.payload, WinchPayload) and platform.payload._drone is platform.drone


def test_fake_platform_shares_its_drone_with_the_camera(drone):
    platform = FakePlatform(drone)
    assert platform.drone is drone
    assert isinstance(platform.camera, FakeCamera) and platform.camera._drone is drone


def test_a_given_sim_address_is_used_as_is():
    assert SimPlatform("udpin://0.0.0.0:14999").drone._address == "udpin://0.0.0.0:14999"


async def test_stopping_a_platform_stops_the_camera_rests_the_payload_and_disconnects_the_drone(drone):
    platform = FakePlatform(drone)
    await platform.camera.start()
    await platform.stop()
    assert platform.camera._task is None
    assert [action for _, action, _ in platform.payload.log] == ["stop"]
    assert drone._task is None


async def test_starting_a_platform_connects_the_drone_before_the_payload_and_camera(drone, monkeypatch):
    platform = FakePlatform(drone)
    order = []
    connect, start_payload, start_camera = drone.connect, platform.payload.start, platform.camera.start

    async def record(name, start):
        order.append(name)
        await start()

    monkeypatch.setattr(drone, "connect", lambda: record("drone", connect))
    monkeypatch.setattr(platform.payload, "start", lambda: record("payload", start_payload))
    monkeypatch.setattr(platform.camera, "start", lambda: record("camera", start_camera))
    await platform.start()
    assert order == ["drone", "payload", "camera"]
    await platform.stop()


async def test_a_failing_camera_stop_still_rests_the_payload_and_disconnects_the_drone(drone, monkeypatch):
    platform = FakePlatform(drone)
    await platform.start()
    error = RuntimeError("camera thread stuck")

    async def broken_stop():
        raise error

    monkeypatch.setattr(platform.camera, "stop", broken_stop)
    with pytest.raises(RuntimeError) as raised:
        await platform.stop()
    assert raised.value is error
    assert [action for _, action, _ in platform.payload.log][-1] == "stop"
    assert drone._task is None
