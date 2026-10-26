from bv_autonomy.components.camera import BevyCamera, FakeCamera, Rx0Camera
from bv_autonomy.components.camera.fake_camera import BEVY_INTRINSICS
from bv_autonomy.components.camera.rx0_camera import RX0_INTRINSICS
from bv_autonomy.components.drone import MavsdkDrone
from bv_autonomy.components.payload import WinchPayload
from bv_autonomy.components.platform import FakePlatform, PhysicalPlatform, SimPlatform, sim_platform


def test_sim_platform_flies_px4_sitl_with_bevys_camera():
    platform = SimPlatform()
    assert isinstance(platform.drone, MavsdkDrone)
    assert platform.drone._address == sim_platform.default_connection()
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


def test_the_sim_link_listens_for_px4_on_windows_and_calls_it_on_the_mac(monkeypatch):
    monkeypatch.setattr(sim_platform.sys, "platform", "win32")
    assert sim_platform.default_connection() == "udpin://0.0.0.0:14540"
    monkeypatch.setattr(sim_platform.sys, "platform", "darwin")
    assert sim_platform.default_connection() == "udpout://127.0.0.1:14580"


def test_a_given_sim_address_is_used_as_is():
    assert SimPlatform("udpin://0.0.0.0:14999").drone._address == "udpin://0.0.0.0:14999"


async def test_stopping_a_platform_stops_the_camera_rests_the_payload_and_disconnects_the_drone(drone):
    platform = FakePlatform(drone)
    await platform.camera.start()
    await platform.stop()
    assert platform.camera._task is None
    assert [action for _, action, _ in platform.payload.log] == ["stop"]
    assert drone._task is None


