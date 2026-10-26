import logging

from bv_autonomy.components.camera import BevyCamera, FakeCamera, Rx0Camera
from bv_autonomy.components.camera.fake_camera import BEVY_INTRINSICS
from bv_autonomy.components.camera.rx0_camera import RX0_INTRINSICS
from bv_autonomy.components.drone import MavsdkDrone
from bv_autonomy.components.payload import FakePayload
from bv_autonomy.components.platform import FakePlatform, PhysicalPlatform, SimPlatform


def test_sim_platform_flies_px4_sitl_with_bevys_camera():
    platform = SimPlatform()
    assert isinstance(platform.drone, MavsdkDrone)
    assert platform.drone._address == "udpout://127.0.0.1:14580"
    assert isinstance(platform.camera, BevyCamera) and platform.camera._drone is platform.drone
    assert platform.intrinsics is BEVY_INTRINSICS


def test_physical_platform_uses_the_rx0_and_warns_it_has_no_payload_driver(caplog):
    with caplog.at_level(logging.WARNING):
        platform = PhysicalPlatform("serial:///dev/ttyUSB0:57600")
    assert platform.drone._address == "serial:///dev/ttyUSB0:57600"
    assert isinstance(platform.camera, Rx0Camera) and platform.camera._drone is platform.drone
    assert platform.intrinsics is RX0_INTRINSICS
    assert isinstance(platform.payload, FakePayload)
    assert "no payload driver" in caplog.text


def test_fake_platform_shares_its_drone_with_the_camera(drone):
    platform = FakePlatform(drone)
    assert platform.drone is drone
    assert isinstance(platform.camera, FakeCamera) and platform.camera._drone is drone
