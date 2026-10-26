from bv_autonomy.components.camera import FakeCamera
from bv_autonomy.components.camera.fake_camera import BEVY_INTRINSICS
from bv_autonomy.components.drone import FakeDrone
from bv_autonomy.components.payload import FakePayload
from bv_autonomy.components.platform.base import Platform
from bv_autonomy.components.platform.fake_world import Target
from bv_autonomy.types import Intrinsics


class FakePlatform(Platform):
    """Everything in memory: the given FakeDrone, a FakeCamera that draws targets, and a FakePayload."""

    def __init__(self, drone: FakeDrone, targets: list[Target] | None = None,
                 intrinsics: Intrinsics = BEVY_INTRINSICS, fps: float = 15.0):
        self.drone = drone
        self.camera = FakeCamera(drone, targets, intrinsics, fps)
        self.payload = FakePayload()
        self.intrinsics = intrinsics
