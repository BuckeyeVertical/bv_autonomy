"""Detector that 'sees' placed targets by projecting them into the frame, with optional misses and noise."""

import random

from bv_autonomy.components.detector.base import Detector
from bv_autonomy.fake_world import Target, project
from bv_autonomy.types import Detection, Frame


class FakeDetector(Detector):
    """Reports every target in view from the frame's pose, without looking at the pixels.

    miss_rate drops a detection at random and box_noise_px jitters the box, so mission logic
    that confirms over several frames gets exercised. seed makes a run repeatable.
    """

    def __init__(self, targets: list[Target], confidence: float = 0.9, miss_rate: float = 0.0,
                 box_noise_px: float = 0.0, seed: int | None = None):
        self._targets = list(targets)
        self._confidence = confidence
        self._miss_rate = miss_rate
        self._noise = box_noise_px
        self._random = random.Random(seed)
        self._started = False

    async def start(self) -> None:
        self._started = True

    async def stop(self) -> None:
        self._started = False

    async def detect(self, frame: Frame) -> list[Detection]:
        if not self._started:
            raise RuntimeError("FakeDetector.detect() called before start()")
        height, width = frame.image.shape[:2]
        detections = []
        for target in self._targets:
            box = project(target, frame.pose, frame.intrinsics, width, height)
            if box is None or self._random.random() < self._miss_rate:
                continue
            if self._noise:
                box = tuple(v + self._random.gauss(0.0, self._noise) for v in box)
            detections.append(Detection(target.label, self._confidence, box, frame))
        return detections
