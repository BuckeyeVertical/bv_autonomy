"""Detector using a LightlyTrain LT-DETR checkpoint, as in bv_core's MLDetector."""

import asyncio
import gc
from pathlib import Path

import numpy as np

from bv_autonomy import paths
from bv_autonomy.components.detector.base import Detector
from bv_autonomy.types import Detection, Frame


class LtdetrDetector(Detector):
    """Runs LT-DETR on whole frames, one at a time, and keeps detections above their label's threshold.

    Labels come from the checkpoint itself (e.g. {0: "person", 1: "tent"}). Thresholds are
    per label, like bv_core's detection_thresholds; labels left out use default_threshold.
    """

    def __init__(
        self,
        model_path: str | Path = paths.LTDETR_WEIGHTS,
        thresholds: dict[str, float] | None = None,
        default_threshold: float = 0.5,
        device: str | None = None,       # "cuda", "mps" or "cpu"; None lets LightlyTrain pick
    ):
        self._model_path = Path(model_path).expanduser()
        self._thresholds = dict(thresholds or {})
        self._default_threshold = default_threshold
        self._device = device
        self._model = None
        self._labels: dict[int, str] = {}
        self._lock = asyncio.Lock()      # one inference at a time

        for label, value in [("default", default_threshold), *self._thresholds.items()]:
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"threshold for {label} must be from 0 to 1, got {value}")

    async def start(self) -> None:
        if self._model is not None:
            return
        if not self._model_path.is_file():
            raise RuntimeError(f"LT-DETR weights not found: {self._model_path}")
        self._model = await asyncio.to_thread(_load, self._model_path, self._device)
        self._labels = dict(self._model.classes)
        unknown = set(self._thresholds) - set(self._labels.values())
        if unknown:
            raise ValueError(f"thresholds given for labels the model doesn't have: {sorted(unknown)}")

    async def stop(self) -> None:
        async with self._lock:
            self._model = None
            await asyncio.to_thread(_free_gpu_memory)

    async def detect(self, frame: Frame) -> list[Detection]:
        async with self._lock:
            if self._model is None:
                raise RuntimeError("LtdetrDetector.detect() called before start()")
            # Ask the model for everything above the lowest threshold, then apply each label's own.
            lowest = min([self._default_threshold, *self._thresholds.values()])
            labels, boxes, scores = await asyncio.to_thread(_predict, self._model, frame.image, lowest)

        detections = []
        for class_id, box, score in zip(labels, boxes, scores):
            label = self._labels.get(int(class_id))
            if label is None or score < self._thresholds.get(label, self._default_threshold):
                continue
            detections.append(Detection(
                label=label,
                confidence=float(score),
                box=tuple(float(v) for v in box),
                frame=frame,
            ))
        return detections


def _load(model_path: Path, device: str | None):
    import lightly_train   # heavy (pulls in torch), so only loaded when a detector starts

    return lightly_train.load_model(str(model_path), device=device)


def _predict(model, image_bgr: np.ndarray, threshold: float):
    from PIL import Image

    results = model.predict(Image.fromarray(image_bgr[:, :, ::-1]), threshold=threshold)
    labels = _to_numpy(results.get("labels"), np.int32)
    boxes = _to_numpy(results.get("bboxes"), np.float32).reshape((-1, 4))
    scores = _to_numpy(results.get("scores"), np.float32)
    return labels, boxes, scores


def _to_numpy(value, dtype) -> np.ndarray:
    if value is None:
        return np.array([], dtype=dtype)
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=dtype)


def _free_gpu_memory() -> None:
    # Dropping the model isn't enough on the Orin: torch keeps the memory cached until told.
    gc.collect()
    try:
        import torch
    except ImportError:
        return
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
    elif torch.backends.mps.is_available():
        torch.mps.empty_cache()
