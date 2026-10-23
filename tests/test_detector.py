import asyncio
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from bv_autonomy.components.detector import ltdetr_detector as ltdetr


@pytest.fixture
def model_loader(tmp_path, monkeypatch):
    weights = tmp_path / "weights.pt"
    weights.write_bytes(b"test checkpoint")
    loader = Mock(return_value=SimpleNamespace(classes={0: "person", 1: "tent", 2: "marker"}))
    monkeypatch.setattr(ltdetr, "_load", loader)
    monkeypatch.setattr(ltdetr, "_free_gpu_memory", lambda: None)
    return weights, loader


async def test_label_thresholds_include_boundary_use_default_and_discard_unknown_ids(model_loader, frame, monkeypatch):
    weights, _ = model_loader
    detector = ltdetr.LtdetrDetector(weights, thresholds={"person": 0.75, "tent": 0.25}, default_threshold=0.5)
    predict = Mock(return_value=(
        np.array([0, 0, 1, 1, 2, 2, 99]),
        np.arange(28).reshape(7, 4),
        np.array([0.74, 0.75, 0.24, 0.25, 0.49, 0.5, 1.0]),
    ))
    monkeypatch.setattr(ltdetr, "_predict", predict)
    await detector.start()
    found = await detector.detect(frame)
    assert [d.label for d in found] == ["person", "tent", "marker"]
    assert [d.confidence for d in found] == [0.75, 0.25, 0.5]
    assert [d.box for d in found] == [(4., 5., 6., 7.), (12., 13., 14., 15.), (20., 21., 22., 23.)]
    assert all(d.frame is frame for d in found)
    assert predict.call_args.args[1] is frame.image
    assert predict.call_args.args[2] == 0.25
    await detector.stop()


async def test_stop_waits_for_inference_and_restart_loads_again(model_loader, frame, monkeypatch):
    weights, loader = model_loader
    detector = ltdetr.LtdetrDetector(weights)
    entered, release = threading.Event(), threading.Event()
    freed = threading.Event()

    def predict(*args):
        entered.set()
        assert release.wait(2), "test did not release inference"
        assert not freed.is_set()
        return [], [], []

    monkeypatch.setattr(ltdetr, "_predict", predict)
    monkeypatch.setattr(ltdetr, "_free_gpu_memory", freed.set)
    await detector.start()
    await detector.start()
    loader.assert_called_once()
    inference = asyncio.create_task(detector.detect(frame))
    stopping = None
    try:
        assert await asyncio.to_thread(entered.wait, 2)
        stopping = asyncio.create_task(detector.stop())
        await asyncio.sleep(0)
        assert not stopping.done()
        assert not freed.is_set()
    finally:
        release.set()
        await asyncio.wait_for(inference, 2)
        if stopping is not None:
            await asyncio.wait_for(stopping, 2)
    assert freed.is_set()
    with pytest.raises(RuntimeError, match="before start"):
        await detector.detect(frame)
    await detector.start()
    assert loader.call_count == 2
    await detector.stop()


async def test_failed_label_validation_leaves_detector_unstarted(model_loader):
    weights, _ = model_loader
    detector = ltdetr.LtdetrDetector(weights, thresholds={"typo": 0.8})
    try:
        with pytest.raises(ValueError, match="typo"):
            await detector.start()
        assert detector._model is None
    finally:
        await detector.stop()
