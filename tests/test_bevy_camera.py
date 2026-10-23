import asyncio
import json
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
import pytest_asyncio

from bv_autonomy.components.camera import bevy_camera as bevy


def packet(sequence=1, **changes):
    image = np.full((12, 16, 3), (10, 20, 30), dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    jpeg = encoded.tobytes()
    header = dict(schema="bv.camera_frame", version=1, encoding="jpeg", camera_model="pinhole",
                  data_length=len(jpeg), width=16, height=12, sequence=sequence,
                  fx=120, fy=130, cx=7, cy=5)
    header.update(changes)
    return json.dumps(header).encode() + b"\n", jpeg


@pytest_asyncio.fixture
async def stream_camera(drone, monkeypatch):
    connections = asyncio.Queue()

    async def connect(*args, **kwargs):
        return await connections.get()

    # StreamReader exercises real framing and EOF handling without opening a simulator socket.
    monkeypatch.setattr(bevy.asyncio, "open_connection", connect)
    monkeypatch.setattr(bevy, "_RECONNECT_DELAY_S", 0)
    camera = bevy.BevyCamera(drone)
    await camera.start()
    try:
        yield camera, connections
    finally:
        await camera.stop()


async def test_fragmented_stream_keeps_only_latest_frame_and_capture_pose(stream_camera, drone, monkeypatch):
    camera, connections = stream_camera
    reader, writer = asyncio.StreamReader(), Mock()
    connections.put_nowait((reader, writer))
    first_header, first_jpeg = packet(1)
    second_header, second_jpeg = packet(2)
    reader.feed_data(first_header[:7])
    await asyncio.sleep(0)
    assert not camera._new_frame.is_set()
    reader.feed_data(first_header[7:] + first_jpeg + second_header + second_jpeg[:20])
    await asyncio.wait_for(camera._new_frame.wait(), 2)
    camera._new_frame.clear()
    captured_pose = drone.pose()
    monkeypatch.setattr(drone, "pose", lambda: captured_pose)
    reader.feed_data(second_jpeg[20:])
    await asyncio.wait_for(camera._new_frame.wait(), 2)
    monkeypatch.setattr(drone, "pose", lambda: pytest.fail("pose must be captured on receipt, not decode"))
    result = await camera.latest(timeout=0)
    assert result.sequence == 2
    assert result.pose is captured_pose
    assert result.image.shape == (12, 16, 3)
    assert result.image[0, 0] == pytest.approx((10, 20, 30), abs=2)
    assert (result.intrinsics.fx, result.intrinsics.fy, result.intrinsics.cx, result.intrinsics.cy) == (120, 130, 7, 5)
    assert np.all(result.intrinsics.dist == 0)
    assert await camera.latest(timeout=0.001) is None
    await camera.stop()
    writer.close.assert_called_once()


@pytest.mark.parametrize("damage", ["invalid_json", "oversize", "truncated"])
async def test_bad_stream_is_closed_and_camera_recovers_on_reconnect(stream_camera, damage):
    camera, connections = stream_camera
    broken, good = asyncio.StreamReader(), asyncio.StreamReader()
    old_writer, new_writer = Mock(), Mock()
    connections.put_nowait((broken, old_writer))
    connections.put_nowait((good, new_writer))
    header, jpeg = packet()
    if damage == "invalid_json":
        broken.feed_data(b"{\n")
    elif damage == "oversize":
        broken.feed_data(packet(data_length=bevy._MAX_FRAME_BYTES + 1)[0])
    else:
        broken.feed_data(header + jpeg[:10])
    broken.feed_eof()
    good.feed_data(header + jpeg)
    result = await camera.latest(timeout=2)
    assert result is not None and result.sequence == 1
    old_writer.close.assert_called_once()


@pytest.mark.parametrize("damage", ["invalid_jpeg", "wrong_dimensions"])
async def test_latest_rejects_bad_image_and_can_consume_next_frame(stream_camera, damage):
    camera, connections = stream_camera
    reader = asyncio.StreamReader()
    connections.put_nowait((reader, Mock()))
    if damage == "invalid_jpeg":
        header, _ = packet(data_length=4)
        reader.feed_data(header + b"junk")
    else:
        header, jpeg = packet(width=100)
        reader.feed_data(header + jpeg)
    with pytest.raises(ValueError, match="doesn't match"):
        await camera.latest(timeout=2)
    header, jpeg = packet(2)
    reader.feed_data(header + jpeg)
    result = await camera.latest(timeout=2)
    assert result is not None and result.sequence == 2
