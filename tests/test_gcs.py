import asyncio

import aiohttp
import pytest
import pytest_asyncio

from bv_autonomy.components.gcs import BvGcs, NullGcs
from bv_autonomy.types import ApprovalRequest, Waypoint


async def receive_type(ws, kind):
    async with asyncio.timeout(2):
        while True:
            message = await ws.receive_json()
            if message["type"] == kind:
                return message


@pytest_asyncio.fixture
async def server(tmp_path):
    gcs = BvGcs(host="127.0.0.1", port=0, web_dist=tmp_path / "web")
    try:
        await gcs.start()
        port = gcs._runner.addresses[0][1]
        async with aiohttp.ClientSession() as client:
            yield gcs, client, f"http://127.0.0.1:{port}"
    finally:
        await gcs.stop()


@pytest_asyncio.fixture
async def websocket(server):
    gcs, client, url = server
    async with client.ws_connect(f"{url}/ws") as ws:
        snapshot = await receive_type(ws, "snapshot")
        assert snapshot["mission_state"] is None
        yield gcs, ws


@pytest.mark.parametrize("state", [None, "takeoff", "scan", "aborted"])
async def test_end_laps_refused_outside_laps(websocket, state):
    gcs, ws = websocket
    if state is not None:
        gcs.report("mission_state", {"state": state})
    await ws.send_json({"type": "end_laps"})
    ack = await receive_type(ws, "end_laps_ack")
    assert ack["accepted"] is False
    assert not gcs.signal("end_laps").is_set()


async def test_end_laps_accepted_once_then_duplicate_refused(websocket):
    gcs, ws = websocket
    gcs.report("mission_state", {"state": "lap"})
    await ws.send_json({"type": "end_laps"})
    assert (await receive_type(ws, "end_laps_ack"))["accepted"] is True
    assert gcs.signal("end_laps").is_set()
    await ws.send_json({"type": "end_laps"})
    ack = await receive_type(ws, "end_laps_ack")
    assert ack["accepted"] is False
    assert "already" in ack["message"]


@pytest.mark.parametrize("state", [None, "takeoff", "lap", "scan", "aborted"])
async def test_return_home_accepted_in_any_state(websocket, state):
    gcs, ws = websocket
    if state is not None:
        gcs.report("mission_state", {"state": state})
    await ws.send_json({"type": "return_home"})
    ack = await receive_type(ws, "return_home_ack")
    assert ack["accepted"] is True
    assert gcs.signal("return_home").is_set()


async def test_reconnecting_client_receives_latest_snapshot(server):
    gcs, client, url = server
    gcs.report("mission_state", {"state": "lap"})
    progress = {"phase": "lap", "completed": 2, "total": 7}
    gcs.report("path_progress", progress)
    async with client.ws_connect(f"{url}/ws") as ws:
        snapshot = await receive_type(ws, "snapshot")
    assert snapshot["mission_state"] == "lap"
    assert snapshot["path_progress"] == progress


@pytest.mark.parametrize("approved", [True, False])
async def test_operator_decision_resolves_request_and_serves_crop(server, websocket, approved):
    gcs, client, url = server
    _, ws = websocket
    request = ApprovalRequest("person", Waypoint(36, -96, 0), 0.9, b"jpeg-crop", Waypoint(36, -96, 20))
    task = asyncio.create_task(gcs.ask(request, timeout=2))
    pending = (await receive_type(ws, "pending"))["pending"]
    assert pending["class_name"] == "person"
    assert pending["latitude"] == 36
    async with client.get(url + pending["image_url"]) as response:
        assert response.status == 200
        assert response.content_type == "image/jpeg"
        assert await response.read() == request.crop
    await ws.send_json({"type": "decision", "detection_id": pending["detection_id"], "approved": approved})
    ack = await receive_type(ws, "decision_ack")
    assert ack["accepted"] is True
    assert await asyncio.wait_for(task, 2) is approved


async def test_approval_timeout_returns_none(server):
    gcs, _, _ = server
    request = ApprovalRequest("tent", Waypoint(0, 0, 0), 0.8, b"", Waypoint(0, 0, 10))
    assert await gcs.ask(request, timeout=0.01) is None
    assert gcs._pending is None


async def test_stale_decision_refused(websocket):
    _, ws = websocket
    await ws.send_json({"type": "decision", "detection_id": "old", "approved": True})
    ack = await receive_type(ws, "decision_ack")
    assert ack["accepted"] is False


async def test_invalid_json_does_not_break_websocket(websocket):
    _, ws = websocket
    await ws.send_str("{")
    await ws.send_json({"type": "return_home"})
    assert (await receive_type(ws, "return_home_ack"))["accepted"] is True


async def test_missing_crop_returns_404(server):
    _, client, url = server
    async with client.get(f"{url}/frame/missing") as response:
        assert response.status == 404


async def test_health_reports_mission_state(server):
    gcs, client, url = server
    gcs.report("mission_state", {"state": "lap"})
    async with client.get(f"{url}/healthz") as response:
        assert response.status == 200
        assert (await response.json())["mission_state"] == "lap"


async def test_null_gcs_never_approves_or_sets_signals():
    gcs = NullGcs()
    request = ApprovalRequest("person", Waypoint(0, 0, 0), 0.9, b"", Waypoint(0, 0, 10))
    await gcs.start()
    assert await gcs.ask(request, timeout=10) is None
    assert gcs.signal("end_laps") is gcs.signal("end_laps")
    assert not gcs.signal("end_laps").is_set()
    await gcs.stop()


@pytest.fixture
def approval():
    return ApprovalRequest("person", Waypoint(36, -96, 0), 0.9, b"jpeg", Waypoint(36, -96, 20))


async def test_superseded_approval_cannot_answer_or_clear_new_request(websocket, approval):
    gcs, ws = websocket
    first = asyncio.create_task(gcs.ask(approval, timeout=0))
    old = (await receive_type(ws, "pending"))["pending"]
    second = asyncio.create_task(gcs.ask(approval, timeout=0))
    current = (await receive_type(ws, "pending"))["pending"]
    assert await asyncio.wait_for(first, 2) is None
    assert current["detection_id"] != old["detection_id"]
    assert not second.done()
    await ws.send_json({"type": "decision", "detection_id": old["detection_id"], "approved": True})
    assert (await receive_type(ws, "decision_ack"))["accepted"] is False
    assert not second.done()
    await ws.send_json({"type": "decision", "detection_id": current["detection_id"], "approved": False})
    assert (await receive_type(ws, "decision_ack"))["accepted"] is True
    assert await asyncio.wait_for(second, 2) is False


async def test_cancelled_approval_clears_operator_pending_state(websocket, approval):
    gcs, ws = websocket
    task = asyncio.create_task(gcs.ask(approval, timeout=0))
    await receive_type(ws, "pending")
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert (await receive_type(ws, "pending"))["pending"] is None
    assert gcs._answer is None


async def test_stop_resolves_unbounded_approval_and_closes_connection(websocket, approval):
    gcs, ws = websocket
    task = asyncio.create_task(gcs.ask(approval, timeout=0))
    await receive_type(ws, "pending")
    await asyncio.wait_for(gcs.stop(), 2)
    assert await asyncio.wait_for(task, 2) is None
    assert gcs._pending is None
    async with asyncio.timeout(2):
        while not ws.closed:
            await ws.receive()


async def test_second_decision_in_same_event_loop_turn_is_refused(websocket, approval):
    gcs, ws = websocket
    task = asyncio.create_task(gcs.ask(approval, timeout=0))
    pending = (await receive_type(ws, "pending"))["pending"]
    decision = {"detection_id": pending["detection_id"], "approved": True}
    try:
        # Two clients can answer before ask() resumes to clear the pending request.
        assert gcs._on_decision(decision)["accepted"] is True
        assert gcs._on_decision(decision)["accepted"] is False
    finally:
        assert await asyncio.wait_for(task, 2) is True


async def test_crop_cache_evicts_oldest_without_losing_newer_images(server, websocket, approval, monkeypatch):
    gcs, client, url = server
    _, ws = websocket
    monkeypatch.setattr(gcs, "_frame_cache_size", 2)
    image_urls = []
    for _ in range(3):
        task = asyncio.create_task(gcs.ask(approval, timeout=0))
        image_urls.append((await receive_type(ws, "pending"))["pending"]["image_url"])
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        await receive_type(ws, "pending")
    for index, image_url in enumerate(image_urls):
        async with client.get(url + image_url) as response:
            assert response.status == (404 if index == 0 else 200)
            if index:
                assert await response.read() == approval.crop


async def test_reports_reach_all_clients_with_protocol_payloads(server, websocket):
    gcs, client, url = server
    _, first = websocket
    async with client.ws_connect(f"{url}/ws") as second:
        await receive_type(second, "snapshot")
        gcs.report("mission_state", {"state": "lap"})
        gcs.report("path_progress", {"completed": 1})
        gcs.report("drone_fix", {"latitude": 36, "longitude": -96})
        for ws in (first, second):
            assert await receive_type(ws, "mission_state") == {"type": "mission_state", "data": "lap"}
            assert await receive_type(ws, "path_progress") == {"type": "path_progress", "progress": {"completed": 1}}
            assert await receive_type(ws, "drone_fix") == {"type": "drone_fix", "latitude": 36, "longitude": -96}
