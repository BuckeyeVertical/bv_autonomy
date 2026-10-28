"""Drone-side server for the bv_gcs web dashboard, replacing bv_gcs's ROS approval_node.

Speaks approval_node's protocol (bv_gcs web/src/net/types.ts), so the dashboard works unchanged:
JSON over a WebSocket at /ws, the detection crop at /frame/<id>, and the built frontend at /.
"""

import asyncio
import contextlib
import json
import logging
import time
import uuid
from collections import OrderedDict
from pathlib import Path

from aiohttp import WSMsgType, web

from bv_autonomy import paths
from bv_autonomy.components.drone import Drone
from bv_autonomy.components.gcs.base import Gcs
from bv_autonomy.types import ApprovalRequest, Frame

log = logging.getLogger(__name__)

_PAYLOAD_KEYS = {
    "mission_state": "data",
    "path_progress": "progress",
    "confirm_window": "window",
    "sahi_progress": "progress",
    "drone_fix": None,
}


class BvGcs(Gcs):
    """Relays approvals and status between the mission and the operator's browser.

    The GCS is never an authority: ask() returns None when nobody answers in time, and the
    mission decides what that means. The operator's End laps and Return home buttons only set
    signal("end_laps") and signal("return_home"); the mission has to act on them.

    report() events the dashboard shows:
        report("mission_state", {"state": "lap"})        # "lap" enables End laps
        report("path_progress", {"phase": "scan", "completed": 3, "target": 4, "total": 10,
                                 "segment_start": [lat, lon], "segment_end": [lat, lon]})
    With a drone given, the drone's position is sent on its own every fix_interval seconds.
    """

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 8765,
        web_dist: str | Path = paths.GCS_WEB_DIST,
        drone: Drone | None = None,
        class_ids: dict[str, int] | None = None,
        fix_interval: float = 0.5,
        frame_cache_size: int = 32,
    ):
        """Sets up the server that connects the mission to the operator's dashboard."""
        self._host = host
        self._port = port
        self._dist = Path(web_dist)
        self._drone = drone
        self._class_ids = dict(class_ids or {})
        self._fix_interval = fix_interval
        self._frame_cache_size = frame_cache_size

        self._runner: web.AppRunner | None = None
        self._fix_task: asyncio.Task | None = None
        self._clients: set[web.WebSocketResponse] = set()
        self._state: dict[str, object] = {}
        self._frames: OrderedDict[str, bytes] = OrderedDict()
        self._signals: dict[str, asyncio.Event] = {}
        self._pending: dict | None = None
        self._pending_since = 0.0
        self._answer: asyncio.Future | None = None

    async def start(self) -> None:
        if self._runner is not None:
            return
        self._runner = web.AppRunner(self._build_app())
        await self._runner.setup()
        await web.TCPSite(self._runner, self._host, self._port).start()
        if self._drone is not None:
            self._fix_task = asyncio.create_task(self._send_fixes())
        log.info("GCS dashboard on http://%s:%d", self._host, self._port)

    async def stop(self) -> None:
        if self._fix_task is not None:
            self._fix_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._fix_task
            self._fix_task = None
        if self._answer is not None and not self._answer.done():
            self._answer.set_result(None)
        for ws in list(self._clients):
            await ws.close()
        if self._runner is not None:
            await self._runner.cleanup()
            self._runner = None

    def report(self, event: str, data: dict) -> None:
        value = data.get("state") if event == "mission_state" else data
        if self._state.get(event) == value:
            return
        self._state[event] = value
        self._broadcast(_message(event, value))

    def preview(self, frame: Frame) -> None:
        pass

    async def ask(self, request: ApprovalRequest, timeout: float) -> bool | None:
        if self._answer is not None and not self._answer.done():
            log.warning("superseding pending %s with a new request", self._pending["detection_id"])
            self._answer.set_result(None)

        detection_id = str(uuid.uuid4())
        if request.crop:
            self._frames[detection_id] = request.crop
            while len(self._frames) > self._frame_cache_size:
                self._frames.popitem(last=False)
        answer = asyncio.get_running_loop().create_future()
        self._answer = answer
        self._pending = {
            "detection_id": detection_id,
            "class_id": self._class_ids.get(request.label, -1),
            "class_name": request.label,
            "latitude": request.position.lat,
            "longitude": request.position.lon,
            "altitude": request.position.alt,
            "confidence": request.confidence,
            "drone_latitude": request.drone_position.lat,
            "drone_longitude": request.drone_position.lon,
            "timeout_sec": float(timeout or 0.0),
            "image_url": f"/frame/{detection_id}" if request.crop else None,
        }
        self._pending_since = time.monotonic()
        log.debug("awaiting approval %s: %s at %.6f, %.6f (%.0f s)", detection_id, request.label,
                 request.position.lat, request.position.lon, timeout or 0.0)
        self._broadcast({"type": "pending", "pending": self._pending_for_send()})
        try:
            done, _ = await asyncio.wait({answer}, timeout=timeout or None)
            if not done:
                log.debug("no answer for %s within %.0f s", detection_id, timeout)
                return None
            return answer.result()
        finally:
            if self._answer is answer:
                self._answer = None
                self._pending = None
                self._broadcast({"type": "pending", "pending": None})

    def signal(self, name: str) -> asyncio.Event:
        return self._signals.setdefault(name, asyncio.Event())


    def _pending_for_send(self) -> dict | None:
        if self._pending is None:
            return None
        return {**self._pending, "age_sec": time.monotonic() - self._pending_since}

    def _snapshot(self) -> dict:
        fix = self._state.get("drone_fix")
        return {
            "type": "snapshot",
            "pending": self._pending_for_send(),
            "mission_state": self._state.get("mission_state"),
            "drone_fix": dict(fix) if fix else None,
            "path_progress": self._state.get("path_progress"),
            "confirm_window": self._state.get("confirm_window"),
            "sahi_progress": self._state.get("sahi_progress"),
        }

    def _broadcast(self, payload: dict) -> None:
        if not self._clients:
            return
        text = json.dumps(payload)
        for ws in list(self._clients):
            asyncio.create_task(self._send(ws, text))

    async def _send(self, ws: web.WebSocketResponse, text: str) -> None:
        try:
            await ws.send_str(text)
        except (ConnectionResetError, RuntimeError):
            self._clients.discard(ws)

    async def _send_fixes(self) -> None:
        while True:
            p = self._drone.position()
            self.report("drone_fix", {"latitude": p.lat, "longitude": p.lon})
            await asyncio.sleep(self._fix_interval)

    def _on_decision(self, data: dict) -> dict:
        detection_id = str(data.get("detection_id", ""))
        approved = bool(data.get("approved", False))
        reason = str(data.get("reason", ""))
        pending = self._pending
        waiting = self._answer is not None and not self._answer.done()
        if not waiting or pending is None or pending["detection_id"] != detection_id:
            accepted, message = False, "that detection is no longer waiting for a decision"
        else:
            log.debug("operator %s %s%s", "APPROVED" if approved else "REJECTED", detection_id,
                     f" ({reason})" if reason else "")
            self._answer.set_result(approved)
            accepted, message = True, "approved" if approved else "rejected"
        return {"type": "decision_ack", "detection_id": detection_id, "accepted": accepted, "message": message}

    def _on_request(self, name: str) -> dict:
        refusal = None
        if name == "end_laps":
            if self._state.get("mission_state") != "lap":
                refusal = "End laps is only available during laps"
            elif self.signal(name).is_set():
                refusal = "End laps is already requested"
        if refusal is not None:
            log.debug("refused %s: %s", name.replace("_", " "), refusal)
            return {"type": f"{name}_ack", "accepted": False, "message": refusal}
        log.debug("operator requested %s", name.replace("_", " "))
        self.signal(name).set()
        return {"type": f"{name}_ack", "accepted": True, "message": f"{name.replace('_', ' ')} requested"}


    def _build_app(self) -> web.Application:
        app = web.Application()
        app.router.add_get("/ws", self._ws_handler)
        app.router.add_get("/frame/{detection_id}", self._frame_handler)
        app.router.add_get("/healthz", self._health_handler)
        if (self._dist / "index.html").is_file():
            app.router.add_get("/", self._index_handler)
            app.router.add_static("/", self._dist)
        else:
            app.router.add_get("/", self._no_frontend_handler)
        return app

    async def _ws_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        self._clients.add(ws)
        log.debug("dashboard connected from %s (%d total)", request.remote, len(self._clients))
        try:
            await ws.send_str(json.dumps(self._snapshot()))
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                kind = data.get("type")
                if kind == "decision":
                    reply = self._on_decision(data)
                elif kind in ("end_laps", "return_home"):
                    reply = self._on_request(kind)
                elif kind == "preview":
                    reply = {"type": "preview_state", "enabled": False}
                else:
                    continue
                await ws.send_str(json.dumps(reply))
        finally:
            self._clients.discard(ws)
            log.debug("dashboard disconnected (%d left)", len(self._clients))
        return ws

    async def _frame_handler(self, request: web.Request) -> web.Response:
        crop = self._frames.get(request.match_info["detection_id"])
        if crop is None:
            raise web.HTTPNotFound(text="no cached frame for that detection")
        return web.Response(body=crop, content_type="image/jpeg", headers={"Cache-Control": "public, max-age=3600"})

    async def _health_handler(self, request: web.Request) -> web.Response:
        return web.json_response({
            "base_url": f"{request.scheme}://{request.host}",
            "clients": len(self._clients),
            "video_clients": 0,
            "pending": self._pending_for_send(),
            "mission_state": self._state.get("mission_state"),
            "frontend": str(self._dist) if (self._dist / "index.html").is_file() else "not built",
        })

    async def _index_handler(self, _request: web.Request) -> web.FileResponse:
        return web.FileResponse(self._dist / "index.html", headers={"Cache-Control": "no-cache"})

    async def _no_frontend_handler(self, _request: web.Request) -> web.Response:
        return web.Response(text=f"bv_gcs frontend not found at {self._dist}; build it with npm run build in bv_gcs/web.\n")


def _message(event: str, value) -> dict:
    key = _PAYLOAD_KEYS.get(event, "data")
    if key is None:
        return {"type": event, **value}
    return {"type": event, key: value}
