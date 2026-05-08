"""dashboard.py — Forklift patrol dashboard (top-down warehouse view).

Reads forklift_state.json written by forklift_controller.py every 10 frames
and serves a live browser UI showing a to-scale top-down view of every asset
in scene_assembly.usd: warehouse walls, rack shelving, structural columns,
obstacle cubes, the forklift (live), waypoint route, and key metrics.

Asset geometry is extracted from scene_assembly.usd and the controller's
spatial calibration data so the map matches the physical simulation exactly.

Run:
    cd simulations/forklift-warehouse/03_dashboard
    python3 -m uvicorn dashboard:app --host 0.0.0.0 --port 8080

Then open http://<host>:8080 in a browser.
"""

import asyncio
import json
import os
import pathlib
import signal
import subprocess
import urllib.request

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, PlainTextResponse
from starlette.staticfiles import StaticFiles

PORT = 8080


def _kill_existing_port_user() -> None:
    """Kill any process already bound to PORT so uvicorn can start cleanly."""
    try:
        out = subprocess.check_output(
            ["fuser", f"{PORT}/tcp"], stderr=subprocess.DEVNULL
        ).decode().split()
        my_pid = os.getpid()
        for tok in out:
            pid = int(tok)
            if pid != my_pid:
                os.kill(pid, signal.SIGKILL)
    except (subprocess.CalledProcessError, FileNotFoundError, ValueError):
        pass


_kill_existing_port_user()

STATE_FILE = (
    "/home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot/"
    "simulations/forklift-warehouse/04_current_outputs/forklift_state.json"
)

# Controller HTTP server (aiohttp, running inside Isaac Sim on --network=host)
CONTROLLER_CMD_URL = "http://localhost:8081/cmd"

_EMPTY_STATE: dict = {
    "frame": 0,
    "x": 0.0, "y": 0.0,
    "heading": 0.0, "target_hdg": 0.0, "heading_err": 0.0,
    "wp": 0, "lap": 0, "dist_to_wp": 0.0,
    "lidar_state": "CLEAR",
    "forward_min": 9.9, "repulsion": 0.0,
    "speed_frac": 0.0,
    "waypoints": [],
    "lidar_slices": [False, False, False, False, False, False, False, False, False],
}


# ── Mtime-cached reader — only re-reads file when it changes on disk ──────────
_cached_state: dict = _EMPTY_STATE
_cached_mtime_ns: int = 0


def _read_state() -> dict:
    global _cached_state, _cached_mtime_ns
    try:
        mt = os.stat(STATE_FILE).st_mtime_ns
        if mt != _cached_mtime_ns:
            with open(STATE_FILE, encoding="utf-8") as fh:
                _cached_state = json.load(fh)
            _cached_mtime_ns = mt
        return _cached_state
    except Exception:
        return _EMPTY_STATE


_DASHBOARD_DIR = pathlib.Path(__file__).resolve().parent
_HTML_FILE = _DASHBOARD_DIR / "index.html"


app = FastAPI(title="Forklift Dashboard")
app.mount("/static", StaticFiles(directory=str(_DASHBOARD_DIR)), name="static")


@app.on_event("startup")
async def _free_port():
    _kill_existing_port_user()


@app.get("/", response_class=HTMLResponse)
def index():
    return HTMLResponse(_HTML_FILE.read_text(encoding="utf-8"))


# ── Path-obstacle manifest (written by spawn_path_obstacles.py) ────────────────
_OBSTACLES_FILE = (
    "/home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot/"
    "simulations/forklift-warehouse/04_current_outputs/path_obstacles.json"
)
_cached_obstacles: list = []
_cached_obs_mtime_ns: int = 0


def _read_obstacles() -> list:
    global _cached_obstacles, _cached_obs_mtime_ns
    try:
        mt = os.stat(_OBSTACLES_FILE).st_mtime_ns
        if mt != _cached_obs_mtime_ns:
            with open(_OBSTACLES_FILE, encoding="utf-8") as fh:
                _cached_obstacles = json.load(fh)
            _cached_obs_mtime_ns = mt
        return _cached_obstacles
    except Exception:
        return []


@app.get("/api/obstacles", response_class=JSONResponse)
def get_obstacles():
    return JSONResponse(_read_obstacles())


@app.get("/api/state", response_class=JSONResponse)
def get_state():
    return JSONResponse(_read_state())


def _post_controller_cmd(action: str, value=None) -> str:
    """POST an action to the controller's aiohttp server (best-effort)."""
    body = json.dumps({"action": action, "value": value}).encode()
    req  = urllib.request.Request(
        CONTROLLER_CMD_URL,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=2) as resp:
            return resp.read().decode()
    except Exception as exc:
        return f"Error: {exc}"


@app.get("/api/controller-alive", response_class=JSONResponse)
def controller_alive():
    """Probe the controller HTTP server to see if it's reachable."""
    try:
        req = urllib.request.Request("http://localhost:8081/status", method="GET")
        with urllib.request.urlopen(req, timeout=1):
            pass
        return JSONResponse({"alive": True})
    except Exception:
        return JSONResponse({"alive": False})


@app.post("/api/cmd/pause", response_class=PlainTextResponse)
def cmd_pause():
    return _post_controller_cmd("pause")


@app.post("/api/cmd/resume", response_class=PlainTextResponse)
def cmd_resume():
    return _post_controller_cmd("resume")


@app.post("/api/cmd/reset_location", response_class=PlainTextResponse)
def cmd_reset_location():
    return _post_controller_cmd("reset_location")


@app.post("/api/cmd/override")
def cmd_override(body: dict):
    """Forward an override action (speed, lidar_range, …) to the controller."""
    action = body.get("action", "")
    value = body.get("value")
    if not action:
        return PlainTextResponse("missing action", status_code=400)
    return PlainTextResponse(_post_controller_cmd(action, value))


@app.websocket("/ws")
async def ws_state(websocket: WebSocket):
    await websocket.accept()
    last_mtime: int = 0
    last_push: float = 0.0
    try:
        while True:
            try:
                mt = os.stat(STATE_FILE).st_mtime_ns
            except OSError:
                mt = 0
            now = asyncio.get_event_loop().time()
            # Push immediately on file change, or every 2 s as a heartbeat
            if mt != last_mtime or (now - last_push) >= 2.0:
                last_mtime = mt
                last_push  = now
                await websocket.send_json(_read_state())
            await asyncio.sleep(0.05)  # 50 ms poll interval
    except WebSocketDisconnect:
        pass
