"""
forklift_controller.py — Main loop: wires physics joints, nav, and LIDAR together.

Control API (aiohttp server on port 8081, runs inside Isaac Sim):
  POST http://<host>:8081/cmd   body: {"action": "pause"|"resume"|"reset"|"speed", "value": 0-1}
  GET  http://<host>:8081/status  → live forklift state JSON (same as state file)

Data flow:
  Browser  →  POST /cmd  →  aiohttp server  →  asyncio.Queue  →  main loop
  Main loop  →  state dict  →  GET /status  (in-memory, no file read needed)

All tunable values  → fl_config.py
LIDAR processing    → lidar_processor.py
Waypoint nav / PD   → nav_controller.py

Run via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely
Dev owner: Integration / sim lead
"""
from __future__ import annotations

import asyncio
import importlib.util
import json
import os
import queue
import socket
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

# ── Module loader ──────────────────────────────────────────────────────────────
_BASE = (
    "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot"
    "/simulations/forklift-warehouse/02_core_scripts"
)

def _load(mod_name: str, filename: str):
    path = f"{_BASE}/{filename}"
    sys.modules.pop(mod_name, None)
    spec = importlib.util.spec_from_file_location(mod_name, path)
    mod  = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = mod
    spec.loader.exec_module(mod)
    return mod

config        = _load("fl_config",       "fl_config.py")
_lidar_mod    = _load("lidar_processor", "lidar_processor.py")
_nav_mod      = _load("nav_controller",  "nav_controller.py")

LidarProcessor      = _lidar_mod.LidarProcessor
NavController       = _nav_mod.NavController
get_world_transform = _nav_mod.get_world_transform
# ──────────────────────────────────────────────────────────────────────────────

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics


# ── Shared state (written by main loop, read by HTTP status endpoint) ──────────
_live_state: dict = {}
# SimpleQueue is thread-safe — the HTTP handler thread puts, the asyncio loop gets.
_cmd_queue: queue.SimpleQueue = queue.SimpleQueue()

# Two keys in sys.modules survive script re-execution in the same Isaac Sim process:
#   _SOCK_KEY — the bound socket (created once, never rebound)
#   _SRV_KEY  — the current HTTPServer (stopped and replaced on each re-run)
_SOCK_KEY = "__forklift_cmd_sock__"
_SRV_KEY  = "__forklift_cmd_server__"


# ── HTTP command server (stdlib, daemon thread) ────────────────────────────────

def _start_cmd_server() -> None:
    """Start a stdlib HTTP server in a daemon thread on CMD_SERVER_PORT.

    The listening socket is created and bound only ONCE (first run) and then
    reused on every subsequent re-run — so 'address already in use' is
    impossible regardless of how many times the script is restarted.

    On the very first run (or after an aiohttp/old-code run), any stale socket
    held by Python objects in this process is force-closed via gc before we
    attempt to bind.

    POST /cmd  {"action": "pause"|"resume"|"speed", "value": 0.0-1.0}
    GET  /status  → current forklift state JSON
    """
    import gc

    # Stop the old serve_forever loop (non-blocking — in bg thread).
    old_server = sys.modules.pop(_SRV_KEY, None)
    if old_server is not None:
        threading.Thread(target=old_server.shutdown, daemon=True).start()

    # Reuse the already-bound socket, or create it on first run.
    sock = sys.modules.get(_SOCK_KEY)
    if sock is None:
        # Force-close any Python socket object in this process that is already
        # bound to CMD_SERVER_PORT (covers aiohttp leftovers, old HTTPServer,
        # or any prior code that didn't store the socket in sys.modules).
        for obj in gc.get_objects():
            try:
                if isinstance(obj, socket.socket) and not obj._closed:
                    if obj.getsockname()[1] == config.CMD_SERVER_PORT:
                        obj.close()
                        carb.log_warn(
                            f"[cmd_server] Force-closed stale socket on port "
                            f"{config.CMD_SERVER_PORT}"
                        )
            except Exception:
                pass

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", config.CMD_SERVER_PORT))
        sock.listen(10)
        sys.modules[_SOCK_KEY] = sock

    class _Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # suppress per-request console noise
            pass

        def do_POST(self):
            if self.path != "/cmd":
                self.send_response(404); self.end_headers(); return
            try:
                length = int(self.headers.get("Content-Length", 0))
                body   = json.loads(self.rfile.read(length))
                action = body.get("action", "")
                value  = body.get("value")
                if not action:
                    raise ValueError("missing action")
                _cmd_queue.put({"action": action, "value": value})
                resp = json.dumps({"ok": True, "action": action}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(resp)))
                self.end_headers()
                self.wfile.write(resp)
            except Exception as exc:
                self.send_response(400); self.end_headers()
                self.wfile.write(str(exc).encode())

        def do_GET(self):
            if self.path != "/status":
                self.send_response(404); self.end_headers(); return
            resp = json.dumps(_live_state).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

    # Build the server against the pre-bound socket — no rebind.
    # HTTPServer(bind_and_activate=False) still creates a throwaway socket
    # internally; discard it immediately and substitute our persistent one.
    # Override server_close() so that shutdown() never closes the shared socket.
    class _Server(HTTPServer):
        def server_close(self):
            pass  # keep the shared socket alive across re-runs

    server = _Server(("0.0.0.0", config.CMD_SERVER_PORT), _Handler, bind_and_activate=False)
    server.socket.close()
    server.socket = sock

    sys.modules[_SRV_KEY] = server
    threading.Thread(target=server.serve_forever, daemon=True).start()
    carb.log_warn(
        f"[cmd_server] Listening on http://0.0.0.0:{config.CMD_SERVER_PORT}"
        f"  (POST /cmd, GET /status)"
    )


# ── Logging helper ─────────────────────────────────────────────────────────────

def _log(level: str, msg: str, diag=None) -> None:
    tagged = f"[forklift] {msg}"
    if level == "warn":
        carb.log_warn(tagged)
    elif level == "error":
        carb.log_error(tagged)
    else:
        carb.log_info(tagged)
    if diag is not None:
        diag.write(f"[{level.upper()}] {msg}\n")


# ── Main loop ─────────────────────────────────────────────────────────────────

async def run_forklift() -> None:
    global _live_state

    app   = omni.kit.app.get_app()
    stage = omni.usd.get_context().get_stage()

    if stage is None:
        carb.log_error("[forklift] No stage loaded — open scene_assembly.usd first.")
        return

    # Start HTTP command server (daemon thread, no await needed)
    _start_cmd_server()

    # ── Acquire physics joint prims ────────────────────────────────────────────
    drive_joint   = stage.GetPrimAtPath(config.DRIVE_JOINT_PATH)
    steer_joint   = stage.GetPrimAtPath(config.STEER_JOINT_PATH)
    forklift_prim = stage.GetPrimAtPath(config.FORKLIFT_PRIM)

    if not drive_joint.IsValid():
        raise RuntimeError(f"Drive joint not found: {config.DRIVE_JOINT_PATH!r}")
    if not steer_joint.IsValid():
        raise RuntimeError(f"Steer joint not found: {config.STEER_JOINT_PATH!r}")
    if not forklift_prim.IsValid():
        raise RuntimeError(f"Forklift prim not found: {config.FORKLIFT_PRIM!r}")

    drive_api = UsdPhysics.DriveAPI(drive_joint, "angular")
    steer_api = UsdPhysics.DriveAPI(steer_joint, "angular")

    # ── Pre-play joint init ────────────────────────────────────────────────────
    drive_api.GetTargetVelocityAttr().Set(0.0)
    steer_api.GetTargetPositionAttr().Set(0.0)
    steer_api.GetStiffnessAttr().Set(config.STEER_STIFFNESS_SETTLE)
    steer_api.GetDampingAttr().Set(config.STEER_DAMPING)

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()

    for _ in range(config.SETTLE_FRAMES):
        steer_api.GetTargetPositionAttr().Set(0.0)
        await app.next_update_async()

    steer_api.GetStiffnessAttr().Set(config.STEER_STIFFNESS_DRIVE)

    # ── Open diag log ──────────────────────────────────────────────────────────
    os.makedirs(os.path.dirname(config.DIAG_LOG), exist_ok=True)
    diag = open(config.DIAG_LOG, "w", buffering=1)
    diag.write("frame, fx, fy, heading, target_hdg, err, steer_cmd\n")

    # ── Create sensor and nav ──────────────────────────────────────────────────
    lidar = LidarProcessor()
    await lidar.setup()
    nav = NavController()

    _log("info", f"Patrol START — {len(config.WAYPOINTS)} waypoints, looping", diag)

    frame         = 0
    paused        = False
    speed_override = None   # None = use nav's own speed; 0.0-1.0 = override fraction

    while True:
        if not timeline.is_playing():
            drive_api.GetTargetVelocityAttr().Set(0.0)
            diag.flush()
            await app.next_update_async()
            continue

        # ── Process queued commands (non-blocking) ─────────────────────────────
        while not _cmd_queue.empty():
            try:
                cmd = _cmd_queue.get_nowait()
            except queue.Empty:
                break
            action = cmd.get("action", "")
            value  = cmd.get("value")

            if action == "pause":
                paused = True
                drive_api.GetTargetVelocityAttr().Set(0.0)
                steer_api.GetTargetPositionAttr().Set(0.0)
                _live_state["paused"] = True
                _log("info", "PAUSED", diag)
            elif action == "resume":
                paused = False
                _live_state["paused"] = False
                _log("info", "RESUMED", diag)
            elif action == "speed" and value is not None:
                try:
                    speed_override = max(0.0, min(1.0, float(value)))
                    _log("info", f"Speed override: {speed_override:.0%}", diag)
                except (TypeError, ValueError):
                    pass

        if paused:
            drive_api.GetTargetVelocityAttr().Set(0.0)
            steer_api.GetTargetPositionAttr().Set(0.0)
            try:
                with open(config.STATE_JSON, "w") as _sf:
                    json.dump(_live_state, _sf)
            except Exception:
                pass
            await app.next_update_async()
            continue

        # ── Sense ──────────────────────────────────────────────────────────────
        fx, fy, raw_yaw = get_world_transform(forklift_prim)
        lidar_result    = lidar.process()

        # ── Plan ───────────────────────────────────────────────────────────────
        cmd = nav.step(fx, fy, raw_yaw, lidar_result)

        if cmd is None:
            wp_prev = nav.last_arrived_wp
            _log("info", f"WP {wp_prev} reached  lap={nav.lap}", diag)
            if nav.wp_index == 0 and nav.lap > 0:
                _log("info", f"Lap {nav.lap} complete — looping", diag)
            await app.next_update_async()
            continue

        if cmd.escape_just_started:
            lidar.reset_debounce()
            _log("warn", f"STUCK — escape at ({fx:.1f},{fy:.1f})", diag)

        # ── Apply speed override if set ────────────────────────────────────────
        target_vel = cmd.target_velocity
        if speed_override is not None:
            target_vel = config.DRIVE_VELOCITY * speed_override * (-1 if config.DRIVE_VELOCITY < 0 else 1)

        # ── Act ────────────────────────────────────────────────────────────────
        drive_api.GetTargetVelocityAttr().Set(target_vel)
        steer_api.GetTargetPositionAttr().Set(cmd.steer_angle)

        # ── Update live state (in-memory — served by GET /status) ─────────────
        if frame % 10 == 0:
            _live_state = {
                "frame":        frame,
                "x":            round(fx, 2),
                "y":            round(fy, 2),
                "heading":      round(cmd.smooth_heading, 1),
                "target_hdg":   round(cmd.target_hdg, 1),
                "heading_err":  round(cmd.heading_err, 1),
                "wp":           cmd.wp_index,
                "lap":          cmd.lap,
                "dist_to_wp":   round(cmd.dist_to_wp, 2),
                "lidar_state":  lidar_result.lidar_state,
                "forward_min":  round(lidar_result.forward_min, 2),
                "repulsion":    round(lidar_result.repulsion_steer, 1),
                "speed_frac":   round(abs(target_vel / config.DRIVE_VELOCITY), 2),
                "speed_override": speed_override,
                "lidar_slices": lidar_result.lidar_slices,
                "waypoints":    config.WAYPOINTS,
                "paused":       paused,
            }
            # Also write to disk for dashboard fallback / diag
            try:
                with open(config.STATE_JSON, "w") as _sf:
                    json.dump(_live_state, _sf)
            except Exception:
                pass

        # ── Diag log every 60 frames ───────────────────────────────────────────
        if frame % 60 == 0:
            tag    = lidar_result.lidar_state
            fwd    = lidar_result.forward_min
            rep    = lidar_result.repulsion_steer
            suffix = (
                f"  LIDAR_{tag}(fwd={fwd:.1f}m rep={rep:+.0f}°)"
                if tag != "CLEAR" else
                (f"  APF(rep={rep:+.0f}°)" if abs(rep) > 2.0 else "")
            )
            msg = (
                f"frame={frame:5d}  pos=({fx:.1f},{fy:.1f})  "
                f"hdg={cmd.smooth_heading:.1f}  err={cmd.heading_err:+.1f}  "
                f"dist={cmd.dist_to_wp:.1f}m  wp={cmd.wp_index}  lap={cmd.lap}"
                + suffix
            )
            _log("info", msg, diag)

        frame += 1
        await app.next_update_async()

    diag.write("Controller stopped.\n")
    diag.close()


# ── Task management ───────────────────────────────────────────────────────────

_TASK_KEY = "_forklift_controller_task"
_existing = getattr(asyncio.get_event_loop(), _TASK_KEY, None)
if _existing and not _existing.done():
    _existing.cancel()

_task = asyncio.ensure_future(run_forklift())
setattr(asyncio.get_event_loop(), _TASK_KEY, _task)
