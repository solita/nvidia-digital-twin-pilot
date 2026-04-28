"""
forklift_controller.py — Main loop: wires physics joints, nav, and LIDAR together.

This file is intentionally thin — it only owns:
  - Physics joint setup and the simulation timeline
  - The per-frame orchestration loop (get pose → process LIDAR → compute nav → set joints)
  - State JSON and diag-log output

All tunable values  → config.py          (edit to tune without touching logic)
LIDAR processing    → lidar_processor.py  (sensor dev works here independently)
Waypoint nav / PD   → nav_controller.py   (nav dev works here independently)

Run via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely

Dev owner: Integration / sim lead
"""
from __future__ import annotations

import asyncio
import importlib.util
import json
import os
import sys

# ── Module loader ──────────────────────────────────────────────────────────────
# sys.path manipulation is unreliable in Isaac Sim's embedded Python.
# Load local modules directly by filesystem path instead.

_BASE = (
    "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot"
    "/simulations/forklift-warehouse/02_core_scripts"
)

def _load(mod_name: str, filename: str):
    path = f"{_BASE}/{filename}"
    spec = importlib.util.spec_from_file_location(mod_name, path)
    mod  = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = mod   # register first so cross-imports resolve
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
from pxr import UsdPhysics


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
    app   = omni.kit.app.get_app()
    stage = omni.usd.get_context().get_stage()

    if stage is None:
        carb.log_error("[forklift] No stage loaded — open scene_assembly.usd first, then re-run.")
        return

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

    # Settle: hold still while wheel snaps straight under high stiffness
    for _ in range(config.SETTLE_FRAMES):
        steer_api.GetTargetPositionAttr().Set(0.0)
        await app.next_update_async()

    steer_api.GetStiffnessAttr().Set(config.STEER_STIFFNESS_DRIVE)

    # ── Open diag log ──────────────────────────────────────────────────────────
    os.makedirs(os.path.dirname(config.DIAG_LOG), exist_ok=True)
    diag = open(config.DIAG_LOG, "w", buffering=1)
    diag.write("frame, fx, fy, heading, target_hdg, err, steer_cmd\n")

    # ── Create LIDAR sensor ────────────────────────────────────────────────────
    lidar = LidarProcessor()
    await lidar.setup()

    # ── Create nav controller ──────────────────────────────────────────────────
    nav = NavController()

    _log("info", f"Patrol START — {len(config.WAYPOINTS)} waypoints, looping forever", diag)

    frame = 0

    while True:
        if not timeline.is_playing():
            drive_api.GetTargetVelocityAttr().Set(0.0)
            diag.flush()
            await app.next_update_async()
            continue

        # ── Sense ──────────────────────────────────────────────────────────────
        fx, fy, raw_yaw = get_world_transform(forklift_prim)
        lidar_result    = lidar.process()

        # ── Plan ───────────────────────────────────────────────────────────────
        cmd = nav.step(fx, fy, raw_yaw, lidar_result)

        if cmd is None:
            # Waypoint just reached — log and skip to next physics frame
            wp_prev = nav.last_arrived_wp
            _log("info", f"WP {wp_prev} reached  lap={nav.lap}", diag)
            if nav.wp_index == 0 and nav.lap > 0:
                _log("info", f"Lap {nav.lap} complete — looping", diag)
            await app.next_update_async()
            continue

        # Reset LIDAR debounce at the start of a stuck-escape maneuver
        if cmd.escape_just_started:
            lidar.reset_debounce()
            _log("warn",
                 f"STUCK — escape at ({fx:.1f},{fy:.1f}) "
                 f"rep={lidar_result.repulsion_steer:+.1f}°", diag)

        # ── Act ────────────────────────────────────────────────────────────────
        drive_api.GetTargetVelocityAttr().Set(cmd.target_velocity)
        steer_api.GetTargetPositionAttr().Set(cmd.steer_angle)

        # ── State JSON every 10 frames (dashboard data source) ─────────────────
        if frame % 10 == 0:
            state = {
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
                "speed_frac":   round(cmd.speed_frac, 2),
                "lidar_slices": lidar_result.lidar_slices,
                "waypoints":    config.WAYPOINTS,
            }
            try:
                with open(config.STATE_JSON, "w") as _sf:
                    json.dump(state, _sf)
            except Exception:
                pass

        # ── Diag log every 60 frames ───────────────────────────────────────────
        if frame % 60 == 0:
            lidar_tag = lidar_result.lidar_state
            fwd       = lidar_result.forward_min
            rep       = lidar_result.repulsion_steer
            suffix    = (
                f"  LIDAR_{lidar_tag}(fwd={fwd:.1f}m rep={rep:+.0f}°)"
                if lidar_tag != "CLEAR" else
                (f"  APF(rep={rep:+.0f}°)" if abs(rep) > 2.0 else "")
            )
            msg = (
                f"frame={frame:5d}  pos=({fx:.1f},{fy:.1f})  "
                f"hdg={cmd.smooth_heading:.1f}  target={cmd.target_hdg:.1f}  "
                f"err={cmd.heading_err:+.1f}  steer={cmd.steer_cmd_pd:+.1f}  "
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
