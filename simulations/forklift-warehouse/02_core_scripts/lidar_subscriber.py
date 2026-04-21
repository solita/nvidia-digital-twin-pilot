"""lidar_subscriber.py - Stream lidar data directly to the dashboard.

Reads PhysX lidar depth data from the existing sensor prim and writes the latest
snapshot to a shared JSON file that the dashboard reads directly.

Prerequisite: run lidar_sensor_setup.py first so the PhysX lidar exists at
/World/forklift_b/body/lidar_sensor.

Run via VS Code: Ctrl+Shift+P -> Isaac Sim: Run File Remotely
"""

import asyncio
import json
import os
import time

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import UsdGeom


FORKLIFT_PRIM_PATH = "/World/forklift_b"
LIDAR_PRIM_PATH = "/World/forklift_b/body/lidar_sensor"
MIN_RANGE = 0.4
MAX_RANGE = 40.0
LOG_INTERVAL_FRAMES = 60
SAFETY_DISTANCE_M = 2.0
DASHBOARD_STATE_PATH_CANDIDATES = [
    "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot/simulations/forklift-warehouse/03_dashboard/runtime/lidar_state.json",
    "/home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot/simulations/forklift-warehouse/03_dashboard/runtime/lidar_state.json",
]
_resolved_dashboard_state_path = None


def _log(msg):
    print(f"[lidar_subscriber] {msg}", flush=True)


def _get_xform(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim not found: {prim_path!r}")
    return UsdGeom.Xformable(prim)


def _get_nearest_obstacle(distances):
    if distances.size == 0:
        return float("inf")
    return float(np.min(distances))


def _is_path_blocked(distances):
    return _get_nearest_obstacle(distances) < SAFETY_DISTANCE_M


def _resolve_dashboard_state_path():
    global _resolved_dashboard_state_path
    if _resolved_dashboard_state_path is not None:
        return _resolved_dashboard_state_path

    last_error = None
    for candidate in DASHBOARD_STATE_PATH_CANDIDATES:
        target_dir = os.path.dirname(candidate)
        try:
            os.makedirs(target_dir, exist_ok=True)
            _resolved_dashboard_state_path = candidate
            return _resolved_dashboard_state_path
        except OSError as exc:
            last_error = exc

    raise last_error if last_error is not None else RuntimeError("No writable dashboard state path found")


def _write_dashboard_state(payload):
    dashboard_state_path = _resolve_dashboard_state_path()
    temp_path = dashboard_state_path + ".tmp"
    with open(temp_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle)
    os.replace(temp_path, dashboard_state_path)


async def run_lidar_subscriber():
    app = omni.kit.app.get_app()
    stage = omni.usd.get_context().get_stage()

    lidar_prim = stage.GetPrimAtPath(LIDAR_PRIM_PATH)
    if not lidar_prim.IsValid():
        _log(f"ERROR: LIDAR prim not found at {LIDAR_PRIM_PATH}. Run lidar_sensor_setup.py first.")
        return

    try:
        _get_xform(FORKLIFT_PRIM_PATH)
    except RuntimeError as exc:
        _log(f"ERROR: {exc}")
        return

    from omni.isaac.range_sensor import _range_sensor
    lidar_interface = _range_sensor.acquire_lidar_sensor_interface()

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
        await app.next_update_async()

    try:
        dashboard_state_path = _resolve_dashboard_state_path()
    except OSError as exc:
        _log(f"ERROR: unable to find writable dashboard state path: {exc}")
        return

    _log(f"Writing dashboard state to: {dashboard_state_path}")

    frame_count = 0
    while True:
        await app.next_update_async()

        if not timeline.is_playing():
            _log("Timeline stopped -- exiting")
            break

        frame_count += 1
        depth = lidar_interface.get_linear_depth_data(LIDAR_PRIM_PATH)
        if depth is None:
            if frame_count % LOG_INTERVAL_FRAMES == 0:
                _log(f"frame={frame_count} depth=None (sensor not ready)")
            continue

        depth_arr = np.asarray(depth, dtype=np.float32).ravel()
        distances = depth_arr[
            np.isfinite(depth_arr) & (depth_arr > 0.0) & (depth_arr <= MAX_RANGE)
        ]

        nearest_obstacle_m = _get_nearest_obstacle(distances)
        path_blocked = _is_path_blocked(distances)

        _write_dashboard_state(
            {
                "updated_at": time.time(),
                "nearest_obstacle_m": None if not np.isfinite(nearest_obstacle_m) else float(nearest_obstacle_m),
                "path_blocked": bool(path_blocked),
                "scan": {
                    "range_min": float(MIN_RANGE),
                    "range_max": float(MAX_RANGE),
                    "ranges": distances.astype(np.float32).tolist(),
                    "count": int(distances.size),
                },
            }
        )

        if frame_count % LOG_INTERVAL_FRAMES == 0:
            nearest_text = f"{nearest_obstacle_m:.3f} m" if np.isfinite(nearest_obstacle_m) else "inf"
            _log(
                f"returns={distances.size} "
                f"nearest={nearest_text} "
                f"blocked={path_blocked}"
            )


asyncio.ensure_future(run_lidar_subscriber())
