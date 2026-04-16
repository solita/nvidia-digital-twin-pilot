"""
forklift_controller.py — Forklift autonomous driving controller.

Moves the forklift along a list of (x, y) waypoints by updating its root
Xform transform each simulation step. This approach is physics-independent
and works with any USD forklift asset regardless of joint configuration.

Run via VS Code: open this file and press Ctrl+Shift+P → Isaac Sim: Run Remotely
Scene must already be open in Isaac Sim (scene_assembly.usd).

Dev owner: Dev 2
"""
from __future__ import annotations

import math

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

# ── Configuration ─────────────────────────────────────────────────────────────

FORKLIFT_PRIM_PATH = "/World/forklift_b"

# Waypoints (x, y) in stage units.
# Warehouse bounds: X (-12.5 → 11.5), Y (-18.8 → 20.1) — units are metres.
# These waypoints patrol a rectangle ~1.5 m inside the walls.
WAYPOINTS: list[tuple[float, float]] = [
    (-5.185119007723585, -5.40847410883111),   # start — lower-left aisle
    (-8.0,  17.0),   # drive up left side
    ( 8.0,  17.0),   # cross upper end
    ( 8.0, -15.0),   # drive down right side
    (-5.185119007723585, -5.40847410883111),   # return to start
]

SPEED_PER_STEP = 0.05          # metres per simulation step (~5 cm/step, tune to taste)
WAYPOINT_TOLERANCE = 0.15      # metres — distance at which a waypoint is considered reached


# ── Helpers ───────────────────────────────────────────────────────────────────

def _get_xform(prim_path: str) -> UsdGeom.Xformable:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim not found: {prim_path!r} — check FORKLIFT_PRIM_PATH")
    return UsdGeom.Xformable(prim)


def _get_position(xform: UsdGeom.Xformable) -> tuple[float, float, float]:
    ops = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
    if "xformOp:translate" in ops:
        t = ops["xformOp:translate"].Get()
        return float(t[0]), float(t[1]), float(t[2])
    return 0.0, 0.0, 0.0


def _set_position(xform: UsdGeom.Xformable, x: float, y: float, z: float) -> None:
    ops = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
    if "xformOp:translate" in ops:
        ops["xformOp:translate"].Set(Gf.Vec3d(x, y, z))
    else:
        xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))


def _set_heading(xform: UsdGeom.Xformable, angle_deg: float) -> None:
    """Rotate the forklift to face the given heading (0° = +X axis)."""
    ops = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
    rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), angle_deg)
    q = rot.GetQuat()
    gf_q = Gf.Quatf(float(q.GetReal()),
                    float(q.GetImaginary()[0]),
                    float(q.GetImaginary()[1]),
                    float(q.GetImaginary()[2]))
    if "xformOp:orient" in ops:
        ops["xformOp:orient"].Set(gf_q)
    else:
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(gf_q)


# ── Main controller loop ──────────────────────────────────────────────────────

async def run_forklift() -> None:
    app = omni.kit.app.get_app()
    xform = _get_xform(FORKLIFT_PRIM_PATH)

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
        await app.next_update_async()

    carb.log_info("[forklift_controller] Starting waypoint drive")

    waypoint_index = 0

    while waypoint_index < len(WAYPOINTS):
        wx, wy = WAYPOINTS[waypoint_index]
        cx, cy, cz = _get_position(xform)

        dx = wx - cx
        dy = wy - cy
        dist = math.hypot(dx, dy)

        if dist <= WAYPOINT_TOLERANCE:
            carb.log_info(f"[forklift_controller] Reached waypoint {waypoint_index}: ({wx}, {wy})")
            waypoint_index += 1
            await app.next_update_async()
            continue

        step = min(SPEED_PER_STEP, dist)
        nx = cx + (dx / dist) * step
        ny = cy + (dy / dist) * step

        _set_position(xform, nx, ny, cz)
        _set_heading(xform, math.degrees(math.atan2(dy, dx)))

        await app.next_update_async()

    carb.log_info("[forklift_controller] All waypoints reached.")


import asyncio
asyncio.ensure_future(run_forklift())

