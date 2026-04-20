"""
forklift_controller.py — Forklift autonomous driving controller.

Moves the forklift along a list of (x, y) waypoints by updating its root
Xform transform each simulation step. This approach is physics-independent
and works with any USD forklift asset regardless of joint configuration.

Movement model:
  - Infinite closed loop: runs the waypoint list repeatedly until the
    simulation is stopped.
  - Variable speed: accelerates to SPEED_MAX, decelerates in SLOW_ZONE near
    each waypoint and when the required heading change exceeds TURN_SLOW_THRESH.
  - Smooth heading: rotation is slew-rate limited to HEADING_SLEW_DEG per step,
    so the forklift turns gradually rather than snapping.
  - Obstacle avoidance hook: set OBSTACLE_AVOIDANCE_ENABLED = True once
    obstacle_avoidance.py is implemented (Phase 4).

Spatial data (from helper scripts):
  Warehouse navigable bounds: X(-10.49 → 9.48), Y(-17.69 → 18.99)
  Floor Z: 0.0
  Forklift size: 3.03 m (X) × 1.13 m (Y) × 2.94 m (Z)
  Forklift pivot: offset ~0.63 m behind geometric centre on X axis

NOTE: After scaling the warehouse (Phase 2), re-run get_warehouse_spatial_info.py
and update WAYPOINTS + navigable bounds comment above.

Run via VS Code: open this file and press Ctrl+Shift+P → Isaac Sim: Run File Remotely
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

# Floor Z from get_warehouse_spatial_info.py output
FLOOR_Z = 0.0

# Serpentine patrol: left aisle north, cross top, right aisle south, loop.
# Navigable bounds: X(-10.49 → 9.48), Y(-17.69 → 18.99)
# NOTE: update these after scaling the warehouse (Phase 2).
WAYPOINTS: list[tuple[float, float]] = [
    ( 0.0,   0.0),   # centre of warehouse
    (-9.0, -16.0),   # lower-left corner
    (-9.0,  17.0),   # upper-left corner
    ( 0.0,  17.0),   # top-centre cross
    ( 0.0, -16.0),   # bottom-centre
    ( 8.5, -16.0),   # lower-right corner
    ( 8.5,  17.0),   # upper-right corner
    # loops back to index 0 — no duplicate needed
]

# Speed (metres per simulation step, sim runs ~60 Hz)
SPEED_MAX          = 0.10   # cruising speed  (~6 m/s at 60 Hz — reduce if too fast)
SPEED_MIN          = 0.02   # minimum speed when braking / turning
SLOW_ZONE          = 3.0    # metres from waypoint at which braking begins

# Heading
HEADING_SLEW_DEG   = 4.0    # max heading change per step — lower = smoother turns
TURN_SLOW_THRESH   = 45.0   # heading error (°) above which speed is clamped to SPEED_MIN

# Arrival
WAYPOINT_TOLERANCE = 0.25   # metres — distance at which a waypoint is considered reached

# Obstacle avoidance (Phase 4) — set True once obstacle_avoidance.py is implemented
OBSTACLE_AVOIDANCE_ENABLED = False


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


def _angle_diff(target: float, current: float) -> float:
    """Shortest signed angular difference in degrees, range [-180, 180]."""
    return (target - current + 180) % 360 - 180


def _slew_heading(current: float, target: float, max_step: float) -> float:
    """Advance current heading toward target by at most max_step degrees."""
    diff = _angle_diff(target, current)
    step = max(-max_step, min(max_step, diff))
    return current + step


# ── Main controller loop ──────────────────────────────────────────────────────

async def run_forklift() -> None:
    app = omni.kit.app.get_app()
    xform = _get_xform(FORKLIFT_PRIM_PATH)

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
        await app.next_update_async()

    carb.log_info("[forklift_controller] Starting patrol route (infinite loop)")

    waypoint_index = 0
    lap = 0
    current_heading = 0.0   # degrees; 0 = facing +X axis (matches forklift rest pose)

    while True:
        wx, wy = WAYPOINTS[waypoint_index]
        cx, cy, _ = _get_position(xform)

        dx = wx - cx
        dy = wy - cy
        dist = math.hypot(dx, dy)

        # ── Arrived ───────────────────────────────────────────────────────────
        if dist <= WAYPOINT_TOLERANCE:
            next_index = (waypoint_index + 1) % len(WAYPOINTS)
            if next_index == 0:
                lap += 1
                carb.log_info(f"[forklift_controller] Lap {lap} complete — restarting route")
            else:
                carb.log_info(
                    f"[forklift_controller] Lap {lap} waypoint {waypoint_index} "
                    f"reached: ({wx}, {wy})"
                )
            waypoint_index = next_index
            await app.next_update_async()
            continue

        # ── Target heading ────────────────────────────────────────────────────
        target_heading = math.degrees(math.atan2(dy, dx))
        heading_error  = abs(_angle_diff(target_heading, current_heading))

        # ── Smooth heading (slew-rate limited) ───────────────────────────────
        current_heading = _slew_heading(current_heading, target_heading, HEADING_SLEW_DEG)
        _set_heading(xform, current_heading)

        # ── Variable speed ────────────────────────────────────────────────────
        # Brake when close to waypoint or mid-turn
        proximity_t = min(1.0, dist / SLOW_ZONE)                        # 0→1
        turn_t      = max(0.0, 1.0 - heading_error / TURN_SLOW_THRESH)  # 0→1
        speed       = SPEED_MIN + (SPEED_MAX - SPEED_MIN) * proximity_t * turn_t
        speed       = max(SPEED_MIN, speed)

        # ── Obstacle avoidance hook (Phase 4) ───────────────────────────────
        if OBSTACLE_AVOIDANCE_ENABLED:
            from obstacle_avoidance import is_path_blocked  # noqa: PLC0415
            if is_path_blocked():
                # Hold position until path clears
                await app.next_update_async()
                continue

        # ── Move ──────────────────────────────────────────────────────────────
        step = min(speed, dist)
        nx = cx + (dx / dist) * step
        ny = cy + (dy / dist) * step
        _set_position(xform, nx, ny, FLOOR_Z)

        await app.next_update_async()


import asyncio
asyncio.ensure_future(run_forklift())

