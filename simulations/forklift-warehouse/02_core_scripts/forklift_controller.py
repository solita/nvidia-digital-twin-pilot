"""
forklift_controller.py — Forklift physics-based driving controller.

Drives the forklift along WAYPOINTS in an infinite closed loop using the
asset’s ArticulationRoot and wheel DriveAPIs (no transform manipulation).

Drive model:
  - Rear-wheel drive:  targetVelocity on back_wheel_drive  (RevoluteJoint, axis X)
  - Rear-wheel steer:  targetPosition on back_wheel_swivel (RevoluteJoint, axis Z, ±60°)
  - Heading read each frame from the chassis body world transform.
  - Speed slows near waypoints and during large heading errors.
  - Obstacle avoidance hook: set OBSTACLE_AVOIDANCE_ENABLED = True (Phase 4).

TUNING:
  If the forklift drives backward  → negate DRIVE_VEL_MAX.
  If the forklift steers the wrong way → negate STEER_GAIN.

Joint data (from inspect_forklift_joints.py):
  ArticulationRootAPI at /World/forklift_b
  back_wheel_drive  — DriveAPI:angular, stiffness=100,     damping=10000 (vel-drive)
  back_wheel_swivel — DriveAPI:angular, stiffness=100000,  damping=100   (pos-drive)

Spatial data (from get_warehouse_spatial_info.py — Apr 20 2026):
  Warehouse size:    72.46 m (X) × 116.62 m (Y) × 29.48 m (Z)
  Navigable bounds:  X(-34.716 → 33.709), Y(-53.770 → 60.718)
  Floor Z:           -0.0002

Prerequisites: run setup_physics.py once before running this controller.

Run via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely
Scene must already be open in Isaac Sim (scene_assembly.usd).

Dev owner: Dev 2
"""
from __future__ import annotations

import asyncio
import math

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

# ── Configuration ─────────────────────────────────────────────────────────────

# Prim paths (from inspect_forklift_joints.py)
FORKLIFT_PRIM_PATH = "/World/forklift_b"
BODY_PRIM_PATH     = "/World/forklift_b/body"                               # chassis rigid body
DRIVE_JOINT_PATH   = "/World/forklift_b/back_wheel_joints/back_wheel_drive"  # propulsion
STEER_JOINT_PATH   = "/World/forklift_b/back_wheel_joints/back_wheel_swivel" # steering

# Floor Z from get_warehouse_spatial_info.py output
FLOOR_Z = -0.0002

# Serpentine patrol across the scaled warehouse.
# Navigable bounds from get_warehouse_spatial_info.py (Apr 20 2026):
#   Warehouse: X(-36.73 → 35.72), Y(-54.84 → 61.78)  — 72 × 117 m
#   Navigable: X(-34.716 → 33.709), Y(-53.770 → 60.718)
WAYPOINTS: list[tuple[float, float]] = [
    (-29.09,  -17.48),   # start — forklift rest position (from get_forklift_transform.py)
    (-33.0,   -52.0),   # lower-left corner
    (-33.0,    59.0),   # upper-left corner
    (  0.0,    59.0),   # top-centre cross
    (  0.0,   -52.0),   # bottom-centre
    ( 32.0,   -52.0),   # lower-right corner
    ( 32.0,    59.0),   # upper-right corner
    # loops back to index 0 — no duplicate needed
]

# Drive: target velocity on back_wheel_drive joint (degrees/second, angular)
# DRIVE_DIRECTION: 1.0 = forward, -1.0 = forward is the other way.
DRIVE_DIRECTION  =  -1.0   # flip if forklift drives backward
DRIVE_VEL_MAX    =  5000.0  # cruise speed magnitude (deg/s) — straight-line speed
DRIVE_VEL_TURN   =  1200.0  # speed magnitude while turning (heading error > TURN_SLOW_THRESH)
DRIVE_VEL_MIN    =   300.0  # minimum speed magnitude near waypoints
SLOW_ZONE        =    6.0   # metres from waypoint at which braking begins

# Steer: target position on back_wheel_swivel joint (degrees)
# STEER_DIRECTION: 1.0 = normal, -1.0 = flip if forklift steers the wrong way.
STEER_DIRECTION  =  -1.0   # flip if steering is inverted
STEER_GAIN       =   1.0   # heading error (°) → steer angle multiplier (always positive)
STEER_MAX        =  55.0   # clamped inside joint limit of ±60°

# Heading
TURN_SLOW_THRESH = 45.0    # heading error (°) above which speed → DRIVE_VEL_TURN

# Arrival
WAYPOINT_TOLERANCE = 1.5   # metres — larger than kinematic (physics overshoots slightly)

# Obstacle avoidance (Phase 4) — set True once obstacle_avoidance.py is implemented
OBSTACLE_AVOIDANCE_ENABLED = False

# Physics settling frames after timeline.play()
PHYSICS_SETTLE_FRAMES = 10


# ── Helpers ───────────────────────────────────────────────────────────────────

def _get_body_pose(body_prim) -> tuple[float, float, float]:
    """Return (world_x, world_y, heading_deg) from the chassis rigid body transform."""
    mat = UsdGeom.Xformable(body_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = mat.ExtractTranslation()
    # Forward direction: local +X axis in world space
    fwd = mat.TransformDir(Gf.Vec3d(1, 0, 0))
    heading = math.degrees(math.atan2(float(fwd[1]), float(fwd[0])))
    return float(pos[0]), float(pos[1]), heading


def _set_drive(drive_api: UsdPhysics.DriveAPI, velocity: float) -> None:
    """Set target velocity (deg/s) on the drive wheel joint."""
    drive_api.GetTargetVelocityAttr().Set(velocity)


def _set_steer(steer_api: UsdPhysics.DriveAPI, angle: float) -> None:
    """Set target position (deg) on the steer joint."""
    steer_api.GetTargetPositionAttr().Set(angle)


def _angle_diff(target: float, current: float) -> float:
    """Shortest signed angular difference in degrees, range [-180, 180]."""
    return (target - current + 180) % 360 - 180


# ── Main controller loop ──────────────────────────────────────────────────────

async def run_forklift() -> None:
    app   = omni.kit.app.get_app()
    stage = omni.usd.get_context().get_stage()

    body_prim   = stage.GetPrimAtPath(BODY_PRIM_PATH)
    drive_joint = stage.GetPrimAtPath(DRIVE_JOINT_PATH)
    steer_joint = stage.GetPrimAtPath(STEER_JOINT_PATH)

    if not body_prim.IsValid():
        raise RuntimeError(f"Body prim not found: {BODY_PRIM_PATH!r}")
    if not drive_joint.IsValid():
        raise RuntimeError(f"Drive joint not found: {DRIVE_JOINT_PATH!r}")
    if not steer_joint.IsValid():
        raise RuntimeError(f"Steer joint not found: {STEER_JOINT_PATH!r}")

    drive_api = UsdPhysics.DriveAPI(drive_joint, "angular")
    steer_api  = UsdPhysics.DriveAPI(steer_joint, "angular")

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
        for _ in range(PHYSICS_SETTLE_FRAMES):
            await app.next_update_async()

    carb.log_info("[forklift_controller] Starting physics patrol route (infinite loop)")

    waypoint_index = 0
    lap = 0

    while True:
        # ── Respect timeline stop ─────────────────────────────────────────────
        if not timeline.is_playing():
            _set_drive(drive_api, 0.0)   # brake
            await app.next_update_async()
            continue

        wx, wy = WAYPOINTS[waypoint_index]
        cx, cy, current_heading = _get_body_pose(body_prim)

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
            _set_drive(drive_api, 0.0)   # coast through waypoint
            _set_steer(steer_api, 0.0)   # straighten wheels
            await app.next_update_async()
            continue

        # ── Heading and steering ──────────────────────────────────────────────
        target_heading = math.degrees(math.atan2(dy, dx))
        heading_error  = _angle_diff(target_heading, current_heading)

        steer_angle = max(-STEER_MAX, min(STEER_MAX, heading_error * STEER_GAIN))
        _set_steer(steer_api, steer_angle)

        # ── Variable speed ────────────────────────────────────────────────────
        proximity_t = min(1.0, dist / SLOW_ZONE)
        # Blend between DRIVE_VEL_TURN (full turn) and DRIVE_VEL_MAX (straight)
        turn_t      = max(0.0, 1.0 - abs(heading_error) / TURN_SLOW_THRESH)
        vel_ceiling = DRIVE_VEL_TURN + (DRIVE_VEL_MAX - DRIVE_VEL_TURN) * turn_t
        speed       = DRIVE_VEL_MIN + (vel_ceiling - DRIVE_VEL_MIN) * proximity_t
        speed       = max(DRIVE_VEL_MIN, speed)  # always positive magnitude

        # ── Obstacle avoidance hook (Phase 4) ──────────────────────────────────
        if OBSTACLE_AVOIDANCE_ENABLED:
            from obstacle_avoidance import is_path_blocked  # noqa: PLC0415
            if is_path_blocked():
                _set_drive(drive_api, 0.0)
                await app.next_update_async()
                continue

        _set_drive(drive_api, speed * DRIVE_DIRECTION)
        _set_steer(steer_api, steer_angle * STEER_DIRECTION)
        await app.next_update_async()


# Cancel any previously running forklift task before starting a new one.
_TASK_KEY = "_forklift_controller_task"
_existing = getattr(asyncio.get_event_loop(), _TASK_KEY, None)
if _existing and not _existing.done():
    _existing.cancel()

_task = asyncio.ensure_future(run_forklift())
setattr(asyncio.get_event_loop(), _TASK_KEY, _task)

