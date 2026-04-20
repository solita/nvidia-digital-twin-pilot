"""
forklift_controller.py — Bare minimum: drive the forklift straight forward.

No waypoints, no steering logic, no heading correction.
The forklift drives in whatever direction it is currently facing.
Flip DRIVE_VELOCITY sign if it goes the wrong way.

Run via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely
Scene must already be open in Isaac Sim (scene_assembly.usd).

Dev owner: Dev 2
"""
from __future__ import annotations

import asyncio
import math
import os

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

# ── Configuration ─────────────────────────────────────────────────────────────

DRIVE_JOINT_PATH = "/World/forklift_b/back_wheel_joints/back_wheel_drive"
STEER_JOINT_PATH = "/World/forklift_b/back_wheel_joints/back_wheel_swivel"

DRIVE_VELOCITY = -200.0  # deg/s — flip sign if forklift goes backward
SETTLE_FRAMES  = 60       # frames to hold still before driving starts
RAMP_FRAMES    = 60       # frames to ramp from 0 → full speed (prevents torque spike)

# Steer joint: position mode targeting 0° (wheel straight).
# "Straight" = joint angle 0° = world steer_angle -90° (localRot1 offset is intentional).
# Stiffness must be HIGH enough to resist caster-trail dynamics during motion.
# The startup torque-spike (old bug) is now gone — drive velocity is zeroed before play.
STEER_STIFFNESS_SETTLE = 20000.0  # snaps wheel straight during settle (forklift stationary)
STEER_STIFFNESS_DRIVE  = 40000.0  # strong enough to resist caster drift under load
STEER_DAMPING          = 10000.0  # damping to prevent oscillation

SWIVEL_PRIM_PATH = "/World/forklift_b/back_wheel_swivel"
BODY_PRIM_PATH   = "/World/forklift_b/body"
DIAG_LOG = (
    "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot/"
    "simulations/forklift-warehouse/04_current_outputs/forklift_diag.txt"
)


def _world_yaw_deg(prim) -> float:
    """Return the world-space Z-rotation (yaw) of a prim in degrees."""
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    q = m.ExtractRotationQuat()
    x, y, z = q.GetImaginary()
    w = q.GetReal()
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


# ── Main ──────────────────────────────────────────────────────────────────────

async def run_forklift() -> None:
    app   = omni.kit.app.get_app()
    stage = omni.usd.get_context().get_stage()

    drive_joint = stage.GetPrimAtPath(DRIVE_JOINT_PATH)
    steer_joint = stage.GetPrimAtPath(STEER_JOINT_PATH)

    if not drive_joint.IsValid():
        raise RuntimeError(f"Drive joint not found: {DRIVE_JOINT_PATH!r}")
    if not steer_joint.IsValid():
        raise RuntimeError(f"Steer joint not found: {STEER_JOINT_PATH!r}")

    drive_api = UsdPhysics.DriveAPI(drive_joint, "angular")
    steer_api = UsdPhysics.DriveAPI(steer_joint, "angular")

    swivel_prim = stage.GetPrimAtPath(SWIVEL_PRIM_PATH)
    body_prim   = stage.GetPrimAtPath(BODY_PRIM_PATH)

    # Zero drive velocity in USD before play — prevents leftover -2000 torque spike at t=0
    drive_api.GetTargetVelocityAttr().Set(0.0)

    # Steer: high stiffness during settle snaps wheel to 0° regardless of stored angle.
    steer_api.GetTargetPositionAttr().Set(0.0)
    steer_api.GetStiffnessAttr().Set(STEER_STIFFNESS_SETTLE)
    steer_api.GetDampingAttr().Set(STEER_DAMPING)

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()

    # Hold still while physics settles — high stiffness snaps wheel straight in this window
    for _ in range(SETTLE_FRAMES):
        steer_api.GetTargetPositionAttr().Set(0.0)
        await app.next_update_async()

    # Switch to gentle stiffness for driving (reduces oscillation while moving)
    steer_api.GetStiffnessAttr().Set(STEER_STIFFNESS_DRIVE)

    # Open diag log (overwrite on each run)
    os.makedirs(os.path.dirname(DIAG_LOG), exist_ok=True)
    diag_file = open(DIAG_LOG, "w", buffering=1)  # line-buffered so tail -f works
    diag_file.write("frame, body_yaw, swivel_yaw, steer_angle\n")

    carb.log_info(f"[forklift] GO — ramping to {DRIVE_VELOCITY} deg/s over {RAMP_FRAMES} frames")

    frame = 0
    while True:
        if not timeline.is_playing():
            drive_api.GetTargetVelocityAttr().Set(0.0)
            diag_file.flush()
            await app.next_update_async()
            continue

        # Ramp up from 0 → DRIVE_VELOCITY over RAMP_FRAMES to avoid torque spike
        scale = min(1.0, frame / RAMP_FRAMES)
        drive_api.GetTargetVelocityAttr().Set(DRIVE_VELOCITY * scale)
        steer_api.GetTargetPositionAttr().Set(0.0)  # position mode: steer straight

        # Diagnostic every 30 frames: straight = steer_angle -90°, so deviation = steer_angle + 90°
        if frame % 30 == 0 and swivel_prim.IsValid() and body_prim.IsValid():
            swivel_yaw  = _world_yaw_deg(swivel_prim)
            body_yaw    = _world_yaw_deg(body_prim)
            steer_angle = swivel_yaw - body_yaw
            deviation   = steer_angle + 90.0  # 0° = perfectly straight
            msg = (
                f"frame={frame:4d}  "
                f"body_yaw={body_yaw:7.2f}°  "
                f"steer_angle={steer_angle:6.2f}°  "
                f"deviation_from_straight={deviation:+.2f}°"
            )
            carb.log_warn(f"[forklift diag] {msg}")
            diag_file.write(msg + "\n")

        frame += 1
        await app.next_update_async()


# ── Task management ───────────────────────────────────────────────────────────

_TASK_KEY = "_forklift_controller_task"
_existing = getattr(asyncio.get_event_loop(), _TASK_KEY, None)
if _existing and not _existing.done():
    _existing.cancel()

_task = asyncio.ensure_future(run_forklift())
setattr(asyncio.get_event_loop(), _TASK_KEY, _task)
