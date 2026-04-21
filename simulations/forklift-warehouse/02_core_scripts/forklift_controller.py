"""
forklift_controller.py — Phase 2: heading-corrected single-waypoint drive.

The forklift drives toward a single waypoint due north, using a closed-loop
P controller on steer angle to hold the heading. This validates that heading
correction works before restoring the full 14-waypoint patrol route.

Waypoint: (-29.09, 53.0) — due north of the rest position (-29.09, -17.48)
Reset heading to 90° in reset_forklift.py before running.

Run via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely

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
from pxr import Usd, UsdGeom, UsdPhysics

# ── Configuration ─────────────────────────────────────────────────────────────

DRIVE_JOINT_PATH = "/World/forklift_b/back_wheel_joints/back_wheel_drive"
STEER_JOINT_PATH = "/World/forklift_b/back_wheel_joints/back_wheel_swivel"
FORKLIFT_PRIM    = "/World/forklift_b/body"  # physics rigid body — this is what actually moves

DRIVE_VELOCITY = +400.0   # deg/s wheel spin -- positive = forward/south at heading -90°
SETTLE_FRAMES  =  60      # physics settle before driving
RAMP_FRAMES    =  60      # ramp from 0 → full speed to avoid torque spike

# Steer joint physics — values validated in Phase 1
STEER_STIFFNESS_SETTLE = 20000.0  # snap wheel straight while stationary
STEER_STIFFNESS_DRIVE  = 40000.0  # resist caster drift under load
STEER_DAMPING          = 10000.0  # prevent oscillation

# Heading PD controller
STEER_KP       =   0.30   # proportional gain  (deg steer per deg heading error — scaled to ±60°)
STEER_KD       =   0.10   # derivative gain    (dampens overshoot)
STEER_DEADBAND =   2.5    # deg — ignore errors smaller than this (prevents hunting)
STEER_MAX      =  30.0    # deg — clamp steer command to ±30° for gradual turns
HEADING_SMOOTH =   0.40   # EMA factor for heading (0=frozen, 1=raw) — filters sensor noise

# Patrol route -- designed from warehouse_spatial_info_latest.txt obstacle data.
#
# Key obstacles:
#   West rack:  X = -30.27 to -27.02,  Y = -12.06 to +36.38  (solid collision)
#   Columns:    X = -27.16, -4.37, +8.41, +26.52  (0.60m wide, full height)
#   FL width:   3.03m (half = 1.52m)
#
# Aisles used:
#   South cross  Y = -45  (below all columns south end Y=-27.80)
#   East aisle   X = +20  (between columns at X=8.41 and X=26.52)
#   North cross  Y = +55  (above all racks north end Y=36.38 and pillars Y=45.12)
#   West-center  X = -24  (FL west edge -25.52, rack east edge -27.02 -> 1.50m clearance)
#
# REST_HEADING must be -90 deg so DRIVE_VELOCITY=+200 drives straight south to WP0.
WAYPOINTS = [
    (-15.0, -33.0),   # WP0: south end        -- actual south floor boundary ~Y=-36.4, 3m buffer
    ( 20.0, -33.0),   # WP1: south-east       -- east in south cross-aisle
    ( 20.0,  48.0),   # WP2: north-east       -- actual north wall ~Y=52.3, 4m buffer to turn
    (-24.0,  48.0),   # WP3: north-west       -- west in north cross-aisle
    (-24.0, -33.0),   # WP4: south-west       -- south in west-center aisle
    (-15.0, -17.5),   # WP5: start zone       -- loops back to WP0
]
ARRIVAL_RADIUS = 4.0   # metres -- advance to next waypoint when within this

DIAG_LOG = (
    "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot/"
    "simulations/forklift-warehouse/04_current_outputs/forklift_diag.txt"
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _get_world_transform(prim):
    """Return (x, y, yaw_deg) of a prim in world space."""
    m   = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = m.ExtractTranslation()
    q   = m.ExtractRotationQuat()
    x, y, z = q.GetImaginary()
    w   = q.GetReal()
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return float(pos[0]), float(pos[1]), yaw


def _angle_diff(a: float, b: float) -> float:
    """Signed shortest-path difference a-b, wrapped to [-180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


# ── Main ──────────────────────────────────────────────────────────────────────

async def run_forklift() -> None:
    app   = omni.kit.app.get_app()
    stage = omni.usd.get_context().get_stage()

    drive_joint    = stage.GetPrimAtPath(DRIVE_JOINT_PATH)
    steer_joint    = stage.GetPrimAtPath(STEER_JOINT_PATH)
    forklift_prim  = stage.GetPrimAtPath(FORKLIFT_PRIM)

    if not drive_joint.IsValid():
        raise RuntimeError(f"Drive joint not found: {DRIVE_JOINT_PATH!r}")
    if not steer_joint.IsValid():
        raise RuntimeError(f"Steer joint not found: {STEER_JOINT_PATH!r}")
    if not forklift_prim.IsValid():
        raise RuntimeError(f"Forklift prim not found: {FORKLIFT_PRIM!r}")

    drive_api = UsdPhysics.DriveAPI(drive_joint, "angular")
    steer_api = UsdPhysics.DriveAPI(steer_joint, "angular")

    # ── Pre-play: zero drive velocity in USD (prevents leftover torque spike) ──
    drive_api.GetTargetVelocityAttr().Set(0.0)

    # ── Pre-play: high steer stiffness — snaps wheel straight before physics runs ──
    steer_api.GetTargetPositionAttr().Set(0.0)
    steer_api.GetStiffnessAttr().Set(STEER_STIFFNESS_SETTLE)
    steer_api.GetDampingAttr().Set(STEER_DAMPING)

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()

    # Settle: hold still, wheel snaps straight under high stiffness
    for _ in range(SETTLE_FRAMES):
        steer_api.GetTargetPositionAttr().Set(0.0)
        await app.next_update_async()

    # Switch to drive stiffness
    steer_api.GetStiffnessAttr().Set(STEER_STIFFNESS_DRIVE)

    # ── Init heading state ─────────────────────────────────────────────────────
    fx, fy, raw_yaw = _get_world_transform(forklift_prim)
    smooth_heading  = raw_yaw
    prev_heading_err = 0.0

    # ── Diag log ───────────────────────────────────────────────────────────────
    os.makedirs(os.path.dirname(DIAG_LOG), exist_ok=True)
    diag = open(DIAG_LOG, "w", buffering=1)
    diag.write("frame, fx, fy, heading, target_hdg, err, steer_cmd\n")

    carb.log_info(f"[forklift] Phase 3 patrol START -- {len(WAYPOINTS)} waypoints, looping forever")

    frame      = 0
    wp_index   = 0
    lap        = 0

    while True:  # loop forever
        if not timeline.is_playing():
            drive_api.GetTargetVelocityAttr().Set(0.0)
            diag.flush()
            await app.next_update_async()
            continue

        wx, wy = WAYPOINTS[wp_index]

        # ── Forklift world pose ────────────────────────────────────────────────
        fx, fy, raw_yaw = _get_world_transform(forklift_prim)

        # EMA-smooth heading to filter physics jitter
        smooth_heading = HEADING_SMOOTH * raw_yaw + (1.0 - HEADING_SMOOTH) * smooth_heading


        # ── Arrival check ─────────────────────────────────────────────────────
        dist = math.hypot(wx - fx, wy - fy)
        if dist < ARRIVAL_RADIUS:
            carb.log_info(f"[forklift] WP {wp_index} reached: ({wx},{wy})  lap={lap}")
            wp_index += 1
            if wp_index >= len(WAYPOINTS):
                wp_index = 0
                lap += 1
                carb.log_info(f"[forklift] Lap {lap} complete -- looping")
            continue

        # ── Heading error: direction to waypoint vs current heading ───────────
        target_hdg = math.degrees(math.atan2(wy - fy, wx - fx))
        heading_err = _angle_diff(target_hdg, smooth_heading)

        # ── PD steer command ──────────────────────────────────────────────────
        d_err      = heading_err - prev_heading_err
        steer_cmd  = STEER_KP * heading_err + STEER_KD * d_err
        steer_cmd  = max(-STEER_MAX, min(STEER_MAX, steer_cmd))

        if abs(heading_err) < STEER_DEADBAND:
            steer_cmd = 0.0

        prev_heading_err = heading_err

        # ── Drive ─────────────────────────────────────────────────────────────
        scale = min(1.0, frame / RAMP_FRAMES)
        drive_api.GetTargetVelocityAttr().Set(DRIVE_VELOCITY * scale)
        steer_api.GetTargetPositionAttr().Set(-steer_cmd)  # negated: joint localRot1 90° offset inverts steer direction

        # ── Diag every 60 frames ──────────────────────────────────────────────
        if frame % 60 == 0:
            msg = (
                f"frame={frame:5d}  pos=({fx:.1f},{fy:.1f})  "
                f"hdg={smooth_heading:.1f}  target={target_hdg:.1f}  "
                f"err={heading_err:+.1f}  steer={steer_cmd:+.1f}  "
                f"dist={dist:.1f}m  wp={wp_index}  lap={lap}"
            )
            carb.log_info(f"[forklift] {msg}")
            diag.write(msg + "\n")

        frame += 1
        await app.next_update_async()
    diag.write("ARRIVED — controller stopped\n")
    diag.close()
    carb.log_info("[forklift] All waypoints reached. Stopped.")


# ── Task management ───────────────────────────────────────────────────────────

_TASK_KEY = "_forklift_controller_task"
_existing = getattr(asyncio.get_event_loop(), _TASK_KEY, None)
if _existing and not _existing.done():
    _existing.cancel()

_task = asyncio.ensure_future(run_forklift())
setattr(asyncio.get_event_loop(), _TASK_KEY, _task)
