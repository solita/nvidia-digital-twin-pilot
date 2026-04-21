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
import omni.kit.commands
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

# ── LIDAR sensor (2D, single horizontal ring) ─────────────────────────────────
LIDAR_ENABLED      = True
LIDAR_PRIM_PATH    = "/World/forklift_b/body/lidar"
LIDAR_STOP_DIST    = 3.0    # metres — full stop  (must be > min_range=2.3 to leave detection window)
LIDAR_SLOW_DIST    = 3.5    # metres — slow to half speed when obstacle within this range
LIDAR_DRAW_LINES   = False   # set False to disable viewport ray visualisation
LIDAR_FORWARD_RAY  = 179    # ray index pointing forward — calibrated from live diag (cube ahead → ray 179)
LIDAR_CONE_HALF    = 20     # half-width of forward detection cone in rays (degrees) — narrow to avoid self-hits during turns
LIDAR_MIN_VALID    = 2.35   # metres — software floor for FORWARD cone only (forklift mast extends ~2.2m ahead)
LIDAR_DEBOUNCE_FRAMES = 10  # consecutive frames obstacle must be detected before stopping (filters transient self-hits)
LIDAR_CENTER_HALF     = 7   # rays — ±7° centre sub-sector (pure-ahead zone for avoidance sector split)
LIDAR_AVOID_STEER     = 22.0  # deg — steer angle applied while laterally avoiding an obstacle
LIDAR_AVOID_SPEED     = 0.35  # fraction of DRIVE_VELOCITY while actively manoeuvring around obstacle
LIDAR_AVOID_MARGIN    = 0.5   # metres — one side must be this much clearer than the other to choose avoidance over full stop
LIDAR_AVOID_HOLD_FRAMES = 20  # consecutive clear frames after body has passed before resuming (oscillation damper)
LIDAR_AVOID_TURN_INHIBIT = 25.0  # deg — suppress avoidance trigger when heading error exceeds this (FL is navigating a waypoint turn, not approaching an obstacle)
FORKLIFT_BODY_HALF_LENGTH = 1.8  # metres — half forklift length from body-centre; avoidance holds until obstacle projects this far behind

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


def _log(level: str, msg: str, diag=None) -> None:
    """Log to the carb console and, if diag is open, to the diag file."""
    tagged = f"[forklift] {msg}"
    if level == "warn":
        carb.log_warn(tagged)
    elif level == "error":
        carb.log_error(tagged)
    else:
        carb.log_info(tagged)
    if diag is not None:
        diag.write(f"[{level.upper()}] {msg}\n")


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

    # ── Diag log (opened early so LIDAR startup messages are also captured) ─────
    os.makedirs(os.path.dirname(DIAG_LOG), exist_ok=True)
    diag = open(DIAG_LOG, "w", buffering=1)
    diag.write("frame, fx, fy, heading, target_hdg, err, steer_cmd\n")

    # ── LIDAR: create sensor programmatically, attached to forklift body ───────
    lidar_if = None
    if LIDAR_ENABLED:
        # Remove stale prim from a previous run (script re-runs without full reload)
        if stage.GetPrimAtPath(LIDAR_PRIM_PATH).IsValid():
            omni.kit.commands.execute("DeletePrims", paths=[LIDAR_PRIM_PATH])
            await app.next_update_async()

        omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/lidar",                      # relative — will land at LIDAR_PRIM_PATH
            parent="/World/forklift_b/body",    # mounts on rigid body → moves with forklift
            min_range=0.5,                      # metres — low global floor; per-sector software floors handle self-hits
            max_range=8.0,                      # metres — detection horizon
            draw_lines=LIDAR_DRAW_LINES,        # coloured rays in viewport
            horizontal_fov=360.0,               # full ring
            horizontal_resolution=1.0,          # 1 ray per degree → 360 rays total
            vertical_fov=0.0,                   # 2D single ring
            rotation_rate=0.0,                  # sync to physics step, not spinning
        )
        await app.next_update_async()           # one frame for prim to initialise

        try:
            from omni.isaac.range_sensor import _range_sensor
            lidar_if = _range_sensor.acquire_lidar_sensor_interface()
            _log("info", "LIDAR sensor created and interface acquired", diag)
        except Exception as exc:
            _log("warn", f"LIDAR interface unavailable: {exc}", diag)
            lidar_if = None

    # ── Init heading state ─────────────────────────────────────────────────────
    fx, fy, raw_yaw = _get_world_transform(forklift_prim)
    smooth_heading  = raw_yaw
    prev_heading_err = 0.0

    _log("info", f"Phase 3 patrol START -- {len(WAYPOINTS)} waypoints, looping forever", diag)

    frame      = 0
    wp_index   = 0
    lap        = 0
    lidar_trigger_count = 0      # consecutive frames with obstacle in forward cone
    lidar_clear_count   = 0      # consecutive frames body has been clear (exit-side debounce)
    lidar_stopped       = False   # True while in full-stop state (transition logging)
    lidar_avoiding      = False   # True while in active avoidance maneuver (transition logging)
    avoid_dir           = None    # 'left' or 'right' — avoidance steer direction; persists until body clears
    obstacle_world_pos  = None    # (x, y) world position of obstacle recorded when avoidance starts
    obs_along           = 0.0     # projection of obstacle onto forklift forward axis (negative = behind FL)
    lidar_min_ahead     = 9.9     # nearest valid forward hit this frame

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
            _log("info", f"WP {wp_index} reached: ({wx},{wy})  lap={lap}", diag)
            wp_index += 1
            if wp_index >= len(WAYPOINTS):
                wp_index = 0
                lap += 1
                _log("info", f"Lap {lap} complete -- looping", diag)
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

        # ── LIDAR obstacle check with sector-aware avoidance ──────────────────
        obstacle_ahead  = False
        avoid_active    = False
        lidar_slow      = False
        lidar_min_ahead = 9.9
        left_min = center_min = right_min = 9.9
        obs_along = 0.0
        if LIDAR_ENABLED and lidar_if is not None:
            try:
                depths = lidar_if.get_linear_depth_data(LIDAR_PRIM_PATH)
                if depths is not None and depths.size >= 360:
                    flat = [float(d) for d in depths.flat]
                    n    = len(flat)

                    # Three sub-sectors within the forward cone (indices wrap mod n)
                    c_lo = LIDAR_FORWARD_RAY - LIDAR_CENTER_HALF   # centre sector
                    c_hi = LIDAR_FORWARD_RAY + LIDAR_CENTER_HALF
                    l_lo = LIDAR_FORWARD_RAY - LIDAR_CONE_HALF     # lo-index sector
                    l_hi = c_lo - 1
                    r_lo = c_hi + 1                                 # hi-index sector
                    r_hi = LIDAR_FORWARD_RAY + LIDAR_CONE_HALF

                    l_rays = [flat[i % n] for i in range(l_lo, l_hi + 1)]
                    l_filt = [d for d in l_rays if math.isfinite(d) and d > LIDAR_MIN_VALID and d < 8.0]
                    left_min = min(l_filt) if l_filt else 9.9

                    c_rays = [flat[i % n] for i in range(c_lo, c_hi + 1)]
                    c_filt = [d for d in c_rays if math.isfinite(d) and d > LIDAR_MIN_VALID and d < 8.0]
                    center_min = min(c_filt) if c_filt else 9.9

                    r_rays = [flat[i % n] for i in range(r_lo, r_hi + 1)]
                    r_filt = [d for d in r_rays if math.isfinite(d) and d > LIDAR_MIN_VALID and d < 8.0]
                    right_min = min(r_filt) if r_filt else 9.9

                    lidar_min_ahead = min(left_min, center_min, right_min)

                    # Compute how far the tracked obstacle is along the forklift's forward axis
                    # Negative = obstacle is behind the forklift body centre
                    if obstacle_world_pos is not None:
                        _hr   = math.radians(smooth_heading)
                        _fwd  = (math.cos(_hr), math.sin(_hr))
                        _to   = (obstacle_world_pos[0] - fx, obstacle_world_pos[1] - fy)
                        obs_along = _fwd[0]*_to[0] + _fwd[1]*_to[1]
                    body_cleared = (obs_along < -FORKLIFT_BODY_HALF_LENGTH) or (obstacle_world_pos is None)

                    if lidar_min_ahead < LIDAR_STOP_DIST:
                        lidar_trigger_count += 1
                        lidar_clear_count    = 0
                    else:
                        lidar_trigger_count  = 0
                        if body_cleared:
                            lidar_clear_count += 1
                            if lidar_clear_count >= LIDAR_AVOID_HOLD_FRAMES:
                                avoid_dir           = None
                                obstacle_world_pos  = None
                        else:
                            lidar_clear_count = 0   # wait for body to pass obstacle before counting down

                    lidar_slow = (lidar_min_ahead < LIDAR_SLOW_DIST)

                    if lidar_trigger_count >= LIDAR_DEBOUNCE_FRAMES and abs(heading_err) < LIDAR_AVOID_TURN_INHIBIT:
                        # Only trigger avoidance when driving nearly straight — not during waypoint turns
                        # (during turns the cone sweeps across warehouse structure, causing false triggers)
                        if avoid_dir is None:
                            if left_min > right_min + LIDAR_AVOID_MARGIN:
                                avoid_dir = "left"    # right side blocked → steer left
                            elif right_min > left_min + LIDAR_AVOID_MARGIN:
                                avoid_dir = "right"   # left side blocked → steer right
                            # else: symmetric block → avoid_dir stays None → full stop

                            if avoid_dir is not None:
                                # Record obstacle world position once (used for body-clearance tracking)
                                _hr = math.radians(smooth_heading)
                                obstacle_world_pos = (
                                    fx + math.cos(_hr) * lidar_min_ahead,
                                    fy + math.sin(_hr) * lidar_min_ahead,
                                )

                        if avoid_dir is not None:
                            avoid_active = True
                        else:
                            obstacle_ahead = True     # symmetric block — stop and wait

                    # Keep avoidance alive after obstacle leaves forward cone until body clears
                    # But not during wide turns — let the PD controller navigate the turn
                    if avoid_dir is not None and not body_cleared and not avoid_active and abs(heading_err) < LIDAR_AVOID_TURN_INHIBIT:
                        avoid_active = True
            except Exception as exc:
                _log("warn", f"LIDAR read error: {exc}", diag)

        # ── LIDAR state-change logging (only on transitions) ──────────────────
        if obstacle_ahead and not lidar_stopped:
            _log("warn", f"LIDAR STOP — obstacle {lidar_min_ahead:.2f}m (L={left_min:.1f} C={center_min:.1f} R={right_min:.1f})", diag)
            lidar_stopped  = True
            lidar_avoiding = False
        elif avoid_active and not lidar_avoiding:
            _log("info", f"LIDAR AVOID {avoid_dir.upper()} — {lidar_min_ahead:.2f}m (L={left_min:.1f} C={center_min:.1f} R={right_min:.1f} along={obs_along:.1f}m)", diag)
            lidar_avoiding = True
            lidar_stopped  = False
        elif not obstacle_ahead and not avoid_active and (lidar_stopped or lidar_avoiding):
            if lidar_clear_count >= LIDAR_AVOID_HOLD_FRAMES:
                _log("info", f"LIDAR RESUME — path clear (nearest: {lidar_min_ahead:.2f}m)", diag)
                lidar_stopped  = False
                lidar_avoiding = False

        # ── Drive ─────────────────────────────────────────────────────────────
        if obstacle_ahead:
            target_vel  = 0.0                          # full stop — symmetric block or both sides closed
            final_steer = steer_cmd
        elif avoid_active:
            target_vel  = DRIVE_VELOCITY * LIDAR_AVOID_SPEED
            # avoid_dir='left'  → steer left  (negative PD convention)
            # avoid_dir='right' → steer right (positive PD convention)
            # If avoidance physically steers the wrong way, swap the sign below
            final_steer = -LIDAR_AVOID_STEER if avoid_dir == "left" else LIDAR_AVOID_STEER
        elif lidar_slow:
            target_vel  = DRIVE_VELOCITY * 0.5         # half speed in slow zone
            final_steer = steer_cmd
        else:
            scale       = min(1.0, frame / RAMP_FRAMES)
            target_vel  = DRIVE_VELOCITY * scale       # full speed
            final_steer = steer_cmd
        drive_api.GetTargetVelocityAttr().Set(target_vel)
        steer_api.GetTargetPositionAttr().Set(-final_steer)  # negated: joint localRot1 90° offset inverts steer direction

        # ── Diag every 60 frames ──────────────────────────────────────────────
        if frame % 60 == 0:
            msg = (
                f"frame={frame:5d}  pos=({fx:.1f},{fy:.1f})  "
                f"hdg={smooth_heading:.1f}  target={target_hdg:.1f}  "
                f"err={heading_err:+.1f}  steer={steer_cmd:+.1f}  "
                f"dist={dist:.1f}m  wp={wp_index}  lap={lap}"
                + ("  LIDAR_STOP" if obstacle_ahead else (f"  LIDAR_AVOID(along={obs_along:.1f}m)" if avoid_active else ("  LIDAR_SLOW" if lidar_slow else "")))
            )
            _log("info", msg)
            diag.write(msg + "\n")

        frame += 1
        await app.next_update_async()
    diag.write("ARRIVED — controller stopped\n")
    diag.close()
    _log("info", "All waypoints reached. Stopped.")


# ── Task management ───────────────────────────────────────────────────────────

_TASK_KEY = "_forklift_controller_task"
_existing = getattr(asyncio.get_event_loop(), _TASK_KEY, None)
if _existing and not _existing.done():
    _existing.cancel()

_task = asyncio.ensure_future(run_forklift())
setattr(asyncio.get_event_loop(), _TASK_KEY, _task)
