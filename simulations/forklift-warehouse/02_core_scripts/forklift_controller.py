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
LIDAR_STOP_DIST    = 5.5    # metres — debounced forward stop (wider zone gives 2.7m turning window above 2.80m mast floor)
LIDAR_SLOW_DIST    = 7.0    # metres — slow to half speed; also activates early open-side bias in this zone
LIDAR_DRAW_LINES   = False   # set False to disable viewport ray visualisation
LIDAR_FORWARD_RAY  = 179    # ray index pointing forward — calibrated from live diag (cube ahead → ray 179)
LIDAR_CONE_HALF    = 20     # half-width of forward detection cone in rays (degrees) — narrow to avoid self-hits during turns
LIDAR_MIN_VALID    = 2.80   # metres — software floor for FORWARD cone (mast tip reads up to 2.72m with physics jitter)
LIDAR_REPULSE_GAIN  = 10.0  # deg·m — lateral repulsion gain for 1/d formula (K/d per sector)
LIDAR_REPULSE_RANGE =  3.5  # m — max distance at which a sector obstacle contributes repulsion (shorter = less wall interference with PD heading correction)
LIDAR_REPULSE_ARC   =  130  # deg — ±arc from forward scanned for lateral sectors (excludes directly behind)
LIDAR_AVOID_STEER   =  20.0 # deg — open-side directional bias applied when forward stop is confirmed
LIDAR_FWDSTOP_SPEED =  0.40 # fraction of DRIVE_VELOCITY in confirmed STOP (higher = more turning authority per second)
LIDAR_DEBOUNCE_FRAMES = 5   # consecutive frames forward obstacle must persist before triggering STOP
STUCK_CHECK_FRAMES  =  180  # frames with no movement → trigger escape maneuver
STUCK_ESCAPE_FRAMES =  120  # total escape: 60 frames reverse (break contact) + 60 frames forward (steer clear)
STUCK_MIN_MOVE      =  0.10 # m — minimum displacement per STUCK_CHECK_FRAMES to reset counter

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
    forward_min            = 9.9   # nearest validated forward-cone hit this frame
    repulsion_steer        = 0.0   # APF lateral repulsion this frame (deg, added to PD steer)
    lidar_fwd_stop            = False # forward obstacle confirmed for LIDAR_DEBOUNCE_FRAMES frames
    lidar_fwd_slow            = False # forward obstacle within LIDAR_SLOW_DIST
    lidar_fwd_debounce_count  = 0     # rising counter: increments each STOP frame
    lidar_fwd_clear_count     = 0     # consecutive clear frames (hysteresis for debounce decay)
    lidar_log_state           = ""    # last logged LIDAR state — for transition-only messages
    stuck_check_pos           = None  # (x, y) snapshot for stuck detection
    stuck_frames              = 0     # consecutive frames with no meaningful movement
    stuck_escape_frames       = 0     # countdown: >0 = executing escape maneuver

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

        # ── LIDAR: forward stop/slow + APF repulsion steer ───────────────────
        lidar_fwd_stop  = False
        lidar_fwd_slow  = False
        repulsion_steer = 0.0
        forward_min     = 9.9

        if LIDAR_ENABLED and lidar_if is not None:
            try:
                depths = lidar_if.get_linear_depth_data(LIDAR_PRIM_PATH)
                if depths is not None and depths.size >= 360:
                    flat = [float(d) for d in depths.flat]
                    n    = len(flat)

                    # Forward cone: minimum range for stop/slow (mast self-hit floor applied)
                    fwd_hits = [
                        flat[i % n]
                        for i in range(LIDAR_FORWARD_RAY - LIDAR_CONE_HALF,
                                       LIDAR_FORWARD_RAY + LIDAR_CONE_HALF + 1)
                        if math.isfinite(flat[i % n]) and flat[i % n] > LIDAR_MIN_VALID and flat[i % n] < 8.0
                    ]
                    forward_min = min(fwd_hits) if fwd_hits else 9.9

                    # Hysteresis debounce: count up immediately on each STOP frame,
                    # but only count DOWN after 5 consecutive CLEAR frames.
                    # Prevents sensor jitter from flapping the control state.
                    if forward_min < LIDAR_STOP_DIST:
                        lidar_fwd_debounce_count = min(lidar_fwd_debounce_count + 1, LIDAR_DEBOUNCE_FRAMES)
                        lidar_fwd_clear_count    = 0
                    else:
                        lidar_fwd_clear_count += 1
                        if lidar_fwd_clear_count >= 5:
                            lidar_fwd_debounce_count = max(lidar_fwd_debounce_count - 1, 0)
                    lidar_fwd_stop = (lidar_fwd_debounce_count >= LIDAR_DEBOUNCE_FRAMES)
                    lidar_fwd_slow = (forward_min < LIDAR_SLOW_DIST)

                    # APF sector repulsion: find the CLOSEST obstacle in the left and right
                    # lateral hemispheres. Use 1/d per sector (not a per-ray sum) so repulsion
                    # stays bounded even in a warehouse full of continuous walls.
                    #
                    # offset_deg > 0  → obstacle is to FL's RIGHT → push left (reduce steer)
                    # offset_deg < 0  → obstacle is to FL's LEFT  → push right (increase steer)
                    left_min_repulse  = 9.9   # closest hit in left-side arc
                    right_min_repulse = 9.9   # closest hit in right-side arc
                    for i in range(n):
                        d = flat[i]
                        if not math.isfinite(d) or d > LIDAR_REPULSE_RANGE or d < 0.5:
                            continue
                        offset_deg = ((i - LIDAR_FORWARD_RAY) + 180) % 360 - 180
                        # Skip directly ahead (handled by fwd stop) and directly behind
                        if abs(offset_deg) < 10 or abs(offset_deg) > LIDAR_REPULSE_ARC:
                            continue
                        # Mast self-hit floor for near-forward rays
                        if abs(offset_deg) <= LIDAR_CONE_HALF and d < LIDAR_MIN_VALID:
                            continue
                        if offset_deg > 0:
                            right_min_repulse = min(right_min_repulse, d)
                        else:
                            left_min_repulse  = min(left_min_repulse,  d)

                    # Net repulsion = push from left obstacle − push from right obstacle
                    r_left  = (LIDAR_REPULSE_GAIN / left_min_repulse)  if left_min_repulse  < LIDAR_REPULSE_RANGE else 0.0
                    r_right = (LIDAR_REPULSE_GAIN / right_min_repulse) if right_min_repulse < LIDAR_REPULSE_RANGE else 0.0
                    repulsion_steer = max(-STEER_MAX, min(STEER_MAX, r_left - r_right))

                    # Open-side selection: scan ±25-80° lateral arcs, pick side with more clear space.
                    # Applied in STOP mode (full bias) and early approach (half bias) for earlier turning.
                    def _open_side_scan():
                        ro_r = min(
                            (flat[i % n] for i in range(LIDAR_FORWARD_RAY + 25, LIDAR_FORWARD_RAY + 80)
                             if math.isfinite(flat[i % n]) and flat[i % n] >= 0.5),
                            default=9.9
                        )
                        ro_l = min(
                            (flat[i % n] for i in range(LIDAR_FORWARD_RAY - 80, LIDAR_FORWARD_RAY - 25)
                             if math.isfinite(flat[i % n]) and flat[i % n] >= 0.5),
                            default=9.9
                        )
                        return ro_r, ro_l

                    if lidar_fwd_stop:
                        ro_right, ro_left = _open_side_scan()
                        open_bias = LIDAR_AVOID_STEER if ro_right >= ro_left else -LIDAR_AVOID_STEER
                        repulsion_steer = max(-STEER_MAX, min(STEER_MAX, repulsion_steer + open_bias))
                    elif lidar_fwd_debounce_count >= 2:
                        # Cube approaching, partially confirmed: directional half-bias for earlier course correction
                        ro_right, ro_left = _open_side_scan()
                        early_bias = (LIDAR_AVOID_STEER * 0.5) if ro_right >= ro_left else -(LIDAR_AVOID_STEER * 0.5)
                        repulsion_steer = max(-STEER_MAX, min(STEER_MAX, repulsion_steer + early_bias))
                    elif lidar_fwd_debounce_count == 1 and abs(repulsion_steer) < 5.0:
                        # Single-frame detection: gentle 8° nudge toward the more open side
                        ro_right, ro_left = _open_side_scan()
                        repulsion_steer = 8.0 if ro_right >= ro_left else -8.0

            except Exception as exc:
                _log("warn", f"LIDAR read error: {exc}", diag)

        # ── LIDAR state logging (transition only) ─────────────────────────────
        new_lidar_state = "STOP" if lidar_fwd_stop else ("SLOW" if lidar_fwd_slow else "")
        if new_lidar_state != lidar_log_state:
            if new_lidar_state == "STOP":
                _log("warn", f"LIDAR FWD STOP — {forward_min:.2f}m  rep={repulsion_steer:+.1f}°", diag)
            elif new_lidar_state == "" and lidar_log_state:
                _log("info", f"LIDAR CLEAR — fwd={forward_min:.2f}m", diag)
            lidar_log_state = new_lidar_state

        # ── Drive (APF-blended) ───────────────────────────────────────────────
        # Stuck detection — halt once on first trigger
        if stuck_check_pos is None:
            stuck_check_pos = (fx, fy)
            stuck_frames    = 0
        else:
            moved = math.hypot(fx - stuck_check_pos[0], fy - stuck_check_pos[1])
            if moved > STUCK_MIN_MOVE:
                stuck_check_pos = (fx, fy)
                stuck_frames    = 0
            else:
                stuck_frames += 1
                if stuck_frames == STUCK_CHECK_FRAMES:
                    _log("warn", f"STUCK — escape maneuver at ({fx:.1f},{fy:.1f}) rep={repulsion_steer:+.1f}°", diag)
                    stuck_escape_frames      = STUCK_ESCAPE_FRAMES
                    stuck_frames             = 0
                    stuck_check_pos          = (fx, fy)
                    lidar_fwd_debounce_count = 0   # reset so LIDAR doesn't immediately re-lock after escape
                    lidar_fwd_clear_count    = 0
                    diag.flush()

        # Escape maneuver: two-phase to break contact and steer clear
        # Phase 1 (first half): reverse slowly straight back to break physical contact with column/cube
        # Phase 2 (second half): forward at full speed with max steer toward the open side
        if stuck_escape_frames > 0:
            if stuck_escape_frames > STUCK_ESCAPE_FRAMES // 2:
                # Phase 1: reverse, no steer
                drive_api.GetTargetVelocityAttr().Set(-DRIVE_VELOCITY * 0.40)
                steer_api.GetTargetPositionAttr().Set(0.0)
            else:
                # Phase 2: forward + open-side steer
                escape_steer = STEER_MAX if repulsion_steer >= 0 else -STEER_MAX
                drive_api.GetTargetVelocityAttr().Set(DRIVE_VELOCITY)
                steer_api.GetTargetPositionAttr().Set(-escape_steer)
            stuck_escape_frames -= 1
            frame += 1
            await app.next_update_async()
            continue

        # APF-blended steer: PD attraction (toward waypoint) + repulsion (all obstacles/walls)
        apf_steer = max(-STEER_MAX, min(STEER_MAX, steer_cmd + repulsion_steer))

        if lidar_fwd_stop:
            target_vel  = DRIVE_VELOCITY * LIDAR_FWDSTOP_SPEED
            if abs(heading_err) < 25.0:
                # Driving toward obstacle (not mid-turn): apply STEER_MAX for maximum lateral avoidance
                final_steer = STEER_MAX if repulsion_steer >= 0 else -STEER_MAX
            else:
                # In a waypoint turn: let APF handle it to avoid fighting heading correction
                final_steer = apf_steer
        elif lidar_fwd_slow:
            target_vel  = DRIVE_VELOCITY * 0.5
            final_steer = apf_steer
        else:
            scale       = min(1.0, frame / RAMP_FRAMES)
            target_vel  = DRIVE_VELOCITY * scale
            final_steer = apf_steer
        drive_api.GetTargetVelocityAttr().Set(target_vel)
        steer_api.GetTargetPositionAttr().Set(-final_steer)  # negated: joint localRot1 90° offset inverts steer direction

        # ── Diag every 60 frames ──────────────────────────────────────────────
        if frame % 60 == 0:
            msg = (
                f"frame={frame:5d}  pos=({fx:.1f},{fy:.1f})  "
                f"hdg={smooth_heading:.1f}  target={target_hdg:.1f}  "
                f"err={heading_err:+.1f}  steer={steer_cmd:+.1f}  "
                f"dist={dist:.1f}m  wp={wp_index}  lap={lap}"
                + (f"  LIDAR_STOP(fwd={forward_min:.1f}m rep={repulsion_steer:+.0f}°)" if lidar_fwd_stop
                   else (f"  LIDAR_SLOW(fwd={forward_min:.1f}m rep={repulsion_steer:+.0f}°)" if lidar_fwd_slow
                   else (f"  APF(rep={repulsion_steer:+.0f}°)" if abs(repulsion_steer) > 2.0 else "")))
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
