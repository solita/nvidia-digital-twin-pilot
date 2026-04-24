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
import json
import math
import os

import carb
import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

# ── Configuration ─────────────────────────────────────────────────────────────

# Fleet: all 4 forklift IDs (must match populate_scene.py and warehouse-manager)
FORKLIFT_IDS = ["forklift_0", "forklift_1", "forklift_2", "forklift_3"]

# Legacy single-forklift paths kept for reference; run_forklift() builds per-ID paths.
DRIVE_JOINT_PATH = "/World/forklift_0/back_wheel_joints/back_wheel_drive"
STEER_JOINT_PATH = "/World/forklift_0/back_wheel_joints/back_wheel_swivel"
FORKLIFT_PRIM    = "/World/forklift_0/body"  # physics rigid body — this is what actually moves

DRIVE_VELOCITY = -550.0   # deg/s wheel spin -- negative = forks-forward (body -X direction)
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
#   East aisle   X = +17  (between columns at X=8.41 and X=26.52; centred gives ~9m buffer from east wall)
#   North cross  Y = +55  (above all racks north end Y=36.38 and pillars Y=45.12)
#   West-center  X = -24  (FL west edge -25.52, rack east edge -27.02 -> 1.50m clearance)
#
# REST_HEADING must be -90 deg so DRIVE_VELOCITY=+200 drives straight south to WP0.
WAYPOINTS = [
    ( -8.0, -26.0),   # WP0: south end        -- Y=-26 clears south wall (at Y=-36.4) by 10m, avoids cube at Y=-33
    ( 17.0, -26.0),   # WP1: south-east       -- east in south cross-aisle (X=17: 9m from east column X=26.52)
    ( 17.0,  48.0),   # WP2: north-east       -- actual north wall ~Y=52.3, 4m buffer to turn
    (-24.0,  48.0),   # WP3: north-west       -- west in north cross-aisle
    (-24.0, -26.0),   # WP4: south-west       -- south in west-center aisle, matched to new Y=-26 row
    ( -8.0, -17.5),   # WP5: start zone       -- loops back to WP0
]
ARRIVAL_RADIUS = 2.5   # metres -- advance to next waypoint when within this (tighter = cleaner corners)

DIAG_LOG = (
    "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot/"
    "simulations/forklift-warehouse/04_current_outputs/forklift_diag.txt"
)
STATE_JSON = (
    "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot/"
    "simulations/forklift-warehouse/04_current_outputs/forklift_state.json"
)

# ── LIDAR sensor (2D, single horizontal ring) ─────────────────────────────────
LIDAR_ENABLED      = True
LIDAR_PRIM_PATH    = "/World/forklift_0/body/lidar"  # default; overridden per-forklift in run_forklift()
LIDAR_STOP_DIST    = 5.5    # metres — debounced forward stop
LIDAR_SLOW_DIST    = 8.0    # metres — beginning of proportional speed ramp (widened for forks-side geometry)
LIDAR_DRAW_LINES   = False   # set False to disable viewport ray visualisation
LIDAR_FORWARD_RAY  = 359    # ray index pointing toward forks (body -X) — opposite of ray 179 (body +X)
LIDAR_CONE_HALF    = 20     # half-width of forward detection cone in rays (degrees) — narrow to avoid self-hits during turns
LIDAR_MIN_VALID    = 0.80   # metres — software floor for FORWARD cone (sensor noise floor; was 4.90 which caused blind zone)
LIDAR_MIN_HIT_COUNT_NEAR =  4  # rays needed for close range (0.8–4.5m): narrow column fills ~5 rays → caught
LIDAR_MIN_HIT_COUNT_FAR  = 10  # rays needed for fork-shadow zone (4.5–5.5m): fork tines fill ~8 rays → filtered; 1m cube = ~11 → caught
LIDAR_FAR_FLOOR          = 4.5 # metres — boundary between near and far hit-count thresholds
LIDAR_REPULSE_GAIN  = 10.0  # deg·m — lateral repulsion gain for 1/d formula (K/d per sector)
LIDAR_REPULSE_RANGE =  3.5  # m — max distance at which a sector obstacle contributes repulsion (shorter = less wall interference with PD heading correction)
LIDAR_REPULSE_ARC   =  130  # deg — ±arc from forward scanned for lateral sectors (excludes directly behind)
LIDAR_AVOID_STEER   =   8.0 # deg — open-side directional bias; kept low to prevent aisle drift overwhelming PD heading correction
LIDAR_SIDE_BACK_RANGE = 4.0 # metres — reduced detection range for side/back sectors (half of front 8.0 m)
LIDAR_HARD_STOP_DIST =  2.5 # metres — full stop threshold: very close obstacle, halt immediately
LIDAR_FWDSTOP_SPEED =  0.40 # fraction of DRIVE_VELOCITY at STOP_DIST (ramp floor); overridden to 0 below HARD_STOP_DIST
LIDAR_DEBOUNCE_FRAMES = 5   # consecutive frames forward obstacle must persist before triggering STOP
STUCK_CHECK_FRAMES  =  180  # frames with no movement → trigger escape maneuver
STUCK_ESCAPE_FRAMES =   80  # total escape: 40 frames reverse (break contact) + 40 frames forward (steer clear)
STUCK_MIN_MOVE      =  0.10 # m — minimum displacement per STUCK_CHECK_FRAMES to reset counter

# Forklift bounding-box safe zone — LIDAR hits inside this are self-hits on the
# forklift's own collider/physics mesh and must be ignored.
# Half-extents from get_warehouse_spatial_info.py + 0.15 m noise margin:
#   Forks axis (body X):  3.031/2 = 1.516 m + 0.15 m = 1.67 m
#   Lateral    (body Y):  1.130/2 = 0.565 m + 0.15 m = 0.72 m
FORKLIFT_SAFE_HALF_FORKS   = 1.516 + 0.15
FORKLIFT_SAFE_HALF_LATERAL = 0.565 + 0.15

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


def _compute_lidar_safe_zone():
    """Precompute per-ray minimum distance to exit the forklift bounding box.

    For each of 360 rays, compute how far along that direction the ray must
    travel to leave the forklift's own footprint rectangle.  Any LIDAR depth
    shorter than this value is a self-hit on the forklift's collider mesh.
    """
    safe = []
    for i in range(360):
        alpha_rad = math.radians(((i - LIDAR_FORWARD_RAY) + 180) % 360 - 180)
        ca = abs(math.cos(alpha_rad))
        sa = abs(math.sin(alpha_rad))
        dist_fwd = FORKLIFT_SAFE_HALF_FORKS / ca if ca > 1e-6 else 999.0
        dist_lat = FORKLIFT_SAFE_HALF_LATERAL / sa if sa > 1e-6 else 999.0
        safe.append(min(dist_fwd, dist_lat))
    return safe


_LIDAR_SAFE_ZONE = _compute_lidar_safe_zone()


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

async def run_forklift(forklift_id: str = "forklift_0", start_wp: int = 0) -> None:
    """Drive a single forklift through the patrol loop.

    Args:
        forklift_id: Prim name under /World (e.g. "forklift_0").
        start_wp:    Index into WAYPOINTS to begin the patrol.
    """
    # Per-forklift prim paths
    drive_joint_path = f"/World/{forklift_id}/back_wheel_joints/back_wheel_drive"
    steer_joint_path = f"/World/{forklift_id}/back_wheel_joints/back_wheel_swivel"
    forklift_prim_path = f"/World/{forklift_id}/body"
    lidar_prim_path  = f"/World/{forklift_id}/body/lidar"

    app   = omni.kit.app.get_app()
    stage = omni.usd.get_context().get_stage()

    drive_joint    = stage.GetPrimAtPath(drive_joint_path)
    steer_joint    = stage.GetPrimAtPath(steer_joint_path)
    forklift_prim  = stage.GetPrimAtPath(forklift_prim_path)

    if not drive_joint.IsValid():
        raise RuntimeError(f"[{forklift_id}] Drive joint not found: {drive_joint_path!r}")
    if not steer_joint.IsValid():
        raise RuntimeError(f"[{forklift_id}] Steer joint not found: {steer_joint_path!r}")
    if not forklift_prim.IsValid():
        raise RuntimeError(f"[{forklift_id}] Forklift prim not found: {forklift_prim_path!r}")

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

    # ── Per-forklift diag log and state JSON paths ──────────────────────────────
    diag_log = DIAG_LOG.replace("forklift_diag.txt", f"{forklift_id}_diag.txt")
    state_json = STATE_JSON.replace("forklift_state.json", f"{forklift_id}_state.json")

    os.makedirs(os.path.dirname(diag_log), exist_ok=True)
    diag = open(diag_log, "w", buffering=1)
    diag.write(f"[{forklift_id}] frame, fx, fy, heading, target_hdg, err, steer_cmd\n")

    # ── LIDAR: create sensor programmatically, attached to forklift body ───────
    lidar_if = None
    if LIDAR_ENABLED:
        # Remove stale prim from a previous run (script re-runs without full reload)
        if stage.GetPrimAtPath(lidar_prim_path).IsValid():
            omni.kit.commands.execute("DeletePrims", paths=[lidar_prim_path])
            await app.next_update_async()

        omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/lidar",                      # relative — will land at lidar_prim_path
            parent=f"/World/{forklift_id}/body",  # mounts on rigid body → moves with forklift
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
            _log("info", f"[{forklift_id}] LIDAR sensor created and interface acquired", diag)
        except Exception as exc:
            _log("warn", f"[{forklift_id}] LIDAR interface unavailable: {exc}", diag)
            lidar_if = None

    # ── Init heading state ─────────────────────────────────────────────────────
    fx, fy, raw_yaw = _get_world_transform(forklift_prim)
    smooth_heading  = raw_yaw
    prev_heading_err = 0.0

    _log("info", f"[{forklift_id}] Phase 3 patrol START -- {len(WAYPOINTS)} waypoints, start_wp={start_wp}, looping forever", diag)

    frame      = 0
    wp_index   = start_wp % len(WAYPOINTS)
    lap        = 0
    forward_min            = 9.9   # nearest validated forward-cone hit this frame
    repulsion_steer        = 0.0   # APF lateral repulsion this frame (deg, added to PD steer)
    lidar_fwd_stop            = False # forward obstacle confirmed for LIDAR_DEBOUNCE_FRAMES frames
    lidar_fwd_slow            = False # forward obstacle within LIDAR_SLOW_DIST
    lidar_fwd_debounce_count  = 0     # rising counter: increments each STOP frame
    lidar_fwd_clear_count     = 0     # consecutive clear frames (hysteresis for debounce decay)
    stuck_check_pos           = None  # (x, y) snapshot for stuck detection
    stuck_frames              = 0     # consecutive frames with no meaningful movement
    stuck_escape_frames       = 0     # countdown: >0 = executing escape maneuver
    stuck_wp_count            = 0     # how many escape cycles fired at current wp_index without progress
    lost_heading_count        = 0     # frames where |err| > 150° (heading-loss detector)
    recovering_heading        = False # True while FL is spinning back on-axis

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
            stuck_wp_count = 0   # reset per-WP stuck counter on successful arrival
            wp_index += 1
            if wp_index >= len(WAYPOINTS):
                wp_index = 0
                lap += 1
                _log("info", f"Lap {lap} complete -- looping", diag)
            continue

        # ── Heading error: direction to waypoint vs current heading ───────────
        target_hdg = math.degrees(math.atan2(wy - fy, wx - fx))
        # Forks are the forward direction (body -X), which is 180° from the body +X axis
        # that smooth_heading tracks. Offset by 180° to get the true forks heading.
        heading_err = _angle_diff(smooth_heading + 180.0, target_hdg)  # forks-forward: flipped args so positive err → negative steer_cmd → Set(+) → CCW

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
        lidar_slices    = [False] * 9  # [FL,FC,FR, RF,RB, BR,BL, LB,LF] for dashboard pie chart

        if LIDAR_ENABLED and lidar_if is not None:
            try:
                depths = lidar_if.get_linear_depth_data(lidar_prim_path)
                if depths is not None and depths.size >= 360:
                    flat = [float(d) for d in depths.flat]
                    n    = len(flat)

                    # ── Self-hit filter: discard any depth inside the forklift bbox ──
                    for _si in range(n):
                        if flat[_si] < _LIDAR_SAFE_ZONE[_si % 360]:
                            flat[_si] = float('inf')

                    # ── Side/back range cap: halve detection range outside front cone ──
                    for _si in range(n):
                        _off = ((_si - LIDAR_FORWARD_RAY) + 180) % 360 - 180
                        if abs(_off) > LIDAR_CONE_HALF and flat[_si] > LIDAR_SIDE_BACK_RANGE:
                            flat[_si] = float('inf')

                    # Emergency close-range check: bypass MIN_HIT_COUNT/MIN_VALID for very near obstacles.
                    # 2 rays in tight ±6° cone reading <2.5m = real physical contact threat regardless of fork shadow.
                    emerg_hits = [
                        flat[i % n]
                        for i in range(LIDAR_FORWARD_RAY - 6, LIDAR_FORWARD_RAY + 7)
                        if math.isfinite(flat[i % n]) and 0.3 < flat[i % n] < 2.5
                    ]
                    if len(emerg_hits) >= 2:
                        forward_min = min(emerg_hits)  # overrides the main cone check below

                    # Forward cone: two-tier hit-count threshold.
                    # Near zone (0.8–4.5m): narrow rack column fills ~5 rays → threshold=4.
                    # Far zone  (4.5–5.5m): fork tines fill ~8 rays → threshold=10; 1m cube=~11 → caught.
                    fwd_hits = [
                        flat[i % n]
                        for i in range(LIDAR_FORWARD_RAY - LIDAR_CONE_HALF,
                                       LIDAR_FORWARD_RAY + LIDAR_CONE_HALF + 1)
                        if math.isfinite(flat[i % n]) and flat[i % n] > LIDAR_MIN_VALID and flat[i % n] < 8.0
                    ]
                    near_stop = [d for d in fwd_hits if d < min(LIDAR_STOP_DIST, LIDAR_FAR_FLOOR)]
                    far_stop  = [d for d in fwd_hits if LIDAR_FAR_FLOOR <= d < LIDAR_STOP_DIST]
                    if len(near_stop) >= LIDAR_MIN_HIT_COUNT_NEAR or len(far_stop) >= LIDAR_MIN_HIT_COUNT_FAR:
                        forward_min = min(forward_min, min(fwd_hits))  # take the closer of emergency or main cone

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

                    # Pie-chart slice obstacle flags for dashboard
                    # 9 slices: front×3, right×2, back×2, left×2
                    for i in range(n):
                        d = flat[i]
                        if not math.isfinite(d) or d <= 0.80 or d >= 8.0:
                            continue
                        offset = ((i - LIDAR_FORWARD_RAY) + 180) % 360 - 180
                        # Front zone (±20°): skip fork self-hits
                        if -20 <= offset <= 20:
                            if d < LIDAR_MIN_VALID:
                                continue
                            if offset < -6.67:
                                lidar_slices[0] = True   # front-left
                            elif offset <= 6.67:
                                lidar_slices[1] = True   # front-center
                            else:
                                lidar_slices[2] = True   # front-right
                        elif 20 < offset <= 73.33:
                            lidar_slices[3] = True   # right-front
                        elif 73.33 < offset <= 126.67:
                            lidar_slices[4] = True   # right-back
                        elif offset > 126.67:
                            lidar_slices[5] = True   # back-right
                        elif offset < -126.67:
                            lidar_slices[6] = True   # back-left
                        elif -126.67 <= offset < -73.33:
                            lidar_slices[7] = True   # left-back
                        else:
                            lidar_slices[8] = True   # left-front

            except Exception as exc:
                _log("warn", f"LIDAR read error: {exc}", diag)

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
                    stuck_wp_count += 1
                    if stuck_wp_count >= 5:
                        # Completely blocked at this WP — skip to next and reset counter
                        _log("warn", f"STUCK x5 at WP{wp_index} ({fx:.1f},{fy:.1f}) — skipping waypoint", diag)
                        wp_index = (wp_index + 1) % len(WAYPOINTS)
                        stuck_wp_count = 0
                        stuck_frames   = 0
                        stuck_check_pos = (fx, fy)
                        diag.flush()
                    else:
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
                # Phase 1: reverse (away from forks = positive velocity), no steer
                drive_api.GetTargetVelocityAttr().Set(abs(DRIVE_VELOCITY) * 0.40)
                steer_api.GetTargetPositionAttr().Set(0.0)
            else:
                # Phase 2: forward + open-side steer.
                # If repulsion is negligible (no lateral obstacle detected), use heading_err
                # to steer toward the target waypoint direction instead of a blind sign-flip.
                if abs(repulsion_steer) < 1.0:
                    escape_steer = STEER_MAX if heading_err > 0 else -STEER_MAX
                else:
                    escape_steer = STEER_MAX if repulsion_steer >= 0 else -STEER_MAX
                drive_api.GetTargetVelocityAttr().Set(DRIVE_VELOCITY)
                steer_api.GetTargetPositionAttr().Set(-escape_steer)  # forks-forward: negated
            stuck_escape_frames -= 1
            frame += 1
            await app.next_update_async()
            continue

        # ── Heading recovery: if err ≈ ±180° the PD steer oscillates and the FL drives away ─
        # Creep forward at low speed + full steer until back within 90° of target.
        if recovering_heading:
            if abs(heading_err) < 90.0:
                recovering_heading = False
                lost_heading_count = 0
            else:
                # heading_err < 0 (new convention) = forks need CCW = want Set(+) = -spin must be + = spin must be -
                spin_steer = -STEER_MAX if heading_err < 0 else STEER_MAX
                drive_api.GetTargetVelocityAttr().Set(DRIVE_VELOCITY * 0.20)
                steer_api.GetTargetPositionAttr().Set(-spin_steer)
                frame += 1
                await app.next_update_async()
                continue
        elif abs(heading_err) > 150.0:
            lost_heading_count += 1
            if lost_heading_count >= 10:
                _log("warn", f"HEADING LOST — recovery spin at ({fx:.1f},{fy:.1f}) err={heading_err:+.1f}°", diag)
                recovering_heading = True
        else:
            lost_heading_count = max(0, lost_heading_count - 2)

        # APF-blended steer: PD attraction (toward waypoint) + repulsion (all obstacles/walls)
        apf_steer = max(-STEER_MAX, min(STEER_MAX, steer_cmd + repulsion_steer))

        # Turn-speed limiter: reduce speed when heading error is large so the FL
        # covers less ground per frame while steering through sharp turns/waypoints.
        # 100% when |err|<=15°, ramps to 50% at |err|>=60°.  Multiplies all drive paths.
        turn_scale = max(0.5, 1.0 - max(0.0, abs(heading_err) - 15.0) / 90.0)

        if lidar_fwd_stop or lidar_fwd_slow:
            # Full stop when very close: prevents crawling into walls/columns.
            # Proportional ramp between HARD_STOP_DIST and SLOW_DIST.
            if forward_min < LIDAR_HARD_STOP_DIST:
                speed_frac = 0.0
            else:
                t          = max(0.0, min(1.0, (forward_min - LIDAR_STOP_DIST) / max(LIDAR_SLOW_DIST - LIDAR_STOP_DIST, 0.1)))
                speed_frac = LIDAR_FWDSTOP_SPEED + t * (1.0 - LIDAR_FWDSTOP_SPEED)
            target_vel = DRIVE_VELOCITY * speed_frac * turn_scale
            # Always use APF steer — STEER_MAX override caused spinning when false stop fired
            final_steer = apf_steer
        else:
            scale       = min(1.0, frame / RAMP_FRAMES)
            target_vel  = DRIVE_VELOCITY * scale * turn_scale
            final_steer = apf_steer
        drive_api.GetTargetVelocityAttr().Set(target_vel)
        steer_api.GetTargetPositionAttr().Set(-final_steer)  # forks-forward: negated (joint axis convention)

        # ── State JSON every 10 frames (dashboard data source) ──────────────
        if frame % 10 == 0:
            lidar_tag = (
                "STOP" if lidar_fwd_stop else
                "SLOW" if lidar_fwd_slow else
                "CLEAR"
            )
            state = {
                "forklift_id":  forklift_id,
                "frame":        frame,
                "x":            round(fx, 2),
                "y":            round(fy, 2),
                "heading":      round(smooth_heading, 1),
                "target_hdg":   round(target_hdg, 1),
                "heading_err":  round(heading_err, 1),
                "wp":           wp_index,
                "lap":          lap,
                "dist_to_wp":   round(dist, 2),
                "lidar_state":  lidar_tag,
                "forward_min":  round(forward_min, 2),
                "repulsion":    round(repulsion_steer, 1),
                "speed_frac":   round(target_vel / DRIVE_VELOCITY, 2),
                "lidar_slices": lidar_slices,
                "waypoints":    WAYPOINTS,
            }
            try:
                with open(state_json, "w") as _sf:
                    json.dump(state, _sf)
            except Exception:
                pass

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
    diag.write(f"[{forklift_id}] ARRIVED — controller stopped\n")
    diag.close()
    _log("info", f"[{forklift_id}] All waypoints reached. Stopped.")


# ── Task management — launch all 4 forklifts ─────────────────────────────────

for _fid in FORKLIFT_IDS:
    _TASK_KEY = f"_forklift_controller_task_{_fid}"
    _existing = getattr(asyncio.get_event_loop(), _TASK_KEY, None)
    if _existing and not _existing.done():
        _existing.cancel()

for _i, _fid in enumerate(FORKLIFT_IDS):
    _TASK_KEY = f"_forklift_controller_task_{_fid}"
    _task = asyncio.ensure_future(run_forklift(_fid, start_wp=_i % len(WAYPOINTS)))
    setattr(asyncio.get_event_loop(), _TASK_KEY, _task)
