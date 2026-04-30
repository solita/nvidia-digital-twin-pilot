"""
config.py — All tunable constants for the forklift warehouse simulation.

Edit values here to tune vehicle physics, sensor calibration, waypoints, or
output paths.  No logic lives here — safe to change without touching nav or
sensor code.

Dev owner: anyone — no Isaac Sim knowledge required to tune these.
"""

# ── USD prim paths ─────────────────────────────────────────────────────────────

DRIVE_JOINT_PATH = "/World/forklift_b/back_wheel_joints/back_wheel_drive"
STEER_JOINT_PATH = "/World/forklift_b/back_wheel_joints/back_wheel_swivel"
FORKLIFT_PRIM    = "/World/forklift_b/body"   # physics rigid body

# ── Vehicle physics ────────────────────────────────────────────────────────────

DRIVE_VELOCITY = -550.0   # deg/s wheel spin; negative = forks-forward (body -X)
SETTLE_FRAMES  =  60      # physics-settle frames before driving
RAMP_FRAMES    =  60      # ramp from 0 → full speed to avoid torque spike

# Steer joint — values validated against this forklift mesh in Phase 1
STEER_STIFFNESS_SETTLE = 20_000.0   # snaps wheel straight while stationary
STEER_STIFFNESS_DRIVE  = 40_000.0   # resists caster drift under load
STEER_DAMPING          = 10_000.0   # prevents oscillation

# ── Heading PD controller ──────────────────────────────────────────────────────

STEER_KP       =  0.30   # proportional gain (deg steer per deg heading error)
STEER_KD       =  0.10   # derivative gain (dampens overshoot)
STEER_DEADBAND =  2.5    # deg — ignore errors smaller than this (prevents hunting)
STEER_MAX      = 30.0    # deg — clamp steer command to ±30°
HEADING_SMOOTH =  0.40   # EMA factor for heading (0=frozen, 1=raw)

# ── Patrol waypoints ───────────────────────────────────────────────────────────
# Designed from warehouse spatial info (see 04_helper_scripts/get_warehouse_spatial_info.py).
#
# Key obstacles used in layout:
#   West rack:  X = -30.27 to -27.02,  Y = -12.06 to +36.38
#   Columns:    X = -27.16, -4.37, +8.41, +26.52  (0.60 m wide)
#   FL width:   3.03 m (half = 1.52 m)
#
# Aisle centres chosen:
#   South cross  Y = -26  (10 m above south wall, avoids cube at Y=-33)
#   East aisle   X = +17  (between columns X=8.41 and X=26.52)
#   North cross  Y = +48  (clears racks north end Y=36.38)
#   West-centre  X = -24  (1.50 m clearance from rack east edge X=-27.02)

WAYPOINTS = [
    ( -8.0, -26.0),   # WP0: south end
    ( 17.0, -26.0),   # WP1: south-east
    ( 17.0,  48.0),   # WP2: north-east
    (-24.0,  48.0),   # WP3: north-west
    (-24.0, -26.0),   # WP4: south-west
    ( -8.0, -17.5),   # WP5: start zone → loops back to WP0
]

ARRIVAL_RADIUS = 2.5   # metres — advance when within this distance of a waypoint

# ── LIDAR sensor ───────────────────────────────────────────────────────────────

LIDAR_ENABLED      = True
LIDAR_PRIM_PATH    = "/World/forklift_b/body/lidar"

# Detection distances
LIDAR_HARD_STOP_DIST =  2.5   # m — full stop; obstacle is contact-close
LIDAR_STOP_DIST      =  5.5   # m — debounced forward stop
LIDAR_SLOW_DIST      =  8.0   # m — start proportional speed ramp

# Sensor geometry
LIDAR_DRAW_LINES      = False  # show coloured rays in the Isaac Sim viewport
LIDAR_FORWARD_RAY     = 359   # ray index pointing toward forks (body -X direction)
LIDAR_CONE_HALF       =  20   # half-width of forward detection cone (rays = degrees)
LIDAR_MIN_VALID       = 0.80  # m — software noise floor for forward cone

# Hit-count thresholds (two-tier, eliminates fork-tine ghost hits)
LIDAR_MIN_HIT_COUNT_NEAR =  4   # rays needed in 0.8–4.5 m zone
LIDAR_MIN_HIT_COUNT_FAR  = 10   # rays needed in 4.5–5.5 m zone (fork tines fill ~8)
LIDAR_FAR_FLOOR          = 4.5  # m — boundary between near and far zones

# Debounce
LIDAR_DEBOUNCE_FRAMES = 5    # consecutive STOP frames before triggering
LIDAR_FWDSTOP_SPEED   = 0.40 # fractional speed at STOP_DIST (ramp floor)

# APF lateral repulsion
LIDAR_REPULSE_GAIN  = 10.0  # deg·m — 1/d gain constant
LIDAR_REPULSE_RANGE =  3.5  # m — max range contributing repulsion
LIDAR_REPULSE_ARC   = 130   # deg — ±arc from forward scanned (excludes rear)
LIDAR_AVOID_STEER   =  8.0  # deg — open-side directional bias
LIDAR_SIDE_BACK_RANGE = 4.0 # m — detection range cap outside forward cone

# ── Forklift self-hit bounding box ─────────────────────────────────────────────
# Half-extents from get_warehouse_spatial_info.py plus 0.15 m noise margin.
# LIDAR returns inside this box are self-hits on the forklift's own collider.
#   Forks axis (body X):  3.031 / 2 = 1.516 m
#   Lateral    (body Y):  1.130 / 2 = 0.565 m

FORKLIFT_SAFE_HALF_FORKS   = 1.516 + 0.15
FORKLIFT_SAFE_HALF_LATERAL = 0.565 + 0.15

# ── Stuck detection & escape ───────────────────────────────────────────────────

STUCK_CHECK_FRAMES  = 180   # no movement for this many frames → escape maneuver
STUCK_ESCAPE_FRAMES =  80   # escape duration: 40 reverse + 40 forward-with-steer
STUCK_MIN_MOVE      = 0.10  # m — minimum displacement to reset stuck counter

# ── Output file paths (inside Isaac Sim container) ────────────────────────────

_OUT = (
    "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot/"
    "simulations/forklift-warehouse/04_current_outputs"
)
DIAG_LOG   = f"{_OUT}/forklift_diag.txt"
STATE_JSON = f"{_OUT}/forklift_state.json"

# ── HTTP control server ────────────────────────────────────────────────────────
# aiohttp server started by forklift_controller.py.  Dashboard POSTs to this.
# Container uses --network=host so localhost:CMD_SERVER_PORT is directly
# reachable from the host without any docker port mapping changes.

CMD_SERVER_PORT = 8081

# ── Forklift rest / spawn pose ─────────────────────────────────────────────────
# Where the forklift is placed before each controller run.
# X=-15: open centre-west area between columns at X=-27 and X=-4.
FORKLIFT_ROOT_PRIM = "/World/forklift_b"
REST_X       = -15.0
REST_Y       = -17.5
REST_Z       =   0.0
REST_HEADING =  90.0   # forks-forward: local -X points south toward WP0
