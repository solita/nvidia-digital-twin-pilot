# shotcrete_microburst_v16_proxy_receiver_visual_baseline_full_run_metrics.py
# -----------------------------------------------------------------------------
# Continuous / quasi-continuous injector + face-locked, aim-zone freeze deposit.
#
# Purpose:
#   Improve the earlier freeze-on-impact prototype by:
#   1) freezing only on one EXPLICIT receiver face
#   2) freezing only inside a compact aim-centered zone on that face
#   3) projecting deposited particles onto that face plane
#   4) adding simple per-cell stacking along the face normal
#
# Workflow:
#   1. Open CAVE_clean_master.usdc
#   2. Keep /CAVE_LOWPOLY/temp_collider as receiver
#   3. Keep cave collider off
#   4. Stop
#   5. Double-click this script
#   6. Play
#
# Notes:
#   - This is a pragmatic PoC prototype, not a final physical model.
#   - The freeze trigger is geometric and face-locked, not contact-report based.
#   - Treat only the first fresh run from CAVE_clean_master as valid evidence.
# -----------------------------------------------------------------------------

import builtins
import datetime
import hashlib
import json
import math
import os
import random
import shutil

import omni
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdShade, Vt

# -----------------------------------------------------------------------------
# Scene paths
# -----------------------------------------------------------------------------

NOZZLE_LOCATOR_PATH = "/CAVE_LOWPOLY/NozzleLocator"
AIM_LOCATOR_PATH = "/CAVE_LOWPOLY/AimLocator"
RECEIVER_PATH = "/CAVE_LOWPOLY/temp_collider"

RIG_ROOT = "/ShotcreteCaveRig"
DYNAMIC_ROOT = f"{RIG_ROOT}/Bursts"
DEPOSIT_ROOT = f"{RIG_ROOT}/Deposited"
PARTICLE_SYSTEM_PATH = f"{RIG_ROOT}/particleSystem0"
PBD_MATERIAL_PATH = f"{RIG_ROOT}/pbdParticleMaterial"
NOZZLE_MARKER_PATH = f"{RIG_ROOT}/NozzleMarker"
AIM_MARKER_PATH = f"{RIG_ROOT}/AimMarker"
DYNAMIC_PROTO_PATH = f"{RIG_ROOT}/DynamicSphereProto"
STATIC_PROTO_PATH = f"{RIG_ROOT}/StaticSphereProto"
DEPOSIT_PI_PATH = f"{DEPOSIT_ROOT}/DepositPI"

# -----------------------------------------------------------------------------
# Run / state keys
# -----------------------------------------------------------------------------

STATE_KEY = "_SHOTCRETE_FACE_LOCKED_SPLAT_METRICS_STATE"
SUB_KEY = "_SHOTCRETE_FACE_LOCKED_SPLAT_METRICS_SUB"

OLD_KEYS = [
    "_SHOTCRETE_CONT_FREEZE_STATE",
    "_SHOTCRETE_CONT_FREEZE_SUB",
    "_SHOTCRETE_AB_STATE",
    "_SHOTCRETE_AB_SUB",
    "_SHOTCRETE_V12_PROXY_STATE",
    "_SHOTCRETE_V12_PROXY_SUB",
    "_SHOTCRETE_V12_RET_STATE",
    "_SHOTCRETE_V12_RET_SUB",
    "_SHOTCRETE_V12_RET_TL",
    "_SHOTCRETE_V12_RET_LONG_STATE",
    "_SHOTCRETE_V12_RET_LONG_SUB",
    "_SHOTCRETE_V12_RET_LONG_TL",
    "_SHOTCRETE_V12_RET_LONG_INST_STATE",
    "_SHOTCRETE_V12_RET_LONG_INST_SUB",
    "_SHOTCRETE_V12_RET_LONG_INST_TL",
    "_SHOTCRETE_V12_RET_WARM_STATE",
    "_SHOTCRETE_V12_RET_WARM_SUB",
    "_SHOTCRETE_V12_RET_WARM_TL",
    "_SHOTCRETE_PROXY_STATE",
    "_SHOTCRETE_PROXY_SUB",
    "_SHOTCRETE_PROXY_TIMELINE",
    "_SHOTCRETE_PROXY_SHAPE_STATE",
    "_SHOTCRETE_PROXY_SHAPE_SUB",
    "_SHOTCRETE_PROXY_SHAPE_TIMELINE",
    "_SHOTCRETE_PROXY_SHAPE_MATRIX_FIX1_STATE",
    "_SHOTCRETE_PROXY_SHAPE_MATRIX_FIX1_SUB",
]

# -----------------------------------------------------------------------------
# Timing / delivery
# -----------------------------------------------------------------------------

FPS = 60.0
START_FRAME = 1
EMIT_DURATION_SECONDS = 8.0
EMIT_FRAMES = int(EMIT_DURATION_SECONDS * FPS)

# Continuous / quasi-continuous delivery
PARTICLE_MASS = 0.0022
TARGET_PARTICLES_PER_SEC = 450.0

# -----------------------------------------------------------------------------
# Particle / spray tuning
# -----------------------------------------------------------------------------

PARTICLE_RADIUS = 0.011
PARTICLE_DIAMETER = PARTICLE_RADIUS * 2.0
STATIC_PROTO_RADIUS = PARTICLE_RADIUS * 1.65

SPRAY_SPEED_MPS = 2.15
SPRAY_CONE_DEG = 0.20
NOZZLE_SPAWN_OFFSET_M = 0.045
SPEED_JITTER = 0.010
SPAWN_DISK_RADIUS = 0.0008

# -----------------------------------------------------------------------------
# Retention-style PBD tuning
# -----------------------------------------------------------------------------

PBD_DAMPING = 0.94
PBD_COHESION = 1.05
PBD_VISCOSITY = 1600.0
PBD_SURFACE_TENSION = 0.12
PBD_FRICTION = 0.98
PBD_PARTICLE_FRICTION_SCALE = 1.40
PBD_ADHESION = 5.00
PBD_PARTICLE_ADHESION_SCALE = 2.40
PBD_GRAVITY_SCALE = 0.16

SOLVER_POSITION_ITERS = 48
MAX_NEIGHBORHOOD = 196

# -----------------------------------------------------------------------------
# Face-locked aim-zone freeze settings
# -----------------------------------------------------------------------------
# Explicit receiver face selection.
# axis: 0=x, 1=y, 2=z in RECEIVER LOCAL SPACE
# sign: +1 -> positive face, -1 -> negative face
#
# If deposit appears on the wrong side, change RECEIVER_FACE_SIGN first.
# If it appears on the wrong plane entirely, change RECEIVER_FACE_AXIS.

RECEIVER_FACE_AXIS = 0
RECEIVER_FACE_SIGN = -1

# Capture corridor in front of the chosen face.
# Positive front distance means "in front of / outside the chosen face".
CAPTURE_FRONT_TOL = 0.105
CAPTURE_BACK_TOL = 0.020
PREDICTIVE_CAPTURE_MAX_TIME = 0.100

# Freeze only inside a compact aim-centered zone on the face.
# We now bias capture generosity mostly sideways, not vertically, because the
# current dominant miss mode is material skimming past the side of the proxy
# face rather than falling below it.
AIM_ZONE_RADIUS = 0.60
CAPTURE_TANGENTIAL_PAD_SIDE = 0.180
CAPTURE_TANGENTIAL_PAD_VERTICAL = 0.015
AIM_ELLIPSE_SIDE_RADIUS = AIM_ZONE_RADIUS + 0.20
AIM_ELLIPSE_VERTICAL_RADIUS = AIM_ZONE_RADIUS + 0.01

# Soft side-funnel capture:
# particles that would just miss the receiver laterally can still be snapped
# onto the nearest valid paint column, but only if they are already close to
# the correct face and not below the valid paint window.
SIDE_FUNNEL_CAPTURE_PAD = 0.180
PAINT_REJECT_BELOW_PAD = 0.020
BOTTOM_SOFT_LIFT_BAND = 0.030
BOTTOM_SOFT_LIFT = 0.012

# Travel / directional guardrails
MIN_TRAVEL_BEFORE_FREEZE = 0.04
FREEZE_MIN_FORWARD_DOT = -0.05
MIN_FACE_APPROACH_DOT = 0.02

# Optional thinning
FREEZE_FRACTION = 1.0

# Miss cleanup / runaway cleanup
MISS_BEHIND_DIST = 0.090
MISS_SIDE_PAD = 0.045
MISS_BELOW_LOCAL_PAD = 0.040
MISS_BELOW_WORLD_PAD = 0.180
MAX_DYNAMIC_TRAVEL_M = 4.50

# Safety cap
MAX_DEPOSITED = 60000

# Tangential deposit grid & stacking
DEPOSIT_CELL_SIZE = PARTICLE_DIAMETER * 0.60
DEPOSIT_NORMAL_OFFSET = PARTICLE_RADIUS * 1.05
STACK_STEP = PARTICLE_RADIUS * 0.34

# Face-aligned splat representation
SPLAT_COUNT = 9
SPLAT_RADIUS = PARTICLE_RADIUS * 0.85
SPLAT_CENTER_WEIGHT = 1.0
SPLAT_DIAG_OFFSET = SPLAT_RADIUS * 0.75 * math.sqrt(2.0)
PAINT_CLUSTER_TANGENTIAL_MARGIN = (
    STATIC_PROTO_RADIUS + max(SPLAT_RADIUS, SPLAT_DIAG_OFFSET) + 0.5 * DEPOSIT_CELL_SIZE
)
FACE_PAINT_SIDE_MARGIN = PAINT_CLUSTER_TANGENTIAL_MARGIN + 0.004
FACE_PAINT_TOP_MARGIN = FACE_PAINT_SIDE_MARGIN
FACE_PAINT_BOTTOM_MARGIN = FACE_PAINT_SIDE_MARGIN + 0.025

# -----------------------------------------------------------------------------
# Logging checkpoints / metrics output
# -----------------------------------------------------------------------------

METRIC_TIMES_SEC = [1.0, 3.0, 6.0]
METRIC_LOG_INTERVAL_FRAMES = 60
METRICS_CSV_PATH = "D:/Solita/Cave Sim/Scripts/shotcrete_metrics_broad_splat.csv"

RUN_VERSION_TAG = "v16_1"
METRICS_LATEST_CSV_BASENAME = f"shotcrete_metrics_broad_splat_{RUN_VERSION_TAG}_latest.csv"
METRICS_RUN_CSV_PREFIX = f"shotcrete_metrics_broad_splat_{RUN_VERSION_TAG}_run_"
METRICS_RUN_SUMMARY_TXT_PREFIX = f"shotcrete_metrics_broad_splat_{RUN_VERSION_TAG}_run_"
METRICS_LATEST_SUMMARY_TXT_BASENAME = f"shotcrete_metrics_broad_splat_{RUN_VERSION_TAG}_latest.txt"
METRICS_RUN_JSON_PREFIX = f"shotcrete_metrics_broad_splat_{RUN_VERSION_TAG}_run_"
METRICS_LATEST_JSON_BASENAME = f"shotcrete_metrics_broad_splat_{RUN_VERSION_TAG}_latest.json"
WRITE_LEGACY_GENERIC_CSV = False

FINALIZE_DYNAMIC_LIVE_THRESHOLD = 12
FINALIZE_PROBE_INTERVAL_FRAMES = 30
FINALIZE_QUIET_PROBES = 4
MIN_POST_EMIT_FINALIZE_SECONDS = 2.0
HARD_FINALIZE_EXTRA_SECONDS = 24.0

# Optional manual recenter knob.
# Leave at 0.0 for the current baseline; adjust only if side miss remains
# dominant and you have confirmed the sign from a trusted view.
AIM_SIDE_LOCAL_BIAS_M = 0.0
AIM_VERTICAL_LOCAL_BIAS_M = 0.0

# -----------------------------------------------------------------------------

def log(msg: str) -> None:
    print(f"[SHOTCRETE_FACE_LOCKED_SPLAT_METRICS] {msg}")

def get_stage():
    return omni.usd.get_context().get_stage()

def get_stage_file_path(stage) -> str:
    try:
        layer = stage.GetRootLayer()
        if layer:
            return str(layer.realPath or layer.identifier or "")
    except Exception:
        pass
    return ""

def choose_writable_dir(candidates):
    for path in candidates:
        if not path:
            continue
        try:
            os.makedirs(path, exist_ok=True)
            return path
        except Exception:
            pass
    try:
        return os.getcwd()
    except Exception:
        return ""

def resolve_metrics_output_bundle(stage):
    run_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    stage_path = get_stage_file_path(stage)
    stage_dir = os.path.dirname(stage_path) if stage_path else ""
    legacy_dir = os.path.dirname(METRICS_CSV_PATH) if METRICS_CSV_PATH else ""
    primary_dir = choose_writable_dir([legacy_dir, stage_dir])

    if not primary_dir:
        return {
            "run_id": run_id,
            "stage_path": stage_path,
            "primary_dir": "",
            "run_csv_path": None,
            "latest_csv_path": None,
            "run_summary_txt_path": None,
            "latest_summary_txt_path": None,
            "run_json_path": None,
            "latest_json_path": None,
            "csv_paths": [],
        }

    run_csv_path = os.path.join(primary_dir, f"{METRICS_RUN_CSV_PREFIX}{run_id}.csv")
    latest_csv_path = os.path.join(primary_dir, METRICS_LATEST_CSV_BASENAME)
    csv_paths = [run_csv_path]

    if WRITE_LEGACY_GENERIC_CSV and METRICS_CSV_PATH:
        legacy_generic_path = os.path.join(primary_dir, os.path.basename(METRICS_CSV_PATH))
        if legacy_generic_path not in csv_paths:
            csv_paths.append(legacy_generic_path)

    run_summary_txt_path = os.path.join(primary_dir, f"{METRICS_RUN_SUMMARY_TXT_PREFIX}{run_id}.txt")
    latest_summary_txt_path = os.path.join(primary_dir, METRICS_LATEST_SUMMARY_TXT_BASENAME)
    run_json_path = os.path.join(primary_dir, f"{METRICS_RUN_JSON_PREFIX}{run_id}.json")
    latest_json_path = os.path.join(primary_dir, METRICS_LATEST_JSON_BASENAME)

    return {
        "run_id": run_id,
        "stage_path": stage_path,
        "primary_dir": primary_dir,
        "run_csv_path": run_csv_path,
        "latest_csv_path": latest_csv_path,
        "run_summary_txt_path": run_summary_txt_path,
        "latest_summary_txt_path": latest_summary_txt_path,
        "run_json_path": run_json_path,
        "latest_json_path": latest_json_path,
        "csv_paths": csv_paths,
    }

def metrics_csv_header() -> str:
    return (
        "time_s,snapshot_label,run_id,config_hash,emitted_particles,dynamic_live,max_dynamic_live,"
        "corridor_entries,captured_particles,deposited_splats,"
        "captured_via_corridor,captured_via_predicted,captured_via_crossing,captured_via_side_funnel,"
        "missed_side,missed_below,missed_behind,culled_far,"
        "occupied_cells,coverage_area_m2,coverage_pct,capture_efficiency,"
        "avg_stack,max_stack,avg_thickness_m,max_thickness_m,finalize_reason\n"
    )

def write_metrics_csv_headers(paths):
    if not paths:
        return
    header = metrics_csv_header()
    for path in paths:
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(header)
        except Exception:
            pass

def build_run_config_manifest():
    return {
        "run_version_tag": RUN_VERSION_TAG,
        "fps": FPS,
        "emit_duration_seconds": EMIT_DURATION_SECONDS,
        "target_particles_per_sec": TARGET_PARTICLES_PER_SEC,
        "particle_mass": PARTICLE_MASS,
        "particle_radius": PARTICLE_RADIUS,
        "static_proto_radius": STATIC_PROTO_RADIUS,
        "spray_speed_mps": SPRAY_SPEED_MPS,
        "spray_cone_deg": SPRAY_CONE_DEG,
        "nozzle_spawn_offset_m": NOZZLE_SPAWN_OFFSET_M,
        "speed_jitter": SPEED_JITTER,
        "spawn_disk_radius": SPAWN_DISK_RADIUS,
        "pbd_damping": PBD_DAMPING,
        "pbd_cohesion": PBD_COHESION,
        "pbd_viscosity": PBD_VISCOSITY,
        "pbd_surface_tension": PBD_SURFACE_TENSION,
        "pbd_friction": PBD_FRICTION,
        "pbd_particle_friction_scale": PBD_PARTICLE_FRICTION_SCALE,
        "pbd_adhesion": PBD_ADHESION,
        "pbd_particle_adhesion_scale": PBD_PARTICLE_ADHESION_SCALE,
        "pbd_gravity_scale": PBD_GRAVITY_SCALE,
        "solver_position_iters": SOLVER_POSITION_ITERS,
        "max_neighborhood": MAX_NEIGHBORHOOD,
        "receiver_face_axis": RECEIVER_FACE_AXIS,
        "receiver_face_sign": RECEIVER_FACE_SIGN,
        "capture_front_tol": CAPTURE_FRONT_TOL,
        "capture_back_tol": CAPTURE_BACK_TOL,
        "predictive_capture_max_time": PREDICTIVE_CAPTURE_MAX_TIME,
        "aim_zone_radius": AIM_ZONE_RADIUS,
        "capture_tangential_pad_side": CAPTURE_TANGENTIAL_PAD_SIDE,
        "capture_tangential_pad_vertical": CAPTURE_TANGENTIAL_PAD_VERTICAL,
        "aim_ellipse_side_radius": AIM_ELLIPSE_SIDE_RADIUS,
        "aim_ellipse_vertical_radius": AIM_ELLIPSE_VERTICAL_RADIUS,
        "side_funnel_capture_pad": SIDE_FUNNEL_CAPTURE_PAD,
        "paint_reject_below_pad": PAINT_REJECT_BELOW_PAD,
        "bottom_soft_lift_band": BOTTOM_SOFT_LIFT_BAND,
        "bottom_soft_lift": BOTTOM_SOFT_LIFT,
        "min_travel_before_freeze": MIN_TRAVEL_BEFORE_FREEZE,
        "freeze_min_forward_dot": FREEZE_MIN_FORWARD_DOT,
        "min_face_approach_dot": MIN_FACE_APPROACH_DOT,
        "freeze_fraction": FREEZE_FRACTION,
        "miss_behind_dist": MISS_BEHIND_DIST,
        "miss_side_pad": MISS_SIDE_PAD,
        "miss_below_local_pad": MISS_BELOW_LOCAL_PAD,
        "miss_below_world_pad": MISS_BELOW_WORLD_PAD,
        "max_dynamic_travel_m": MAX_DYNAMIC_TRAVEL_M,
        "max_deposited": MAX_DEPOSITED,
        "deposit_cell_size": DEPOSIT_CELL_SIZE,
        "deposit_normal_offset": DEPOSIT_NORMAL_OFFSET,
        "stack_step": STACK_STEP,
        "splat_count": SPLAT_COUNT,
        "splat_radius": SPLAT_RADIUS,
        "splat_center_weight": SPLAT_CENTER_WEIGHT,
        "splat_diag_offset": SPLAT_DIAG_OFFSET,
        "paint_cluster_tangential_margin": PAINT_CLUSTER_TANGENTIAL_MARGIN,
        "face_paint_side_margin": FACE_PAINT_SIDE_MARGIN,
        "face_paint_top_margin": FACE_PAINT_TOP_MARGIN,
        "face_paint_bottom_margin": FACE_PAINT_BOTTOM_MARGIN,
        "metric_times_sec": list(METRIC_TIMES_SEC),
        "metric_log_interval_frames": METRIC_LOG_INTERVAL_FRAMES,
        "finalize_dynamic_live_threshold": FINALIZE_DYNAMIC_LIVE_THRESHOLD,
        "finalize_probe_interval_frames": FINALIZE_PROBE_INTERVAL_FRAMES,
        "finalize_quiet_probes": FINALIZE_QUIET_PROBES,
        "min_post_emit_finalize_seconds": MIN_POST_EMIT_FINALIZE_SECONDS,
        "hard_finalize_extra_seconds": HARD_FINALIZE_EXTRA_SECONDS,
        "aim_side_local_bias_m": AIM_SIDE_LOCAL_BIAS_M,
        "aim_vertical_local_bias_m": AIM_VERTICAL_LOCAL_BIAS_M,
        "write_legacy_generic_csv": WRITE_LEGACY_GENERIC_CSV,
    }


def compute_config_hash_short(manifest) -> str:
    try:
        payload = json.dumps(manifest, sort_keys=True, separators=(",", ":"))
    except Exception:
        payload = repr(manifest)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]


def summarize_finalize_reason(reason: str) -> str:
    mapping = {
        "quiet_tail": "Emitter finished and live particle count stayed below threshold across quiet probes.",
        "timeline_stopped": "Timeline stopped before quiet-tail or time-cap finalization; final row written immediately.",
        "time_cap": "Hard finalization time cap reached; run closed to avoid hanging metrics.",
    }
    return mapping.get(reason, "Unknown finalization reason.")


def copy_file(src: str, dst: str) -> bool:
    if not src or not dst:
        return False
    try:
        shutil.copyfile(src, dst)
        return True
    except Exception:
        return False


def write_json_file(path: str, payload) -> bool:
    if not path:
        return False
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
            f.write("\n")
        return True
    except Exception:
        return False

def write_text_file(path: str, text: str) -> bool:
    if not path:
        return False
    try:
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return True
    except Exception:
        return False

def delete_prim_if_exists(stage, path: str) -> None:
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        stage.RemovePrim(path)

def clear_old_state():
    sub = getattr(builtins, SUB_KEY, None)
    if sub is not None:
        try:
            sub.unsubscribe()
        except Exception:
            pass
        try:
            delattr(builtins, SUB_KEY)
        except Exception:
            pass

    if hasattr(builtins, STATE_KEY):
        try:
            delattr(builtins, STATE_KEY)
        except Exception:
            pass

    for key in OLD_KEYS:
        val = getattr(builtins, key, None)
        if val is not None and hasattr(val, "unsubscribe"):
            try:
                val.unsubscribe()
            except Exception:
                pass
        if hasattr(builtins, key):
            try:
                delattr(builtins, key)
            except Exception:
                pass

def get_world_pos(stage, path: str) -> Gf.Vec3f:
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Missing prim: {path}")
    world_m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    p = world_m.ExtractTranslation()
    return Gf.Vec3f(float(p[0]), float(p[1]), float(p[2]))

def normalize(v: Gf.Vec3f) -> Gf.Vec3f:
    l = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if l < 1e-8:
        return Gf.Vec3f(0.0, 0.0, -1.0)
    return Gf.Vec3f(v[0]/l, v[1]/l, v[2]/l)

def dot(a: Gf.Vec3f, b: Gf.Vec3f) -> float:
    return float(a[0]*b[0] + a[1]*b[1] + a[2]*b[2])

def length(v: Gf.Vec3f) -> float:
    return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

def orthonormal_basis(dir_vec: Gf.Vec3f):
    d = normalize(dir_vec)
    up = Gf.Vec3f(0.0, 1.0, 0.0)
    if abs(d[1]) > 0.95:
        up = Gf.Vec3f(1.0, 0.0, 0.0)
    u = normalize(Gf.Cross(d, up))
    v = normalize(Gf.Cross(d, u))
    return d, u, v

def rand_in_disk(radius: float):
    a = random.uniform(0.0, math.tau)
    r = radius * math.sqrt(random.random())
    return math.cos(a) * r, math.sin(a) * r

def make_random_dir(base_dir: Gf.Vec3f) -> Gf.Vec3f:
    if SPRAY_CONE_DEG <= 0.0:
        return normalize(base_dir)
    cone_rad = math.radians(SPRAY_CONE_DEG)
    d, u, v = orthonormal_basis(base_dir)
    yaw = random.uniform(0.0, math.tau)
    mag = math.tan(cone_rad) * math.sqrt(random.random())
    offset = u * (math.cos(yaw) * mag) + v * (math.sin(yaw) * mag)
    return normalize(d + offset)

def make_spawn_offset(base_dir: Gf.Vec3f) -> Gf.Vec3f:
    _d, u, v = orthonormal_basis(base_dir)
    x, y = rand_in_disk(SPAWN_DISK_RADIUS)
    return u * x + v * y

def set_translate(stage, path: str, pos: Gf.Vec3f):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return
    x = UsdGeom.Xformable(prim)
    ops = x.GetOrderedXformOps()
    t_op = None
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            t_op = op
            break
    if t_op is None:
        t_op = x.AddTranslateOp()
    t_op.Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))

def ensure_debug_marker(stage, path: str, color=(1.0, 0.2, 0.2), radius=0.01):
    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(radius)
    prim = sphere.GetPrim()
    prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        Vt.Vec3fArray([Gf.Vec3f(*color)])
    )

def safe_create_attr(api_obj, attr_name, value):
    fn = getattr(api_obj, attr_name, None)
    if callable(fn):
        try:
            fn(value)
            return True
        except Exception:
            return False
    return False

def safe_create_rel(api_obj, rel_name, targets):
    fn = getattr(api_obj, rel_name, None)
    if callable(fn):
        try:
            rel = fn()
            rel.SetTargets([Sdf.Path(t) for t in targets])
            return True
        except Exception:
            return False
    return False

def ensure_rig(stage):
    UsdGeom.Xform.Define(stage, RIG_ROOT)
    UsdGeom.Xform.Define(stage, DYNAMIC_ROOT)
    UsdGeom.Xform.Define(stage, DEPOSIT_ROOT)

    dyn_proto = UsdGeom.Sphere.Define(stage, DYNAMIC_PROTO_PATH)
    dyn_proto.CreateRadiusAttr(PARTICLE_RADIUS * 0.82)
    dyn_proto.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        Vt.Vec3fArray([Gf.Vec3f(0.66, 0.62, 0.56)])
    )

    stat_proto = UsdGeom.Sphere.Define(stage, STATIC_PROTO_PATH)
    stat_proto.CreateRadiusAttr(STATIC_PROTO_RADIUS)
    stat_proto.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        Vt.Vec3fArray([Gf.Vec3f(0.48, 0.48, 0.50)])
    )

    dep = UsdGeom.PointInstancer.Define(stage, DEPOSIT_PI_PATH)
    dep.CreatePrototypesRel().SetTargets([Sdf.Path(STATIC_PROTO_PATH)])
    dep.CreateProtoIndicesAttr(Vt.IntArray())
    dep.CreatePositionsAttr(Vt.Vec3fArray())
    dep.CreateScalesAttr(Vt.Vec3fArray())
    dep.CreateOrientationsAttr(Vt.QuathArray())
    dep.CreateIdsAttr(Vt.Int64Array())

    ensure_debug_marker(stage, NOZZLE_MARKER_PATH, color=(0.2, 1.0, 0.2), radius=0.010)
    ensure_debug_marker(stage, AIM_MARKER_PATH, color=(1.0, 0.2, 0.2), radius=0.007)

def ensure_particle_system(stage):
    ps_prim = PhysxSchema.PhysxParticleSystem.Define(stage, PARTICLE_SYSTEM_PATH).GetPrim()
    ps = PhysxSchema.PhysxParticleSystem(ps_prim)

    safe_create_attr(ps, "CreateParticleContactOffsetAttr", PARTICLE_RADIUS * 2.0)
    safe_create_attr(ps, "CreateContactOffsetAttr", PARTICLE_RADIUS * 2.2)
    safe_create_attr(ps, "CreateRestOffsetAttr", PARTICLE_RADIUS)
    safe_create_attr(ps, "CreateSolidRestOffsetAttr", PARTICLE_RADIUS)
    safe_create_attr(ps, "CreateFluidRestOffsetAttr", PARTICLE_RADIUS * 0.65)
    safe_create_attr(ps, "CreateSolverPositionIterationCountAttr", SOLVER_POSITION_ITERS)
    safe_create_attr(ps, "CreateMaxNeighborhoodAttr", MAX_NEIGHBORHOOD)
    safe_create_attr(ps, "CreateMaxVelocityAttr", 50.0)
    safe_create_attr(ps, "CreateMaxParticlesAttr", 500000)

def ensure_material(stage):
    mat_prim = UsdShade.Material.Define(stage, PBD_MATERIAL_PATH).GetPrim()
    mat_api = PhysxSchema.PhysxPBDMaterialAPI.Apply(mat_prim)

    safe_create_attr(mat_api, "CreateDampingAttr", PBD_DAMPING)
    safe_create_attr(mat_api, "CreateGravityScaleAttr", PBD_GRAVITY_SCALE)
    safe_create_attr(mat_api, "CreateViscosityAttr", PBD_VISCOSITY)
    safe_create_attr(mat_api, "CreateCohesionAttr", PBD_COHESION)
    safe_create_attr(mat_api, "CreateSurfaceTensionAttr", PBD_SURFACE_TENSION)
    safe_create_attr(mat_api, "CreateFrictionAttr", PBD_FRICTION)
    safe_create_attr(mat_api, "CreateParticleFrictionScaleAttr", PBD_PARTICLE_FRICTION_SCALE)
    safe_create_attr(mat_api, "CreateAdhesionAttr", PBD_ADHESION)
    safe_create_attr(mat_api, "CreateParticleAdhesionScaleAttr", PBD_PARTICLE_ADHESION_SCALE)

    try:
        mat_prim.CreateRelationship("physics:material:binding", False).SetTargets([Sdf.Path(PARTICLE_SYSTEM_PATH)])
    except Exception:
        pass

def sync_markers(stage):
    nozzle_world = get_world_pos(stage, NOZZLE_LOCATOR_PATH)
    aim_world = get_world_pos(stage, AIM_LOCATOR_PATH)
    set_translate(stage, NOZZLE_MARKER_PATH, nozzle_world)
    set_translate(stage, AIM_MARKER_PATH, aim_world)


def build_receiver_info(stage):
    prim = stage.GetPrimAtPath(RECEIVER_PATH)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Missing receiver prim: {RECEIVER_PATH}")

    world_m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    inv_m = world_m.GetInverse()

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=True)
    bbox = bbox_cache.ComputeLocalBound(prim)
    rng = bbox.GetRange()
    mn = rng.GetMin()
    mx = rng.GetMax()

    half = [
        abs(float(mx[0] - mn[0])) * 0.5,
        abs(float(mx[1] - mn[1])) * 0.5,
        abs(float(mx[2] - mn[2])) * 0.5,
    ]

    # Receiver local face plane
    face_coord = RECEIVER_FACE_SIGN * half[RECEIVER_FACE_AXIS]

    # Tangential axes
    tangential_axes = [i for i in range(3) if i != RECEIVER_FACE_AXIS]

    # Aim-point projection onto the chosen face plane
    aim_world = get_world_pos(stage, AIM_LOCATOR_PATH)
    aim_local = inv_m.Transform(Gf.Vec3d(float(aim_world[0]), float(aim_world[1]), float(aim_world[2])))
    aim_local = [float(aim_local[0]), float(aim_local[1]), float(aim_local[2])]
    aim_local[RECEIVER_FACE_AXIS] = face_coord
    for ax in tangential_axes:
        if ax == 1:
            aim_local[ax] += AIM_VERTICAL_LOCAL_BIAS_M
        else:
            aim_local[ax] += AIM_SIDE_LOCAL_BIAS_M

    # World-space face normal
    axis_dir_local = [0.0, 0.0, 0.0]
    axis_dir_local[RECEIVER_FACE_AXIS] = RECEIVER_FACE_SIGN
    axis_world = world_m.TransformDir(Gf.Vec3d(axis_dir_local[0], axis_dir_local[1], axis_dir_local[2]))
    face_normal_world = normalize(Gf.Vec3f(float(axis_world[0]), float(axis_world[1]), float(axis_world[2])))

    return {
        "world_m": world_m,
        "inv_m": inv_m,
        "half": half,
        "max_half_extent": max(half),
        "face_axis": RECEIVER_FACE_AXIS,
        "face_sign": RECEIVER_FACE_SIGN,
        "face_coord": face_coord,
        "tangential_axes": tangential_axes,
        "aim_local_on_face": aim_local,
        "aim_world": aim_world,
        "face_normal_world": face_normal_world,
    }

def receiver_local_coords_and_front_dist(pos_world: Gf.Vec3f, recv):
    p_local = recv["inv_m"].Transform(Gf.Vec3d(float(pos_world[0]), float(pos_world[1]), float(pos_world[2])))
    p_local = [float(p_local[0]), float(p_local[1]), float(p_local[2])]
    front_dist = recv["face_sign"] * (p_local[recv["face_axis"]] - recv["face_coord"])
    return p_local, float(front_dist)

def world_from_receiver_local(local_p, recv):
    q = recv["world_m"].Transform(Gf.Vec3d(float(local_p[0]), float(local_p[1]), float(local_p[2])))
    return Gf.Vec3f(float(q[0]), float(q[1]), float(q[2]))

def capture_axis_pad(ax: int) -> float:
    return CAPTURE_TANGENTIAL_PAD_VERTICAL if ax == 1 else CAPTURE_TANGENTIAL_PAD_SIDE

def capture_axis_radius(ax: int) -> float:
    return AIM_ELLIPSE_VERTICAL_RADIUS if ax == 1 else AIM_ELLIPSE_SIDE_RADIUS

def soft_capture_axis_pad(ax: int) -> float:
    if ax == 1:
        return CAPTURE_TANGENTIAL_PAD_VERTICAL
    return CAPTURE_TANGENTIAL_PAD_SIDE + SIDE_FUNNEL_CAPTURE_PAD

def soft_capture_axis_radius(ax: int) -> float:
    if ax == 1:
        return AIM_ELLIPSE_VERTICAL_RADIUS
    return AIM_ELLIPSE_SIDE_RADIUS + SIDE_FUNNEL_CAPTURE_PAD

def paint_axis_limits(ax: int, recv):
    half = recv["half"][ax]
    if ax == 1:
        return (-half + FACE_PAINT_BOTTOM_MARGIN, half - FACE_PAINT_TOP_MARGIN)
    return (-half + FACE_PAINT_SIDE_MARGIN, half - FACE_PAINT_SIDE_MARGIN)

def clamp_local_to_paint_window(local_p, recv):
    q = [float(local_p[0]), float(local_p[1]), float(local_p[2])]
    for ax in recv["tangential_axes"]:
        lo, hi = paint_axis_limits(ax, recv)
        if lo <= hi:
            q[ax] = min(max(q[ax], lo), hi)
        else:
            q[ax] = 0.5 * (lo + hi)
    return q

def sanitize_capture_local(local_p, recv, allow_side_funnel: bool = False):
    q = [float(local_p[0]), float(local_p[1]), float(local_p[2])]
    used_side_funnel = False

    for ax in recv["tangential_axes"]:
        lo, hi = paint_axis_limits(ax, recv)
        if ax == 1:
            if q[ax] < lo - PAINT_REJECT_BELOW_PAD:
                return None, False
            if q[ax] < lo:
                if q[ax] < lo - BOTTOM_SOFT_LIFT_BAND:
                    return None, False
                q[ax] = min(lo + BOTTOM_SOFT_LIFT, hi)
            elif q[ax] > hi:
                q[ax] = hi
            continue

        if q[ax] < lo:
            if allow_side_funnel and q[ax] >= lo - SIDE_FUNNEL_CAPTURE_PAD:
                q[ax] = lo
                used_side_funnel = True
            else:
                return None, False
        elif q[ax] > hi:
            if allow_side_funnel and q[ax] <= hi + SIDE_FUNNEL_CAPTURE_PAD:
                q[ax] = hi
                used_side_funnel = True
            else:
                return None, False

    q[recv["face_axis"]] = recv["face_coord"]
    return q, used_side_funnel

def is_inside_capture_window(local_p, recv):
    tangential_axes = recv["tangential_axes"]
    half = recv["half"]
    aim_local = recv["aim_local_on_face"]

    for ax in tangential_axes:
        if abs(local_p[ax]) > (half[ax] + capture_axis_pad(ax)):
            return False

    t0 = tangential_axes[0]
    t1 = tangential_axes[1]
    d0 = local_p[t0] - aim_local[t0]
    d1 = local_p[t1] - aim_local[t1]
    r0 = max(capture_axis_radius(t0), 1e-6)
    r1 = max(capture_axis_radius(t1), 1e-6)
    ell = (d0 / r0) * (d0 / r0) + (d1 / r1) * (d1 / r1)
    if ell > 1.0:
        return False

    return True

def is_inside_soft_capture_window(local_p, recv):
    tangential_axes = recv["tangential_axes"]
    half = recv["half"]
    aim_local = recv["aim_local_on_face"]

    for ax in tangential_axes:
        if abs(local_p[ax]) > (half[ax] + soft_capture_axis_pad(ax)):
            return False

    t0 = tangential_axes[0]
    t1 = tangential_axes[1]
    d0 = local_p[t0] - aim_local[t0]
    d1 = local_p[t1] - aim_local[t1]
    r0 = max(soft_capture_axis_radius(t0), 1e-6)
    r1 = max(soft_capture_axis_radius(t1), 1e-6)
    ell = (d0 / r0) * (d0 / r0) + (d1 / r1) * (d1 / r1)
    if ell > 1.0:
        return False

    q, _used_side_funnel = sanitize_capture_local(local_p, recv, allow_side_funnel=True)
    return q is not None

def is_in_capture_corridor(local_p, front_dist, recv):
    if front_dist < -CAPTURE_BACK_TOL or front_dist > CAPTURE_FRONT_TOL:
        return False
    return is_inside_capture_window(local_p, recv)

def append_deposited_points(stage, state, new_positions):
    if not new_positions:
        return

    dep = UsdGeom.PointInstancer(stage.GetPrimAtPath(DEPOSIT_PI_PATH))
    if not dep:
        return

    old_positions = list(dep.GetPositionsAttr().Get() or [])
    old_proto = list(dep.GetProtoIndicesAttr().Get() or [])
    old_scales = list(dep.GetScalesAttr().Get() or [])
    old_orients = list(dep.GetOrientationsAttr().Get() or [])
    old_ids = list(dep.GetIdsAttr().Get() or [])

    room = MAX_DEPOSITED - len(old_positions)
    if room <= 0:
        return

    new_positions = new_positions[:room]

    for p in new_positions:
        old_positions.append(Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
        old_proto.append(0)
        old_scales.append(Gf.Vec3f(1.0, 1.0, 1.0))
        old_orients.append(Gf.Quath(1.0, Gf.Vec3h(0.0, 0.0, 0.0)))
        old_ids.append(state["next_deposit_id"])
        state["next_deposit_id"] += 1

    dep.GetPositionsAttr().Set(Vt.Vec3fArray(old_positions))
    dep.GetProtoIndicesAttr().Set(Vt.IntArray(old_proto))
    dep.GetScalesAttr().Set(Vt.Vec3fArray(old_scales))
    dep.GetOrientationsAttr().Set(Vt.QuathArray(old_orients))
    dep.GetIdsAttr().Set(Vt.Int64Array(old_ids))

    state["deposited_count"] = len(old_positions)

def emit_particle_set(stage, state, n_particles: int):
    if n_particles <= 0:
        return

    idx = state["set_count"]
    path = f"{DYNAMIC_ROOT}/Set_{idx:05d}"

    nozzle_world = get_world_pos(stage, NOZZLE_LOCATOR_PATH)
    aim_world = get_world_pos(stage, AIM_LOCATOR_PATH)
    base_dir = normalize(aim_world - nozzle_world)

    positions = []
    velocities = []
    proto_indices = []
    scales = []
    orientations = []
    ids = []

    for _ in range(n_particles):
        d = make_random_dir(base_dir)
        p = nozzle_world + base_dir * NOZZLE_SPAWN_OFFSET_M + make_spawn_offset(base_dir)
        speed = SPRAY_SPEED_MPS + random.uniform(-SPEED_JITTER, SPEED_JITTER)

        positions.append(Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
        velocities.append(Gf.Vec3f(float(d[0] * speed), float(d[1] * speed), float(d[2] * speed)))
        proto_indices.append(0)
        scales.append(Gf.Vec3f(1.0, 1.0, 1.0))
        orientations.append(Gf.Quath(1.0, Gf.Vec3h(0.0, 0.0, 0.0)))
        ids.append(state["next_particle_id"])
        state["next_particle_id"] += 1

    inst = UsdGeom.PointInstancer.Define(stage, path)
    inst.CreatePrototypesRel().SetTargets([Sdf.Path(DYNAMIC_PROTO_PATH)])
    inst.CreateProtoIndicesAttr(Vt.IntArray(proto_indices))
    inst.CreatePositionsAttr(Vt.Vec3fArray(positions))
    inst.CreateVelocitiesAttr(Vt.Vec3fArray(velocities))
    inst.CreateScalesAttr(Vt.Vec3fArray(scales))
    inst.CreateOrientationsAttr(Vt.QuathArray(orientations))
    inst.CreateIdsAttr(Vt.Int64Array(ids))

    prim = inst.GetPrim()
    pset = PhysxSchema.PhysxParticleSetAPI.Apply(prim)
    ok = safe_create_rel(pset, "CreateParticleSystemRel", [PARTICLE_SYSTEM_PATH])
    safe_create_attr(pset, "CreateSelfCollisionAttr", True)
    safe_create_attr(pset, "CreateFluidAttr", False)
    safe_create_attr(pset, "CreateParticleGroupAttr", 0)

    particle_api = PhysxSchema.PhysxParticleAPI.Apply(prim)
    safe_create_attr(particle_api, "CreateParticleMassAttr", PARTICLE_MASS)

    if not ok:
        try:
            prim.CreateRelationship("physxParticle:particleSystem", False).SetTargets([Sdf.Path(PARTICLE_SYSTEM_PATH)])
        except Exception:
            pass

    state["set_count"] += 1
    state["emitted_particles"] += n_particles

    if state["set_count"] <= 10 or state["set_count"] % 25 == 0:
        log(
            f"EMIT set={state['set_count']} "
            f"n={n_particles} emitted_total={state['emitted_particles']}"
        )


def project_to_face_and_stack(p_world: Gf.Vec3f, recv, state):
    # Kept for compatibility; returns the center of the splat cluster.
    pts = project_splat_points_to_face(p_world, recv, state)
    return pts[0] if pts else p_world

def project_splat_points_from_local(p_local, recv, state):
    world_m = recv["world_m"]

    face_axis = recv["face_axis"]
    tangential_axes = recv["tangential_axes"]
    face_coord = recv["face_coord"]

    paint_local = clamp_local_to_paint_window(p_local, recv)

    # Tangential coords relative to face-centered aim point
    t0 = tangential_axes[0]
    t1 = tangential_axes[1]
    aim_local = recv["aim_local_on_face"]

    rel0 = paint_local[t0] - aim_local[t0]
    rel1 = paint_local[t1] - aim_local[t1]

    # Grid cell for stacking
    c0 = int(round(rel0 / DEPOSIT_CELL_SIZE))
    c1 = int(round(rel1 / DEPOSIT_CELL_SIZE))
    cell = (c0, c1)

    stack_idx = state["deposit_cell_counts"].get(cell, 0)
    state["deposit_cell_counts"][cell] = stack_idx + 1

    # Base face-locked center point
    center_local = [0.0, 0.0, 0.0]
    center_local[0] = float(paint_local[0])
    center_local[1] = float(paint_local[1])
    center_local[2] = float(paint_local[2])
    center_local[t0] = aim_local[t0] + c0 * DEPOSIT_CELL_SIZE
    center_local[t1] = aim_local[t1] + c1 * DEPOSIT_CELL_SIZE
    center_local = clamp_local_to_paint_window(center_local, recv)
    center_local[face_axis] = (
        face_coord
        + recv["face_sign"] * DEPOSIT_NORMAL_OFFSET
        + recv["face_sign"] * stack_idx * STACK_STEP
    )

    offsets = [
        (0.0, 0.0),
        (SPLAT_RADIUS, 0.0),
        (-SPLAT_RADIUS, 0.0),
        (0.0, SPLAT_RADIUS),
        (0.0, -SPLAT_RADIUS),
        (SPLAT_RADIUS * 0.75, SPLAT_RADIUS * 0.75),
        (SPLAT_RADIUS * 0.75, -SPLAT_RADIUS * 0.75),
        (-SPLAT_RADIUS * 0.75, SPLAT_RADIUS * 0.75),
        (-SPLAT_RADIUS * 0.75, -SPLAT_RADIUS * 0.75),
    ]
    offsets = offsets[:SPLAT_COUNT]

    pts = []
    for du, dv in offsets:
        q = center_local.copy()
        q[t0] += du
        q[t1] += dv
        q = clamp_local_to_paint_window(q, recv)
        p_world_proj = world_m.Transform(Gf.Vec3d(q[0], q[1], q[2]))
        pts.append(Gf.Vec3f(float(p_world_proj[0]), float(p_world_proj[1]), float(p_world_proj[2])))

    return pts

def project_splat_points_to_face(p_world: Gf.Vec3f, recv, state):
    p_local, _front_dist = receiver_local_coords_and_front_dist(p_world, recv)
    return project_splat_points_from_local(p_local, recv, state)

def should_freeze_particle(
    pos: Gf.Vec3f,
    vel: Gf.Vec3f,
    nozzle_world: Gf.Vec3f,
    spray_world: Gf.Vec3f,
    recv,
    frame_dt: float,
):
    travel = length(pos - nozzle_world)
    if travel < MIN_TRAVEL_BEFORE_FREEZE:
        return False, None, None

    vel_len = length(vel)
    vel_dir = None
    vel_local = None
    if vel_len > 1e-8:
        vel_dir = normalize(vel)
        if dot(vel_dir, spray_world) < FREEZE_MIN_FORWARD_DOT:
            return False, None, None
        if dot(vel_dir, recv["face_normal_world"]) > -MIN_FACE_APPROACH_DOT:
            return False, None, None
        vel_local = recv["inv_m"].TransformDir(Gf.Vec3d(float(vel[0]), float(vel[1]), float(vel[2])))
        vel_local = [float(vel_local[0]), float(vel_local[1]), float(vel_local[2])]

    p_local, front_dist = receiver_local_coords_and_front_dist(pos, recv)

    # Direct corridor capture
    if is_in_capture_corridor(p_local, front_dist, recv):
        snapped_local, _used_side_funnel = sanitize_capture_local(p_local, recv, allow_side_funnel=False)
        if snapped_local is not None:
            if FREEZE_FRACTION >= 1.0 or random.random() <= FREEZE_FRACTION:
                return True, snapped_local, "corridor"

    # Predictive capture: if the particle is clearly on course to the face shortly,
    # capture it at its projected face intercept rather than waiting for a literal
    # contact sample that may never line up with the frame boundaries.
    if vel_local is not None and front_dist > 0.0:
        front_speed = recv["face_sign"] * vel_local[recv["face_axis"]]
        if front_speed < -1e-6:
            time_to_face = front_dist / (-front_speed)
            if 0.0 <= time_to_face <= PREDICTIVE_CAPTURE_MAX_TIME:
                pred_local = [
                    p_local[0] + vel_local[0] * time_to_face,
                    p_local[1] + vel_local[1] * time_to_face,
                    p_local[2] + vel_local[2] * time_to_face,
                ]
                pred_local[recv["face_axis"]] = recv["face_coord"]
                if is_inside_soft_capture_window(pred_local, recv):
                    snapped_local, used_side_funnel = sanitize_capture_local(pred_local, recv, allow_side_funnel=True)
                    if snapped_local is not None:
                        if FREEZE_FRACTION >= 1.0 or random.random() <= FREEZE_FRACTION:
                            return True, snapped_local, ("side_funnel" if used_side_funnel else "predicted")

    # Crossing capture: robust to frame skips / fast traversal through the corridor
    prev_world = pos - vel * max(frame_dt, 1.0 / FPS)
    prev_local, prev_front_dist = receiver_local_coords_and_front_dist(prev_world, recv)
    crossed_face = (prev_front_dist > CAPTURE_FRONT_TOL and front_dist < -CAPTURE_BACK_TOL)
    if crossed_face:
        denom = prev_front_dist - front_dist
        alpha = 0.5
        if abs(denom) > 1e-6:
            alpha = min(max(prev_front_dist / denom, 0.0), 1.0)
        cross_local = [
            prev_local[0] + alpha * (p_local[0] - prev_local[0]),
            prev_local[1] + alpha * (p_local[1] - prev_local[1]),
            prev_local[2] + alpha * (p_local[2] - prev_local[2]),
        ]
        cross_local[recv["face_axis"]] = recv["face_coord"]
        if is_inside_soft_capture_window(cross_local, recv):
            snapped_local, used_side_funnel = sanitize_capture_local(cross_local, recv, allow_side_funnel=True)
            if snapped_local is not None:
                if FREEZE_FRACTION >= 1.0 or random.random() <= FREEZE_FRACTION:
                    return True, snapped_local, ("side_funnel" if used_side_funnel else "crossed")

    return False, None, None

def classify_miss_particle(pos: Gf.Vec3f, vel: Gf.Vec3f, nozzle_world: Gf.Vec3f, recv):
    travel = length(pos - nozzle_world)
    if travel < MIN_TRAVEL_BEFORE_FREEZE:
        return None

    p_local, front_dist = receiver_local_coords_and_front_dist(pos, recv)

    tangential_axes = recv["tangential_axes"]

    if front_dist < -MISS_BEHIND_DIST:
        return "missed_behind"

    # Prefer paint-window-relative culling so the scene stops being dominated by
    # particles that are already clearly below or beyond the usable proxy face.
    if 1 in tangential_axes:
        lo_y, _hi_y = paint_axis_limits(1, recv)
        if p_local[1] < (lo_y - MISS_BELOW_LOCAL_PAD) and float(vel[1]) <= 0.0 and front_dist < (CAPTURE_FRONT_TOL + 0.18):
            return "missed_below"
    else:
        if float(pos[1]) < float(recv["aim_world"][1]) - (recv["max_half_extent"] + MISS_BELOW_WORLD_PAD):
            return "missed_below"

    side_over = 0.0
    for ax in tangential_axes:
        if ax == 1:
            continue
        lo, hi = paint_axis_limits(ax, recv)
        if p_local[ax] < lo:
            side_over = max(side_over, lo - p_local[ax])
        elif p_local[ax] > hi:
            side_over = max(side_over, p_local[ax] - hi)

    if side_over > (SIDE_FUNNEL_CAPTURE_PAD + MISS_SIDE_PAD) and front_dist < (CAPTURE_FRONT_TOL + 0.18):
        return "missed_side"

    if travel > MAX_DYNAMIC_TRAVEL_M:
        return "culled_far"

    return None

def maybe_mark_corridor_entry(state, particle_id: int, p_local, front_dist, recv):
    if particle_id in state["corridor_entry_ids"]:
        return
    if front_dist < 0.0 or front_dist > (CAPTURE_FRONT_TOL + 0.12):
        return
    if not is_inside_soft_capture_window(p_local, recv):
        return
    state["corridor_entry_ids"].add(particle_id)
    state["entered_capture_corridor"] += 1

def run_freeze_pass(stage, state, current_frame: int, frame_dt: float):
    if state["deposited_count"] >= MAX_DEPOSITED:
        return

    nozzle_world = get_world_pos(stage, NOZZLE_LOCATOR_PATH)
    aim_world = get_world_pos(stage, AIM_LOCATOR_PATH)
    spray_world = normalize(aim_world - nozzle_world)

    new_deposits = []
    root = stage.GetPrimAtPath(DYNAMIC_ROOT)
    if not root or not root.IsValid():
        return

    capture_count = 0
    culled_counts = {
        "missed_side": 0,
        "missed_below": 0,
        "missed_behind": 0,
        "culled_far": 0,
    }

    for child in root.GetChildren():
        inst = UsdGeom.PointInstancer(child)
        if not inst:
            continue

        pos_attr = inst.GetPositionsAttr()
        vel_attr = inst.GetVelocitiesAttr()
        id_attr = inst.GetIdsAttr()
        proto_attr = inst.GetProtoIndicesAttr()
        scale_attr = inst.GetScalesAttr()
        orient_attr = inst.GetOrientationsAttr()

        positions = list(pos_attr.Get() or [])
        velocities = list(vel_attr.Get() or [])
        ids = list(id_attr.Get() or [])
        proto = list(proto_attr.Get() or [])
        scales = list(scale_attr.Get() or [])
        orients = list(orient_attr.Get() or [])

        n = min(len(positions), len(velocities), len(ids), len(proto), len(scales), len(orients))
        if n == 0:
            continue

        keep_positions = []
        keep_velocities = []
        keep_ids = []
        keep_proto = []
        keep_scales = []
        keep_orients = []

        for i in range(n):
            p = Gf.Vec3f(float(positions[i][0]), float(positions[i][1]), float(positions[i][2]))
            v = Gf.Vec3f(float(velocities[i][0]), float(velocities[i][1]), float(velocities[i][2]))
            particle_id = int(ids[i])

            p_local, front_dist = receiver_local_coords_and_front_dist(p, state["receiver_info"])
            maybe_mark_corridor_entry(state, particle_id, p_local, front_dist, state["receiver_info"])

            do_capture, capture_local, capture_mode = should_freeze_particle(
                p,
                v,
                nozzle_world,
                spray_world,
                state["receiver_info"],
                frame_dt,
            )

            if state["deposited_count"] + len(new_deposits) < MAX_DEPOSITED and do_capture:
                splat_pts = project_splat_points_from_local(capture_local, state["receiver_info"], state)
                room = MAX_DEPOSITED - (state["deposited_count"] + len(new_deposits))
                if room > 0:
                    new_deposits.extend(splat_pts[:room])

                capture_count += 1
                if particle_id not in state["captured_particle_ids"]:
                    state["captured_particle_ids"].add(particle_id)
                    state["captured_parent_particles"] += 1
                    if capture_mode == "crossed":
                        state["captured_via_crossing"] += 1
                    elif capture_mode == "predicted":
                        state["captured_via_predicted"] += 1
                    elif capture_mode == "side_funnel":
                        state["captured_via_side_funnel"] += 1
                    else:
                        state["captured_via_corridor"] += 1
                continue

            miss_reason = classify_miss_particle(p, v, nozzle_world, state["receiver_info"])
            if miss_reason is not None:
                culled_counts[miss_reason] += 1
                if particle_id not in state["missed_particle_ids"]:
                    state["missed_particle_ids"].add(particle_id)
                    state[miss_reason] += 1
                continue

            keep_positions.append(p)
            keep_velocities.append(v)
            keep_ids.append(particle_id)
            keep_proto.append(int(proto[i]))
            keep_scales.append(Gf.Vec3f(float(scales[i][0]), float(scales[i][1]), float(scales[i][2])))
            q = orients[i]
            keep_orients.append(
                Gf.Quath(
                    float(q.GetReal()),
                    Gf.Vec3h(q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]),
                )
            )

        pos_attr.Set(Vt.Vec3fArray(keep_positions))
        vel_attr.Set(Vt.Vec3fArray(keep_velocities))
        id_attr.Set(Vt.Int64Array(keep_ids))
        proto_attr.Set(Vt.IntArray(keep_proto))
        scale_attr.Set(Vt.Vec3fArray(keep_scales))
        orient_attr.Set(Vt.QuathArray(keep_orients))

    if new_deposits:
        append_deposited_points(stage, state, new_deposits)
        state["deposited_splats_total"] = state["deposited_count"]

    if capture_count or any(v > 0 for v in culled_counts.values()):
        log(
            f"CAPTURE frame={current_frame} "
            f"captured_parents={capture_count} "
            f"deposited_splats_total={state['deposited_count']} "
            f"miss_side={culled_counts['missed_side']} "
            f"miss_below={culled_counts['missed_below']} "
            f"miss_behind={culled_counts['missed_behind']} "
            f"culled_far={culled_counts['culled_far']}"
        )


def compute_deposit_metrics(state):
    occupied_cells = len(state["deposit_cell_counts"])
    deposited_splats = state["deposited_count"]
    captured_particles = state["captured_parent_particles"]

    cell_area_m2 = DEPOSIT_CELL_SIZE * DEPOSIT_CELL_SIZE
    coverage_area_m2 = occupied_cells * cell_area_m2

    aim_zone_area_m2 = math.pi * AIM_ELLIPSE_SIDE_RADIUS * AIM_ELLIPSE_VERTICAL_RADIUS
    coverage_pct = 0.0
    if aim_zone_area_m2 > 1e-12:
        coverage_pct = 100.0 * coverage_area_m2 / aim_zone_area_m2

    max_stack = 0
    avg_stack = 0.0
    if occupied_cells > 0:
        stacks = list(state["deposit_cell_counts"].values())
        max_stack = max(stacks)
        avg_stack = sum(stacks) / float(len(stacks))

    max_thickness_m = 0.0
    avg_thickness_m = 0.0
    if max_stack > 0:
        max_thickness_m = DEPOSIT_NORMAL_OFFSET + max(0, max_stack - 1) * STACK_STEP
    if avg_stack > 0.0:
        avg_thickness_m = DEPOSIT_NORMAL_OFFSET + max(0.0, avg_stack - 1.0) * STACK_STEP

    capture_efficiency = 0.0
    if state["emitted_particles"] > 0:
        capture_efficiency = captured_particles / float(state["emitted_particles"])

    return {
        "captured_particles": captured_particles,
        "deposited_splats": deposited_splats,
        "occupied_cells": occupied_cells,
        "coverage_area_m2": coverage_area_m2,
        "coverage_pct": coverage_pct,
        "capture_efficiency": capture_efficiency,
        "max_stack": max_stack,
        "avg_stack": avg_stack,
        "max_thickness_m": max_thickness_m,
        "avg_thickness_m": avg_thickness_m,
    }

def metrics_csv_row(state, snapshot_label: str, current_time_sec: float, dynamic_live: int, metrics=None, finalize_reason: str = ""):
    m = metrics or compute_deposit_metrics(state)
    return (
        f"{current_time_sec:.3f},"
        f"{snapshot_label},"
        f"{state.get('run_id', '')},"
        f"{state.get('config_hash', '')},"
        f"{state['emitted_particles']},"
        f"{dynamic_live},"
        f"{state['max_dynamic_live']},"
        f"{state['entered_capture_corridor']},"
        f"{m['captured_particles']},"
        f"{m['deposited_splats']},"
        f"{state['captured_via_corridor']},"
        f"{state['captured_via_predicted']},"
        f"{state['captured_via_crossing']},"
        f"{state['captured_via_side_funnel']},"
        f"{state['missed_side']},"
        f"{state['missed_below']},"
        f"{state['missed_behind']},"
        f"{state['culled_far']},"
        f"{m['occupied_cells']},"
        f"{m['coverage_area_m2']:.6f},"
        f"{m['coverage_pct']:.3f},"
        f"{m['capture_efficiency']:.3f},"
        f"{m['avg_stack']:.3f},"
        f"{m['max_stack']},"
        f"{m['avg_thickness_m']:.6f},"
        f"{m['max_thickness_m']:.6f},"
        f"{finalize_reason}\n"
    )

def append_metrics_csv(state, snapshot_label: str, current_time_sec: float, dynamic_live: int, metrics=None, finalize_reason: str = ""):
    paths = list(state.get("metrics_csv_paths") or [])
    if not paths:
        legacy_path = state.get("metrics_csv_path")
        if legacy_path:
            paths = [legacy_path]
    if not paths:
        return

    line = metrics_csv_row(
        state,
        snapshot_label,
        current_time_sec,
        dynamic_live,
        metrics=metrics,
        finalize_reason=finalize_reason,
    )
    for path in paths:
        try:
            with open(path, "a", encoding="utf-8") as f:
                f.write(line)
        except Exception:
            pass

def count_dynamic_particles(stage):
    total = 0
    root = stage.GetPrimAtPath(DYNAMIC_ROOT)
    if not root or not root.IsValid():
        return total
    for child in root.GetChildren():
        try:
            inst = UsdGeom.PointInstancer(child)
            if not inst:
                continue
            arr = inst.GetPositionsAttr().Get()
            if arr is not None:
                total += len(arr)
        except Exception:
            pass
    return total

def record_metrics_snapshot(stage, state, snapshot_label: str, current_time_sec: float, dynamic_live=None, metrics=None, finalize_reason: str = ""):
    if dynamic_live is None:
        dynamic_live = count_dynamic_particles(stage)
    if metrics is None:
        metrics = compute_deposit_metrics(state)
    state["max_dynamic_live"] = max(state["max_dynamic_live"], dynamic_live)

    extra = f" finalize_reason={finalize_reason}" if finalize_reason else ""
    log(
        f"{snapshot_label} t={current_time_sec:.2f}s "
        f"emitted={state['emitted_particles']} "
        f"dynamic_live={dynamic_live} "
        f"corridor={state['entered_capture_corridor']} "
        f"captured={metrics['captured_particles']} "
        f"splats={metrics['deposited_splats']} "
        f"capture_eff={metrics['capture_efficiency']:.3f} "
        f"miss_side={state['missed_side']} "
        f"miss_below={state['missed_below']} "
        f"miss_behind={state['missed_behind']} "
        f"cells={metrics['occupied_cells']} "
        f"coverage_pct={metrics['coverage_pct']:.2f} "
        f"avg_stack={metrics['avg_stack']:.2f} "
        f"max_stack={metrics['max_stack']} "
        f"avg_thick_mm={metrics['avg_thickness_m']*1000.0:.1f} "
        f"max_thick_mm={metrics['max_thickness_m']*1000.0:.1f}"
        f"{extra}"
    )

    csv_key = (snapshot_label, int(round(current_time_sec * 1000.0)), finalize_reason)
    if csv_key not in state["csv_logged_keys"]:
        append_metrics_csv(
            state,
            snapshot_label,
            current_time_sec,
            dynamic_live,
            metrics=metrics,
            finalize_reason=finalize_reason,
        )
        state["csv_logged_keys"].add(csv_key)

    return dynamic_live, metrics

def log_metrics_snapshot(stage, state, current_frame: int, snapshot_label: str):
    current_time_sec = current_frame / FPS
    record_metrics_snapshot(stage, state, snapshot_label, current_time_sec)

def maybe_log_metrics(stage, state, current_frame: int):
    if current_frame >= state["next_status_frame"]:
        log_metrics_snapshot(stage, state, current_frame, "STATUS")
        state["next_status_frame"] = current_frame + METRIC_LOG_INTERVAL_FRAMES

    for t in METRIC_TIMES_SEC:
        frame_t = int(round(t * FPS))
        if current_frame >= frame_t and t not in state["logged_times"]:
            log_metrics_snapshot(stage, state, current_frame, f"METRIC_{t:.1f}s")
            state["logged_times"].add(t)

def dominant_loss_channel(state):
    items = [
        ("missed_side", state["missed_side"]),
        ("missed_below", state["missed_below"]),
        ("missed_behind", state["missed_behind"]),
        ("culled_far", state["culled_far"]),
    ]
    return max(items, key=lambda kv: kv[1])

def build_run_summary_lines(stage, state, current_time_sec: float, dynamic_live: int, metrics, finalize_reason: str):
    dominant_name, dominant_value = dominant_loss_channel(state)
    interpretation = ""
    if dominant_name == "missed_side" and dominant_value > 0:
        interpretation = "Dominant remaining loss is lateral overspray / side interception."
    elif dominant_name == "missed_below" and dominant_value > 0:
        interpretation = "Dominant remaining loss is below-face miss / lower-edge interception."
    elif dominant_name == "missed_behind" and dominant_value > 0:
        interpretation = "Dominant remaining loss is overshoot behind the proxy face."
    elif dominant_name == "culled_far" and dominant_value > 0:
        interpretation = "Dominant remaining loss is long-tail runaway particles far from the receiver."
    else:
        interpretation = "No dominant miss channel detected."

    finalize_semantics = summarize_finalize_reason(finalize_reason)
    accounted_losses = state['missed_side'] + state['missed_below'] + state['missed_behind'] + state['culled_far']
    accounted_total = metrics['captured_particles'] + accounted_losses
    conservation_delta = state['emitted_particles'] - accounted_total

    return [
        f"Shotcrete metrics summary ({RUN_VERSION_TAG})",
        f"Run ID: {state.get('run_id', '')}",
        f"Config hash: {state.get('config_hash', '')}",
        f"Stage file: {state.get('stage_path', '')}",
        f"Finalize reason: {finalize_reason}",
        f"Finalize semantics: {finalize_semantics}",
        f"Time_s: {current_time_sec:.3f}",
        f"Run CSV: {state.get('run_csv_path', '')}",
        f"Latest CSV alias (written only after finalize): {state.get('latest_csv_path', '')}",
        f"Run TXT: {state.get('metrics_run_summary_txt_path', '')}",
        f"Latest TXT alias (written only after finalize): {state.get('metrics_latest_summary_txt_path', '')}",
        f"Run JSON: {state.get('run_json_path', '')}",
        f"Latest JSON alias (written only after finalize): {state.get('latest_json_path', '')}",
        "",
        f"Emitted particles: {state['emitted_particles']}",
        f"Dynamic live (final): {dynamic_live}",
        f"Dynamic live (max): {state['max_dynamic_live']}",
        f"Corridor entries: {state['entered_capture_corridor']}",
        f"Captured parent particles: {metrics['captured_particles']}",
        f"Deposited splats: {metrics['deposited_splats']}",
        f"Capture via corridor: {state['captured_via_corridor']}",
        f"Capture via predicted: {state['captured_via_predicted']}",
        f"Capture via crossing: {state['captured_via_crossing']}",
        f"Capture via side funnel: {state['captured_via_side_funnel']}",
        f"Missed side: {state['missed_side']}",
        f"Missed below: {state['missed_below']}",
        f"Missed behind: {state['missed_behind']}",
        f"Culled far: {state['culled_far']}",
        f"Accounted total (captured + misses): {accounted_total}",
        f"Conservation delta (emitted - accounted): {conservation_delta}",
        "",
        f"Occupied cells: {metrics['occupied_cells']}",
        f"Coverage area m2: {metrics['coverage_area_m2']:.6f}",
        f"Coverage pct: {metrics['coverage_pct']:.3f}",
        f"Capture efficiency: {metrics['capture_efficiency']:.3f}",
        f"Avg stack: {metrics['avg_stack']:.3f}",
        f"Max stack: {metrics['max_stack']}",
        f"Avg thickness mm: {metrics['avg_thickness_m'] * 1000.0:.3f}",
        f"Max thickness mm: {metrics['max_thickness_m'] * 1000.0:.3f}",
        "",
        f"Dominant loss channel: {dominant_name} ({dominant_value})",
        f"Interpretation: {interpretation}",
        f"Aim side local bias m: {AIM_SIDE_LOCAL_BIAS_M:.4f}",
        f"Aim vertical local bias m: {AIM_VERTICAL_LOCAL_BIAS_M:.4f}",
        f"Latest alias updated: {state.get('latest_alias_updated', False)}",
        "",
        "Recommended next step: validate that this full-run CSV matches the visual branch before proxy-to-cave mapping.",
    ]


def write_run_summary_txt(stage, state, current_time_sec: float, dynamic_live: int, metrics, finalize_reason: str):
    path = state.get("metrics_run_summary_txt_path")
    if not path:
        return False

    lines = build_run_summary_lines(stage, state, current_time_sec, dynamic_live, metrics, finalize_reason)
    try:
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        return True
    except Exception:
        return False

def build_run_summary_payload(stage, state, current_time_sec: float, dynamic_live: int, metrics, finalize_reason: str):
    dominant_name, dominant_value = dominant_loss_channel(state)
    accounted_losses = state['missed_side'] + state['missed_below'] + state['missed_behind'] + state['culled_far']
    accounted_total = metrics['captured_particles'] + accounted_losses
    conservation_delta = state['emitted_particles'] - accounted_total
    return {
        "run_version_tag": RUN_VERSION_TAG,
        "run_id": state.get('run_id', ''),
        "config_hash": state.get('config_hash', ''),
        "stage_path": state.get('stage_path', ''),
        "finalize_reason": finalize_reason,
        "finalize_semantics": summarize_finalize_reason(finalize_reason),
        "time_s": round(float(current_time_sec), 6),
        "paths": {
            "run_csv": state.get('run_csv_path', ''),
            "latest_csv_alias": state.get('latest_csv_path', ''),
            "run_summary_txt": state.get('metrics_run_summary_txt_path', ''),
            "latest_summary_txt_alias": state.get('metrics_latest_summary_txt_path', ''),
            "run_json": state.get('run_json_path', ''),
            "latest_json_alias": state.get('latest_json_path', ''),
        },
        "counts": {
            "emitted_particles": state['emitted_particles'],
            "dynamic_live_final": dynamic_live,
            "dynamic_live_max": state['max_dynamic_live'],
            "corridor_entries": state['entered_capture_corridor'],
            "captured_parent_particles": metrics['captured_particles'],
            "deposited_splats": metrics['deposited_splats'],
            "captured_via_corridor": state['captured_via_corridor'],
            "captured_via_predicted": state['captured_via_predicted'],
            "captured_via_crossing": state['captured_via_crossing'],
            "captured_via_side_funnel": state['captured_via_side_funnel'],
            "missed_side": state['missed_side'],
            "missed_below": state['missed_below'],
            "missed_behind": state['missed_behind'],
            "culled_far": state['culled_far'],
            "accounted_total": accounted_total,
            "conservation_delta": conservation_delta,
        },
        "coverage": {
            "occupied_cells": metrics['occupied_cells'],
            "coverage_area_m2": metrics['coverage_area_m2'],
            "coverage_pct": metrics['coverage_pct'],
            "capture_efficiency": metrics['capture_efficiency'],
            "avg_stack": metrics['avg_stack'],
            "max_stack": metrics['max_stack'],
            "avg_thickness_m": metrics['avg_thickness_m'],
            "max_thickness_m": metrics['max_thickness_m'],
        },
        "dominant_loss_channel": {
            "name": dominant_name,
            "value": dominant_value,
        },
        "bias_knobs": {
            "aim_side_local_bias_m": AIM_SIDE_LOCAL_BIAS_M,
            "aim_vertical_local_bias_m": AIM_VERTICAL_LOCAL_BIAS_M,
        },
        "latest_alias_updated": state.get('latest_alias_updated', False),
        "manifest": state.get('run_config_manifest', {}),
    }


def sync_latest_aliases(state) -> bool:
    ok_csv = copy_file(state.get('run_csv_path', ''), state.get('latest_csv_path', ''))
    ok_txt = copy_file(state.get('metrics_run_summary_txt_path', ''), state.get('metrics_latest_summary_txt_path', ''))
    ok_json = copy_file(state.get('run_json_path', ''), state.get('latest_json_path', ''))
    state['latest_alias_updated'] = bool(ok_csv and ok_txt and ok_json)
    return state['latest_alias_updated']


def finalize_run(stage, state, current_time_sec: float, finalize_reason: str):
    if state.get("finalized") or state.get("finalize_in_progress"):
        return

    state["finalize_in_progress"] = True
    try:
        dynamic_live = count_dynamic_particles(stage)
        metrics = compute_deposit_metrics(state)
        record_metrics_snapshot(
            stage,
            state,
            "FINAL",
            current_time_sec,
            dynamic_live=dynamic_live,
            metrics=metrics,
            finalize_reason=finalize_reason,
        )

        state["finalized"] = True
        state["finalize_reason"] = finalize_reason
        state["final_time_sec"] = current_time_sec

        txt_ok = write_run_summary_txt(stage, state, current_time_sec, dynamic_live, metrics, finalize_reason)
        payload = build_run_summary_payload(stage, state, current_time_sec, dynamic_live, metrics, finalize_reason)
        json_ok = write_json_file(state.get("run_json_path"), payload)
        # --- AUTO PROMOTE JSON TO LATEST (no manual renaming ever again) ---
        latest_json_path = state.get("latest_json_path")
        run_json_path = state.get("run_json_path")

        if run_json_path and latest_json_path:
            try:
                shutil.copyfile(run_json_path, latest_json_path)
            except Exception:
                pass
        alias_ok = sync_latest_aliases(state)
        if alias_ok:
            txt_ok = write_run_summary_txt(stage, state, current_time_sec, dynamic_live, metrics, finalize_reason)
            payload = build_run_summary_payload(stage, state, current_time_sec, dynamic_live, metrics, finalize_reason)
            json_ok = write_json_file(state.get("run_json_path"), payload)
            latest_json_path = state.get("latest_json_path")
            run_json_path = state.get("run_json_path")

            if run_json_path and latest_json_path:
                try:
                    shutil.copyfile(run_json_path, latest_json_path)
                except Exception:
                    pass
            sync_latest_aliases(state)

        log(
            f"FINALIZED run_id={state.get('run_id', '')} "
            f"reason={finalize_reason} "
            f"run_csv={state.get('run_csv_path', '')} "
            f"latest_csv={state.get('latest_csv_path', '')} "
            f"summary_ok={txt_ok} json_ok={json_ok} latest_ok={alias_ok}"
        )
        if state.get("metrics_run_summary_txt_path"):
            log(f"RUN_SUMMARY_TXT = {state['metrics_run_summary_txt_path']}")
        if state.get("metrics_latest_summary_txt_path"):
            log(f"LATEST_SUMMARY_TXT = {state['metrics_latest_summary_txt_path']}")
        if state.get("run_json_path"):
            log(f"RUN_SUMMARY_JSON = {state['run_json_path']}")
        if state.get("latest_json_path"):
            log(f"LATEST_SUMMARY_JSON = {state['latest_json_path']}")
    finally:
        state["finalize_in_progress"] = False

def maybe_finalize_run(stage, state, current_frame: int, current_time_sec: float):
    if state.get("finalized"):
        return

    if current_time_sec >= state.get("hard_finalize_time_sec", 1.0e9):
        finalize_run(stage, state, current_time_sec, "time_cap")
        return

    emit_end_frame = START_FRAME + EMIT_FRAMES
    if current_frame < emit_end_frame + int(MIN_POST_EMIT_FINALIZE_SECONDS * FPS):
        return
    if current_frame < state["next_finalize_probe_frame"]:
        return

    dynamic_live = count_dynamic_particles(stage)
    state["max_dynamic_live"] = max(state["max_dynamic_live"], dynamic_live)
    state["next_finalize_probe_frame"] = current_frame + FINALIZE_PROBE_INTERVAL_FRAMES

    if dynamic_live <= FINALIZE_DYNAMIC_LIVE_THRESHOLD:
        state["quiet_probe_hits"] += 1
    else:
        state["quiet_probe_hits"] = 0

    if state["quiet_probe_hits"] >= FINALIZE_QUIET_PROBES:
        finalize_run(stage, state, current_time_sec, "quiet_tail")

def on_update(_event):
    stage = get_stage()
    state = getattr(builtins, STATE_KEY, None)
    if state is None:
        return

    tl = omni.timeline.get_timeline_interface()
    current_time_sec = float(tl.get_current_time())

    if not tl.is_playing():
        if state.get("has_started") and not state.get("finalized"):
            finalize_run(stage, state, current_time_sec, "timeline_stopped")
        return

    current_frame = int(current_time_sec * FPS + 0.5)
    if current_frame < START_FRAME:
        return
    if current_frame == state["last_frame"]:
        return

    state["has_started"] = True
    if state.get("finalized"):
        state["last_frame"] = current_frame
        return

    prev_frame = state["last_frame"]
    frame_step = 1 if prev_frame < 0 else max(1, current_frame - prev_frame)
    frame_dt = frame_step / FPS
    state["last_frame"] = current_frame

    emit_end_frame = START_FRAME + EMIT_FRAMES
    if current_frame < emit_end_frame:
        state["particle_accum"] += TARGET_PARTICLES_PER_SEC * frame_dt
        n = int(math.floor(state["particle_accum"]))
        if n > 0:
            state["particle_accum"] -= n
            emit_particle_set(stage, state, n)

    run_freeze_pass(stage, state, current_frame, frame_dt)
    maybe_log_metrics(stage, state, current_frame)
    maybe_finalize_run(stage, state, current_frame, current_time_sec)

def build():
    stage = get_stage()
    clear_old_state()

    delete_prim_if_exists(stage, RIG_ROOT)
    delete_prim_if_exists(stage, "/ShotcreteShapeFreezeRig")
    root = stage.GetPseudoRoot()
    for child in list(root.GetChildren()):
        if child.GetName().startswith("ShotcreteProbeRun_"):
            stage.RemovePrim(child.GetPath())

    ensure_rig(stage)
    ensure_particle_system(stage)
    ensure_material(stage)
    sync_markers(stage)

    recv = stage.GetPrimAtPath(RECEIVER_PATH)
    if recv and recv.IsValid():
        try:
            UsdGeom.Imageable(recv).MakeInvisible()
        except Exception:
            pass

    metrics_bundle = resolve_metrics_output_bundle(stage)
    write_metrics_csv_headers(metrics_bundle["csv_paths"])

    state = {
        "last_frame": -1,
        "set_count": 0,
        "next_particle_id": 1,
        "next_deposit_id": 1,
        "particle_accum": 0.0,
        "emitted_particles": 0,
        "deposited_count": 0,
        "deposited_splats_total": 0,
        "captured_parent_particles": 0,
        "captured_via_corridor": 0,
        "captured_via_predicted": 0,
        "captured_via_crossing": 0,
        "captured_via_side_funnel": 0,
        "entered_capture_corridor": 0,
        "missed_side": 0,
        "missed_below": 0,
        "missed_behind": 0,
        "culled_far": 0,
        "max_dynamic_live": 0,
        "run_id": metrics_bundle["run_id"],
        "stage_path": metrics_bundle["stage_path"],
        "run_csv_path": metrics_bundle["run_csv_path"],
        "latest_csv_path": metrics_bundle["latest_csv_path"],
        "metrics_run_summary_txt_path": metrics_bundle["run_summary_txt_path"],
        "metrics_latest_summary_txt_path": metrics_bundle["latest_summary_txt_path"],
        "run_json_path": metrics_bundle["run_json_path"],
        "latest_json_path": metrics_bundle["latest_json_path"],
        "metrics_csv_paths": metrics_bundle["csv_paths"],
        "metrics_csv_path": metrics_bundle["run_csv_path"],
        "run_config_manifest": None,
        "config_hash": "",
        "has_started": False,
        "finalized": False,
        "finalize_in_progress": False,
        "finalize_reason": "",
        "final_time_sec": 0.0,
        "quiet_probe_hits": 0,
        "next_finalize_probe_frame": START_FRAME + EMIT_FRAMES + FINALIZE_PROBE_INTERVAL_FRAMES,
        "hard_finalize_time_sec": (START_FRAME / FPS) + EMIT_DURATION_SECONDS + HARD_FINALIZE_EXTRA_SECONDS,
        "next_status_frame": START_FRAME + METRIC_LOG_INTERVAL_FRAMES,
        "logged_times": set(),
        "csv_logged_keys": set(),
        "deposit_cell_counts": {},
        "corridor_entry_ids": set(),
        "captured_particle_ids": set(),
        "missed_particle_ids": set(),
        "receiver_info": build_receiver_info(stage),
    }
    state["run_config_manifest"] = build_run_config_manifest()
    state["config_hash"] = compute_config_hash_short(state["run_config_manifest"])
    state["latest_alias_updated"] = False

    setattr(builtins, STATE_KEY, state)

    sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        on_update, name="shotcrete_face_locked_capture_corridor_metrics"
    )
    setattr(builtins, SUB_KEY, sub)

    log("Ready.")
    log("Mode = continuous/quasi-continuous + face-locked BROAD-SPLAT + side-funnel capture + lower-face guard + FULL-RUN METRICS")
    log(f"Receiver = {RECEIVER_PATH}")
    log(f"TARGET_PARTICLES_PER_SEC = {TARGET_PARTICLES_PER_SEC}")
    log(f"RUN_ID = {state['run_id']}")
    log(f"CONFIG_HASH = {state['config_hash']}")
    if state.get("run_csv_path"):
        log(f"RUN_CSV = {state['run_csv_path']}")
    if state.get("latest_csv_path"):
        log(f"LATEST_CSV_ALIAS = {state['latest_csv_path']}")
    if state.get("metrics_run_summary_txt_path"):
        log(f"RUN_SUMMARY_TXT = {state['metrics_run_summary_txt_path']}")
    if state.get("metrics_latest_summary_txt_path"):
        log(f"LATEST_SUMMARY_TXT_ALIAS = {state['metrics_latest_summary_txt_path']}")
    if state.get("run_json_path"):
        log(f"RUN_SUMMARY_JSON = {state['run_json_path']}")
    if state.get("latest_json_path"):
        log(f"LATEST_SUMMARY_JSON_ALIAS = {state['latest_json_path']}")
    log(f"EMIT_DURATION_SECONDS = {EMIT_DURATION_SECONDS}")
    log(f"RECEIVER_FACE_AXIS = {RECEIVER_FACE_AXIS}")
    log(f"RECEIVER_FACE_SIGN = {RECEIVER_FACE_SIGN}")
    log(f"AIM_ZONE_RADIUS = {AIM_ZONE_RADIUS}")
    log(f"CAPTURE_FRONT_TOL = {CAPTURE_FRONT_TOL}")
    log(f"CAPTURE_BACK_TOL = {CAPTURE_BACK_TOL}")
    log(f"CAPTURE_TANGENTIAL_PAD_SIDE = {CAPTURE_TANGENTIAL_PAD_SIDE}")
    log(f"CAPTURE_TANGENTIAL_PAD_VERTICAL = {CAPTURE_TANGENTIAL_PAD_VERTICAL}")
    log(f"AIM_ELLIPSE_SIDE_RADIUS = {AIM_ELLIPSE_SIDE_RADIUS}")
    log(f"AIM_ELLIPSE_VERTICAL_RADIUS = {AIM_ELLIPSE_VERTICAL_RADIUS}")
    log(f"SIDE_FUNNEL_CAPTURE_PAD = {SIDE_FUNNEL_CAPTURE_PAD}")
    log(f"FACE_PAINT_SIDE_MARGIN = {FACE_PAINT_SIDE_MARGIN}")
    log(f"FACE_PAINT_BOTTOM_MARGIN = {FACE_PAINT_BOTTOM_MARGIN}")
    log("Metrics now separate captured parent particles from deposited splat instances.")
    log("Capture now includes side-funnel snapping for modest lateral overshoot and a stricter lower-face guard.")
    log("CSV now logs STATUS and METRIC checkpoints without relying on exact frame multiples.")


# -----------------------------------------------------------------------------
# v16.2 exact-field export overrides
# -----------------------------------------------------------------------------

RUN_VERSION_TAG = "v16_2"
METRICS_LATEST_CSV_BASENAME = f"shotcrete_metrics_broad_splat_{RUN_VERSION_TAG}_latest.csv"
METRICS_RUN_CSV_PREFIX = f"shotcrete_metrics_broad_splat_{RUN_VERSION_TAG}_run_"
METRICS_RUN_SUMMARY_TXT_PREFIX = f"shotcrete_metrics_broad_splat_{RUN_VERSION_TAG}_run_"
METRICS_LATEST_SUMMARY_TXT_BASENAME = f"shotcrete_metrics_broad_splat_{RUN_VERSION_TAG}_latest.txt"
METRICS_RUN_JSON_PREFIX = f"shotcrete_metrics_broad_splat_{RUN_VERSION_TAG}_run_"
METRICS_LATEST_JSON_BASENAME = f"shotcrete_metrics_broad_splat_{RUN_VERSION_TAG}_latest.json"
EXACT_FIELD_RUN_JSON_PREFIX = "shotcrete_proxy_field_v16_2_run_"
EXACT_FIELD_LATEST_JSON_BASENAME = "shotcrete_proxy_field_v16_2_latest.json"
EXACT_FIELD_RUN_TXT_PREFIX = "shotcrete_proxy_field_v16_2_run_"
EXACT_FIELD_LATEST_TXT_BASENAME = "shotcrete_proxy_field_v16_2_latest.txt"


def _v16_2_exact_field_paths(state):
    primary_dir = os.path.dirname(state.get("run_csv_path", "") or "")
    run_id = state.get("run_id", "") or datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    if not primary_dir:
        return {"run_json": None, "latest_json": None, "run_txt": None, "latest_txt": None}
    return {
        "run_json": os.path.join(primary_dir, f"{EXACT_FIELD_RUN_JSON_PREFIX}{run_id}.json"),
        "latest_json": os.path.join(primary_dir, EXACT_FIELD_LATEST_JSON_BASENAME),
        "run_txt": os.path.join(primary_dir, f"{EXACT_FIELD_RUN_TXT_PREFIX}{run_id}.txt"),
        "latest_txt": os.path.join(primary_dir, EXACT_FIELD_LATEST_TXT_BASENAME),
    }


def _v16_2_receiver_manifest(recv):
    wm = recv["world_m"]
    rows = []
    for r in range(4):
        rows.append([float(wm[r][0]), float(wm[r][1]), float(wm[r][2]), float(wm[r][3])])
    return {
        "face_axis": int(recv["face_axis"]),
        "face_sign": float(recv["face_sign"]),
        "face_coord": float(recv["face_coord"]),
        "half": [float(x) for x in recv["half"]],
        "tangential_axes": [int(x) for x in recv["tangential_axes"]],
        "aim_local_on_face": [float(x) for x in recv["aim_local_on_face"]],
        "aim_world": [float(recv["aim_world"][0]), float(recv["aim_world"][1]), float(recv["aim_world"][2])],
        "face_normal_world": [float(recv["face_normal_world"][0]), float(recv["face_normal_world"][1]), float(recv["face_normal_world"][2])],
        "world_m_rows": rows,
    }


def _v16_2_exact_proxy_field_payload(stage, state, current_time_sec, metrics, finalize_reason):
    recv = state["receiver_info"]
    t0, t1 = recv["tangential_axes"]
    aim_local = recv["aim_local_on_face"]
    face_axis = recv["face_axis"]
    face_coord = recv["face_coord"]
    face_sign = recv["face_sign"]
    cell_area = DEPOSIT_CELL_SIZE * DEPOSIT_CELL_SIZE
    cells = []
    total_volume_proxy = 0.0
    for key in sorted(state["deposit_cell_counts"].keys()):
        c0, c1 = int(key[0]), int(key[1])
        stack_count = int(state["deposit_cell_counts"][key])
        thickness = stack_count * STACK_STEP
        local_center = [0.0, 0.0, 0.0]
        local_center[t0] = aim_local[t0] + c0 * DEPOSIT_CELL_SIZE
        local_center[t1] = aim_local[t1] + c1 * DEPOSIT_CELL_SIZE
        local_center = clamp_local_to_paint_window(local_center, recv)
        local_center[face_axis] = face_coord + face_sign * (DEPOSIT_NORMAL_OFFSET + max(stack_count - 1, 0) * STACK_STEP)
        world_center = world_from_receiver_local(local_center, recv)
        vol = cell_area * thickness
        total_volume_proxy += vol
        cells.append({
            "cell_key": [c0, c1],
            "stack_count": stack_count,
            "thickness_m": float(thickness),
            "cell_area_m2": float(cell_area),
            "volume_proxy_m3": float(vol),
            "local_center": [float(local_center[0]), float(local_center[1]), float(local_center[2])],
            "world_center": [float(world_center[0]), float(world_center[1]), float(world_center[2])],
        })
    nozzle_world = get_world_pos(stage, NOZZLE_LOCATOR_PATH)
    return {
        "run_version_tag": RUN_VERSION_TAG,
        "run_id": state.get("run_id", ""),
        "config_hash": state.get("config_hash", ""),
        "stage_path": state.get("stage_path", ""),
        "time_s": round(float(current_time_sec), 6),
        "finalize_reason": finalize_reason,
        "receiver_path": RECEIVER_PATH,
        "nozzle_locator_path": NOZZLE_LOCATOR_PATH,
        "aim_locator_path": AIM_LOCATOR_PATH,
        "nozzle_world": [float(nozzle_world[0]), float(nozzle_world[1]), float(nozzle_world[2])],
        "receiver_manifest": _v16_2_receiver_manifest(recv),
        "field_constants": {
            "deposit_cell_size_m": float(DEPOSIT_CELL_SIZE),
            "deposit_normal_offset_m": float(DEPOSIT_NORMAL_OFFSET),
            "stack_step_m": float(STACK_STEP),
            "splat_count": int(SPLAT_COUNT),
            "particle_radius_m": float(PARTICLE_RADIUS),
        },
        "baseline_summary": {
            "occupied_cells": int(metrics["occupied_cells"]),
            "coverage_area_m2": float(metrics["coverage_area_m2"]),
            "volume_proxy_m3": float(sum((v * STACK_STEP) * cell_area for v in state["deposit_cell_counts"].values())),
            "capture_efficiency": float(metrics["capture_efficiency"]),
        },
        "cells": cells,
        "cell_count": len(cells),
        "coverage_area_m2": float(len(cells) * cell_area),
        "volume_proxy_m3": float(total_volume_proxy),
    }


def _v16_2_exact_proxy_field_lines(payload):
    return [
        f"Exact proxy field export ({RUN_VERSION_TAG})",
        f"Run id: {payload.get('run_id', '')}",
        f"Config hash: {payload.get('config_hash', '')}",
        f"Finalize reason: {payload.get('finalize_reason', '')}",
        f"Time s: {payload.get('time_s', 0.0):.3f}",
        f"Receiver path: {payload.get('receiver_path', '')}",
        f"Target source cell count: {payload.get('baseline_summary', {}).get('occupied_cells', 0)}",
        f"Exported exact field cells: {payload.get('cell_count', 0)}",
        f"Coverage area m2: {payload.get('coverage_area_m2', 0.0):.6f}",
        f"Volume proxy m3: {payload.get('volume_proxy_m3', 0.0):.6f}",
        f"Nozzle world: {payload.get('nozzle_world', [])}",
    ]


def sync_latest_aliases(state) -> bool:
    ok_csv = copy_file(state.get('run_csv_path', ''), state.get('latest_csv_path', ''))
    ok_txt = copy_file(state.get('metrics_run_summary_txt_path', ''), state.get('metrics_latest_summary_txt_path', ''))
    ok_json = copy_file(state.get('run_json_path', ''), state.get('latest_json_path', ''))
    field_paths = _v16_2_exact_field_paths(state)
    ok_field_json = copy_file(field_paths.get('run_json', ''), field_paths.get('latest_json', '')) if field_paths.get('run_json') else False
    ok_field_txt = copy_file(field_paths.get('run_txt', ''), field_paths.get('latest_txt', '')) if field_paths.get('run_txt') else False
    state['latest_alias_updated'] = bool(ok_csv and ok_txt and ok_json and ok_field_json and ok_field_txt)
    return state['latest_alias_updated']


def finalize_run(stage, state, current_time_sec: float, finalize_reason: str):
    if state.get("finalized") or state.get("finalize_in_progress"):
        return

    state["finalize_in_progress"] = True
    try:
        dynamic_live = count_dynamic_particles(stage)
        metrics = compute_deposit_metrics(state)
        record_metrics_snapshot(
            stage,
            state,
            "FINAL",
            current_time_sec,
            dynamic_live=dynamic_live,
            metrics=metrics,
            finalize_reason=finalize_reason,
        )

        state["finalized"] = True
        state["finalize_reason"] = finalize_reason
        state["final_time_sec"] = current_time_sec

        txt_ok = write_run_summary_txt(stage, state, current_time_sec, dynamic_live, metrics, finalize_reason)
        payload = build_run_summary_payload(stage, state, current_time_sec, dynamic_live, metrics, finalize_reason)
        json_ok = write_json_file(state.get("run_json_path"), payload)

        field_payload = _v16_2_exact_proxy_field_payload(stage, state, current_time_sec, metrics, finalize_reason)
        field_paths = _v16_2_exact_field_paths(state)
        field_json_ok = write_json_file(field_paths.get("run_json"), field_payload)
        field_txt_ok = write_text_file(field_paths.get("run_txt"), "\n".join(_v16_2_exact_proxy_field_lines(field_payload)))

        alias_ok = sync_latest_aliases(state)

        log(
            f"FINALIZED run_id={state.get('run_id', '')} reason={finalize_reason} "
            f"run_csv={state.get('run_csv_path', '')} latest_csv={state.get('latest_csv_path', '')} "
            f"summary_ok={txt_ok} json_ok={json_ok} exact_field_json_ok={field_json_ok} exact_field_txt_ok={field_txt_ok} latest_ok={alias_ok}"
        )
        if field_paths.get("run_json"):
            log(f"EXACT_FIELD_RUN_JSON = {field_paths['run_json']}")
        if field_paths.get("latest_json"):
            log(f"EXACT_FIELD_LATEST_JSON = {field_paths['latest_json']}")
        if field_paths.get("run_txt"):
            log(f"EXACT_FIELD_RUN_TXT = {field_paths['run_txt']}")
        if field_paths.get("latest_txt"):
            log(f"EXACT_FIELD_LATEST_TXT = {field_paths['latest_txt']}")
    finally:
        state["finalize_in_progress"] = False

build()
