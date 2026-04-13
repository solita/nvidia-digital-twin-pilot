import datetime
import json
import math
import glob
import os
import shutil

import omni
from pxr import Gf, Sdf, Usd, UsdGeom, Vt

# -----------------------------------------------------------------------------
# v17.8c exact proxy-field import -> cave mapping
#
# VISUAL-DEMO revision:
#   - keep importer math / fallback behavior
#   - make mapped result clearly visible on boolean cave mesh
#   - larger spheres
#   - stronger surface offset
#   - per-cell scale from stack_count
#   - mapped = orange, fallback = yellow
#   - rejected debug hidden by default
# -----------------------------------------------------------------------------

RUN_VERSION_TAG = "v17_8c"

LEGACY_OUTPUT_DIR_HINT = "D:/Solita/Cave Sim/Scripts"
EXACT_FIELD_LATEST_JSON_BASENAME = "shotcrete_proxy_field_v16_2_latest.json"

CAVE_TARGET_MESH_PATH = "/CAVE_collision_proxy/CAVE_LOWPOLY/cave_collision/Cube_001"
SOURCE_TO_TARGET_TRANSLATE = (0.0, 0.0, 0.0)
SOURCE_TO_TARGET_SCALE = 1.0

MAX_TRANSFER_DISTANCE_M = 0.45
MIN_RECEIVE_HEMISPHERE_DOT = 0.15
ENABLE_OCCLUSION_CHECK = True
SEGMENT_EPS = 1e-5
END_EXCLUSIVE_EPS = 5e-4

# ---------------------------------------------------------------------
# Import/fallback controls
# ---------------------------------------------------------------------
ENABLE_DEMO_FALLBACK_FOR_REJECT_RECEIVE = True
ENABLE_VERBOSE_REJECT_RECEIVE_LOG = True
MAX_VERBOSE_REJECT_LOGS = 32
SECOND_PASS_RELAXED_RECEIVE_DOT = None

# ---------------------------------------------------------------------
# DEMO VISUALIZATION CONTROLS
# ---------------------------------------------------------------------
DEMO_VISUAL_MODE = True

VIS_MAPPED_RADIUS_M = 0.06
VIS_FALLBACK_RADIUS_M = 0.09
VIS_HIDE_ALIGNMENT_OVERLAY = True

# Make the sprayed region clearly visible.
MAPPED_VISUAL_OFFSET_M = 0.018
FALLBACK_VISUAL_OFFSET_M = 0.022

# Prototype radii (base before per-instance scale)
MAPPED_SPHERE_RADIUS = VIS_MAPPED_RADIUS_M
FALLBACK_SPHERE_RADIUS = VIS_FALLBACK_RADIUS_M
REJECTED_SPHERE_RADIUS = 0.010

# Per-instance scale derived from stack_count
MAPPED_SCALE_MIN = 1.6
MAPPED_SCALE_MAX = 3.6
MAPPED_SCALE_PER_STACK = 0.10

FALLBACK_SCALE_BOOST = 0.35

# Hide rejected debug by default in demo mode
SHOW_REJECTED_DEBUG = False
SHOW_MAPPED_DEBUG = True
SHOW_FALLBACK_DEBUG = True

MAP_RIG_ROOT = "/ShotcreteCaveMapRig"
MAPPED_PROTO_PATH = f"{MAP_RIG_ROOT}/MappedProto"
REJECTED_PROTO_PATH = f"{MAP_RIG_ROOT}/RejectedProto"
FALLBACK_PROTO_PATH = f"{MAP_RIG_ROOT}/FallbackProto"
MAPPED_PI_PATH = f"{MAP_RIG_ROOT}/MappedCellsPI"
REJECTED_PI_PATH = f"{MAP_RIG_ROOT}/RejectedSourceCellsPI"
FALLBACK_PI_PATH = f"{MAP_RIG_ROOT}/FallbackCellsPI"

MAPPING_RUN_CSV_PREFIX = f"shotcrete_proxy_to_cave_map_{RUN_VERSION_TAG}_run_"
MAPPING_LATEST_CSV_BASENAME = f"shotcrete_proxy_to_cave_map_{RUN_VERSION_TAG}_latest.csv"
MAPPING_RUN_TXT_PREFIX = f"shotcrete_proxy_to_cave_map_{RUN_VERSION_TAG}_run_"
MAPPING_LATEST_TXT_BASENAME = f"shotcrete_proxy_to_cave_map_{RUN_VERSION_TAG}_latest.txt"
MAPPING_RUN_JSON_PREFIX = f"shotcrete_proxy_to_cave_map_{RUN_VERSION_TAG}_run_"
MAPPING_LATEST_JSON_BASENAME = f"shotcrete_proxy_to_cave_map_{RUN_VERSION_TAG}_latest.json"


def log(msg: str) -> None:
    print(f"[SHOTCRETE_PROXY_TO_CAVE_MAP_{RUN_VERSION_TAG.upper()}] {msg}")


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


def resolve_output_bundle(stage):
    run_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    stage_path = get_stage_file_path(stage)
    stage_dir = os.path.dirname(stage_path) if stage_path else ""
    primary_dir = choose_writable_dir([LEGACY_OUTPUT_DIR_HINT, stage_dir])
    return {
        "run_id": run_id,
        "stage_path": stage_path,
        "primary_dir": primary_dir,
        "run_csv_path": os.path.join(primary_dir, f"{MAPPING_RUN_CSV_PREFIX}{run_id}.csv") if primary_dir else None,
        "latest_csv_path": os.path.join(primary_dir, MAPPING_LATEST_CSV_BASENAME) if primary_dir else None,
        "run_txt_path": os.path.join(primary_dir, f"{MAPPING_RUN_TXT_PREFIX}{run_id}.txt") if primary_dir else None,
        "latest_txt_path": os.path.join(primary_dir, MAPPING_LATEST_TXT_BASENAME) if primary_dir else None,
        "run_json_path": os.path.join(primary_dir, f"{MAPPING_RUN_JSON_PREFIX}{run_id}.json") if primary_dir else None,
        "latest_json_path": os.path.join(primary_dir, MAPPING_LATEST_JSON_BASENAME) if primary_dir else None,
    }


def copy_file(src, dst):
    try:
        shutil.copyfile(src, dst)
        return True
    except Exception:
        return False


def write_json_file(path, payload):
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
            f.write("\n")
        return True
    except Exception:
        return False


def write_text_file(path, text):
    try:
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
            if not text.endswith("\n"):
                f.write("\n")
        return True
    except Exception:
        return False


def load_exact_field_payload(stage):
    stage_path = get_stage_file_path(stage)
    stage_dir = os.path.dirname(stage_path) if stage_path else ""
    candidates = [
        os.path.join(LEGACY_OUTPUT_DIR_HINT, EXACT_FIELD_LATEST_JSON_BASENAME),
        os.path.join(stage_dir, EXACT_FIELD_LATEST_JSON_BASENAME) if stage_dir else "",
    ]
    for path in candidates:
        if path and os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                payload = json.load(f)
            payload["_loaded_from"] = path
            return payload

    run_globs = []
    if LEGACY_OUTPUT_DIR_HINT:
        run_globs.append(os.path.join(LEGACY_OUTPUT_DIR_HINT, "shotcrete_proxy_field_v16_2_run_*.json"))
    if stage_dir:
        run_globs.append(os.path.join(stage_dir, "shotcrete_proxy_field_v16_2_run_*.json"))

    run_candidates = []
    for pattern in run_globs:
        run_candidates.extend(glob.glob(pattern))
    run_candidates = sorted(
        set(p for p in run_candidates if os.path.isfile(p)),
        key=lambda p: os.path.getmtime(p),
        reverse=True,
    )

    if run_candidates:
        path = run_candidates[0]
        with open(path, "r", encoding="utf-8") as f:
            payload = json.load(f)
        payload["_loaded_from"] = path
        payload["_autodetected_run_file"] = True
        return payload

    raise RuntimeError(
        f"Could not find {EXACT_FIELD_LATEST_JSON_BASENAME} or any "
        f"shotcrete_proxy_field_v16_2_run_*.json file. "
        f"Run v16.2 first and keep its outputs in Scripts."
    )


def normalize(v):
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return Gf.Vec3f(0.0, 0.0, 0.0)
    return Gf.Vec3f(x / n, y / n, z / n)


def dot(a, b):
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def distance_sq(a, b):
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = float(a[2]) - float(b[2])
    return dx * dx + dy * dy + dz * dz


def cross(a, b):
    return Gf.Vec3f(
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    )


def add(a, b):
    return Gf.Vec3f(float(a[0]) + float(b[0]), float(a[1]) + float(b[1]), float(a[2]) + float(b[2]))


def sub(a, b):
    return Gf.Vec3f(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]), float(a[2]) - float(b[2]))


def mul(a, s):
    return Gf.Vec3f(float(a[0]) * s, float(a[1]) * s, float(a[2]) * s)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def closest_point_on_triangle(p, a, b, c):
    ab = sub(b, a)
    ac = sub(c, a)
    ap = sub(p, a)
    d1 = dot(ab, ap)
    d2 = dot(ac, ap)
    if d1 <= 0.0 and d2 <= 0.0:
        return a

    bp = sub(p, b)
    d3 = dot(ab, bp)
    d4 = dot(ac, bp)
    if d3 >= 0.0 and d4 <= d3:
        return b

    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / (d1 - d3)
        return add(a, mul(ab, v))

    cp = sub(p, c)
    d5 = dot(ab, cp)
    d6 = dot(ac, cp)
    if d6 >= 0.0 and d5 <= d6:
        return c

    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / (d2 - d6)
        return add(a, mul(ac, w))

    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        bc = sub(c, b)
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return add(b, mul(bc, w))

    denom = 1.0 / (va + vb + vc)
    v = vb * denom
    w = vc * denom
    return add(a, add(mul(ab, v), mul(ac, w)))


def segment_intersects_triangle_before_end(p0, p1, tri, eps=SEGMENT_EPS, end_exclusive_eps=END_EXCLUSIVE_EPS):
    d = sub(p1, p0)
    e1 = sub(tri["b"], tri["a"])
    e2 = sub(tri["c"], tri["a"])
    h = cross(d, e2)
    a = dot(e1, h)
    if -eps < a < eps:
        return False
    f = 1.0 / a
    s = sub(p0, tri["a"])
    u = f * dot(s, h)
    if u < 0.0 or u > 1.0:
        return False
    q = cross(s, e1)
    v = f * dot(d, q)
    if v < 0.0 or u + v > 1.0:
        return False
    t = f * dot(e2, q)
    seg_len = math.sqrt(max(distance_sq(p0, p1), eps))
    if t > eps and t < seg_len - end_exclusive_eps:
        return True
    return False


def tri_normal(a, b, c):
    return normalize(cross(sub(b, a), sub(c, a)))


def triangulate_mesh(mesh_prim):
    mesh = UsdGeom.Mesh(mesh_prim)
    points_attr = mesh.GetPointsAttr().Get()
    face_counts = mesh.GetFaceVertexCountsAttr().Get()
    face_indices = mesh.GetFaceVertexIndicesAttr().Get()
    if not points_attr or not face_counts or not face_indices:
        raise RuntimeError(f"Mesh missing topology data: {mesh_prim.GetPath()}")

    world_m = UsdGeom.Xformable(mesh_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    points = []
    for p in points_attr:
        q = world_m.Transform(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
        points.append(Gf.Vec3f(float(q[0]), float(q[1]), float(q[2])))

    tris = []
    cursor = 0
    tri_index = 0
    for face_n in face_counts:
        n = int(face_n)
        if n < 3:
            cursor += n
            continue
        ids = [int(face_indices[cursor + i]) for i in range(n)]
        cursor += n
        p0 = points[ids[0]]
        for i in range(1, n - 1):
            p1 = points[ids[i]]
            p2 = points[ids[i + 1]]
            normal = tri_normal(p0, p1, p2)
            tris.append(
                {
                    "index": tri_index,
                    "a": p0,
                    "b": p1,
                    "c": p2,
                    "normal": normal,
                }
            )
            tri_index += 1

    if not tris:
        raise RuntimeError(f"Mesh triangulation produced no usable triangles: {mesh_prim.GetPath()}")
    return tris


def find_nearest_triangle(point_world, tris):
    best = None
    best_d2 = None
    for tri in tris:
        q = closest_point_on_triangle(point_world, tri["a"], tri["b"], tri["c"])
        d2 = distance_sq(point_world, q)
        if best is None or d2 < best_d2:
            best = (tri, q, d2)
            best_d2 = d2
    return best


def delete_prim_if_exists(stage, path):
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        stage.RemovePrim(path)


def ensure_debug_vis(stage):
    UsdGeom.Xform.Define(stage, MAP_RIG_ROOT)

    mapped_proto = UsdGeom.Sphere.Define(stage, MAPPED_PROTO_PATH)
    mapped_proto.CreateRadiusAttr(MAPPED_SPHERE_RADIUS)
    mapped_proto.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        Vt.Vec3fArray([Gf.Vec3f(1.00, 0.45, 0.05)])   # strong orange
    )

    rej_proto = UsdGeom.Sphere.Define(stage, REJECTED_PROTO_PATH)
    rej_proto.CreateRadiusAttr(REJECTED_SPHERE_RADIUS)
    rej_proto.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        Vt.Vec3fArray([Gf.Vec3f(1.00, 0.10, 0.10)])   # red
    )

    fallback_proto = UsdGeom.Sphere.Define(stage, FALLBACK_PROTO_PATH)
    fallback_proto.CreateRadiusAttr(FALLBACK_SPHERE_RADIUS)
    fallback_proto.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        Vt.Vec3fArray([Gf.Vec3f(1.00, 0.95, 0.15)])   # yellow
    )

    mapped = UsdGeom.PointInstancer.Define(stage, MAPPED_PI_PATH)
    mapped.CreatePrototypesRel().SetTargets([Sdf.Path(MAPPED_PROTO_PATH)])
    mapped.CreateProtoIndicesAttr(Vt.IntArray())
    mapped.CreatePositionsAttr(Vt.Vec3fArray())
    mapped.CreateScalesAttr(Vt.Vec3fArray())
    mapped.CreateOrientationsAttr(Vt.QuathArray())
    mapped.CreateIdsAttr(Vt.Int64Array())

    rejected = UsdGeom.PointInstancer.Define(stage, REJECTED_PI_PATH)
    rejected.CreatePrototypesRel().SetTargets([Sdf.Path(REJECTED_PROTO_PATH)])
    rejected.CreateProtoIndicesAttr(Vt.IntArray())
    rejected.CreatePositionsAttr(Vt.Vec3fArray())
    rejected.CreateScalesAttr(Vt.Vec3fArray())
    rejected.CreateOrientationsAttr(Vt.QuathArray())
    rejected.CreateIdsAttr(Vt.Int64Array())

    fallback = UsdGeom.PointInstancer.Define(stage, FALLBACK_PI_PATH)
    fallback.CreatePrototypesRel().SetTargets([Sdf.Path(FALLBACK_PROTO_PATH)])
    fallback.CreateProtoIndicesAttr(Vt.IntArray())
    fallback.CreatePositionsAttr(Vt.Vec3fArray())
    fallback.CreateScalesAttr(Vt.Vec3fArray())
    fallback.CreateOrientationsAttr(Vt.QuathArray())
    fallback.CreateIdsAttr(Vt.Int64Array())


def set_point_instancer(stage, path, positions, scales):
    prim = stage.GetPrimAtPath(path)
    pi = UsdGeom.PointInstancer(prim)
    n = len(positions)
    pi.GetProtoIndicesAttr().Set(Vt.IntArray([0] * n))
    pi.GetPositionsAttr().Set(Vt.Vec3fArray(positions))
    pi.GetScalesAttr().Set(Vt.Vec3fArray(scales))
    pi.GetOrientationsAttr().Set(Vt.QuathArray([Gf.Quath(1.0, 0.0, 0.0, 0.0)] * n))
    pi.GetIdsAttr().Set(Vt.Int64Array(list(range(n))))


def build_stats(values):
    vals = [float(v) for v in values if v is not None]
    if not vals:
        return {"count": 0}
    vals_sorted = sorted(vals)

    def q(p):
        idx = max(0, min(len(vals_sorted) - 1, int(round(p * (len(vals_sorted) - 1)))))
        return vals_sorted[idx]

    return {
        "count": len(vals),
        "avg": sum(vals) / len(vals),
        "p05": q(0.05),
        "p50": q(0.50),
        "p95": q(0.95),
        "min": vals_sorted[0],
        "max": vals_sorted[-1],
    }


def csv_header():
    return (
        "cell_i,cell_j,stack_count,thickness_m,cell_area_m2,volume_proxy_m3,status,reason,"
        "distance_m,receive_dot,receive_threshold_used,triangle_index,used_fallback,"
        "src_x,src_y,src_z,target_x,target_y,target_z,normal_x,normal_y,normal_z\n"
    )


def vec3_to_list(v):
    return [float(v[0]), float(v[1]), float(v[2])]


def make_receive_diag(cell_key, src, target_p, tri, distance_m, receive_dot, threshold_used, status, reason, used_fallback):
    return {
        "cell": [int(cell_key[0]), int(cell_key[1])],
        "status": status,
        "reason": reason,
        "used_fallback": bool(used_fallback),
        "distance_m": float(distance_m) if distance_m is not None else None,
        "receive_dot": float(receive_dot) if receive_dot is not None else None,
        "receive_threshold_used": float(threshold_used) if threshold_used is not None else None,
        "triangle_index": int(tri["index"]) if tri is not None else None,
        "src_world": vec3_to_list(src) if src is not None else None,
        "target_world": vec3_to_list(target_p) if target_p is not None else None,
        "target_normal": vec3_to_list(tri["normal"]) if tri is not None else None,
    }


def console_log_reject_receive(diag):
    cell = diag["cell"]
    log(
        "REJECT_RECEIVE "
        f"cell={cell} "
        f"distance_m={diag['distance_m']:.6f} "
        f"receive_dot={diag['receive_dot']:.6f} "
        f"threshold={diag['receive_threshold_used']:.6f} "
        f"triangle_index={diag['triangle_index']} "
        f"src={diag['src_world']} "
        f"target={diag['target_world']} "
        f"normal={diag['target_normal']}"
    )


def try_relaxed_receive_pass(receive_dot):
    if SECOND_PASS_RELAXED_RECEIVE_DOT is None:
        return False, None
    if receive_dot >= float(SECOND_PASS_RELAXED_RECEIVE_DOT):
        return True, float(SECOND_PASS_RELAXED_RECEIVE_DOT)
    return False, float(SECOND_PASS_RELAXED_RECEIVE_DOT)


def scale_from_stack(stack_count, extra=0.0):
    s = MAPPED_SCALE_MIN + (float(stack_count) * MAPPED_SCALE_PER_STACK) + extra
    s = clamp(s, MAPPED_SCALE_MIN, MAPPED_SCALE_MAX)
    return Gf.Vec3f(s, s, s)


def build():
    stage = get_stage()
    payload = load_exact_field_payload(stage)

    mesh_prim = stage.GetPrimAtPath(CAVE_TARGET_MESH_PATH)
    if not mesh_prim or not mesh_prim.IsValid():
        raise RuntimeError(f"Missing cave target mesh: {CAVE_TARGET_MESH_PATH}")

    tris = triangulate_mesh(mesh_prim)
    bundle = resolve_output_bundle(stage)

    delete_prim_if_exists(stage, MAP_RIG_ROOT)
    ensure_debug_vis(stage)

    nozzle_world = Gf.Vec3f(*[float(x) for x in payload["nozzle_world"]])

    rows = [csv_header()]
    mapped_pts = []
    mapped_scales = []

    rejected_pts = []
    rejected_scales = []

    fallback_pts = []
    fallback_scales = []

    diagnostics = []
    reject_receive_details = []

    accepted_volume = 0.0
    accepted_area = 0.0
    accepted_cells = 0
    fallback_receive_cells = 0

    reject_counts = {
        "reject_distance": 0,
        "reject_receive": 0,
        "reject_occlusion": 0,
        "reject_no_candidate": 0,
    }

    verbose_reject_count = 0

    for cell in payload.get("cells", []):
        c0, c1 = int(cell["cell_key"][0]), int(cell["cell_key"][1])
        stack_count = int(cell["stack_count"])

        src = Gf.Vec3f(
            float(cell["world_center"][0]) * SOURCE_TO_TARGET_SCALE + SOURCE_TO_TARGET_TRANSLATE[0],
            float(cell["world_center"][1]) * SOURCE_TO_TARGET_SCALE + SOURCE_TO_TARGET_TRANSLATE[1],
            float(cell["world_center"][2]) * SOURCE_TO_TARGET_SCALE + SOURCE_TO_TARGET_TRANSLATE[2],
        )

        nearest = find_nearest_triangle(src, tris)
        if nearest is None:
            reject_counts["reject_no_candidate"] += 1
            if SHOW_REJECTED_DEBUG:
                rejected_pts.append(src)
                rejected_scales.append(Gf.Vec3f(1.0, 1.0, 1.0))

            diag = {
                "cell": [c0, c1],
                "status": "rejected",
                "reason": "reject_no_candidate",
                "used_fallback": False,
            }
            diagnostics.append(diag)

            rows.append(
                f"{c0},{c1},{stack_count},{cell['thickness_m']:.6f},{cell['cell_area_m2']:.6f},"
                f"{cell['volume_proxy_m3']:.6f},rejected,reject_no_candidate,,,,False,"
                f"{src[0]:.6f},{src[1]:.6f},{src[2]:.6f},,,,,,\n"
            )
            continue

        tri, q, d2 = nearest
        distance_m = math.sqrt(max(d2, 0.0))

        spray_dir = normalize(sub(q, nozzle_world))
        receive_dot = -dot(spray_dir, tri["normal"])
        threshold_used = float(MIN_RECEIVE_HEMISPHERE_DOT)

        reason = None
        used_fallback = False
        mapped_p = None

        if distance_m > MAX_TRANSFER_DISTANCE_M:
            reason = "reject_distance"
        else:
            receive_ok = receive_dot >= MIN_RECEIVE_HEMISPHERE_DOT

            if not receive_ok:
                second_pass_ok, relaxed_threshold = try_relaxed_receive_pass(receive_dot)
                if second_pass_ok:
                    receive_ok = True
                    threshold_used = float(relaxed_threshold)

            if not receive_ok:
                reason = "reject_receive"

        if reason is None and ENABLE_OCCLUSION_CHECK:
            for other in tris:
                if other["index"] == tri["index"]:
                    continue
                if segment_intersects_triangle_before_end(src, q, other):
                    reason = "reject_occlusion"
                    break

        if reason == "reject_receive":
            diag = make_receive_diag(
                cell_key=[c0, c1],
                src=src,
                target_p=q,
                tri=tri,
                distance_m=distance_m,
                receive_dot=receive_dot,
                threshold_used=threshold_used,
                status="rejected",
                reason="reject_receive",
                used_fallback=False,
            )
            reject_receive_details.append(diag)
            diagnostics.append(diag)

            if ENABLE_VERBOSE_REJECT_RECEIVE_LOG and verbose_reject_count < MAX_VERBOSE_REJECT_LOGS:
                console_log_reject_receive(diag)
                verbose_reject_count += 1

            if ENABLE_DEMO_FALLBACK_FOR_REJECT_RECEIVE:
                mapped_p = add(q, mul(tri["normal"], FALLBACK_VISUAL_OFFSET_M))
                fallback_pts.append(mapped_p)
                fallback_scales.append(scale_from_stack(stack_count, extra=FALLBACK_SCALE_BOOST))
                used_fallback = True
                fallback_receive_cells += 1
                reason = None

                diag = make_receive_diag(
                    cell_key=[c0, c1],
                    src=src,
                    target_p=mapped_p,
                    tri=tri,
                    distance_m=distance_m,
                    receive_dot=receive_dot,
                    threshold_used=threshold_used,
                    status="accepted_fallback",
                    reason="accept_fallback_receive",
                    used_fallback=True,
                )
                diagnostics.append(diag)

        if reason:
            reject_counts[reason] += 1

            if SHOW_REJECTED_DEBUG:
                rejected_pts.append(src)
                rejected_scales.append(Gf.Vec3f(1.0, 1.0, 1.0))

            if reason != "reject_receive":
                diagnostics.append(
                    make_receive_diag(
                        cell_key=[c0, c1],
                        src=src,
                        target_p=q,
                        tri=tri,
                        distance_m=distance_m,
                        receive_dot=receive_dot,
                        threshold_used=threshold_used,
                        status="rejected",
                        reason=reason,
                        used_fallback=False,
                    )
                )

            rows.append(
                f"{c0},{c1},{stack_count},{cell['thickness_m']:.6f},{cell['cell_area_m2']:.6f},"
                f"{cell['volume_proxy_m3']:.6f},rejected,{reason},{distance_m:.6f},{receive_dot:.6f},"
                f"{threshold_used:.6f},{tri['index']},False,"
                f"{src[0]:.6f},{src[1]:.6f},{src[2]:.6f},"
                f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},"
                f"{tri['normal'][0]:.6f},{tri['normal'][1]:.6f},{tri['normal'][2]:.6f}\n"
            )
            continue

        if mapped_p is None:
            mapped_p = add(q, mul(tri["normal"], MAPPED_VISUAL_OFFSET_M))
            mapped_pts.append(mapped_p)
            mapped_scales.append(scale_from_stack(stack_count, extra=0.0))
        else:
            # fallback already added to fallback_pts / fallback_scales
            pass

        accepted_cells += 1
        accepted_volume += float(cell["volume_proxy_m3"])
        accepted_area += float(cell["cell_area_m2"])

        status_str = "accepted_fallback" if used_fallback else "accepted"
        reason_str = "accept_fallback_receive" if used_fallback else "accepted"

        diagnostics.append(
            make_receive_diag(
                cell_key=[c0, c1],
                src=src,
                target_p=mapped_p,
                tri=tri,
                distance_m=distance_m,
                receive_dot=receive_dot,
                threshold_used=threshold_used,
                status=status_str,
                reason=reason_str,
                used_fallback=used_fallback,
            )
        )

        rows.append(
            f"{c0},{c1},{stack_count},{cell['thickness_m']:.6f},{cell['cell_area_m2']:.6f},"
            f"{cell['volume_proxy_m3']:.6f},{status_str},{reason_str},{distance_m:.6f},{receive_dot:.6f},"
            f"{threshold_used:.6f},{tri['index']},{str(used_fallback)},"
            f"{src[0]:.6f},{src[1]:.6f},{src[2]:.6f},"
            f"{mapped_p[0]:.6f},{mapped_p[1]:.6f},{mapped_p[2]:.6f},"
            f"{tri['normal'][0]:.6f},{tri['normal'][1]:.6f},{tri['normal'][2]:.6f}\n"
        )

    set_point_instancer(stage, MAPPED_PI_PATH, mapped_pts, mapped_scales)
    set_point_instancer(stage, REJECTED_PI_PATH, rejected_pts, rejected_scales)
    set_point_instancer(stage, FALLBACK_PI_PATH, fallback_pts, fallback_scales)

    total_cells = len(payload.get("cells", []))
    total_area = float(payload.get("coverage_area_m2", 0.0))
    total_volume = float(payload.get("volume_proxy_m3", 0.0))

    result = {
        "run_version_tag": RUN_VERSION_TAG,
        "run_id": bundle["run_id"],
        "target_stage_path": bundle["stage_path"],
        "source_field_loaded_from": payload.get("_loaded_from", ""),
        "source_run_id": payload.get("run_id", ""),
        "source_config_hash": payload.get("config_hash", ""),
        "source_finalize_reason": payload.get("finalize_reason", ""),
        "target_mesh_path": CAVE_TARGET_MESH_PATH,
        "triangle_count": len(tris),
        "mapping_method": "exact_proxy_field_import_receive_only_with_diagnostics",
        "settings": {
            "max_transfer_distance_m": MAX_TRANSFER_DISTANCE_M,
            "min_receive_hemisphere_dot": MIN_RECEIVE_HEMISPHERE_DOT,
            "enable_occlusion_check": ENABLE_OCCLUSION_CHECK,
            "enable_demo_fallback_for_reject_receive": ENABLE_DEMO_FALLBACK_FOR_REJECT_RECEIVE,
            "second_pass_relaxed_receive_dot": SECOND_PASS_RELAXED_RECEIVE_DOT,
            "mapped_visual_offset_m": MAPPED_VISUAL_OFFSET_M,
            "fallback_visual_offset_m": FALLBACK_VISUAL_OFFSET_M,
            "demo_visual_mode": DEMO_VISUAL_MODE,
            "mapped_sphere_radius": MAPPED_SPHERE_RADIUS,
            "fallback_sphere_radius": FALLBACK_SPHERE_RADIUS,
            "show_rejected_debug": SHOW_REJECTED_DEBUG,
        },
        "counts": {
            "source_cells": total_cells,
            "accepted_cells": accepted_cells,
            "rejected_cells": total_cells - accepted_cells,
            "accept_fraction": (accepted_cells / float(total_cells)) if total_cells else 0.0,
            "fallback_receive_cells": fallback_receive_cells,
            **reject_counts,
        },
        "areas": {
            "source_area_m2": total_area,
            "mapped_area_m2": accepted_area,
            "coverage_ratio": (accepted_area / total_area) if total_area else 0.0,
        },
        "volumes": {
            "source_volume_proxy_m3": total_volume,
            "mapped_volume_proxy_m3": accepted_volume,
            "volume_ratio": (accepted_volume / total_volume) if total_volume else 0.0,
        },
        "receive_dot_stats": build_stats(
            [d.get("receive_dot") for d in diagnostics if d.get("receive_dot") is not None]
        ),
        "distance_stats": build_stats(
            [d.get("distance_m") for d in diagnostics if d.get("distance_m") is not None]
        ),
        "source_field_summary": {
            "cell_count": total_cells,
            "coverage_area_m2": total_area,
            "volume_proxy_m3": total_volume,
            "baseline_summary": payload.get("baseline_summary", {}),
        },
        "reject_receive_details": reject_receive_details,
    }

    txt_lines = [
        f"Shotcrete proxy->cave exact-field import mapping ({RUN_VERSION_TAG})",
        f"Source field JSON: {payload.get('_loaded_from', '')}",
        f"Source run id: {payload.get('run_id', '')}",
        f"Target mesh path: {CAVE_TARGET_MESH_PATH}",
        f"Triangle count: {len(tris)}",
        f"Source cells: {total_cells}",
        f"Accepted cells: {accepted_cells}",
        f"Rejected cells: {total_cells - accepted_cells}",
        f"Fallback receive cells used: {fallback_receive_cells}",
        f"Accept fraction: {result['counts']['accept_fraction']:.4f}",
        f"Reject distance: {reject_counts['reject_distance']}",
        f"Reject receive: {reject_counts['reject_receive']}",
        f"Reject occlusion: {reject_counts['reject_occlusion']}",
        f"Reject no candidate: {reject_counts['reject_no_candidate']}",
        f"Source area m2: {total_area:.6f}",
        f"Mapped area m2: {accepted_area:.6f}",
        f"Coverage ratio: {result['areas']['coverage_ratio']:.4f}",
        f"Source volume proxy m3: {total_volume:.6f}",
        f"Mapped volume proxy m3: {accepted_volume:.6f}",
        f"Volume ratio: {result['volumes']['volume_ratio']:.4f}",
        f"receive_dot avg/p05/p50/p95: "
        f"{result['receive_dot_stats'].get('avg', 0.0):.4f} / "
        f"{result['receive_dot_stats'].get('p05', 0.0):.4f} / "
        f"{result['receive_dot_stats'].get('p50', 0.0):.4f} / "
        f"{result['receive_dot_stats'].get('p95', 0.0):.4f}",
        f"distance avg/p05/p50/p95: "
        f"{result['distance_stats'].get('avg', 0.0):.4f} / "
        f"{result['distance_stats'].get('p05', 0.0):.4f} / "
        f"{result['distance_stats'].get('p50', 0.0):.4f} / "
        f"{result['distance_stats'].get('p95', 0.0):.4f}",
        "",
        "Visualization settings:",
        f"  DEMO_VISUAL_MODE = {DEMO_VISUAL_MODE}",
        f"  MAPPED_VISUAL_OFFSET_M = {MAPPED_VISUAL_OFFSET_M}",
        f"  FALLBACK_VISUAL_OFFSET_M = {FALLBACK_VISUAL_OFFSET_M}",
        f"  MAPPED_SPHERE_RADIUS = {MAPPED_SPHERE_RADIUS}",
        f"  FALLBACK_SPHERE_RADIUS = {FALLBACK_SPHERE_RADIUS}",
        f"  SHOW_REJECTED_DEBUG = {SHOW_REJECTED_DEBUG}",
        "",
        "Reject_receive details:",
    ]

    if reject_receive_details:
        for item in reject_receive_details:
            txt_lines.append(
                f"  cell={item['cell']} "
                f"distance_m={item['distance_m']:.6f} "
                f"receive_dot={item['receive_dot']:.6f} "
                f"threshold={item['receive_threshold_used']:.6f} "
                f"triangle_index={item['triangle_index']} "
                f"src={item['src_world']} "
                f"target={item['target_world']} "
                f"normal={item['target_normal']}"
            )
    else:
        txt_lines.append("  none")

    write_text_file(bundle["run_txt_path"], "\n".join(txt_lines))
    write_json_file(bundle["run_json_path"], result)
    with open(bundle["run_csv_path"], "w", encoding="utf-8") as f:
        f.writelines(rows)

    copy_file(bundle["run_txt_path"], bundle["latest_txt_path"])
    copy_file(bundle["run_json_path"], bundle["latest_json_path"])
    copy_file(bundle["run_csv_path"], bundle["latest_csv_path"])

    log(
        f"DONE source_cells={total_cells} accepted={accepted_cells} "
        f"rejected={total_cells - accepted_cells} "
        f"fallback_receive_cells={fallback_receive_cells} "
        f"coverage_ratio={result['areas']['coverage_ratio']:.4f} "
        f"volume_ratio={result['volumes']['volume_ratio']:.4f}"
    )
    log(f"RUN_TXT = {bundle['run_txt_path']}")
    log(f"RUN_JSON = {bundle['run_json_path']}")
    log(f"RUN_CSV = {bundle['run_csv_path']}")
    log(f"VIS mapped_pts={len(mapped_pts)} fallback_pts={len(fallback_pts)} rejected_pts={len(rejected_pts)}")


build()