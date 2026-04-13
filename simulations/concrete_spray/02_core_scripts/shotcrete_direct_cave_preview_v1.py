import builtins
import datetime
import json
import math
import os
import random

import omni
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt


NOZZLE_LOCATOR_PATH = "/CAVE_collision_proxy/PreviewSpraySource/NozzleLocator"
AIM_LOCATOR_PATH = "/CAVE_collision_proxy/PreviewSpraySource/AimLocator"
PREVIEW_SOURCE_ROOT_PATH = "/CAVE_collision_proxy/PreviewSpraySource"
SOURCE_PROXY_COLLIDER_PATH = "/CAVE_collision_proxy/PreviewSpraySource/temp_collider"
SOURCE_PROXY_MESH_PATH = "/CAVE_collision_proxy/PreviewSpraySource/Cube"
CAVE_TARGET_MESH_PATH = "/CAVE_collision_proxy/CAVE_LOWPOLY/cave_collision/Cube_001"

RIG_ROOT = "/ShotcreteDirectCavePreviewRig"
DYNAMIC_PI_PATH = f"{RIG_ROOT}/DynamicPI"
DEPOSIT_PI_PATH = f"{RIG_ROOT}/DepositPI"
WET_COAT_PI_PATH = f"{RIG_ROOT}/WetCoatPI"
CONTINUOUS_COAT_MESH_PATH = f"{RIG_ROOT}/ContinuousCoatMesh"
DYNAMIC_PROTO_PATH = f"{DYNAMIC_PI_PATH}/DynamicProto"
DEPOSIT_PROTO_PATH = f"{DEPOSIT_PI_PATH}/DepositProto"
WET_COAT_PROTO_PATH = f"{WET_COAT_PI_PATH}/WetCoatProto"
NOZZLE_MARKER_PATH = f"{RIG_ROOT}/NozzleMarker"
AIM_MARKER_PATH = f"{RIG_ROOT}/AimMarker"
RAY_PATH = f"{RIG_ROOT}/NozzleToAimRay"
LOOKS_ROOT = f"{RIG_ROOT}/Looks"
DYNAMIC_MATERIAL_PATH = f"{LOOKS_ROOT}/DynamicParticleMaterial"
DEPOSIT_MATERIAL_PATH = f"{LOOKS_ROOT}/DepositBodyMaterial"
WET_COAT_MATERIAL_PATH = f"{LOOKS_ROOT}/WetCoatMaterial"
CONTINUOUS_COAT_MATERIAL_PATH = f"{LOOKS_ROOT}/ContinuousCoatMaterial"

STATE_KEY = "_SHOTCRETE_DIRECT_CAVE_PREVIEW_STATE"
SUB_KEY = "_SHOTCRETE_DIRECT_CAVE_PREVIEW_SUB"

FPS = 60.0
START_FRAME = 1
EMIT_DURATION_SECONDS = 6.0
EMIT_FRAMES = int(EMIT_DURATION_SECONDS * FPS)

TARGET_PARTICLES_PER_SEC = 520.0
PARTICLE_RADIUS = 0.035
DEPOSIT_RADIUS = 0.050
SPRAY_SPEED_MPS = 16.0
SPRAY_CONE_DEG = 3.8
NOZZLE_SPAWN_OFFSET_M = 0.18
SPAWN_DISK_RADIUS_M = 0.035
SPEED_JITTER_MPS = 1.4
GRAVITY_MPS2 = Gf.Vec3f(0.0, -1.6, 0.0)
VELOCITY_DAMP_PER_FRAME = 0.996

CAVE_CAPTURE_DISTANCE_M = 0.12
MIN_SURFACE_APPROACH_DOT = 0.08
DEPOSIT_SURFACE_GAP_M = 0.004
DEPOSIT_THICKNESS_SCALE_MIN = 0.24
DEPOSIT_THICKNESS_SCALE_STEP = 0.11
DEPOSIT_THICKNESS_SCALE_MAX = 1.30
DEPOSIT_SPREAD_SCALE_MIN = 1.42
DEPOSIT_SPREAD_SCALE_STEP = 0.18
DEPOSIT_SPREAD_SCALE_MAX = 3.00
DEPOSIT_CELL_SIZE_M = 0.100
WET_COAT_RADIUS = 0.060
WET_COAT_SURFACE_GAP_M = 0.002
WET_COAT_THICKNESS_SCALE = 0.11
WET_COAT_MIN_THICKNESS_SCALE = 0.065
WET_COAT_SPREAD_MULT = 1.70
WET_COAT_SPREAD_BONUS = 0.18
SWEEP_PATCH_WIDTH_M = 3.20
SWEEP_PATCH_HEIGHT_M = 2.30
SWEEP_RASTER_PASSES = 15
SWEEP_TARGET_JITTER_M = 0.110
NOZZLE_MOTION_FOLLOW_U = 0.82
NOZZLE_MOTION_FOLLOW_V = 0.66
NOZZLE_MOTION_BOB_AMPLITUDE_M = 0.10
NOZZLE_MOTION_SWAY_AMPLITUDE_M = 0.06
CONTINUOUS_COAT_GRID_STEP_M = 0.100
CONTINUOUS_COAT_MARGIN_M = 0.320
CONTINUOUS_COAT_SURFACE_SAMPLE_HALF_DEPTH_M = 2.25
CONTINUOUS_COAT_SURFACE_GAP_M = 0.006
CONTINUOUS_COAT_SMOOTH_PASSES = 4
CONTINUOUS_COAT_VERTEX_MIN_PRESENCE = 0.10
CONTINUOUS_COAT_FACE_MIN_PRESENCE = 0.16
CONTINUOUS_COAT_RAW_SUPPORT_MIN = 0.06
CONTINUOUS_COAT_MAX_EDGE_STRETCH = 2.50
CONTINUOUS_COAT_MAX_FACE_AREA_MULT = 4.50
CONTINUOUS_COAT_MAX_THICKNESS_M = 0.055
CONTINUOUS_COAT_INFLUENCE_RADIUS_M = 0.230

TRI_GRID_CELL_SIZE_M = 1.00
MAX_LIVE_PARTICLES = 2800
MAX_DEPOSITED = 12000
MAX_TRAVEL_M = 45.0

STATUS_LOG_INTERVAL_FRAMES = 30
FINALIZE_LIVE_THRESHOLD = 8
FINALIZE_PROBE_INTERVAL_FRAMES = 30
FINALIZE_QUIET_PROBES = 4
MIN_POST_EMIT_FINALIZE_SECONDS = 1.5
HARD_FINALIZE_EXTRA_SECONDS = 12.0

HIDE_EXISTING_PREVIEW_OVERLAYS = True
HIDE_SOURCE_PROXY_GEOMETRY = True
SHOW_DEPOSIT_SPLATS = False
SHOW_WET_COAT_SPLATS = True
SHOW_CONTINUOUS_COAT_MESH = False

RUN_VERSION_TAG = "direct_cave_preview_v1"


def log(msg: str) -> None:
    print(f"[SHOTCRETE_DIRECT_CAVE_PREVIEW] {msg}")


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


def ensure_dir(path: str) -> str:
    os.makedirs(path, exist_ok=True)
    return path


def workspace_output_dir() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(script_dir)
    return ensure_dir(os.path.join(workspace_dir, "04_current_outputs"))


def output_bundle(stage):
    run_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = workspace_output_dir()
    return {
        "run_id": run_id,
        "stage_path": get_stage_file_path(stage),
        "run_json_path": os.path.join(out_dir, f"shotcrete_{RUN_VERSION_TAG}_run_{run_id}.json"),
        "latest_json_path": os.path.join(out_dir, f"shotcrete_{RUN_VERSION_TAG}_latest.json"),
        "run_txt_path": os.path.join(out_dir, f"shotcrete_{RUN_VERSION_TAG}_run_{run_id}.txt"),
        "latest_txt_path": os.path.join(out_dir, f"shotcrete_{RUN_VERSION_TAG}_latest.txt"),
    }


def write_json(path: str, payload) -> bool:
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
            f.write("\n")
        return True
    except Exception:
        return False


def write_text(path: str, text: str) -> bool:
    try:
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
            if not text.endswith("\n"):
                f.write("\n")
        return True
    except Exception:
        return False


def normalize(v):
    n = math.sqrt(float(v[0]) * float(v[0]) + float(v[1]) * float(v[1]) + float(v[2]) * float(v[2]))
    if n < 1e-8:
        return Gf.Vec3f(0.0, 0.0, -1.0)
    return Gf.Vec3f(float(v[0]) / n, float(v[1]) / n, float(v[2]) / n)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def dot(a, b):
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def add(a, b):
    return Gf.Vec3f(float(a[0]) + float(b[0]), float(a[1]) + float(b[1]), float(a[2]) + float(b[2]))


def sub(a, b):
    return Gf.Vec3f(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]), float(a[2]) - float(b[2]))


def mul(a, s):
    return Gf.Vec3f(float(a[0]) * s, float(a[1]) * s, float(a[2]) * s)


def cross(a, b):
    return Gf.Vec3f(
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    )


def distance_sq(a, b):
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = float(a[2]) - float(b[2])
    return dx * dx + dy * dy + dz * dz


def length(v):
    return math.sqrt(max(distance_sq(v, Gf.Vec3f(0.0, 0.0, 0.0)), 0.0))


def get_world_pos(stage, path: str) -> Gf.Vec3f:
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Missing prim: {path}")
    world_m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    p = world_m.ExtractTranslation()
    return Gf.Vec3f(float(p[0]), float(p[1]), float(p[2]))


def set_translate(stage, path: str, pos) -> None:
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)
    translate_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if translate_op is None:
        translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))


def set_curve_points(stage, path: str, p0, p1) -> None:
    curve = UsdGeom.BasisCurves.Define(stage, path)
    curve.CreateTypeAttr(UsdGeom.Tokens.linear)
    curve.CreateBasisAttr(UsdGeom.Tokens.bezier)
    curve.CreateCurveVertexCountsAttr(Vt.IntArray([2]))
    curve.CreateWidthsAttr(Vt.FloatArray([0.03, 0.03]))
    curve.CreatePointsAttr(
        Vt.Vec3fArray(
            [
                Gf.Vec3f(float(p0[0]), float(p0[1]), float(p0[2])),
                Gf.Vec3f(float(p1[0]), float(p1[1]), float(p1[2])),
            ]
        )
    )
    curve.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        Vt.Vec3fArray([Gf.Vec3f(1.0, 0.45, 0.05)])
    )


def ensure_marker(stage, path: str, radius: float, color) -> None:
    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(radius)
    sphere.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        Vt.Vec3fArray([Gf.Vec3f(*color)])
    )


def ensure_preview_surface_material(
    stage,
    material_path: str,
    shader_name: str,
    diffuse_color,
    roughness: float,
    specular_color=None,
    opacity: float = 1.0,
    ior: float = 1.47,
):
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/{shader_name}")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse_color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(float(ior))
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
    if specular_color is not None:
        shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
        shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*specular_color))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def bind_material(stage, prim_path: str, material_path: str) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    material_prim = stage.GetPrimAtPath(material_path)
    if not prim or not prim.IsValid() or not material_prim or not material_prim.IsValid():
        return
    UsdShade.MaterialBindingAPI(prim).Bind(UsdShade.Material(material_prim))


def hide_prim(stage, path: str) -> None:
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        try:
            UsdGeom.Imageable(prim).MakeInvisible()
        except Exception:
            pass


def delete_prim_if_exists(stage, path: str) -> None:
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        stage.RemovePrim(path)


def prune_preview_source_geometry(stage) -> None:
    root = stage.GetPrimAtPath(PREVIEW_SOURCE_ROOT_PATH)
    if not root or not root.IsValid():
        return

    keep_paths = {NOZZLE_LOCATOR_PATH, AIM_LOCATOR_PATH}
    for child in list(root.GetChildren()):
        child_path = str(child.GetPath())
        if child_path in keep_paths:
            continue
        delete_prim_if_exists(stage, child_path)


def orthonormal_basis(dir_vec):
    d = normalize(dir_vec)
    up = Gf.Vec3f(0.0, 1.0, 0.0)
    if abs(float(d[1])) > 0.95:
        up = Gf.Vec3f(1.0, 0.0, 0.0)
    u = normalize(cross(d, up))
    v = normalize(cross(d, u))
    return d, u, v


def random_in_disk(radius: float):
    ang = random.uniform(0.0, math.tau)
    mag = radius * math.sqrt(random.random())
    return math.cos(ang) * mag, math.sin(ang) * mag


def make_random_dir(base_dir):
    if SPRAY_CONE_DEG <= 0.0:
        return normalize(base_dir)
    cone_rad = math.radians(SPRAY_CONE_DEG)
    d, u, v = orthonormal_basis(base_dir)
    yaw = random.uniform(0.0, math.tau)
    mag = math.tan(cone_rad) * math.sqrt(random.random())
    offset = add(mul(u, math.cos(yaw) * mag), mul(v, math.sin(yaw) * mag))
    return normalize(add(d, offset))


def make_spawn_offset(base_dir):
    _d, u, v = orthonormal_basis(base_dir)
    x, y = random_in_disk(SPAWN_DISK_RADIUS_M)
    return add(mul(u, x), mul(v, y))


def identity_orientation():
    return Gf.Quath(1.0, Gf.Vec3h(0.0, 0.0, 0.0))


def orientation_from_normal(surface_normal):
    base = Gf.Vec3f(0.0, 0.0, 1.0)
    target = normalize(surface_normal)
    cosine = clamp(dot(base, target), -1.0, 1.0)
    if cosine > 0.9999:
        return identity_orientation()
    if cosine < -0.9999:
        return Gf.Quath(0.0, Gf.Vec3h(1.0, 0.0, 0.0))

    axis = normalize(cross(base, target))
    half_angle = math.acos(cosine) * 0.5
    sin_half = math.sin(half_angle)
    return Gf.Quath(
        float(math.cos(half_angle)),
        Gf.Vec3h(float(axis[0] * sin_half), float(axis[1] * sin_half), float(axis[2] * sin_half)),
    )


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


def segment_hit_triangle(p0, p1, tri, eps=1e-6):
    d = sub(p1, p0)
    e1 = sub(tri["b"], tri["a"])
    e2 = sub(tri["c"], tri["a"])
    h = cross(d, e2)
    det = dot(e1, h)
    if -eps < det < eps:
        return None
    inv_det = 1.0 / det
    s = sub(p0, tri["a"])
    u = inv_det * dot(s, h)
    if u < 0.0 or u > 1.0:
        return None
    q = cross(s, e1)
    v = inv_det * dot(d, q)
    if v < 0.0 or u + v > 1.0:
        return None
    t = inv_det * dot(e2, q)
    if t < 0.0 or t > 1.0:
        return None
    hit = add(p0, mul(d, t))
    return t, hit


def tri_normal(a, b, c):
    return normalize(cross(sub(b, a), sub(c, a)))


def oriented_surface_normal(surface_normal, incoming_dir):
    n = normalize(surface_normal)
    if dot(n, incoming_dir) > 0.0:
        return mul(n, -1.0)
    return n


def aabb_min(a, b, c):
    return Gf.Vec3f(
        min(float(a[0]), float(b[0]), float(c[0])),
        min(float(a[1]), float(b[1]), float(c[1])),
        min(float(a[2]), float(b[2]), float(c[2])),
    )


def aabb_max(a, b, c):
    return Gf.Vec3f(
        max(float(a[0]), float(b[0]), float(c[0])),
        max(float(a[1]), float(b[1]), float(c[1])),
        max(float(a[2]), float(b[2]), float(c[2])),
    )


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
            tris.append(
                {
                    "index": tri_index,
                    "a": p0,
                    "b": p1,
                    "c": p2,
                    "normal": tri_normal(p0, p1, p2),
                    "bb_min": aabb_min(p0, p1, p2),
                    "bb_max": aabb_max(p0, p1, p2),
                }
            )
            tri_index += 1
    if not tris:
        raise RuntimeError(f"No triangles found on mesh: {mesh_prim.GetPath()}")
    return tris


def cell_key_for_point(p):
    return (
        int(math.floor(float(p[0]) / TRI_GRID_CELL_SIZE_M)),
        int(math.floor(float(p[1]) / TRI_GRID_CELL_SIZE_M)),
        int(math.floor(float(p[2]) / TRI_GRID_CELL_SIZE_M)),
    )


def iter_cell_keys_for_bounds(mn, mx):
    ix0, iy0, iz0 = cell_key_for_point(mn)
    ix1, iy1, iz1 = cell_key_for_point(mx)
    for ix in range(ix0, ix1 + 1):
        for iy in range(iy0, iy1 + 1):
            for iz in range(iz0, iz1 + 1):
                yield (ix, iy, iz)


def build_triangle_grid(tris):
    grid = {}
    for tri in tris:
        for key in iter_cell_keys_for_bounds(tri["bb_min"], tri["bb_max"]):
            grid.setdefault(key, []).append(tri["index"])
    return grid


def candidate_triangles_for_segment(state, p0, p1, pad):
    mn = Gf.Vec3f(
        min(float(p0[0]), float(p1[0])) - pad,
        min(float(p0[1]), float(p1[1])) - pad,
        min(float(p0[2]), float(p1[2])) - pad,
    )
    mx = Gf.Vec3f(
        max(float(p0[0]), float(p1[0])) + pad,
        max(float(p0[1]), float(p1[1])) + pad,
        max(float(p0[2]), float(p1[2])) + pad,
    )
    seen = set()
    candidates = []
    for key in iter_cell_keys_for_bounds(mn, mx):
        for index in state["triangle_grid"].get(key, []):
            if index in seen:
                continue
            seen.add(index)
            candidates.append(state["tris"][index])
    return candidates


def first_surface_hit_for_segment(state, p0, p1, incoming_dir):
    best = None
    for tri in candidate_triangles_for_segment(state, p0, p1, 0.02):
        hit = segment_hit_triangle(p0, p1, tri)
        if hit is None:
            continue
        t, q = hit
        surface_normal = oriented_surface_normal(tri["normal"], incoming_dir)
        if best is None or t < best["t"]:
            best = {
                "t": t,
                "point": q,
                "tri": tri,
                "normal": surface_normal,
            }
    return best


def compute_spray_anchor(stage, state):
    nozzle_world = get_world_pos(stage, NOZZLE_LOCATOR_PATH)
    aim_world = get_world_pos(stage, AIM_LOCATOR_PATH)
    spray_dir = normalize(sub(aim_world, nozzle_world))
    probe_end = add(nozzle_world, mul(spray_dir, MAX_TRAVEL_M))
    hit = first_surface_hit_for_segment(state, nozzle_world, probe_end, spray_dir)
    if hit is not None:
        center = hit["point"]
        normal = hit["normal"]
    else:
        center = aim_world
        normal = mul(spray_dir, -1.0)

    tangent_u = cross(normal, spray_dir)
    if length(tangent_u) < 1e-4:
        _d, tangent_u, tangent_v = orthonormal_basis(normal)
    else:
        tangent_u = normalize(tangent_u)
        tangent_v = normalize(cross(tangent_u, normal))

    if float(tangent_v[1]) < 0.0:
        tangent_u = mul(tangent_u, -1.0)
        tangent_v = mul(tangent_v, -1.0)

    return {
        "center": Gf.Vec3f(float(center[0]), float(center[1]), float(center[2])),
        "normal": Gf.Vec3f(float(normal[0]), float(normal[1]), float(normal[2])),
        "u": Gf.Vec3f(float(tangent_u[0]), float(tangent_u[1]), float(tangent_u[2])),
        "v": Gf.Vec3f(float(tangent_v[0]), float(tangent_v[1]), float(tangent_v[2])),
    }


def compute_spray_target_world(state, current_frame: int):
    anchor = state.get("spray_anchor")
    if not anchor:
        return None

    sweep = compute_sweep_sample(current_frame)
    offset_u = (sweep["horizontal_t"] - 0.5) * SWEEP_PATCH_WIDTH_M
    offset_v = sweep["vertical_t"] * SWEEP_PATCH_HEIGHT_M

    return add(anchor["center"], add(mul(anchor["u"], offset_u), mul(anchor["v"], offset_v)))


def compute_sweep_sample(current_frame: int):
    if EMIT_FRAMES <= 1:
        progress = 0.0
    else:
        progress = clamp((current_frame - START_FRAME) / float(EMIT_FRAMES - 1), 0.0, 1.0)

    passes = max(int(SWEEP_RASTER_PASSES), 1)
    sweep_progress = min(progress * passes, passes - 1.0e-6)
    pass_index = int(math.floor(sweep_progress))
    pass_t = sweep_progress - pass_index
    horizontal_t = pass_t if (pass_index % 2 == 0) else (1.0 - pass_t)
    vertical_t = 0.0 if passes <= 1 else (0.5 - (float(pass_index) / float(passes - 1)))

    return {
        "progress": progress,
        "passes": passes,
        "pass_index": pass_index,
        "horizontal_t": horizontal_t,
        "vertical_t": vertical_t,
    }


def compute_nozzle_world(stage, state, current_frame: int):
    base_nozzle_world = state.get("base_nozzle_world")
    anchor = state.get("spray_anchor")
    if base_nozzle_world is None or not anchor:
        return get_world_pos(stage, NOZZLE_LOCATOR_PATH)

    sweep = compute_sweep_sample(current_frame)
    offset_u = (sweep["horizontal_t"] - 0.5) * SWEEP_PATCH_WIDTH_M * NOZZLE_MOTION_FOLLOW_U
    offset_v = sweep["vertical_t"] * SWEEP_PATCH_HEIGHT_M * NOZZLE_MOTION_FOLLOW_V
    bob_phase = sweep["progress"] * math.tau * 2.0
    sway_phase = sweep["progress"] * math.tau
    bob = math.sin(bob_phase) * NOZZLE_MOTION_BOB_AMPLITUDE_M
    sway = math.cos(sway_phase) * NOZZLE_MOTION_SWAY_AMPLITUDE_M

    return add(
        base_nozzle_world,
        add(
            mul(anchor["u"], offset_u + sway),
            add(mul(anchor["v"], offset_v), mul(anchor["normal"], bob)),
        ),
    )


def continuous_coat_patch_size():
    return (
        SWEEP_PATCH_WIDTH_M + 2.0 * CONTINUOUS_COAT_MARGIN_M,
        SWEEP_PATCH_HEIGHT_M + 2.0 * CONTINUOUS_COAT_MARGIN_M,
    )


def anchor_uv_from_world(anchor, point):
    rel = sub(point, anchor["center"])
    return dot(rel, anchor["u"]), dot(rel, anchor["v"])


def sample_surface_for_coat_grid(state, anchor, u_offset: float, v_offset: float):
    plane_point = add(anchor["center"], add(mul(anchor["u"], u_offset), mul(anchor["v"], v_offset)))
    probe_start = add(plane_point, mul(anchor["normal"], CONTINUOUS_COAT_SURFACE_SAMPLE_HALF_DEPTH_M))
    probe_end = add(plane_point, mul(anchor["normal"], -CONTINUOUS_COAT_SURFACE_SAMPLE_HALF_DEPTH_M))
    hit = first_surface_hit_for_segment(state, probe_start, probe_end, mul(anchor["normal"], -1.0))
    if hit is None:
        return {
            "point": plane_point,
            "normal": anchor["normal"],
            "valid": False,
        }
    return {
        "point": hit["point"],
        "normal": hit["normal"],
        "valid": True,
    }


def build_continuous_coat_surface_grid(state, anchor):
    patch_width_m, patch_height_m = continuous_coat_patch_size()
    cols = max(4, int(math.ceil(patch_width_m / CONTINUOUS_COAT_GRID_STEP_M)) + 1)
    rows = max(4, int(math.ceil(patch_height_m / CONTINUOUS_COAT_GRID_STEP_M)) + 1)
    step_u = patch_width_m / float(cols - 1)
    step_v = patch_height_m / float(rows - 1)

    plane_points = []
    uv_offsets = []

    for row in range(rows):
        if rows <= 1:
            v_offset = 0.0
        else:
            v_offset = (0.5 - (float(row) / float(rows - 1))) * patch_height_m
        for col in range(cols):
            if cols <= 1:
                u_offset = 0.0
            else:
                u_offset = ((float(col) / float(cols - 1)) - 0.5) * patch_width_m
            uv_offsets.append((u_offset, v_offset))
            plane_points.append(
                Gf.Vec3f(
                    float(anchor["center"][0] + anchor["u"][0] * u_offset + anchor["v"][0] * v_offset),
                    float(anchor["center"][1] + anchor["u"][1] * u_offset + anchor["v"][1] * v_offset),
                    float(anchor["center"][2] + anchor["u"][2] * u_offset + anchor["v"][2] * v_offset),
                )
            )

    return {
        "width": cols,
        "height": rows,
        "step_u": step_u,
        "step_v": step_v,
        "half_width": 0.5 * patch_width_m,
        "half_height": 0.5 * patch_height_m,
        "patch_width_m": patch_width_m,
        "patch_height_m": patch_height_m,
        "uv_offsets": uv_offsets,
        "plane_points": plane_points,
    }


def scale_from_stack(stack_count: int) -> Gf.Vec3f:
    layers = max(stack_count - 1, 0)
    spread = min(DEPOSIT_SPREAD_SCALE_MIN + DEPOSIT_SPREAD_SCALE_STEP * math.sqrt(layers), DEPOSIT_SPREAD_SCALE_MAX)
    thickness = min(DEPOSIT_THICKNESS_SCALE_MIN + DEPOSIT_THICKNESS_SCALE_STEP * layers, DEPOSIT_THICKNESS_SCALE_MAX)
    return Gf.Vec3f(spread, spread, thickness)


def deposit_center(hit_point, surface_normal, scale):
    thickness_radius = DEPOSIT_RADIUS * float(scale[2])
    return add(hit_point, mul(surface_normal, DEPOSIT_SURFACE_GAP_M + thickness_radius))


def wet_coat_scale_from_stack(stack_count: int) -> Gf.Vec3f:
    base = scale_from_stack(stack_count)
    layers = max(stack_count - 1, 0)
    spread = float(base[0]) * (WET_COAT_SPREAD_MULT + WET_COAT_SPREAD_BONUS * math.sqrt(layers))
    thickness = max(float(base[2]) * WET_COAT_THICKNESS_SCALE, WET_COAT_MIN_THICKNESS_SCALE)
    return Gf.Vec3f(spread, spread, thickness)


def wet_coat_center(hit_point, surface_normal, scale):
    thickness_radius = WET_COAT_RADIUS * float(scale[2])
    return add(hit_point, mul(surface_normal, WET_COAT_SURFACE_GAP_M + thickness_radius))


def continuous_coat_thickness_from_stack(stack_count: int) -> float:
    scale = scale_from_stack(stack_count)
    thickness = DEPOSIT_RADIUS * float(scale[2]) * 0.90
    return clamp(thickness, 0.0, CONTINUOUS_COAT_MAX_THICKNESS_M)


def smooth_scalar_grid(values, valid_mask, width: int, height: int, passes: int):
    current = list(values)
    for _ in range(max(int(passes), 0)):
        nxt = [0.0] * len(current)
        for row in range(height):
            for col in range(width):
                idx = row * width + col
                if not valid_mask[idx]:
                    continue
                accum = 0.0
                total = 0.0
                for d_row in (-1, 0, 1):
                    rr = row + d_row
                    if rr < 0 or rr >= height:
                        continue
                    for d_col in (-1, 0, 1):
                        cc = col + d_col
                        if cc < 0 or cc >= width:
                            continue
                        nidx = rr * width + cc
                        if not valid_mask[nidx]:
                            continue
                        weight = 4.0 if (d_row == 0 and d_col == 0) else (2.0 if (d_row == 0 or d_col == 0) else 1.0)
                        accum += current[nidx] * weight
                        total += weight
                nxt[idx] = (accum / total) if total > 1e-8 else 0.0
        current = nxt
    return current


def triangle_area(a, b, c) -> float:
    return 0.5 * length(cross(sub(b, a), sub(c, a)))


def quad_area(a, b, c, d) -> float:
    return triangle_area(a, b, c) + triangle_area(a, c, d)


def set_point_instancer(stage, path: str, proto_path: str, positions, scales, ids, orientations=None) -> None:
    pi = UsdGeom.PointInstancer.Define(stage, path)
    pi.CreatePrototypesRel().SetTargets([Sdf.Path(proto_path)])
    count = len(positions)
    pi.GetProtoIndicesAttr().Set(Vt.IntArray([0] * count))
    pi.GetPositionsAttr().Set(Vt.Vec3fArray(positions))
    pi.GetScalesAttr().Set(Vt.Vec3fArray(scales))
    if orientations is None:
        orientations = [identity_orientation()] * count
    pi.GetOrientationsAttr().Set(Vt.QuathArray(orientations))
    pi.GetIdsAttr().Set(Vt.Int64Array(ids))


def set_preview_mesh(stage, path: str, points, face_counts, face_indices, color) -> None:
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(points))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.catmullClark)
    mesh.CreateDoubleSidedAttr(True)
    mesh.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        Vt.Vec3fArray([Gf.Vec3f(*color)])
    )


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


def ensure_rig(stage):
    UsdGeom.Xform.Define(stage, RIG_ROOT)
    UsdGeom.Xform.Define(stage, LOOKS_ROOT)

    UsdGeom.PointInstancer.Define(stage, DYNAMIC_PI_PATH)
    UsdGeom.PointInstancer.Define(stage, DEPOSIT_PI_PATH)
    UsdGeom.PointInstancer.Define(stage, WET_COAT_PI_PATH)

    ensure_preview_surface_material(
        stage,
        DYNAMIC_MATERIAL_PATH,
        "DynamicParticleShader",
        diffuse_color=(0.96, 0.77, 0.42),
        roughness=0.28,
        specular_color=(0.14, 0.11, 0.06),
        opacity=1.0,
    )
    ensure_preview_surface_material(
        stage,
        DEPOSIT_MATERIAL_PATH,
        "DepositBodyShader",
        diffuse_color=(0.33, 0.34, 0.35),
        roughness=0.52,
        specular_color=(0.06, 0.06, 0.06),
        opacity=1.0,
    )
    ensure_preview_surface_material(
        stage,
        WET_COAT_MATERIAL_PATH,
        "WetCoatShader",
        diffuse_color=(0.23, 0.24, 0.25),
        roughness=0.12,
        specular_color=(0.22, 0.22, 0.22),
        opacity=0.96,
        ior=1.50,
    )
    ensure_preview_surface_material(
        stage,
        CONTINUOUS_COAT_MATERIAL_PATH,
        "ContinuousCoatShader",
        diffuse_color=(0.34, 0.35, 0.36),
        roughness=0.20,
        specular_color=(0.18, 0.18, 0.18),
        opacity=0.985,
        ior=1.50,
    )

    dyn_proto = UsdGeom.Sphere.Define(stage, DYNAMIC_PROTO_PATH)
    dyn_proto.CreateRadiusAttr(PARTICLE_RADIUS)
    dyn_proto.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        Vt.Vec3fArray([Gf.Vec3f(0.98, 0.77, 0.42)])
    )
    bind_material(stage, DYNAMIC_PROTO_PATH, DYNAMIC_MATERIAL_PATH)

    dep_proto = UsdGeom.Sphere.Define(stage, DEPOSIT_PROTO_PATH)
    dep_proto.CreateRadiusAttr(DEPOSIT_RADIUS)
    dep_proto.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        Vt.Vec3fArray([Gf.Vec3f(0.56, 0.56, 0.59)])
    )
    bind_material(stage, DEPOSIT_PROTO_PATH, DEPOSIT_MATERIAL_PATH)

    wet_proto = UsdGeom.Sphere.Define(stage, WET_COAT_PROTO_PATH)
    wet_proto.CreateRadiusAttr(WET_COAT_RADIUS)
    wet_proto.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        Vt.Vec3fArray([Gf.Vec3f(0.28, 0.28, 0.29)])
    )
    bind_material(stage, WET_COAT_PROTO_PATH, WET_COAT_MATERIAL_PATH)

    set_point_instancer(stage, DYNAMIC_PI_PATH, DYNAMIC_PROTO_PATH, [], [], [])
    set_point_instancer(stage, DEPOSIT_PI_PATH, DEPOSIT_PROTO_PATH, [], [], [])
    set_point_instancer(stage, WET_COAT_PI_PATH, WET_COAT_PROTO_PATH, [], [], [])
    set_preview_mesh(stage, CONTINUOUS_COAT_MESH_PATH, [], [], [], (0.35, 0.36, 0.37))
    bind_material(stage, CONTINUOUS_COAT_MESH_PATH, CONTINUOUS_COAT_MATERIAL_PATH)

    ensure_marker(stage, NOZZLE_MARKER_PATH, 0.10, (0.25, 1.0, 0.30))
    ensure_marker(stage, AIM_MARKER_PATH, 0.08, (1.0, 0.25, 0.25))


def sync_debug_helpers(stage):
    state = getattr(builtins, STATE_KEY, None)
    nozzle_world = state.get("current_nozzle_world") if state else None
    if nozzle_world is None:
        nozzle_world = get_world_pos(stage, NOZZLE_LOCATOR_PATH)
    aim_world = state.get("spray_target_world") if state else None
    if aim_world is None:
        aim_world = get_world_pos(stage, AIM_LOCATOR_PATH)
    set_translate(stage, NOZZLE_MARKER_PATH, nozzle_world)
    set_translate(stage, AIM_MARKER_PATH, aim_world)
    set_curve_points(stage, RAY_PATH, nozzle_world, aim_world)


def maybe_hide_preview_clutter(stage):
    if not HIDE_EXISTING_PREVIEW_OVERLAYS:
        return
    for path in [
        "/ShotcreteCaveMapRig",
        "/ShotcreteVisualAlignmentOverlayRig",
        "/ShotcreteAlignmentAuditRig",
    ]:
        hide_prim(stage, path)
    if HIDE_SOURCE_PROXY_GEOMETRY:
        prune_preview_source_geometry(stage)
        hide_prim(stage, SOURCE_PROXY_COLLIDER_PATH)
        hide_prim(stage, SOURCE_PROXY_MESH_PATH)


def emit_particles(stage, state, n_particles: int):
    if n_particles <= 0:
        return
    nozzle_world = state.get("current_nozzle_world")
    if nozzle_world is None:
        nozzle_world = get_world_pos(stage, NOZZLE_LOCATOR_PATH)
    spray_target_world = state.get("spray_target_world")
    if spray_target_world is None:
        spray_target_world = get_world_pos(stage, AIM_LOCATOR_PATH)
    for _ in range(n_particles):
        if len(state["live_particles"]) >= MAX_LIVE_PARTICLES:
            break
        particle_target_world = spray_target_world
        anchor = state.get("spray_anchor")
        if anchor and SWEEP_TARGET_JITTER_M > 0.0:
            jitter_u, jitter_v = random_in_disk(SWEEP_TARGET_JITTER_M)
            particle_target_world = add(
                spray_target_world,
                add(mul(anchor["u"], jitter_u), mul(anchor["v"], jitter_v)),
            )
        base_dir = normalize(sub(particle_target_world, nozzle_world))
        d = make_random_dir(base_dir)
        p = add(add(nozzle_world, mul(base_dir, NOZZLE_SPAWN_OFFSET_M)), make_spawn_offset(base_dir))
        speed = SPRAY_SPEED_MPS + random.uniform(-SPEED_JITTER_MPS, SPEED_JITTER_MPS)
        state["live_particles"].append(
            {
                "id": state["next_particle_id"],
                "pos": p,
                "vel": mul(d, speed),
            }
        )
        state["next_particle_id"] += 1
        state["emitted_particles"] += 1


def deposit_cell_key(point_world):
    return (
        int(round(float(point_world[0]) / DEPOSIT_CELL_SIZE_M)),
        int(round(float(point_world[1]) / DEPOSIT_CELL_SIZE_M)),
        int(round(float(point_world[2]) / DEPOSIT_CELL_SIZE_M)),
    )


def append_deposit(state, hit_point, surface_normal):
    cell = deposit_cell_key(hit_point)
    stack_count = state["deposit_cell_counts"].get(cell, 0) + 1
    state["deposit_cell_counts"][cell] = stack_count
    surface_normal = normalize(surface_normal)
    deposit_index = state["deposit_cell_index"].get(cell)
    scale = scale_from_stack(stack_count)

    # Represent the retained spray as a surface-aligned splat per cell so thickness
    # builds outward from the cave wall instead of stacking loose spheres in space.
    if deposit_index is None:
        if len(state["deposit_positions"]) < MAX_DEPOSITED:
            point = Gf.Vec3f(float(hit_point[0]), float(hit_point[1]), float(hit_point[2]))
            normal = Gf.Vec3f(float(surface_normal[0]), float(surface_normal[1]), float(surface_normal[2]))
            p = deposit_center(point, normal, scale)
            state["deposit_cell_index"][cell] = len(state["deposit_positions"])
            state["deposit_cell_points"][cell] = point
            state["deposit_cell_normals"][cell] = normal
            state["deposit_positions"].append(Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
            state["deposit_scales"].append(scale)
            state["deposit_orientations"].append(orientation_from_normal(normal))
            state["deposit_ids"].append(state["next_deposit_id"])
            state["next_deposit_id"] += 1
    else:
        prev_hits = max(stack_count - 1, 1)
        keep_weight = float(prev_hits) / float(stack_count)
        add_weight = 1.0 / float(stack_count)
        prev_point = state["deposit_cell_points"][cell]
        prev_normal = state["deposit_cell_normals"][cell]
        blended_point = add(mul(prev_point, keep_weight), mul(hit_point, add_weight))
        blended_normal = normalize(add(mul(prev_normal, float(prev_hits)), surface_normal))
        p = deposit_center(blended_point, blended_normal, scale)
        state["deposit_cell_points"][cell] = Gf.Vec3f(
            float(blended_point[0]), float(blended_point[1]), float(blended_point[2])
        )
        state["deposit_cell_normals"][cell] = Gf.Vec3f(
            float(blended_normal[0]), float(blended_normal[1]), float(blended_normal[2])
        )
        state["deposit_positions"][deposit_index] = Gf.Vec3f(float(p[0]), float(p[1]), float(p[2]))
        state["deposit_scales"][deposit_index] = scale
        state["deposit_orientations"][deposit_index] = orientation_from_normal(blended_normal)
    state["captured_particles"] += 1


def find_capture_hit(state, prev_p, curr_p, vel):
    candidates = candidate_triangles_for_segment(state, prev_p, curr_p, CAVE_CAPTURE_DISTANCE_M * 1.5)
    if not candidates:
        return None

    vel_dir = normalize(vel)
    best_hit = None
    for tri in candidates:
        hit = segment_hit_triangle(prev_p, curr_p, tri)
        if hit is None:
            continue
        t, q = hit
        surface_normal = oriented_surface_normal(tri["normal"], vel_dir)
        receive_dot = -dot(vel_dir, surface_normal)
        if receive_dot < MIN_SURFACE_APPROACH_DOT:
            continue
        if best_hit is None or t < best_hit["t"]:
            best_hit = {
                "kind": "segment_hit",
                "t": t,
                "point": q,
                "tri": tri,
                "normal": surface_normal,
                "distance_m": 0.0,
                "receive_dot": receive_dot,
            }

    if best_hit is not None:
        return best_hit

    best_d2 = None
    best_near = None
    for tri in candidates:
        q = closest_point_on_triangle(curr_p, tri["a"], tri["b"], tri["c"])
        d2 = distance_sq(curr_p, q)
        if best_near is None or d2 < best_d2:
            best_d2 = d2
            best_near = (tri, q)

    if best_near is None:
        return None

    tri, q = best_near
    distance_m = math.sqrt(max(best_d2, 0.0))
    if distance_m > CAVE_CAPTURE_DISTANCE_M:
        return None

    surface_normal = oriented_surface_normal(tri["normal"], vel_dir)
    receive_dot = -dot(vel_dir, surface_normal)
    if receive_dot < MIN_SURFACE_APPROACH_DOT:
        return None

    return {
        "kind": "near_surface",
        "t": 1.0,
        "point": q,
        "tri": tri,
        "normal": surface_normal,
        "distance_m": distance_m,
        "receive_dot": receive_dot,
    }


def sync_dynamic_particles(stage, state):
    positions = []
    scales = []
    ids = []
    for particle in state["live_particles"]:
        p = particle["pos"]
        positions.append(Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
        scales.append(Gf.Vec3f(1.0, 1.0, 1.0))
        ids.append(int(particle["id"]))
    set_point_instancer(stage, DYNAMIC_PI_PATH, DYNAMIC_PROTO_PATH, positions, scales, ids)


def sync_deposits(stage, state):
    if not SHOW_DEPOSIT_SPLATS:
        set_point_instancer(stage, DEPOSIT_PI_PATH, DEPOSIT_PROTO_PATH, [], [], [])
        return
    set_point_instancer(
        stage,
        DEPOSIT_PI_PATH,
        DEPOSIT_PROTO_PATH,
        state["deposit_positions"],
        state["deposit_scales"],
        state["deposit_ids"],
        state["deposit_orientations"],
    )


def sync_wet_coat(stage, state):
    if not SHOW_WET_COAT_SPLATS:
        set_point_instancer(stage, WET_COAT_PI_PATH, WET_COAT_PROTO_PATH, [], [], [])
        return

    positions = []
    scales = []
    ids = []
    orientations = []

    for cell, deposit_index in sorted(state["deposit_cell_index"].items(), key=lambda item: item[1]):
        point = state["deposit_cell_points"].get(cell)
        normal = state["deposit_cell_normals"].get(cell)
        stack_count = state["deposit_cell_counts"].get(cell, 0)
        if point is None or normal is None or stack_count <= 0:
            continue

        scale = wet_coat_scale_from_stack(stack_count)
        p = wet_coat_center(point, normal, scale)
        orientation = (
            state["deposit_orientations"][deposit_index]
            if deposit_index < len(state["deposit_orientations"])
            else orientation_from_normal(normal)
        )
        instance_id = (
            int(state["deposit_ids"][deposit_index]) if deposit_index < len(state["deposit_ids"]) else int(deposit_index + 1)
        )

        positions.append(Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
        scales.append(scale)
        ids.append(instance_id)
        orientations.append(orientation)

    set_point_instancer(
        stage,
        WET_COAT_PI_PATH,
        WET_COAT_PROTO_PATH,
        positions,
        scales,
        ids,
        orientations,
    )


def sync_continuous_coat(stage, state):
    grid = state.get("continuous_coat_grid")
    if not grid:
        state["continuous_coat_stats"] = {"vertices": 0, "faces": 0, "area_m2": 0.0}
        set_preview_mesh(stage, CONTINUOUS_COAT_MESH_PATH, [], [], [], (0.35, 0.36, 0.37))
        return

    width = int(grid["width"])
    height = int(grid["height"])
    total = width * height
    anchor = state.get("spray_anchor")
    all_valid_mask = [True] * total

    support_seed = [0.0] * total
    thickness_seed = [0.0] * total
    presence_seed = [0.0] * total
    point_x_seed = [0.0] * total
    point_y_seed = [0.0] * total
    point_z_seed = [0.0] * total
    normal_x_seed = [0.0] * total
    normal_y_seed = [0.0] * total
    normal_z_seed = [0.0] * total
    influence_radius_sq = CONTINUOUS_COAT_INFLUENCE_RADIUS_M * CONTINUOUS_COAT_INFLUENCE_RADIUS_M
    sigma = max(CONTINUOUS_COAT_INFLUENCE_RADIUS_M * 0.58, 1.0e-4)
    radius_cols = max(2, int(math.ceil(CONTINUOUS_COAT_INFLUENCE_RADIUS_M / grid["step_u"])))
    radius_rows = max(2, int(math.ceil(CONTINUOUS_COAT_INFLUENCE_RADIUS_M / grid["step_v"])))

    for cell, stack_count in state["deposit_cell_counts"].items():
        if stack_count <= 0 or anchor is None:
            continue
        point = state["deposit_cell_points"].get(cell)
        normal = state["deposit_cell_normals"].get(cell)
        if point is None or normal is None:
            continue
        u_offset, v_offset = anchor_uv_from_world(anchor, point)
        col_f = (u_offset + grid["half_width"]) / grid["step_u"]
        row_f = (grid["half_height"] - v_offset) / grid["step_v"]
        col = int(round(col_f))
        row = int(round(row_f))

        thickness_m = continuous_coat_thickness_from_stack(stack_count)
        for d_row in range(-radius_rows, radius_rows + 1):
            rr = row + d_row
            if rr < 0 or rr >= height:
                continue
            for d_col in range(-radius_cols, radius_cols + 1):
                cc = col + d_col
                if cc < 0 or cc >= width:
                    continue
                du = (float(cc) - col_f) * grid["step_u"]
                dv = (float(rr) - row_f) * grid["step_v"]
                dist_sq = du * du + dv * dv
                if dist_sq > influence_radius_sq:
                    continue
                idx = rr * width + cc
                influence = math.exp(-dist_sq / (2.0 * sigma * sigma))
                support_seed[idx] += influence
                presence_seed[idx] += influence
                thickness_seed[idx] += thickness_m * influence
                point_x_seed[idx] += float(point[0]) * influence
                point_y_seed[idx] += float(point[1]) * influence
                point_z_seed[idx] += float(point[2]) * influence
                normal_x_seed[idx] += float(normal[0]) * influence
                normal_y_seed[idx] += float(normal[1]) * influence
                normal_z_seed[idx] += float(normal[2]) * influence

    raw_support = list(support_seed)
    support = smooth_scalar_grid(support_seed, all_valid_mask, width, height, CONTINUOUS_COAT_SMOOTH_PASSES + 1)
    presence_seed = smooth_scalar_grid(presence_seed, all_valid_mask, width, height, CONTINUOUS_COAT_SMOOTH_PASSES)
    thickness = smooth_scalar_grid(thickness_seed, all_valid_mask, width, height, CONTINUOUS_COAT_SMOOTH_PASSES + 1)
    point_x = smooth_scalar_grid(point_x_seed, all_valid_mask, width, height, CONTINUOUS_COAT_SMOOTH_PASSES + 1)
    point_y = smooth_scalar_grid(point_y_seed, all_valid_mask, width, height, CONTINUOUS_COAT_SMOOTH_PASSES + 1)
    point_z = smooth_scalar_grid(point_z_seed, all_valid_mask, width, height, CONTINUOUS_COAT_SMOOTH_PASSES + 1)
    normal_x = smooth_scalar_grid(normal_x_seed, all_valid_mask, width, height, CONTINUOUS_COAT_SMOOTH_PASSES + 1)
    normal_y = smooth_scalar_grid(normal_y_seed, all_valid_mask, width, height, CONTINUOUS_COAT_SMOOTH_PASSES + 1)
    normal_z = smooth_scalar_grid(normal_z_seed, all_valid_mask, width, height, CONTINUOUS_COAT_SMOOTH_PASSES + 1)

    points = []
    active_vertices = 0
    presence = [1.0 - math.exp(-value * 0.92) for value in presence_seed]
    local_supported = [value >= CONTINUOUS_COAT_RAW_SUPPORT_MIN for value in raw_support]
    for idx, plane_point in enumerate(grid["plane_points"]):
        support_value = support[idx]
        if support_value > 1.0e-5:
            base_point = Gf.Vec3f(
                float(point_x[idx] / support_value),
                float(point_y[idx] / support_value),
                float(point_z[idx] / support_value),
            )
            base_normal = normalize(
                Gf.Vec3f(
                    float(normal_x[idx] / support_value),
                    float(normal_y[idx] / support_value),
                    float(normal_z[idx] / support_value),
                )
            )
        else:
            base_point = plane_point
            base_normal = anchor["normal"] if anchor is not None else Gf.Vec3f(0.0, 0.0, 1.0)

        edge_alpha = clamp(
            (presence[idx] - CONTINUOUS_COAT_VERTEX_MIN_PRESENCE) / (1.0 - CONTINUOUS_COAT_VERTEX_MIN_PRESENCE),
            0.0,
            1.0,
        )
        edge_alpha = edge_alpha ** 1.35
        if not local_supported[idx]:
            edge_alpha = 0.0
        final_thickness = min(thickness[idx] * edge_alpha, CONTINUOUS_COAT_MAX_THICKNESS_M)
        p = add(base_point, mul(base_normal, CONTINUOUS_COAT_SURFACE_GAP_M + final_thickness))
        points.append(Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
        if edge_alpha > 1.0e-4:
            active_vertices += 1

    face_counts = []
    face_indices = []
    total_area_m2 = 0.0
    max_face_edge_m = max(grid["step_u"], grid["step_v"]) * CONTINUOUS_COAT_MAX_EDGE_STRETCH
    max_face_area_m2 = (grid["step_u"] * grid["step_v"]) * CONTINUOUS_COAT_MAX_FACE_AREA_MULT
    for row in range(height - 1):
        for col in range(width - 1):
            i0 = row * width + col
            i1 = row * width + (col + 1)
            i2 = (row + 1) * width + col
            i3 = (row + 1) * width + (col + 1)
            if not (local_supported[i0] and local_supported[i1] and local_supported[i2] and local_supported[i3]):
                continue
            presence_avg = 0.25 * (presence[i0] + presence[i1] + presence[i2] + presence[i3])
            if presence_avg < CONTINUOUS_COAT_FACE_MIN_PRESENCE:
                continue
            edge_lengths = (
                math.sqrt(distance_sq(points[i0], points[i1])),
                math.sqrt(distance_sq(points[i1], points[i3])),
                math.sqrt(distance_sq(points[i3], points[i2])),
                math.sqrt(distance_sq(points[i2], points[i0])),
            )
            if max(edge_lengths) > max_face_edge_m:
                continue
            face_area_m2 = quad_area(points[i0], points[i1], points[i3], points[i2])
            if face_area_m2 > max_face_area_m2:
                continue
            face_counts.append(4)
            face_indices.extend([i0, i1, i3, i2])
            total_area_m2 += face_area_m2

    state["continuous_coat_stats"] = {
        "vertices": active_vertices,
        "faces": len(face_counts),
        "area_m2": total_area_m2,
    }
    if not SHOW_CONTINUOUS_COAT_MESH:
        set_preview_mesh(stage, CONTINUOUS_COAT_MESH_PATH, [], [], [], (0.35, 0.36, 0.37))
        bind_material(stage, CONTINUOUS_COAT_MESH_PATH, CONTINUOUS_COAT_MATERIAL_PATH)
        return
    set_preview_mesh(stage, CONTINUOUS_COAT_MESH_PATH, points, face_counts, face_indices, (0.35, 0.36, 0.37))
    bind_material(stage, CONTINUOUS_COAT_MESH_PATH, CONTINUOUS_COAT_MATERIAL_PATH)


def update_particles(stage, state, frame_dt: float):
    nozzle_world = state.get("current_nozzle_world")
    if nozzle_world is None:
        nozzle_world = get_world_pos(stage, NOZZLE_LOCATOR_PATH)
    gravity_step = mul(GRAVITY_MPS2, frame_dt)
    velocity_damp = VELOCITY_DAMP_PER_FRAME ** max(int(round(frame_dt * FPS)), 1)

    kept = []
    new_captures = 0
    near_surface_captures = 0
    segment_captures = 0
    culled_far = 0

    for particle in state["live_particles"]:
        prev_p = particle["pos"]
        vel = add(particle["vel"], gravity_step)
        vel = mul(vel, velocity_damp)
        curr_p = add(prev_p, mul(vel, frame_dt))

        if length(sub(curr_p, nozzle_world)) > MAX_TRAVEL_M:
            culled_far += 1
            state["culled_far"] += 1
            continue

        hit = find_capture_hit(state, prev_p, curr_p, vel)
        if hit is not None:
            append_deposit(state, hit["point"], hit["normal"])
            new_captures += 1
            if hit["kind"] == "segment_hit":
                segment_captures += 1
            else:
                near_surface_captures += 1
            continue

        particle["pos"] = curr_p
        particle["vel"] = vel
        kept.append(particle)

    state["live_particles"] = kept
    state["captured_via_segment"] += segment_captures
    state["captured_via_near_surface"] += near_surface_captures
    state["max_live_particles"] = max(state["max_live_particles"], len(kept))

def summary_payload(stage, state, finalize_reason: str):
    nozzle_world = state.get("current_nozzle_world")
    if nozzle_world is None:
        nozzle_world = get_world_pos(stage, NOZZLE_LOCATOR_PATH)
    aim_world = state.get("spray_target_world")
    if aim_world is None:
        aim_world = get_world_pos(stage, AIM_LOCATOR_PATH)
    layer_counts = list(state["deposit_cell_counts"].values())
    avg_layers = (float(sum(layer_counts)) / float(len(layer_counts))) if layer_counts else 0.0
    max_layers = max(layer_counts) if layer_counts else 0
    sweep_patch_area_m2 = SWEEP_PATCH_WIDTH_M * SWEEP_PATCH_HEIGHT_M
    captured_per_m2 = (float(state["captured_particles"]) / sweep_patch_area_m2) if sweep_patch_area_m2 > 1e-8 else 0.0
    return {
        "run_version_tag": RUN_VERSION_TAG,
        "run_id": state["run_id"],
        "stage_path": state["stage_path"],
        "finalize_reason": finalize_reason,
        "nozzle_locator_path": NOZZLE_LOCATOR_PATH,
        "aim_locator_path": AIM_LOCATOR_PATH,
        "cave_target_mesh_path": CAVE_TARGET_MESH_PATH,
        "nozzle_world": [float(nozzle_world[0]), float(nozzle_world[1]), float(nozzle_world[2])],
        "aim_world": [float(aim_world[0]), float(aim_world[1]), float(aim_world[2])],
        "counts": {
            "emitted_particles": state["emitted_particles"],
            "captured_particles": state["captured_particles"],
            "captured_via_segment": state["captured_via_segment"],
            "captured_via_near_surface": state["captured_via_near_surface"],
            "deposit_instances": len(state["deposit_positions"]),
            "wet_coat_instances": len(state["deposit_cell_index"]),
            "continuous_coat_vertices": state["continuous_coat_stats"]["vertices"],
            "continuous_coat_faces": state["continuous_coat_stats"]["faces"],
            "continuous_coat_area_m2": state["continuous_coat_stats"]["area_m2"],
            "live_particles_final": len(state["live_particles"]),
            "max_live_particles": state["max_live_particles"],
            "culled_far": state["culled_far"],
            "deposit_cells": len(state["deposit_cell_counts"]),
            "avg_layers_per_cell": avg_layers,
            "max_layers_per_cell": max_layers,
            "captured_particles_per_m2": captured_per_m2,
        },
        "config": {
            "emit_duration_seconds": EMIT_DURATION_SECONDS,
            "target_particles_per_sec": TARGET_PARTICLES_PER_SEC,
            "particle_radius": PARTICLE_RADIUS,
            "deposit_radius": DEPOSIT_RADIUS,
            "deposit_cell_size_m": DEPOSIT_CELL_SIZE_M,
            "spray_speed_mps": SPRAY_SPEED_MPS,
            "spray_cone_deg": SPRAY_CONE_DEG,
            "sweep_patch_width_m": SWEEP_PATCH_WIDTH_M,
            "sweep_patch_height_m": SWEEP_PATCH_HEIGHT_M,
            "sweep_patch_area_m2": sweep_patch_area_m2,
            "sweep_raster_passes": SWEEP_RASTER_PASSES,
            "sweep_target_jitter_m": SWEEP_TARGET_JITTER_M,
            "nozzle_motion_enabled": True,
            "nozzle_motion_follow_u": NOZZLE_MOTION_FOLLOW_U,
            "nozzle_motion_follow_v": NOZZLE_MOTION_FOLLOW_V,
            "nozzle_motion_bob_amplitude_m": NOZZLE_MOTION_BOB_AMPLITUDE_M,
            "nozzle_motion_sway_amplitude_m": NOZZLE_MOTION_SWAY_AMPLITUDE_M,
            "wet_coat_enabled": True,
            "wet_coat_spread_mult": WET_COAT_SPREAD_MULT,
            "wet_coat_thickness_scale": WET_COAT_THICKNESS_SCALE,
            "continuous_coat_enabled": True,
            "continuous_coat_grid_step_m": CONTINUOUS_COAT_GRID_STEP_M,
            "continuous_coat_margin_m": CONTINUOUS_COAT_MARGIN_M,
            "gravity_mps2": [float(GRAVITY_MPS2[0]), float(GRAVITY_MPS2[1]), float(GRAVITY_MPS2[2])],
            "capture_distance_m": CAVE_CAPTURE_DISTANCE_M,
            "min_surface_approach_dot": MIN_SURFACE_APPROACH_DOT,
            "triangle_count": len(state["tris"]),
            "triangle_grid_cell_size_m": TRI_GRID_CELL_SIZE_M,
        },
    }


def summary_lines(payload):
    counts = payload["counts"]
    config = payload["config"]
    return [
        f"Shotcrete direct cave preview ({RUN_VERSION_TAG})",
        f"Run id: {payload['run_id']}",
        f"Finalize reason: {payload['finalize_reason']}",
        f"Stage path: {payload['stage_path']}",
        f"Cave mesh: {payload['cave_target_mesh_path']}",
        f"Emitted particles: {counts['emitted_particles']}",
        f"Captured particles: {counts['captured_particles']}",
        f"Captured via segment: {counts['captured_via_segment']}",
        f"Captured via near surface: {counts['captured_via_near_surface']}",
        f"Deposit instances: {counts['deposit_instances']}",
        f"Wet coat instances: {counts['wet_coat_instances']}",
        f"Continuous coat vertices: {counts['continuous_coat_vertices']}",
        f"Continuous coat faces: {counts['continuous_coat_faces']}",
        f"Continuous coat area m2: {counts['continuous_coat_area_m2']:.3f}",
        f"Deposit cells: {counts['deposit_cells']}",
        f"Avg layers per cell: {counts['avg_layers_per_cell']:.2f}",
        f"Max layers per cell: {counts['max_layers_per_cell']}",
        f"Captured particles per m2: {counts['captured_particles_per_m2']:.1f}",
        f"Live particles final: {counts['live_particles_final']}",
        f"Max live particles: {counts['max_live_particles']}",
        f"Culled far: {counts['culled_far']}",
        f"Spray speed mps: {config['spray_speed_mps']:.3f}",
        f"Spray cone deg: {config['spray_cone_deg']:.3f}",
        f"Sweep patch width m: {config['sweep_patch_width_m']:.3f}",
        f"Sweep patch height m: {config['sweep_patch_height_m']:.3f}",
        f"Sweep patch area m2: {config['sweep_patch_area_m2']:.3f}",
        f"Sweep raster passes: {config['sweep_raster_passes']}",
        f"Nozzle motion enabled: {config['nozzle_motion_enabled']}",
        f"Capture distance m: {config['capture_distance_m']:.3f}",
        f"Triangle count: {config['triangle_count']}",
    ]


def finalize_run(stage, state, finalize_reason: str):
    if state.get("finalized"):
        return
    state["finalized"] = True
    sync_dynamic_particles(stage, state)
    sync_deposits(stage, state)
    sync_wet_coat(stage, state)
    sync_continuous_coat(stage, state)
    payload = summary_payload(stage, state, finalize_reason)
    lines = summary_lines(payload)
    write_json(state["run_json_path"], payload)
    write_json(state["latest_json_path"], payload)
    write_text(state["run_txt_path"], "\n".join(lines))
    write_text(state["latest_txt_path"], "\n".join(lines))
    log(
        f"FINALIZED run_id={state['run_id']} reason={finalize_reason} "
        f"emitted={state['emitted_particles']} captured={state['captured_particles']} "
        f"deposits={len(state['deposit_positions'])}"
    )


def maybe_finalize(stage, state, current_frame: int, current_time_sec: float):
    if state.get("finalized"):
        return
    if current_time_sec >= state["hard_finalize_time_sec"]:
        finalize_run(stage, state, "time_cap")
        return
    emit_end_frame = START_FRAME + EMIT_FRAMES
    if current_frame < emit_end_frame + int(MIN_POST_EMIT_FINALIZE_SECONDS * FPS):
        return
    if current_frame < state["next_finalize_probe_frame"]:
        return
    state["next_finalize_probe_frame"] = current_frame + FINALIZE_PROBE_INTERVAL_FRAMES
    if len(state["live_particles"]) <= FINALIZE_LIVE_THRESHOLD:
        state["quiet_probe_hits"] += 1
    else:
        state["quiet_probe_hits"] = 0
    if state["quiet_probe_hits"] >= FINALIZE_QUIET_PROBES:
        finalize_run(stage, state, "quiet_tail")


def on_update(_event):
    stage = get_stage()
    state = getattr(builtins, STATE_KEY, None)
    if state is None:
        return

    timeline = omni.timeline.get_timeline_interface()

    if not timeline.is_playing():
        if state.get("has_started") and not state.get("finalized"):
            finalize_run(stage, state, "timeline_stopped")
        return

    state["has_started"] = True
    state["tick_frame"] += 1
    current_frame = state["tick_frame"]
    current_time_sec = current_frame / FPS
    if current_frame < START_FRAME:
        return

    frame_dt = 1.0 / FPS
    state["last_frame"] = current_frame

    emit_end_frame = START_FRAME + EMIT_FRAMES
    state["spray_target_world"] = compute_spray_target_world(state, current_frame)
    state["current_nozzle_world"] = compute_nozzle_world(stage, state, current_frame)
    if current_frame < emit_end_frame:
        state["particle_accum"] += TARGET_PARTICLES_PER_SEC * frame_dt
        n = int(math.floor(state["particle_accum"]))
        if n > 0:
            state["particle_accum"] -= n
            emit_particles(stage, state, n)

    update_particles(stage, state, frame_dt)
    sync_dynamic_particles(stage, state)
    sync_deposits(stage, state)
    sync_wet_coat(stage, state)
    sync_continuous_coat(stage, state)
    sync_debug_helpers(stage)

    if current_frame >= state["next_status_frame"]:
        state["next_status_frame"] = current_frame + STATUS_LOG_INTERVAL_FRAMES
        log(
            f"STATUS frame={current_frame} live={len(state['live_particles'])} "
            f"emitted={state['emitted_particles']} captured={state['captured_particles']} "
            f"deposits={len(state['deposit_positions'])} "
            f"coat_faces={state['continuous_coat_stats']['faces']}"
        )

    maybe_finalize(stage, state, current_frame, current_time_sec)


def build():
    stage = get_stage()
    if stage is None:
        raise RuntimeError("No open stage.")

    for required_path in [NOZZLE_LOCATOR_PATH, AIM_LOCATOR_PATH, CAVE_TARGET_MESH_PATH]:
        prim = stage.GetPrimAtPath(required_path)
        if not prim or not prim.IsValid():
            raise RuntimeError(f"Missing required prim for preview branch: {required_path}")

    clear_old_state()
    delete_prim_if_exists(stage, RIG_ROOT)
    ensure_rig(stage)
    maybe_hide_preview_clutter(stage)

    cave_mesh_prim = stage.GetPrimAtPath(CAVE_TARGET_MESH_PATH)
    tris = triangulate_mesh(cave_mesh_prim)
    triangle_grid = build_triangle_grid(tris)
    spray_probe_state = {
        "tris": tris,
        "triangle_grid": triangle_grid,
    }
    spray_anchor = compute_spray_anchor(stage, spray_probe_state)
    continuous_coat_grid = build_continuous_coat_surface_grid(spray_probe_state, spray_anchor)

    bundle = output_bundle(stage)
    base_nozzle_world = get_world_pos(stage, NOZZLE_LOCATOR_PATH)
    state = {
        "run_id": bundle["run_id"],
        "stage_path": bundle["stage_path"],
        "run_json_path": bundle["run_json_path"],
        "latest_json_path": bundle["latest_json_path"],
        "run_txt_path": bundle["run_txt_path"],
        "latest_txt_path": bundle["latest_txt_path"],
        "last_frame": -1,
        "tick_frame": 0,
        "particle_accum": 0.0,
        "next_particle_id": 1,
        "next_deposit_id": 1,
        "emitted_particles": 0,
        "captured_particles": 0,
        "captured_via_segment": 0,
        "captured_via_near_surface": 0,
        "culled_far": 0,
        "max_live_particles": 0,
        "live_particles": [],
        "deposit_positions": [],
        "deposit_scales": [],
        "deposit_orientations": [],
        "deposit_ids": [],
        "deposit_cell_counts": {},
        "deposit_cell_index": {},
        "deposit_cell_points": {},
        "deposit_cell_normals": {},
        "tris": tris,
        "triangle_grid": triangle_grid,
        "continuous_coat_grid": continuous_coat_grid,
        "continuous_coat_stats": {
            "vertices": 0,
            "faces": 0,
            "area_m2": 0.0,
        },
        "base_nozzle_world": Gf.Vec3f(
            float(base_nozzle_world[0]),
            float(base_nozzle_world[1]),
            float(base_nozzle_world[2]),
        ),
        "current_nozzle_world": Gf.Vec3f(
            float(base_nozzle_world[0]),
            float(base_nozzle_world[1]),
            float(base_nozzle_world[2]),
        ),
        "spray_anchor": spray_anchor,
        "spray_target_world": spray_anchor["center"],
        "has_started": False,
        "finalized": False,
        "quiet_probe_hits": 0,
        "next_finalize_probe_frame": START_FRAME + EMIT_FRAMES + FINALIZE_PROBE_INTERVAL_FRAMES,
        "hard_finalize_time_sec": (START_FRAME / FPS) + EMIT_DURATION_SECONDS + HARD_FINALIZE_EXTRA_SECONDS,
        "next_status_frame": START_FRAME + STATUS_LOG_INTERVAL_FRAMES,
    }

    setattr(builtins, STATE_KEY, state)
    sync_debug_helpers(stage)
    sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        on_update, name="shotcrete_direct_cave_preview"
    )
    setattr(builtins, SUB_KEY, sub)

    log("Ready.")
    log("Branch = direct-cave preview")
    log(f"Nozzle = {NOZZLE_LOCATOR_PATH}")
    log(f"Aim = {AIM_LOCATOR_PATH}")
    log(f"Cave mesh = {CAVE_TARGET_MESH_PATH}")
    log(f"Triangle count = {len(tris)}")
    log(
        "Spray patch center = "
        f"({spray_anchor['center'][0]:.3f}, {spray_anchor['center'][1]:.3f}, {spray_anchor['center'][2]:.3f})"
    )
    log(f"Sweep patch size m = {SWEEP_PATCH_WIDTH_M:.3f} x {SWEEP_PATCH_HEIGHT_M:.3f}")
    log(
        "Continuous coat grid = "
        f"{continuous_coat_grid['width']} x {continuous_coat_grid['height']} "
        f"over {continuous_coat_grid['patch_width_m']:.3f} x {continuous_coat_grid['patch_height_m']:.3f} m"
    )
    log(f"Sweep raster passes = {SWEEP_RASTER_PASSES}")
    log(
        "Nozzle motion = "
        f"follow_u {NOZZLE_MOTION_FOLLOW_U:.2f}, follow_v {NOZZLE_MOTION_FOLLOW_V:.2f}, "
        f"bob {NOZZLE_MOTION_BOB_AMPLITUDE_M:.2f} m, sway {NOZZLE_MOTION_SWAY_AMPLITUDE_M:.2f} m"
    )
    log(f"RUN_ID = {state['run_id']}")
    log(f"RUN_TXT = {state['run_txt_path']}")
    log(f"RUN_JSON = {state['run_json_path']}")
    log(f"SPRAY_SPEED_MPS = {SPRAY_SPEED_MPS}")
    log(f"TARGET_PARTICLES_PER_SEC = {TARGET_PARTICLES_PER_SEC}")
    log(f"CAVE_CAPTURE_DISTANCE_M = {CAVE_CAPTURE_DISTANCE_M}")


build()
