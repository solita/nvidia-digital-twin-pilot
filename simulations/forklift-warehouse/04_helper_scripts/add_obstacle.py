"""
add_obstacle.py — Add one realistic pushcart + pallet + KLT-bin stack obstacle.

Run once in the Isaac Sim Script Editor (or via launcher).

What it does
------------
Creates /World/Obstacles/Obstacle_01 as a parent Xform group containing:
  • Pushcart   — SM_PushcartA_02 flatbed trolley (base of the stack)
  • KLT_001 … KLT_064 — 4 × 4 × 4 grid of SmallKLT bins on top of the pushcart

Z positions are determined at runtime using UsdGeom.BBoxCache so they are
always correct regardless of each asset's internal origin offset.

Reference paths are resolved to absolute URLs from existing warehouse prims,
avoiding the "Could not open asset" error from preserved relative paths.

Next steps (manual, in the Stage panel)
-----------------------------------------
1. Ctrl+D duplicate /World/Obstacles/Obstacle_01 for each remaining cube.
2. Set translate XY of each copy:
     Cube_01  (-15.01, -26.38)
     Cube_02  ( -6.80, -26.54)
     Cube_03  ( -6.80, -27.56)
     Cube_04  (-14.14, -29.98)
     Cube_06  ( -1.24, -33.05)
3. Delete /World/Cube, /World/Cube_01 … /World/Cube_06.
"""

import carb
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom

# ── Config ─────────────────────────────────────────────────────────────────────

PUSHCART_SOURCE = "/World/warehouse/SM_PushcartA_02_22"
KLT_SOURCE      = "/World/warehouse/KLT_Bins/SmallKLT_Visual_02"

GROUP_PRIM = "/World/Obstacles/Obstacle_01"
POSITION   = Gf.Vec3d(-15.01, -23.87, 0.0)   # same XY as first cube

# KLT grid: 4 cols × 4 rows × 4 layers  (= 64 bins)
KLT_STEP_X = 0.22   # m between bin centres along X (0.20 m wide + 0.02 m gap)
KLT_STEP_Y = 0.32   # m between bin centres along Y (0.30 m wide + 0.02 m gap)
KLT_COLS   = 4
KLT_ROWS   = 4
KLT_LAYERS = 4

# ── Helpers ────────────────────────────────────────────────────────────────────

def _resolve_asset(stage, source_path: str) -> "str | None":
    """Resolve the reference/payload on *source_path* to an absolute URL."""
    prim = stage.GetPrimAtPath(source_path)
    if not prim.IsValid():
        carb.log_error(f"[add_obstacle] Source prim not found: {source_path}")
        return None
    for spec in prim.GetPrimStack():
        for item in spec.referenceList.prependedItems:
            if item.assetPath:
                return spec.layer.ComputeAbsolutePath(item.assetPath)
        for item in spec.payloadList.prependedItems:
            if item.assetPath:
                return spec.layer.ComputeAbsolutePath(item.assetPath)
    carb.log_error(f"[add_obstacle] Cannot resolve asset from {source_path}")
    return None


def _place(stage, wrapper_path: str, asset_abs: str, pos: Gf.Vec3d) -> None:
    """
    Two-level prim: wrapper (translate only) + asset child (reference only).

    Keeping translate and reference on separate prims prevents xformOpOrder
    conflicts that cause all bins to collapse to the same position.
    """
    if stage.GetPrimAtPath(wrapper_path).IsValid():
        stage.RemovePrim(Sdf.Path(wrapper_path))
    w = UsdGeom.Xform.Define(stage, wrapper_path)
    w.AddTranslateOp().Set(pos)
    a = UsdGeom.Xform.Define(stage, f"{wrapper_path}/asset")
    a.GetPrim().GetReferences().AddReference(asset_abs)


def _top_z(stage, prim_path: str, bbox_cache: UsdGeom.BBoxCache) -> float:
    """Return the world-space max Z of *prim_path* using the given BBoxCache."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        carb.log_warn(f"[add_obstacle] _top_z: prim not found {prim_path}, using 0")
        return 0.0
    bound = bbox_cache.ComputeWorldBound(prim)
    return bound.GetRange().GetMax()[2]

# ── Main ───────────────────────────────────────────────────────────────────────

def add_obstacle() -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        carb.log_error("[add_obstacle] No stage loaded.")
        return

    pushcart_asset = _resolve_asset(stage, PUSHCART_SOURCE)
    klt_asset      = _resolve_asset(stage, KLT_SOURCE)
    if not all([pushcart_asset, klt_asset]):
        return

    carb.log_info(f"[add_obstacle] pushcart → {pushcart_asset}")
    carb.log_info(f"[add_obstacle] klt bin → {klt_asset}")

    # BBoxCache — reused for all measurements; useExtentsHint speeds it up
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[UsdGeom.Tokens.default_],
        useExtentsHint=True,
    )

    # Create /World/Obstacles group if needed.
    if not stage.GetPrimAtPath("/World/Obstacles").IsValid():
        UsdGeom.Xform.Define(stage, "/World/Obstacles")

    # Remove any stale Obstacle_01.
    if stage.GetPrimAtPath(GROUP_PRIM).IsValid():
        stage.RemovePrim(Sdf.Path(GROUP_PRIM))

    # Parent group at world POSITION (XY cursor + floor Z).
    group = UsdGeom.Xform.Define(stage, GROUP_PRIM)
    group.AddTranslateOp().Set(POSITION)

    # ── 1. Pushcart — sits on the floor (local Z = 0) ─────────────────────────
    _place(stage, f"{GROUP_PRIM}/Pushcart", pushcart_asset, Gf.Vec3d(0, 0, 0))
    pushcart_top = _top_z(stage, f"{GROUP_PRIM}/Pushcart/asset", bbox_cache)
    # top_z is world-space; convert to local by subtracting the group's world Z
    pushcart_top_local = pushcart_top - POSITION[2]
    carb.log_info(f"[add_obstacle] Pushcart top (local Z) = {pushcart_top_local:.3f} m")

    # ── 2. KLT bins — 4 × 4 grid, 4 layers high, direct on pushcart ─────────
    # Probe one bin at Z=0 to measure its actual full height.
    _place(stage, f"{GROUP_PRIM}/_klt_probe", klt_asset, Gf.Vec3d(0, 0, 0))
    probe_bbox = bbox_cache.ComputeWorldBound(
        stage.GetPrimAtPath(f"{GROUP_PRIM}/_klt_probe/asset")
    ).GetRange()
    klt_full_h = probe_bbox.GetMax()[2] - probe_bbox.GetMin()[2]
    klt_half_h = klt_full_h / 2.0
    stage.RemovePrim(Sdf.Path(f"{GROUP_PRIM}/_klt_probe"))
    carb.log_info(
        f"[add_obstacle] KLT measured: full_h={klt_full_h:.3f} m, "
        f"half_h={klt_half_h:.3f} m"
    )

    offset_x = (KLT_COLS - 1) * KLT_STEP_X / 2.0
    offset_y = (KLT_ROWS - 1) * KLT_STEP_Y / 2.0
    bin_idx = 1
    for layer in range(KLT_LAYERS):
        # bin centre Z: pushcart top + half-height + (full-height × layer index)
        lz = pushcart_top_local + klt_half_h + layer * klt_full_h
        for row in range(KLT_ROWS):
            for col in range(KLT_COLS):
                lx = col * KLT_STEP_X - offset_x
                ly = row * KLT_STEP_Y - offset_y
                _place(stage, f"{GROUP_PRIM}/KLT_{bin_idx:03d}", klt_asset,
                       Gf.Vec3d(lx, ly, lz))
                bin_idx += 1

    stack_top = pushcart_top_local + klt_full_h * KLT_LAYERS
    total_bins = bin_idx - 1
    carb.log_info(
        f"[add_obstacle] Done — pushcart + {total_bins} KLT bins "
        f"({KLT_COLS}×{KLT_ROWS}×{KLT_LAYERS}), stack top ≈ {stack_top:.3f} m"
    )
    carb.log_info("  Collision active on child meshes — LIDAR detects the full stack.")
    carb.log_info("  Ctrl+D the group in Stage panel for each remaining cube position.")


add_obstacle()

