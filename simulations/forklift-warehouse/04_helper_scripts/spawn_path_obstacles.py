"""
spawn_path_obstacles.py — Spawn SimReady-style cones and cardboard boxes along
the forklift patrol path so LIDAR/APF has obstacles to avoid.

Re-runnable: this script deletes and recreates /World/Obstacles each run.

Run via VS Code: Ctrl+Shift+P -> Isaac Sim: Run File Remotely
Scene must already be open in Isaac Sim.
"""
from __future__ import annotations

import json
import os
import random

import carb
import omni.kit.commands
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

# -- Configuration -------------------------------------------------------------
OBSTACLES_GROUP = "/World/Obstacles"
SEED = 42

CONE_ASSETS = [
    "https://omniverse-content-staging.s3.us-west-2.amazonaws.com/Assets/simready_content/common_assets/props/trafficcone_a05/trafficcone_a05.usd",
    "https://omniverse-content-staging.s3.us-west-2.amazonaws.com/Assets/simready_content/common_assets/props/trafficcone_a06/trafficcone_a06.usd",
]
BOX_ASSETS = [
    "https://omniverse-content-staging.s3.us-west-2.amazonaws.com/Assets/simready_content/common_assets/props/cubebox_a09/cubebox_a09.usd",
]

STACK_EXPORTPALLET_ASSET = (
    "https://omniverse-content-staging.s3.us-west-2.amazonaws.com/Assets/simready_content/common_assets/props/exportpallet_a01/exportpallet_a01.usd"
)
STACK_PALETTE_INTERNAL_PRIM = "/World/warehouse/SM_PaletteA_360"
STACK_KLT_BIN_INTERNAL_PRIM = "/World/warehouse/KLT_Bins/SmallKLT_Visual_156"

STACK_PALETTE_Z_OFFSET = 0.16
STACK_BINS_Z_OFFSET = 0.48
STACK_BIN_GRID_SIZE = 4
STACK_BIN_SPACING = 0.26
STACK_SCALE = 4.0

# pushcart-style stack (pushcart + 4x4x4 KLT bins)
ADD_OBS_PUSHCART_SOURCE = "/World/warehouse/SM_PushcartA_02_22"
ADD_OBS_KLT_SOURCE = "/World/warehouse/KLT_Bins/SmallKLT_Visual_02"
ADD_OBS_KLT_STEP_X = 0.22
ADD_OBS_KLT_STEP_Y = 0.32
ADD_OBS_KLT_COLS = 4
ADD_OBS_KLT_ROWS = 4
ADD_OBS_KLT_LAYERS = 4

OBSTACLE_Z = 0.0

# (kind, x, y)
OBSTACLES = [
    ("cone", 0.0, -24.8),
    ("box", 6.0, -27.2),
    ("cone", 12.0, -24.6),
    ("pushcart", 18.5, -5.0),
    ("pallet", 18.2, 30.0),
    ("cone", 15.5, 15.0),
    ("box", 18.2, 30.0),
    ("cone", 10.0, 46.5),
    ("box", 0.0, 49.5),
    ("cone", -12.0, 46.4),
    ("pallet", -22.5, 30.0),
    ("box", -22.5, 30.0),
    ("cone", -25.5, 10.0),
    ("box", -22.8, -10.0),
    ("cone", -17.0, -22.0),
    ("box", -12.0, -20.0),
]

# -- Stage --------------------------------------------------------------------
stage = omni.usd.get_context().get_stage()

if stage is None:
    carb.log_error("[spawn_path_obstacles] No active USD stage. Load a scene first.")
else:
    def _resolve_asset(source_path: str) -> str | None:
        """Resolve a prim reference/payload to an absolute asset URL."""
        prim = stage.GetPrimAtPath(source_path)
        if not prim.IsValid():
            carb.log_error(f"[spawn_path_obstacles] Source prim not found: {source_path}")
            return None

        for spec in prim.GetPrimStack():
            for item in spec.referenceList.prependedItems:
                if item.assetPath:
                    return spec.layer.ComputeAbsolutePath(item.assetPath)
            for item in spec.payloadList.prependedItems:
                if item.assetPath:
                    return spec.layer.ComputeAbsolutePath(item.assetPath)

        carb.log_error(f"[spawn_path_obstacles] Cannot resolve asset from {source_path}")
        return None

    def _apply_collision_apis(target_prim):
        if not target_prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(target_prim)
        if not target_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            mesh_col = UsdPhysics.MeshCollisionAPI.Apply(target_prim)
            mesh_col.GetApproximationAttr().Set("convexHull")

    def _apply_collision_to_meshes(root_prim):
        """Apply collision and mesh-collision APIs to all mesh descendants."""
        for prim in Usd.PrimRange(root_prim):
            if prim.IsA(UsdGeom.Mesh):
                _apply_collision_apis(prim)

    def _place_ref_asset(wrapper_path: str, asset_abs: str, pos: Gf.Vec3d):
        """Create wrapper xform (translate) with a child that carries the reference."""
        if stage.GetPrimAtPath(wrapper_path).IsValid():
            stage.RemovePrim(Sdf.Path(wrapper_path))

        wrapper = UsdGeom.Xform.Define(stage, wrapper_path)
        wrapper.AddTranslateOp().Set(pos)

        asset_prim = UsdGeom.Xform.Define(stage, f"{wrapper_path}/asset").GetPrim()
        asset_prim.GetReferences().AddReference(asset_abs)
        _apply_collision_apis(wrapper.GetPrim())
        _apply_collision_to_meshes(asset_prim)

    def _build_pushcart_stack(prim_path: str, pushcart_asset: str, klt_asset: str):
        """Build pushcart-style pushcart + 4x4x4 KLT stack under prim_path."""
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            includedPurposes=[UsdGeom.Tokens.default_],
            useExtentsHint=True,
        )

        # 1) Pushcart at local origin.
        _place_ref_asset(f"{prim_path}/pushcart", pushcart_asset, Gf.Vec3d(0.0, 0.0, 0.0))
        pushcart_asset_prim = stage.GetPrimAtPath(f"{prim_path}/pushcart/asset")
        pushcart_top_world_z = bbox_cache.ComputeWorldBound(pushcart_asset_prim).GetRange().GetMax()[2]

        parent_world_z = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath(prim_path)).GetRange().GetMin()[2]
        pushcart_top_local_z = pushcart_top_world_z - parent_world_z

        # 2) Probe one KLT to measure actual height.
        probe_path = f"{prim_path}/_klt_probe"
        _place_ref_asset(probe_path, klt_asset, Gf.Vec3d(0.0, 0.0, 0.0))
        probe_range = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath(f"{probe_path}/asset")).GetRange()
        klt_full_h = probe_range.GetMax()[2] - probe_range.GetMin()[2]
        klt_half_h = klt_full_h * 0.5
        stage.RemovePrim(Sdf.Path(probe_path))

        offset_x = (ADD_OBS_KLT_COLS - 1) * ADD_OBS_KLT_STEP_X * 0.5
        offset_y = (ADD_OBS_KLT_ROWS - 1) * ADD_OBS_KLT_STEP_Y * 0.5

        bin_idx = 1
        for layer in range(ADD_OBS_KLT_LAYERS):
            lz = pushcart_top_local_z + klt_half_h + layer * klt_full_h
            for row in range(ADD_OBS_KLT_ROWS):
                for col in range(ADD_OBS_KLT_COLS):
                    lx = col * ADD_OBS_KLT_STEP_X - offset_x
                    ly = row * ADD_OBS_KLT_STEP_Y - offset_y
                    _place_ref_asset(
                        f"{prim_path}/klt_{bin_idx:03d}",
                        klt_asset,
                        Gf.Vec3d(lx, ly, lz),
                    )
                    bin_idx += 1

    group_prim = stage.GetPrimAtPath(OBSTACLES_GROUP)
    if group_prim.IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[OBSTACLES_GROUP])
        carb.log_info(f"[spawn_path_obstacles] Deleted existing group: {OBSTACLES_GROUP}")

    UsdGeom.Xform.Define(stage, OBSTACLES_GROUP)

    add_obs_pushcart_asset = _resolve_asset(ADD_OBS_PUSHCART_SOURCE)
    add_obs_klt_asset = _resolve_asset(ADD_OBS_KLT_SOURCE)
    add_obs_enabled = bool(add_obs_pushcart_asset and add_obs_klt_asset)
    if not add_obs_enabled:
        carb.log_warn("[spawn_path_obstacles] pushcart kind disabled (asset resolution failed).")

    random.seed(SEED)

    for idx, (kind, x, y) in enumerate(OBSTACLES, start=1):
        prim_path = f"{OBSTACLES_GROUP}/{kind}_{idx:02d}"
        prim = stage.DefinePrim(prim_path, "Xform")

        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(x, y, OBSTACLE_Z))
        yaw = random.uniform(0.0, 360.0)
        rot_q = Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw).GetQuat()
        q = Gf.Quatf(
            float(rot_q.GetReal()),
            float(rot_q.GetImaginary()[0]),
            float(rot_q.GetImaginary()[1]),
            float(rot_q.GetImaginary()[2]),
        )
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(q)

        if kind == "cone":
            asset_url = random.choice(CONE_ASSETS)
            prim.GetReferences().AddReference(asset_url)
            _apply_collision_apis(prim)
        elif kind == "box":
            asset_url = random.choice(BOX_ASSETS)
            prim.GetReferences().AddReference(asset_url)
            _apply_collision_apis(prim)
        elif kind == "pallet":
            xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(STACK_SCALE, STACK_SCALE, STACK_SCALE)
            )
            # Build a composed pallet: export pallet (bottom), palette (middle), bins grid (top).
            export_pallet = stage.DefinePrim(f"{prim_path}/exportpallet", "Xform")
            export_pallet.GetReferences().AddReference(STACK_EXPORTPALLET_ASSET)
            _apply_collision_apis(export_pallet)

            palette = stage.DefinePrim(f"{prim_path}/palette", "Xform")
            palette.GetReferences().AddInternalReference(STACK_PALETTE_INTERNAL_PRIM)
            palette_xf = UsdGeom.Xformable(palette)
            palette_xf.ClearXformOpOrder()
            palette_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, STACK_PALETTE_Z_OFFSET))
            _apply_collision_apis(palette)

            half = (STACK_BIN_GRID_SIZE - 1) * 0.5
            for row in range(STACK_BIN_GRID_SIZE):
                for col in range(STACK_BIN_GRID_SIZE):
                    bin_prim = stage.DefinePrim(f"{prim_path}/bin_{row}_{col}", "Xform")
                    bin_prim.GetReferences().AddInternalReference(STACK_KLT_BIN_INTERNAL_PRIM)
                    bin_xf = UsdGeom.Xformable(bin_prim)
                    bin_xf.ClearXformOpOrder()
                    bin_x = (col - half) * STACK_BIN_SPACING
                    bin_y = (row - half) * STACK_BIN_SPACING
                    bin_xf.AddTranslateOp().Set(Gf.Vec3d(bin_x, bin_y, STACK_BINS_Z_OFFSET))
                    _apply_collision_apis(bin_prim)
        elif kind == "pushcart":
            if not add_obs_enabled:
                carb.log_warn(
                    f"[spawn_path_obstacles] pushcart assets unavailable at index {idx}; skipping"
                )
                continue
            _build_pushcart_stack(prim_path, add_obs_pushcart_asset, add_obs_klt_asset)
            _apply_collision_apis(prim)
            _apply_collision_to_meshes(prim)
        else:
            carb.log_warn(f"[spawn_path_obstacles] Unknown obstacle kind '{kind}' at index {idx}; skipping")

    # Export obstacle manifest so the dashboard can pick it up dynamically.
    # Compute bounding boxes so the dashboard draws each obstacle to scale.
    _bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[UsdGeom.Tokens.default_],
        useExtentsHint=True,
    )
    _obs_out = []
    for _idx, (_kind, _x, _y) in enumerate(OBSTACLES, start=1):
        _prim_path = f"{OBSTACLES_GROUP}/{_kind}_{_idx:02d}"
        _entry = {"kind": _kind, "x": _x, "y": _y}
        _prim = stage.GetPrimAtPath(_prim_path)
        if _prim.IsValid():
            _world_range = _bbox_cache.ComputeWorldBound(_prim).GetRange()
            _entry["size_x"] = round(_world_range.GetMax()[0] - _world_range.GetMin()[0], 3)
            _entry["size_y"] = round(_world_range.GetMax()[1] - _world_range.GetMin()[1], 3)
        _obs_out.append(_entry)
    _manifest_path = (
        "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot/"
        "simulations/forklift-warehouse/04_current_outputs/path_obstacles.json"
    )
    with open(_manifest_path, "w", encoding="utf-8") as _fh:
        json.dump(_obs_out, _fh, indent=2)
    carb.log_info(f"[spawn_path_obstacles] Wrote obstacle manifest to {_manifest_path}")

    carb.log_info(f"[spawn_path_obstacles] Spawned {len(OBSTACLES)} obstacles under {OBSTACLES_GROUP}")
