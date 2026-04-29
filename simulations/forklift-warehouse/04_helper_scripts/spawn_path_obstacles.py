"""
spawn_path_obstacles.py — Spawn SimReady-style cones and cardboard boxes along
the forklift patrol path so LIDAR/APF has obstacles to avoid.

Re-runnable: this script deletes and recreates /World/Obstacles each run.

Run via VS Code: Ctrl+Shift+P -> Isaac Sim: Run File Remotely
Scene must already be open in Isaac Sim.
"""
from __future__ import annotations

import random

import carb
import omni.kit.commands
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

# -- Configuration -------------------------------------------------------------
OBSTACLES_GROUP = "/World/Obstacles"
SEED = 42

CONE_ASSETS = [
    "https://omniverse-content-staging.s3.us-west-2.amazonaws.com/Assets/simready_content/common_assets/props/trafficcone_a05/trafficcone_a05.usd",
    "https://omniverse-content-staging.s3.us-west-2.amazonaws.com/Assets/simready_content/common_assets/props/trafficcone_a06/trafficcone_a06.usd",
]
BOX_ASSETS = [
    "https://omniverse-content-staging.s3.us-west-2.amazonaws.com/Assets/simready_content/common_assets/props/cubebox_a02/cubebox_a02.usd",
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

OBSTACLE_Z = 0.0

# (kind, x, y)
OBSTACLES = [
    ("cone", 0.0, -24.8),
    ("box", 6.0, -27.2),
    ("cone", 12.0, -24.6),
    ("stack", 18.5, -5.0),
    ("stack", 18.2, 30.0),
    ("cone", 15.5, 15.0),
    ("box", 18.2, 30.0),
    ("cone", 10.0, 46.5),
    ("box", 0.0, 49.5),
    ("cone", -12.0, 46.4),
    ("stack", -22.5, 30.0),
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
    def _apply_collision_apis(target_prim):
        if not target_prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(target_prim)
        if not target_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            mesh_col = UsdPhysics.MeshCollisionAPI.Apply(target_prim)
            mesh_col.GetApproximationAttr().Set("convexHull")

    group_prim = stage.GetPrimAtPath(OBSTACLES_GROUP)
    if group_prim.IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[OBSTACLES_GROUP])
        carb.log_info(f"[spawn_path_obstacles] Deleted existing group: {OBSTACLES_GROUP}")

    UsdGeom.Xform.Define(stage, OBSTACLES_GROUP)

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
        elif kind == "stack":
            xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(STACK_SCALE, STACK_SCALE, STACK_SCALE)
            )
            # Build a composed stack: export pallet (bottom), palette (middle), bins grid (top).
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
        else:
            carb.log_warn(f"[spawn_path_obstacles] Unknown obstacle kind '{kind}' at index {idx}; skipping")

    carb.log_info(f"[spawn_path_obstacles] Spawned {len(OBSTACLES)} obstacles under {OBSTACLES_GROUP}")
