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

OBSTACLE_Z = 0.0

# (kind, x, y)
OBSTACLES = [
    ("cone", 0.0, -24.8),
    ("box", 6.0, -27.2),
    ("cone", 12.0, -24.6),
    ("box", 18.5, -5.0),
    ("cone", 15.5, 15.0),
    ("box", 18.2, 30.0),
    ("cone", 10.0, 46.5),
    ("box", 0.0, 49.5),
    ("cone", -12.0, 46.4),
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
    group_prim = stage.GetPrimAtPath(OBSTACLES_GROUP)
    if group_prim.IsValid():
        omni.kit.commands.execute("DeletePrims", paths=[OBSTACLES_GROUP])
        carb.log_info(f"[spawn_path_obstacles] Deleted existing group: {OBSTACLES_GROUP}")

    UsdGeom.Xform.Define(stage, OBSTACLES_GROUP)

    random.seed(SEED)

    for idx, (kind, x, y) in enumerate(OBSTACLES, start=1):
        prim_path = f"{OBSTACLES_GROUP}/{kind}_{idx:02d}"
        prim = stage.DefinePrim(prim_path, "Xform")

        if kind == "cone":
            asset_url = random.choice(CONE_ASSETS)
        else:
            asset_url = random.choice(BOX_ASSETS)

        prim.GetReferences().AddReference(asset_url)

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

        # Ensure collision so LIDAR raycasts detect the obstacle
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
        if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            mesh_col = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_col.GetApproximationAttr().Set("convexHull")

    carb.log_info(f"[spawn_path_obstacles] Spawned {len(OBSTACLES)} obstacles under {OBSTACLES_GROUP}")
