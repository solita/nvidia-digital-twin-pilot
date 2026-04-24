"""
reset_forklift.py — Snap all forklifts back to their authored start positions and
cancel any running controller tasks.

Run via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely
The timeline will be stopped and all forklifts will return to their rest poses.
"""
from __future__ import annotations

import asyncio

import carb
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

# ── Must match forklift_controller.py and populate_scene.py ──────────────────

FORKLIFTS = [
    {"id": "forklift_0", "x": -15.0, "y": -25.0, "heading": 90.0},
    {"id": "forklift_1", "x":  -5.0, "y": -25.0, "heading": 90.0},
    {"id": "forklift_2", "x":   5.0, "y": -25.0, "heading": 90.0},
    {"id": "forklift_3", "x":  15.0, "y": -25.0, "heading": 90.0},
]
REST_Z = 0.0

# ── Cancel any running controller tasks ──────────────────────────────────────
for fl in FORKLIFTS:
    _TASK_KEY = f"_forklift_controller_task_{fl['id']}"
    _existing = getattr(asyncio.get_event_loop(), _TASK_KEY, None)
    if _existing and not _existing.done():
        _existing.cancel()
        carb.log_info(f"[reset_forklift] Cancelled controller task for {fl['id']}.")

# ── Stop timeline ─────────────────────────────────────────────────────────────
omni.timeline.get_timeline_interface().stop()

# ── Reset transforms ──────────────────────────────────────────────────────────
stage = omni.usd.get_context().get_stage()

for fl in FORKLIFTS:
    prim_path = f"/World/{fl['id']}"
    prim = stage.GetPrimAtPath(prim_path)

    if not prim.IsValid():
        carb.log_error(f"[reset_forklift] Prim not found: {prim_path!r}")
        continue

    xform = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}

    # Position
    if "xformOp:translate" in ops:
        ops["xformOp:translate"].Set(Gf.Vec3d(fl["x"], fl["y"], REST_Z))
    else:
        xform.AddTranslateOp().Set(Gf.Vec3d(fl["x"], fl["y"], REST_Z))

    # Heading
    rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), fl["heading"])
    q = rot.GetQuat()
    gf_q = Gf.Quatf(
        float(q.GetReal()),
        float(q.GetImaginary()[0]),
        float(q.GetImaginary()[1]),
        float(q.GetImaginary()[2]),
    )
    if "xformOp:orient" in ops:
        ops["xformOp:orient"].Set(gf_q)
    else:
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(gf_q)

    carb.log_info(
        f"[reset_forklift] {fl['id']} reset to ({fl['x']}, {fl['y']}, {REST_Z}), "
        f"heading {fl['heading']}°"
    )

    # Reset steer joint angle to 0
    steer_path = f"/World/{fl['id']}/back_wheel_joints/back_wheel_swivel"
    steer_joint = stage.GetPrimAtPath(steer_path)
    if steer_joint.IsValid():
        steer_api = UsdPhysics.DriveAPI(steer_joint, "angular")
        steer_api.GetTargetPositionAttr().Set(0.0)
        steer_api.GetStiffnessAttr().Set(3000.0)
        steer_api.GetDampingAttr().Set(10000.0)

print(f"[reset_forklift] Done — all {len(FORKLIFTS)} forklifts reset, controller stopped.")
