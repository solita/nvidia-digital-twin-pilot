"""
reset_forklift.py — Snap the forklift back to its authored start position and cancel
any running controller task.

Run via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely
The timeline will be stopped and the forklift will return to its rest pose.
"""
from __future__ import annotations

import asyncio

import carb
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

# ── Must match forklift_controller.py ────────────────────────────────────────
FORKLIFT_PRIM_PATH = "/World/forklift_b"

# Rest position and heading from get_forklift_transform.py
# REST_HEADING set to 90° (north) to match the straight-line test waypoint
# so the controller starts with zero initial turn.
# Change back to 177.39 when restoring the full patrol route.
REST_X       = -29.090045
REST_Y       = -17.476563
REST_Z       =  0.0
REST_HEADING =  90.0   # face north — aligns with straight-line test waypoint (-29.09, 53.0)

# ── Cancel any running controller task ───────────────────────────────────────
_TASK_KEY = "_forklift_controller_task"
_existing = getattr(asyncio.get_event_loop(), _TASK_KEY, None)
if _existing and not _existing.done():
    _existing.cancel()
    carb.log_info("[reset_forklift] Cancelled running controller task.")

# ── Stop timeline ─────────────────────────────────────────────────────────────
omni.timeline.get_timeline_interface().stop()

# ── Reset transform ───────────────────────────────────────────────────────────
stage = omni.usd.get_context().get_stage()
prim  = stage.GetPrimAtPath(FORKLIFT_PRIM_PATH)

if not prim.IsValid():
    carb.log_error(f"[reset_forklift] Prim not found: {FORKLIFT_PRIM_PATH!r}")
else:
    xform = UsdGeom.Xformable(prim)
    ops   = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}

    # Position
    if "xformOp:translate" in ops:
        ops["xformOp:translate"].Set(Gf.Vec3d(REST_X, REST_Y, REST_Z))
    else:
        xform.AddTranslateOp().Set(Gf.Vec3d(REST_X, REST_Y, REST_Z))

    # Heading
    from pxr import Gf as _Gf
    rot  = _Gf.Rotation(_Gf.Vec3d(0, 0, 1), REST_HEADING)
    q    = rot.GetQuat()
    gf_q = _Gf.Quatf(float(q.GetReal()),
                     float(q.GetImaginary()[0]),
                     float(q.GetImaginary()[1]),
                     float(q.GetImaginary()[2]))
    if "xformOp:orient" in ops:
        ops["xformOp:orient"].Set(gf_q)
    else:
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(gf_q)

    carb.log_info(
        f"[reset_forklift] Forklift reset to ({REST_X}, {REST_Y}, {REST_Z}), "
        f"heading {REST_HEADING}°"
    )
    print(f"[reset_forklift] Done — forklift back at start, controller stopped.")
