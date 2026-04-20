"""
attach_follow_camera.py — Attach 4 follow cameras to the forklift (Back, Front, Left, Right).

All cameras are parented to /World/forklift_b/body so they move with the forklift.
Re-running the script recreates them, so you can tune offsets and re-run freely.

Switch between cameras using the viewport dropdown (top-left of the viewport).
Default active camera after running: CamBack (overview from behind).

Run via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely
"""
from __future__ import annotations

import omni.kit.viewport.utility
import omni.usd
from pxr import Gf, UsdGeom

BODY_PRIM_PATH = "/World/forklift_b/body"

# ── Camera definitions: (name, offset_xyz, rotateXYZ) ────────────────────────
#   offset in forklift local space (X=forward, Y=left, Z=up)
#   rotateXYZ applied after translate to aim the lens at the forklift
#
#   Rotation convention (Isaac Sim / USD):
#     default camera looks down -Z.  RotateXYZ tilts/pans it into place.
#     (90 - tilt_down, 0, -90) → rear camera looking forward+down
#
CAMERAS = [
    {
        "name":     "CamBack",
        "offset":   Gf.Vec3d(-14.0,  0.0,  8.0),   # behind + above
        "rotation": Gf.Vec3f(90 - 25, 0.0, -90.0),  # tilt 25° down, face forward
    },
    {
        "name":     "CamFront",
        "offset":   Gf.Vec3d( 10.0,  0.0,  5.0),   # in front + above
        "rotation": Gf.Vec3f(90 - 20, 0.0,  90.0),  # face backward toward forklift
    },
    {
        "name":     "CamLeft",
        "offset":   Gf.Vec3d(  0.0, 10.0,  5.0),   # left side + above
        "rotation": Gf.Vec3f(90 - 20, 0.0,   0.0),  # face right toward forklift
    },
    {
        "name":     "CamRight",
        "offset":   Gf.Vec3d(  0.0,-10.0,  5.0),   # right side + above
        "rotation": Gf.Vec3f(90 - 20, 0.0, 180.0),  # face left toward forklift
    },
]

# Which camera to activate in the viewport after running
DEFAULT_CAMERA = "CamBack"

# Legacy path to clean up
_OLD_CAMERA_PATH = "/World/forklift_b/FollowCam"

# ── Main ──────────────────────────────────────────────────────────────────────
stage = omni.usd.get_context().get_stage()

if not stage:
    print("[attach_follow_camera] ERROR: No stage loaded.")
else:
    # Remove legacy single camera if present
    old = stage.GetPrimAtPath(_OLD_CAMERA_PATH)
    if old.IsValid():
        stage.RemovePrim(_OLD_CAMERA_PATH)
        print(f"[attach_follow_camera] Removed legacy camera at {_OLD_CAMERA_PATH}")

    active_path = None

    for cam_def in CAMERAS:
        path = f"{BODY_PRIM_PATH}/{cam_def['name']}"

        # Always recreate so re-running picks up changed values
        existing = stage.GetPrimAtPath(path)
        if existing.IsValid():
            stage.RemovePrim(path)

        camera = UsdGeom.Camera.Define(stage, path)
        xform  = UsdGeom.Xformable(camera.GetPrim())
        xform.AddTranslateOp().Set(cam_def["offset"])
        xform.AddRotateXYZOp().Set(cam_def["rotation"])
        camera.CreateFocalLengthAttr(24.0)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 10000.0))
        print(f"[attach_follow_camera] Created {cam_def['name']} at {path}")

        if cam_def["name"] == DEFAULT_CAMERA:
            active_path = path

    # Switch viewport to default camera
    viewport = omni.kit.viewport.utility.get_active_viewport()
    if viewport and active_path:
        viewport.camera_path = active_path
        print(f"[attach_follow_camera] Viewport set to {DEFAULT_CAMERA}")
        print("  → Switch cameras via viewport top-left dropdown: CamBack / CamFront / CamLeft / CamRight")
    else:
        print("[attach_follow_camera] WARNING: Could not switch viewport automatically.")

