"""
attach_follow_camera.py — Attach a follow camera to the forklift and switch the
active viewport to it so the view tracks the forklift during simulation.

Run ONCE via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely
The viewport will switch to the follow camera immediately.
To return to the free viewport camera: in the viewport top-left dropdown
change "FollowCam" back to "Perspective".

Tune OFFSET_* to adjust the camera position relative to the forklift.
"""
from __future__ import annotations

import carb
import omni.kit.viewport.utility
import omni.usd
from pxr import Gf, UsdGeom

FORKLIFT_PRIM_PATH  = "/World/forklift_b"
CAMERA_PRIM_PATH    = "/World/forklift_b/body/FollowCam"  # parented to the moving rigid body

# Camera offset from forklift centre (metres, in forklift local space)
OFFSET_X  =  -14.0   # behind the forklift
OFFSET_Y  =   0.0   # centred left/right
OFFSET_Z  =   8.0   # above the forklift

# Camera tilt — how many degrees to look downward toward the forklift
TILT_DOWN_DEG = 25.0

# Old camera path (before fix) — remove if still present
_OLD_CAMERA_PATH = "/World/forklift_b/FollowCam"

# ── Create or find the camera prim ────────────────────────────────────────────
stage = omni.usd.get_context().get_stage()

if not stage:
    print("[attach_follow_camera] ERROR: No stage loaded.")
else:
    # Remove old camera if it exists
    old_cam = stage.GetPrimAtPath(_OLD_CAMERA_PATH)
    if old_cam.IsValid():
        stage.RemovePrim(_OLD_CAMERA_PATH)
        print(f"[attach_follow_camera] Removed old camera at {_OLD_CAMERA_PATH}")

    # Always remove and recreate so re-running the script picks up new offset values
    cam_prim = stage.GetPrimAtPath(CAMERA_PRIM_PATH)
    if cam_prim.IsValid():
        stage.RemovePrim(CAMERA_PRIM_PATH)
        print(f"[attach_follow_camera] Removed existing camera at {CAMERA_PRIM_PATH}")

    camera = UsdGeom.Camera.Define(stage, CAMERA_PRIM_PATH)
    xform  = UsdGeom.Xformable(camera.GetPrim())

    # Position offset behind and above the forklift
    xform.AddTranslateOp().Set(Gf.Vec3d(OFFSET_X, OFFSET_Y, OFFSET_Z))

    # Tilt down to look at the forklift
    xform.AddRotateXYZOp().Set(Gf.Vec3f(90.0 - TILT_DOWN_DEG, 0.0, -90.0))

    camera.CreateFocalLengthAttr(24.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 10000.0))

    print(f"[attach_follow_camera] Created follow camera at {CAMERA_PRIM_PATH}")

    # ── Switch active viewport to this camera ─────────────────────────────────
    viewport = omni.kit.viewport.utility.get_active_viewport()
    if viewport:
        viewport.camera_path = CAMERA_PRIM_PATH
        print(f"[attach_follow_camera] Viewport switched to {CAMERA_PRIM_PATH}")
        print("  → To go back to free camera: viewport dropdown → Perspective")
    else:
        print("[attach_follow_camera] WARNING: Could not get active viewport.")
