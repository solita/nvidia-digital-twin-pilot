"""
setup_physics.py — One-time physics setup for the forklift warehouse scene.

Adds to the stage (skipping anything that already exists):
  1. PhysicsScene  at /World/PhysicsScene   — gravity, solver settings
  2. Ground plane  at /World/PhysicsGround  — invisible infinite floor collider
     positioned at floor level so the forklift lands on it when physics plays.

Run ONCE via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely
Then save the scene (Ctrl+S) so the physics prims persist.
scene_assembly.usd must already be open.

After running this script, forklift_controller.py will drive the forklift
using its wheel joints (back_wheel_drive + back_wheel_swivel) instead of
direct transform manipulation.
"""
from __future__ import annotations

import carb
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

# Floor Z level from get_warehouse_spatial_info.py
FLOOR_Z = -0.0002

# ── Helpers ───────────────────────────────────────────────────────────────────

def _prim_exists(stage, path: str) -> bool:
    return stage.GetPrimAtPath(path).IsValid()


def log(msg: str) -> None:
    carb.log_info(f"[setup_physics] {msg}")
    print(f"[setup_physics] {msg}")


# ── Main ──────────────────────────────────────────────────────────────────────

stage = omni.usd.get_context().get_stage()
if stage is None:
    print("[setup_physics] ERROR: No stage loaded — open scene_assembly.usd first.")
else:
    # ── 1. PhysicsScene ───────────────────────────────────────────────────────
    PHYSICS_SCENE_PATH = "/World/PhysicsScene"
    if _prim_exists(stage, PHYSICS_SCENE_PATH):
        log(f"PhysicsScene already exists at {PHYSICS_SCENE_PATH} — skipping.")
    else:
        scene = UsdPhysics.Scene.Define(stage, PHYSICS_SCENE_PATH)
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(9.81)
        log(f"Created PhysicsScene at {PHYSICS_SCENE_PATH}  (gravity = -Z, 9.81 m/s²)")

    # ── 2. Invisible ground plane collider ────────────────────────────────────
    #    A thin cube large enough to cover the whole warehouse (2000 × 2000 m),
    #    positioned so its top face sits exactly at FLOOR_Z.
    GROUND_PATH = "/World/PhysicsGround"
    if _prim_exists(stage, GROUND_PATH):
        log(f"Ground plane already exists at {GROUND_PATH} — skipping.")
    else:
        cube = UsdGeom.Cube.Define(stage, GROUND_PATH)
        cube.CreateSizeAttr(1.0)    # unit cube; scaled below

        xform = UsdGeom.Xformable(cube.GetPrim())
        # Centre the cube 0.5m below floor so its top face is at FLOOR_Z
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, FLOOR_Z - 0.5))
        xform.AddScaleOp().Set(Gf.Vec3d(2000.0, 2000.0, 1.0))

        # Collision
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        # Make invisible (collider only, no visual clutter)
        UsdGeom.Imageable(cube.GetPrim()).GetVisibilityAttr().Set(
            UsdGeom.Tokens.invisible
        )

        log(
            f"Created ground plane at {GROUND_PATH}  "
            f"(2000×2000 m, top face at Z={FLOOR_Z:.4f}, invisible)"
        )

    # ── Done ──────────────────────────────────────────────────────────────────
    print("\n[setup_physics] Setup complete.")
    print("  → Save the scene now (Ctrl+S) so physics prims persist.")
    print("  → Then run forklift_controller.py to start physics-driven patrol.")
