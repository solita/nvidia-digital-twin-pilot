"""
inspect_forklift_joints.py — Print the full prim tree, joints, and physics APIs
on the forklift_b asset.

This tells us:
  - Whether an ArticulationRoot already exists (and where)
  - All joint names, types, and their parent/child bodies
  - Which prims have CollisionAPI, RigidBodyAPI, PhysicsMassAPI
  - The joint drive configuration (if any)

Run via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely
Scene must already be open in Isaac Sim (scene_assembly.usd).
"""
from __future__ import annotations

import io
import os
import sys

import omni.usd
from pxr import UsdGeom, UsdPhysics, Usd

FORKLIFT_PRIM_PATH = "/World/forklift_b"
OUTPUT_FILE = "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot/simulations/forklift-warehouse/04_current_outputs/forklift_joints_latest.txt"

# ── Tee stdout to file ────────────────────────────────────────────────────────
_buffer = io.StringIO()
_orig_stdout = sys.stdout

class _Tee:
    def __init__(self, *streams): self.streams = streams
    def write(self, data): [s.write(data) for s in self.streams]
    def flush(self): [s.flush() for s in self.streams]

sys.stdout = _Tee(_orig_stdout, _buffer)

# ── Main ──────────────────────────────────────────────────────────────────────
stage = omni.usd.get_context().get_stage()
root  = stage.GetPrimAtPath(FORKLIFT_PRIM_PATH)

if not root.IsValid():
    print(f"ERROR: Prim not found: {FORKLIFT_PRIM_PATH!r}")
else:
    print("=" * 70)
    print(f"Forklift prim: {FORKLIFT_PRIM_PATH}")
    print("=" * 70)

    # ── 1. Walk entire subtree ────────────────────────────────────────────────
    print("\n── Full prim tree ──")
    for prim in Usd.PrimRange(root):
        indent = "  " * (len(prim.GetPath().pathString.split("/")) - len(FORKLIFT_PRIM_PATH.split("/")))
        apis = prim.GetAppliedSchemas()
        api_str = f"  [{', '.join(apis)}]" if apis else ""
        print(f"{indent}{prim.GetName()}  ({prim.GetTypeName()}){api_str}")

    # ── 2. ArticulationRoot ───────────────────────────────────────────────────
    print("\n── ArticulationRoot ──")
    found_art = False
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            print(f"  FOUND at: {prim.GetPath()}")
            found_art = True
    if not found_art:
        print("  NOT FOUND — will need to be added")

    # ── 3. All joints ─────────────────────────────────────────────────────────
    print("\n── Joints ──")
    joints_found = []
    for prim in Usd.PrimRange(root):
        type_name = prim.GetTypeName()
        if "Joint" in type_name:
            joints_found.append(prim)
            print(f"\n  Path      : {prim.GetPath()}")
            print(f"  Type      : {type_name}")
            # body0 / body1
            b0 = prim.GetAttribute("physics:body0").Get()
            b1 = prim.GetAttribute("physics:body1").Get()
            print(f"  body0     : {b0}")
            print(f"  body1     : {b1}")
            # axis
            ax = prim.GetAttribute("physics:axis").Get()
            print(f"  axis      : {ax}")
            # limits
            lo = prim.GetAttribute("physics:lowerLimit").Get()
            hi = prim.GetAttribute("physics:upperLimit").Get()
            print(f"  limits    : lower={lo}  upper={hi}")
            # drives
            for drive_type in ("angular", "linear"):
                drive_api_name = f"PhysicsDriveAPI:{drive_type}"
                if prim.HasAPI(UsdPhysics.DriveAPI, drive_type):
                    da = UsdPhysics.DriveAPI(prim, drive_type)
                    print(f"  DriveAPI ({drive_type}): "
                          f"stiffness={da.GetStiffnessAttr().Get()}  "
                          f"damping={da.GetDampingAttr().Get()}  "
                          f"maxForce={da.GetMaxForceAttr().Get()}  "
                          f"targetVel={da.GetTargetVelocityAttr().Get()}")
            schemas = prim.GetAppliedSchemas()
            if schemas:
                print(f"  Schemas   : {schemas}")

    if not joints_found:
        print("  NONE FOUND — asset may not have pre-configured joints")

    # ── 4. RigidBodyAPI prims ─────────────────────────────────────────────────
    print("\n── RigidBodyAPI prims ──")
    rb_found = False
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rb = UsdPhysics.RigidBodyAPI(prim)
            print(f"  {prim.GetPath()}  kinematic={prim.GetAttribute('physics:kinematicEnabled').Get()}")
            rb_found = True
    if not rb_found:
        print("  NONE FOUND")

    # ── 5. CollisionAPI prims ─────────────────────────────────────────────────
    print("\n── CollisionAPI prims ──")
    col_found = False
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            print(f"  {prim.GetPath()}")
            col_found = True
    if not col_found:
        print("  NONE FOUND")

    print("\n" + "=" * 70)

# ── Write to file ─────────────────────────────────────────────────────────────
sys.stdout = _orig_stdout
try:
    out_path = os.path.normpath(OUTPUT_FILE)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        f.write(_buffer.getvalue())
    print(f"Output written to: {out_path}")
except Exception as e:
    print(f"WARNING: Could not write output file: {e}")
