"""
get_forklift_transform.py — Print the current transform state of the forklift prim.

Useful for tuning WAYPOINTS, SPEED_PER_STEP, and WAYPOINT_TOLERANCE in
forklift_controller.py:
  - Run while the scene is open in Isaac Sim to read the current position,
    orientation, and scale of the forklift.
  - Manually move the forklift in the viewport to a desired waypoint position,
    then run this script to capture the exact (x, y, z) coordinates.

Run via VS Code: open this file and press Ctrl+Shift+P → Isaac Sim: Run File Remotely
Scene must already be open in Isaac Sim (scene_assembly.usd).
"""

import io
import math
import os
import sys

import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

OUTPUT_FILE = "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot/simulations/forklift-warehouse/04_current_outputs/forklift_transform_latest.txt"

# Tee stdout to buffer so we can write to file at the end
_buffer = io.StringIO()
_orig_stdout = sys.stdout

class _Tee:
    def __init__(self, *streams): self.streams = streams
    def write(self, data): [s.write(data) for s in self.streams]
    def flush(self): [s.flush() for s in self.streams]

sys.stdout = _Tee(_orig_stdout, _buffer)

# ── Config ────────────────────────────────────────────────────────────────────

FORKLIFT_PRIM_PATH = "/World/forklift_b"

# ── Helpers ───────────────────────────────────────────────────────────────────

def _get_xform_ops(prim_path: str) -> dict:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim not found: {prim_path!r}")
    xform = UsdGeom.Xformable(prim)
    return prim, xform, {op.GetOpName(): op for op in xform.GetOrderedXformOps()}


def _quat_to_euler_deg(q: Gf.Quatf) -> tuple[float, float, float]:
    """Convert a quaternion to Euler angles (roll, pitch, yaw) in degrees."""
    r = float(q.GetReal())
    i, j, k = q.GetImaginary()

    # roll (x-axis)
    sinr_cosp = 2 * (r * i + j * k)
    cosr_cosp = 1 - 2 * (i * i + j * j)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    # pitch (y-axis)
    sinp = 2 * (r * j - k * i)
    pitch = math.degrees(math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp))

    # yaw (z-axis)
    siny_cosp = 2 * (r * k + i * j)
    cosy_cosp = 1 - 2 * (j * j + k * k)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw


# ── Main ──────────────────────────────────────────────────────────────────────

prim, xform, ops = _get_xform_ops(FORKLIFT_PRIM_PATH)

print("=" * 60)
print(f"Prim path : {FORKLIFT_PRIM_PATH}")
print(f"Prim type : {prim.GetTypeName()}")
print("-" * 60)

# Translation
if "xformOp:translate" in ops:
    t = ops["xformOp:translate"].Get()
    print(f"Translation (x, y, z) : ({t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f})")
    print(f"  → paste as waypoint : ({t[0]}, {t[1]})")
else:
    print("Translation           : <no xformOp:translate found>")

# Orientation
if "xformOp:orient" in ops:
    q = ops["xformOp:orient"].Get()
    roll, pitch, yaw = _quat_to_euler_deg(q)
    print(f"Orientation (quat)    : real={q.GetReal():.4f}  imag={tuple(q.GetImaginary())}")
    print(f"Orientation (euler°)  : roll={roll:.2f}  pitch={pitch:.2f}  yaw={yaw:.2f}")
    print(f"  → heading (yaw)     : {yaw:.2f}°  (0° = +X axis)")
elif "xformOp:rotateXYZ" in ops:
    r = ops["xformOp:rotateXYZ"].Get()
    print(f"Rotation XYZ (deg)    : ({r[0]:.4f}, {r[1]:.4f}, {r[2]:.4f})")
else:
    print("Orientation           : <no orient or rotateXYZ op found>")

# Scale
if "xformOp:scale" in ops:
    s = ops["xformOp:scale"].Get()
    print(f"Scale (x, y, z)       : ({s[0]:.4f}, {s[1]:.4f}, {s[2]:.4f})")

# Bounding box
bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
bbox = bbox_cache.ComputeWorldBound(prim)
r = bbox.GetRange()
size = r.GetMax() - r.GetMin()
print("-" * 60)
print(f"World bbox min        : {tuple(round(v, 4) for v in r.GetMin())}")
print(f"World bbox max        : {tuple(round(v, 4) for v in r.GetMax())}")
print(f"World bbox size       : {tuple(round(v, 4) for v in size)}")
print(f"  → approx length     : {size[1]:.3f} m  (Y axis)")
print(f"  → approx width      : {size[0]:.3f} m  (X axis)")
print(f"  → approx height     : {size[2]:.3f} m  (Z axis)")

# Physics (if any)
print("-" * 60)
if prim.HasAPI(UsdPhysics.RigidBodyAPI):
    rb = UsdPhysics.RigidBodyAPI(prim)
    print(f"RigidBodyAPI          : present")
    vel_attr = prim.GetAttribute("physics:velocity")
    ang_attr = prim.GetAttribute("physics:angularVelocity")
    if vel_attr:
        print(f"Linear velocity       : {vel_attr.Get()}")
    if ang_attr:
        print(f"Angular velocity      : {ang_attr.Get()}")
else:
    print("RigidBodyAPI          : not present (transform-driven mode)")

# All xform ops summary
print("-" * 60)
print("XformOp stack:")
for op in xform.GetOrderedXformOps():
    print(f"  {op.GetOpName():<30} precision={op.GetPrecision()}")

print("=" * 60)

# ── Write output to file ──────────────────────────────────────────────────────
sys.stdout = _orig_stdout
try:
    out_path = os.path.normpath(OUTPUT_FILE)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        f.write(_buffer.getvalue())
    print(f"\nOutput written to: {out_path}")
except Exception as e:
    print(f"WARNING: Could not write output file: {e}")
