"""
get_warehouse_bounding_box.py — Print warehouse spatial data useful for tuning
forklift_controller.py waypoints and clearance values.

Outputs:
  - Warehouse world bounding box + dimensions
  - Floor Z level (for setting forklift Z)
  - Forklift half-extents (hardcoded from get_forklift_transform.py output)
  - Navigable rectangle after subtracting forklift clearance — paste directly into WAYPOINTS
  - Clearance check: does the forklift fit inside the warehouse?
  - Child prims with their individual bounding boxes (walls, racking, obstacles)

Run via VS Code: open this file and press Ctrl+Shift+P → Isaac Sim: Run File Remotely
Scene must already be open in Isaac Sim (scene_assembly.usd).
"""

import io
import os
import sys

import omni.usd
from pxr import Gf, UsdGeom

# ── Config ────────────────────────────────────────────────────────────────────

WAREHOUSE_PRIM_PATH = "/World/warehouse"

# Output file — written alongside console output
# Path is inside the container; maps to host via Docker bind mount:
#   container: /isaac-sim/.local/share/ov/data/  →  host: /home/ubuntu/docker/isaac-sim/data/
OUTPUT_FILE = "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot/simulations/forklift-warehouse/04_current_outputs/warehouse_spatial_info_latest.txt"

# Forklift half-extents from get_forklift_transform.py (bbox size / 2)
# Size: X=3.031m (width), Y=1.130m (length), Z=2.935m (height)
FORKLIFT_HALF_X = 3.031 / 2   # 1.516 m
FORKLIFT_HALF_Y = 1.130 / 2   # 0.565 m

# Extra clearance margin to add on top of forklift half-extents (metres)
CLEARANCE_MARGIN = 0.5

# ── Tee stdout → buffer (written to OUTPUT_FILE at end) ──────────────────────

_buffer = io.StringIO()
_orig_stdout = sys.stdout

class _Tee:
    def __init__(self, *streams):
        self.streams = streams
    def write(self, data):
        for s in self.streams:
            s.write(data)
    def flush(self):
        for s in self.streams:
            s.flush()

sys.stdout = _Tee(_orig_stdout, _buffer)

# ── Helpers ───────────────────────────────────────────────────────────────────

def _bbox(prim, cache):
    b = cache.ComputeWorldBound(prim)
    return b.GetRange()


def _fmt(v):
    return f"({v[0]:.4f}, {v[1]:.4f}, {v[2]:.4f})"


# ── Main ──────────────────────────────────────────────────────────────────────

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath(WAREHOUSE_PRIM_PATH)

if not prim.IsValid():
    print(f"ERROR: Prim not found at {WAREHOUSE_PRIM_PATH!r} — update WAREHOUSE_PRIM_PATH")
else:
    bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
    r = _bbox(prim, bbox_cache)
    size = r.GetMax() - r.GetMin()

    print("=" * 60)
    print(f"Prim path  : {WAREHOUSE_PRIM_PATH}")
    print(f"Prim type  : {prim.GetTypeName()}")
    print("-" * 60)

    # ── Warehouse bbox ─────────────────────────────────────────────────────────
    print("Warehouse bounding box:")
    print(f"  Min      : {_fmt(r.GetMin())}")
    print(f"  Max      : {_fmt(r.GetMax())}")
    print(f"  Size X   : {size[0]:.3f} m")
    print(f"  Size Y   : {size[1]:.3f} m")
    print(f"  Size Z   : {size[2]:.3f} m  (height)")
    print(f"  Floor Z  : {r.GetMin()[2]:.4f}  ← use as forklift Z in WAYPOINTS")
    print("-" * 60)

    # ── Navigable rectangle ────────────────────────────────────────────────────
    pad_x = FORKLIFT_HALF_X + CLEARANCE_MARGIN
    pad_y = FORKLIFT_HALF_Y + CLEARANCE_MARGIN

    nav_min_x = r.GetMin()[0] + pad_x
    nav_max_x = r.GetMax()[0] - pad_x
    nav_min_y = r.GetMin()[1] + pad_y
    nav_max_y = r.GetMax()[1] - pad_y
    floor_z   = r.GetMin()[2]

    print(f"Navigable rectangle (forklift half-extents + {CLEARANCE_MARGIN}m margin):")
    print(f"  X  : {nav_min_x:.3f}  →  {nav_max_x:.3f}")
    print(f"  Y  : {nav_min_y:.3f}  →  {nav_max_y:.3f}")
    print(f"  Z  : {floor_z:.4f}  (floor level)")
    print()
    print("  Suggested WAYPOINTS for a perimeter patrol:")
    corners = [
        (nav_min_x, nav_min_y),
        (nav_min_x, nav_max_y),
        (nav_max_x, nav_max_y),
        (nav_max_x, nav_min_y),
        (nav_min_x, nav_min_y),  # close the loop
    ]
    print("  WAYPOINTS: list[tuple[float, float]] = [")
    labels = ["lower-left", "upper-left", "upper-right", "lower-right", "return to start"]
    for (x, y), label in zip(corners, labels):
        print(f"      ({x:.3f}, {y:.3f}),   # {label}")
    print("  ]")
    print("-" * 60)

    # ── Clearance check ────────────────────────────────────────────────────────
    nav_width  = nav_max_x - nav_min_x
    nav_length = nav_max_y - nav_min_y
    fits = nav_width > 0 and nav_length > 0
    print(f"Clearance check:")
    print(f"  Navigable width  : {nav_width:.3f} m  {'OK' if nav_width > 0 else 'WARNING: forklift too wide!'}")
    print(f"  Navigable length : {nav_length:.3f} m  {'OK' if nav_length > 0 else 'WARNING: forklift too long!'}")
    print(f"  Forklift fits    : {'YES' if fits else 'NO — reduce CLEARANCE_MARGIN or check scale'}")
    print("-" * 60)

    # ── Child prims — obstacle/rack only (skip full list for speed) ───────────
    OBSTACLE_KEYWORDS = ("rack", "shelf", "shelv", "pallet", "column", "beam",
                         "bracket", "storage", "post", "pillar")
    children = list(prim.GetChildren())
    obstacles = []
    for child in children:
        name_lower = child.GetPath().name.lower()
        if any(kw in name_lower for kw in OBSTACLE_KEYWORDS):
            cr  = _bbox(child, bbox_cache)
            ctr = (cr.GetMin() + cr.GetMax()) / 2
            cs  = cr.GetMax() - cr.GetMin()
            obstacles.append((child.GetPath().pathString, ctr, cs, cr))

    if obstacles:
        print(f"\nObstacles / racks ({len(obstacles)} matching prims):")
        print(f"  {'Name':<45} {'CentreX':>8} {'CentreY':>8} {'SizeX':>7} {'SizeY':>7}")
        print(f"  {'-'*45} {'-'*8} {'-'*8} {'-'*7} {'-'*7}")
        for path, ctr, cs, cr in sorted(obstacles, key=lambda t: t[1][1]):
            print(f"  {path:<45} {ctr[0]:>8.2f} {ctr[1]:>8.2f} {cs[0]:>7.2f} {cs[1]:>7.2f}")
    else:
        print(f"\nNo rack/shelf/pallet prims detected by keyword (checked {len(children)} children).")

    # ── Forklift position ──────────────────────────────────────────────────────
    print("\n" + "-" * 60)
    fl_prim = stage.GetPrimAtPath("/World/forklift_b/body")
    if fl_prim.IsValid():
        from pxr import Usd
        import math
        m   = UsdGeom.Xformable(fl_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = m.ExtractTranslation()
        q   = m.ExtractRotationQuat()
        xi, yi, zi = q.GetImaginary()
        w   = q.GetReal()
        yaw = math.degrees(math.atan2(2.0*(w*zi + xi*yi), 1.0 - 2.0*(yi*yi + zi*zi)))
        print(f"Forklift position (/World/forklift_b/body):")
        print(f"  X={pos[0]:.3f}  Y={pos[1]:.3f}  Z={pos[2]:.3f}  heading={yaw:.2f} deg")
    else:
        print("Forklift prim /World/forklift_b/body not found (run with scene loaded).")

    print("=" * 60)

# ── Write captured output to file ─────────────────────────────────────────────
sys.stdout = _orig_stdout
try:
    out_path = os.path.normpath(OUTPUT_FILE)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        f.write(_buffer.getvalue())
    print(f"\nOutput written to: {out_path}")
except Exception as e:
    print(f"WARNING: Could not write output file: {e}")