"""
scan_scene_assets.py — Full asset catalog of scene_assembly.usd.

Walks every prim in the live stage and writes two output files:

  04_current_outputs/scene_catalog.json   ← machine-readable (for scripting)
  04_current_outputs/scene_catalog.txt    ← human-readable (for planning)

Captured data per prim:
  - USD path, type, depth
  - Payload / reference asset paths (S3 / Nucleus URLs)
  - Physics schemas present
  - Bounding box computed ONLY for /World direct children and obstacle cubes
    (skips deep warehouse mesh hierarchy — keeps runtime under 5 seconds)

Run via VS Code: Ctrl+Shift+P → Isaac Sim: Run File Remotely
Scene must already be open (scene_assembly.usd).

Dev owner: anyone — read-only, does not modify the stage.
"""
from __future__ import annotations

import io
import json
import os
from dataclasses import asdict, dataclass, field
from typing import Optional

import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

# ── Output paths (container-side) ─────────────────────────────────────────────
_OUT = (
    "/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot/"
    "simulations/forklift-warehouse/04_current_outputs"
)
JSON_OUT = f"{_OUT}/scene_catalog.json"
TEXT_OUT = f"{_OUT}/scene_catalog.txt"


# ── Data model ────────────────────────────────────────────────────────────────

@dataclass
class BBox:
    min_x: float; min_y: float; min_z: float
    max_x: float; max_y: float; max_z: float
    ctr_x: float; ctr_y: float; ctr_z: float
    size_x: float; size_y: float; size_z: float


@dataclass
class PrimRecord:
    path:          str
    name:          str
    type_name:     str
    specifier:     str
    depth:         int
    children:      int
    payloads:      list[str]        = field(default_factory=list)
    references:    list[str]        = field(default_factory=list)
    physics:       list[str]        = field(default_factory=list)
    has_collision: bool             = False
    bbox:          Optional[BBox]   = None


# ── Helpers ───────────────────────────────────────────────────────────────────

def _bbox_record(prim: Usd.Prim, cache: UsdGeom.BBoxCache) -> Optional[BBox]:
    try:
        r = cache.ComputeWorldBound(prim).GetRange()
        if r.IsEmpty():
            return None
        mn, mx = r.GetMin(), r.GetMax()
        ctr = (mn + mx) / 2
        sz  = mx - mn
        return BBox(
            min_x=round(mn[0], 3), min_y=round(mn[1], 3), min_z=round(mn[2], 3),
            max_x=round(mx[0], 3), max_y=round(mx[1], 3), max_z=round(mx[2], 3),
            ctr_x=round(ctr[0], 3), ctr_y=round(ctr[1], 3), ctr_z=round(ctr[2], 3),
            size_x=round(sz[0], 3), size_y=round(sz[1], 3), size_z=round(sz[2], 3),
        )
    except Exception:
        return None


def _payloads(prim: Usd.Prim) -> list[str]:
    try:
        return [pl.assetPath for pl in prim.GetPayloads().GetAddedOrExplicitItems()
                if pl.assetPath]
    except Exception:
        return []


def _references(prim: Usd.Prim) -> list[str]:
    try:
        refs = prim.GetMetadata(Sdf.PrimSpec.ReferencesKey)
        if refs is None:
            return []
        return [r.assetPath for r in refs.GetAddedOrExplicitItems() if r.assetPath]
    except Exception:
        return []


def _physics_schemas(prim: Usd.Prim) -> list[str]:
    schemas = []
    if UsdPhysics.RigidBodyAPI(prim):   schemas.append("RigidBody")
    if UsdPhysics.CollisionAPI(prim):   schemas.append("Collision")
    if UsdPhysics.MassAPI(prim):        schemas.append("Mass")
    if UsdPhysics.DriveAPI(prim, "angular"): schemas.append("Drive:angular")
    if UsdPhysics.DriveAPI(prim, "linear"):  schemas.append("Drive:linear")
    return schemas


def _specifier(prim: Usd.Prim) -> str:
    return {Sdf.SpecifierDef: "def", Sdf.SpecifierOver: "over",
            Sdf.SpecifierClass: "class"}.get(prim.GetSpecifier(), "?")


# ── Main scan ─────────────────────────────────────────────────────────────────

def scan() -> list[PrimRecord]:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage loaded — open scene_assembly.usd first.")

    # BBox cache — only used for a small number of prims (see below)
    cache   = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    records: list[PrimRecord] = []
    total   = 0

    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith("/__") or path == "/":
            continue
        total += 1

        depth = path.count("/") - 1
        name  = prim.GetName()
        pays  = _payloads(prim)
        refs  = _references(prim)
        phys  = _physics_schemas(prim)

        # Compute bbox ONLY for:
        #   • direct children of /World (depth 1) — warehouse, forklift, cubes
        #   • anything with a payload/reference (asset roots, usually depth 1-2)
        #   • anything with physics (collision bodies)
        #   • anything named "*cube*" (obstacle cubes)
        # Everything deeper inside the warehouse mesh hierarchy is skipped.
        needs_bbox = (
            depth == 1
            or pays or refs or phys
            or "cube" in name.lower()
        )
        bbox = _bbox_record(prim, cache) if needs_bbox else None

        records.append(PrimRecord(
            path          = path,
            name          = name,
            type_name     = prim.GetTypeName() or "(none)",
            specifier     = _specifier(prim),
            depth         = depth,
            children      = len(list(prim.GetChildren())),
            payloads      = pays,
            references    = refs,
            physics       = phys,
            has_collision = "Collision" in phys,
            bbox          = bbox,
        ))

    print(f"Scanned {total} prims, kept {len(records)}, computed bbox for "
          f"{sum(1 for r in records if r.bbox)} prims.")
    return records


# ── Text report ───────────────────────────────────────────────────────────────

def _text_report(records: list[PrimRecord]) -> str:
    buf = io.StringIO()
    W   = buf.write

    W("=" * 72 + "\n")
    W("scene_assembly.usd  —  Asset Catalog\n")
    W("=" * 72 + "\n\n")

    # External assets
    assets = [(r.path, u) for r in records for u in r.payloads + r.references]
    W(f"EXTERNAL ASSETS  ({len(assets)})\n")
    W("-" * 72 + "\n")
    for path, url in sorted(assets):
        W(f"  {path}\n    → {url}\n")
    W("\n")

    # Prim type summary (depth ≤ 2 only for readability)
    shallow = [r for r in records if r.depth <= 2]
    by_type: dict[str, list] = {}
    for r in shallow:
        by_type.setdefault(r.type_name, []).append(r)
    W(f"PRIM TYPES (depth ≤ 2,  {len(shallow)} prims,  {len(by_type)} types)\n")
    W("-" * 72 + "\n")
    for t, rs in sorted(by_type.items(), key=lambda x: -len(x[1])):
        W(f"  {t:<30}  {len(rs):>4}\n")
    W("\n")

    # /World direct children with bbox
    world_children = [r for r in records if r.depth == 1]
    W(f"/World CHILDREN  ({len(world_children)})\n")
    W("-" * 72 + "\n")
    W(f"  {'Name':<30} {'Type':<18} {'CtrX':>8} {'CtrY':>8} {'SzX':>7} {'SzY':>7} {'SzZ':>7}\n")
    W(f"  {'-'*30} {'-'*18} {'-'*8} {'-'*8} {'-'*7} {'-'*7} {'-'*7}\n")
    for r in sorted(world_children, key=lambda x: x.name):
        b = r.bbox
        if b:
            W(f"  {r.name:<30} {r.type_name:<18} {b.ctr_x:>8.2f} {b.ctr_y:>8.2f}"
              f" {b.size_x:>7.2f} {b.size_y:>7.2f} {b.size_z:>7.2f}\n")
        else:
            W(f"  {r.name:<30} {r.type_name:<18}  (no bbox)\n")
        if r.payloads:
            for p in r.payloads:
                W(f"    payload → {p}\n")
    W("\n")

    # Obstacle cubes
    cubes = [r for r in records if "cube" in r.name.lower()]
    W(f"OBSTACLE CUBES  ({len(cubes)})\n")
    W("-" * 72 + "\n")
    W(f"  {'Path':<35} {'CtrX':>8} {'CtrY':>8} {'CtrZ':>8} {'SzX':>7} {'SzY':>7}\n")
    W(f"  {'-'*35} {'-'*8} {'-'*8} {'-'*8} {'-'*7} {'-'*7}\n")
    for r in sorted(cubes, key=lambda x: x.path):
        b = r.bbox
        if b:
            W(f"  {r.path:<35} {b.ctr_x:>8.2f} {b.ctr_y:>8.2f} {b.ctr_z:>8.2f}"
              f" {b.size_x:>7.2f} {b.size_y:>7.2f}\n")
        else:
            W(f"  {r.path:<35}  (no bbox)\n")
    W("\n")

    # Physics prims
    phys_prims = [r for r in records if r.physics]
    W(f"PHYSICS-ENABLED PRIMS  ({len(phys_prims)})\n")
    W("-" * 72 + "\n")
    for r in sorted(phys_prims, key=lambda x: x.path):
        W(f"  {r.path:<55} {', '.join(r.physics)}\n")
    W("\n")

    W("=" * 72 + "\n")
    return buf.getvalue()


# ── Entry point ───────────────────────────────────────────────────────────────

records = scan()
os.makedirs(_OUT, exist_ok=True)

# JSON
with open(JSON_OUT, "w") as f:
    json.dump(
        {"prim_count": len(records),
         "prims": [{**asdict(r), "bbox": asdict(r.bbox) if r.bbox else None}
                   for r in records]},
        f, indent=2,
    )

# Text
text = _text_report(records)
with open(TEXT_OUT, "w") as f:
    f.write(text)

print(text)
print(f"JSON → {JSON_OUT}")
print(f"Text → {TEXT_OUT}")

