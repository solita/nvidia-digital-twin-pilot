"""
populate_scene.py — Add 4 forklifts, 4 shelf racks, and 2 docks to scene_assembly.usd.

Clones the existing /World/forklift_b prim four times (forklift_0 … forklift_3),
deactivates the original template, and adds simple box-based shelf racks and dock
markers so the scene matches the warehouse-manager (4 forklifts) and dashboard
(4 racks, 2 docks).

Run inside Isaac Sim with scene_assembly.usd already open:
  Ctrl+Shift+P → Isaac Sim: Run File Remotely

After running, save the scene (Ctrl+S).
"""
from __future__ import annotations

import carb
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, Vt

# ── Forklift layout ──────────────────────────────────────────────────────────

TEMPLATE_PRIM = "/World/forklift_b"

FORKLIFTS = [
    {"id": "forklift_0", "x": -15.0, "y": -25.0, "heading": 90.0},
    {"id": "forklift_1", "x":  -5.0, "y": -25.0, "heading": 90.0},
    {"id": "forklift_2", "x":   5.0, "y": -25.0, "heading": 90.0},
    {"id": "forklift_3", "x":  15.0, "y": -25.0, "heading": 90.0},
]

# ── Shelf racks ───────────────────────────────────────────────────────────────
# Four racks placed between the warehouse columns (X = -27, -4, +8, +26).
# Each rack is a visible cube with collision, 2m wide × 25m long × 4m tall.

RACKS = [
    {"label": "Rack_A", "x": -16.0, "y": 10.0, "z": 2.0, "sx": 2.0, "sy": 25.0, "sz": 4.0},
    {"label": "Rack_B", "x":   2.0, "y": 10.0, "z": 2.0, "sx": 2.0, "sy": 25.0, "sz": 4.0},
    {"label": "Rack_C", "x":  18.0, "y": 10.0, "z": 2.0, "sx": 2.0, "sy": 25.0, "sz": 4.0},
    {"label": "Rack_D", "x": -26.0, "y": 10.0, "z": 2.0, "sx": 2.0, "sy": 25.0, "sz": 4.0},
]

# ── Loading docks ─────────────────────────────────────────────────────────────
DOCKS = [
    {"label": "Dock_1", "x": -30.0, "y": -32.0, "z": 0.25, "sx": 6.0, "sy": 3.0, "sz": 0.5},
    {"label": "Dock_2", "x":  25.0, "y": -32.0, "z": 0.25, "sx": 6.0, "sy": 3.0, "sz": 0.5},
]

# ── Helpers ───────────────────────────────────────────────────────────────────

def log(msg: str) -> None:
    tagged = f"[populate_scene] {msg}"
    carb.log_info(tagged)
    print(tagged)


def _set_translate(xformable, x, y, z):
    ops = {op.GetOpName(): op for op in xformable.GetOrderedXformOps()}
    if "xformOp:translate" in ops:
        ops["xformOp:translate"].Set(Gf.Vec3d(x, y, z))
    else:
        xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z))


def _set_orient(xformable, heading_deg):
    rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), heading_deg)
    q = rot.GetQuat()
    gf_q = Gf.Quatf(
        float(q.GetReal()),
        float(q.GetImaginary()[0]),
        float(q.GetImaginary()[1]),
        float(q.GetImaginary()[2]),
    )
    ops = {op.GetOpName(): op for op in xformable.GetOrderedXformOps()}
    if "xformOp:orient" in ops:
        ops["xformOp:orient"].Set(gf_q)
    else:
        xformable.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(gf_q)


def _create_box(stage, path, x, y, z, sx, sy, sz, color):
    """Create a visible cube with collision at the given position and scale."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)

    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xform.AddScaleOp().Set(Gf.Vec3d(sx, sy, sz))

    # Collision so LIDAR and physics interact with it
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    # Display color
    cube.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))

    return cube


# ── Main ──────────────────────────────────────────────────────────────────────

def _populate():
    log("Starting scene population...")

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        log("ERROR: No stage loaded — open scene_assembly.usd first.")
        return

    root_layer = stage.GetRootLayer()
    src_path = Sdf.Path(TEMPLATE_PRIM)

    template = stage.GetPrimAtPath(TEMPLATE_PRIM)
    if not template.IsValid():
        log(f"ERROR: Template prim not found: {TEMPLATE_PRIM}")
        return

    # Re-activate the template if it was deactivated by a previous run,
    # so that Sdf.CopySpec can copy a complete prim hierarchy.
    was_inactive = not template.IsActive()
    if was_inactive:
        template.SetActive(True)
        log(f"Re-activated template prim: {TEMPLATE_PRIM}")

    # ── Clone forklifts ───────────────────────────────────────────────────
    for fl in FORKLIFTS:
        dst_path = Sdf.Path(f"/World/{fl['id']}")

        if stage.GetPrimAtPath(str(dst_path)).IsValid():
            log(f"Prim already exists: {dst_path} — updating transform")
        else:
            ok = Sdf.CopySpec(root_layer, src_path, root_layer, dst_path)
            if not ok:
                log(f"FAILED to copy {src_path} → {dst_path}")
                continue
            log(f"Cloned {src_path} → {dst_path}")

        # Set position and heading
        prim = stage.GetPrimAtPath(str(dst_path))
        xform = UsdGeom.Xformable(prim)
        _set_translate(xform, fl["x"], fl["y"], 0.0)
        _set_orient(xform, fl["heading"])
        # Ensure the clone is active
        if not prim.IsActive():
            prim.SetActive(True)
        log(f"  Placed {fl['id']} at ({fl['x']}, {fl['y']}) heading {fl['heading']}°")

    # ── Deactivate original template ──────────────────────────────────────
    template = stage.GetPrimAtPath(TEMPLATE_PRIM)
    if template.IsValid():
        template.SetActive(False)
        log(f"Deactivated template prim: {TEMPLATE_PRIM}")

    # ── Create shelf racks ────────────────────────────────────────────────
    for rack in RACKS:
        rack_path = f"/World/{rack['label']}"
        if stage.GetPrimAtPath(rack_path).IsValid():
            log(f"Rack already exists: {rack_path} — skipping")
            continue
        _create_box(
            stage, rack_path,
            rack["x"], rack["y"], rack["z"],
            rack["sx"], rack["sy"], rack["sz"],
            color=(0.29, 0.33, 0.41),  # dark grey-blue, matches dashboard #4a5568
        )
        log(f"Created {rack['label']} at ({rack['x']}, {rack['y']})")

    # ── Create loading docks ──────────────────────────────────────────────
    for dock in DOCKS:
        dock_path = f"/World/{dock['label']}"
        if stage.GetPrimAtPath(dock_path).IsValid():
            log(f"Dock already exists: {dock_path} — skipping")
            continue
        _create_box(
            stage, dock_path,
            dock["x"], dock["y"], dock["z"],
            dock["sx"], dock["sy"], dock["sz"],
            color=(0.17, 0.42, 0.69),  # blue, matches dashboard #2b6cb0
        )
        log(f"Created {dock['label']} at ({dock['x']}, {dock['y']})")

    # ── Summary ───────────────────────────────────────────────────────────
    log("Scene populated:")
    log(f"  {len(FORKLIFTS)} forklifts (forklift_0 … forklift_3)")
    log(f"  {len(RACKS)} shelf racks (Rack_A … Rack_D)")
    log(f"  {len(DOCKS)} loading docks (Dock_1, Dock_2)")
    log("Save the scene now (Ctrl+S) to persist changes.")


try:
    _populate()
except Exception as e:
    msg = f"[populate_scene] EXCEPTION: {e}"
    print(msg)
    import traceback
    traceback.print_exc()
