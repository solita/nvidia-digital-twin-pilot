"""simplify_warehouse.py — Remove beams/brackets/pillars, tile floor 3×.

The warehouse geometry comes from a remote S3 payload that cannot be
edited directly.  This script:
  1. Flattens the composed warehouse subtree into a local warehouse_clean.usd
  2. Permanently deletes unwanted prim families (beams, brackets, pillars)
  3. Computes the warehouse extent along EXPAND_AXIS
  4. Physically duplicates structural prims (walls, floors, ceilings, lights)
     twice along that axis so the building envelope tiles 3×
  5. Non-structural prims (racks, shelves, pallets) are left untouched at
     their original size and position
  6. Rewires scene_assembly.usd to payload the local clean file
  7. Reloads the stage

After running the unwanted prims are gone from every layer.
Non-structural assets are completely untouched.  The building shell
(walls, ceilings, floors, lights) is physically tiled 3× along one axis.

Run inside Isaac Sim:
  Ctrl+Shift+P → Isaac Sim: Run File Remotely
"""
from __future__ import annotations

import asyncio
import os

import carb
import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, UsdGeom, Vt

WAREHOUSE_PRIM_PATH = "/World/warehouse"
FLOOR_MULTIPLIER = 3              # tile count along expansion axis
EXPAND_AXIS = 0                   # 0 = X, 1 = Y, 2 = Z
LOCAL_WAREHOUSE_FILENAME = "warehouse_clean.usd"
CORNER_FILL_SCALE_Y = 2.0         # Y-scale applied to nearby wall to fill corner gap

# Prim-name prefixes to permanently delete
REMOVE_PREFIXES = (
    "SM_BeamA_",
    "SM_BracketBeam_",
    "SM_BracketSlot",
    "SM_PillarA_",
    "SM_PillarPartA_",
)

# Structural prims to physically duplicate (building envelope + lights)
TILE_PREFIXES = (
    "SM_CeilingA_",
    "SM_floor",
    "SM_WallA_",
    "SM_WallWire_",
    "SM_FloorDecal_",
    "SM_LampCeilingA_",
    "RectLight",
)

# Global / environmental prims — keep once, never duplicate
GLOBAL_PREFIXES = (
    "GroundPlane",
    "DistantLight",
    "Camera",
)


def _should_remove(name: str) -> bool:
    return any(name.startswith(p) for p in REMOVE_PREFIXES)


def _should_tile(name: str) -> bool:
    return any(name.startswith(p) for p in TILE_PREFIXES)


def _is_corner_wall(name: str) -> bool:
    """True if wall prim is a corner piece (InnerCorner, OuterCorner, etc.)."""
    return "Corner" in name


# ── Helpers for reading / offsetting translates in Sdf layer ──────

def _get_translate(layer, prim_path):
    """Return the xformOp:translate as [x, y, z], defaulting to [0,0,0]."""
    attr = layer.GetAttributeAtPath(prim_path.AppendProperty("xformOp:translate"))
    if attr and attr.default is not None:
        t = attr.default
        return [float(t[0]), float(t[1]), float(t[2])]
    return [0.0, 0.0, 0.0]


def _offset_translate(layer, prim_path, axis, offset):
    """Add *offset* to the translate along *axis* for the prim at *prim_path*."""
    prop_path = prim_path.AppendProperty("xformOp:translate")
    attr = layer.GetAttributeAtPath(prop_path)

    if attr and attr.default is not None:
        old = attr.default
        new_t = [float(old[0]), float(old[1]), float(old[2])]
        new_t[axis] += offset
        if isinstance(old, Gf.Vec3f):
            attr.default = Gf.Vec3f(*new_t)
        else:
            attr.default = Gf.Vec3d(*new_t)
    else:
        # No translate exists yet — create one
        prim_spec = layer.GetPrimAtPath(prim_path)
        new_attr = Sdf.AttributeSpec(
            prim_spec, "xformOp:translate", Sdf.ValueTypeNames.Double3
        )
        t = [0.0, 0.0, 0.0]
        t[axis] = offset
        new_attr.default = Gf.Vec3d(*t)
        # Ensure xformOpOrder includes the new translate
        order_path = prim_path.AppendProperty("xformOpOrder")
        order_attr = layer.GetAttributeAtPath(order_path)
        if order_attr and order_attr.default is not None:
            ops = list(order_attr.default)
            if "xformOp:translate" not in ops:
                ops.insert(0, "xformOp:translate")
                order_attr.default = Vt.TokenArray(ops)
        else:
            order_spec = Sdf.AttributeSpec(
                prim_spec, "xformOpOrder", Sdf.ValueTypeNames.TokenArray
            )
            order_spec.default = Vt.TokenArray(["xformOp:translate"])


def _set_scale(layer, prim_path, axis, factor):
    """Set xformOp:scale on *prim_path*, multiplying *axis* by *factor*."""
    prop_path = prim_path.AppendProperty("xformOp:scale")
    attr = layer.GetAttributeAtPath(prop_path)

    if attr and attr.default is not None:
        old = attr.default
        new_s = [float(old[0]), float(old[1]), float(old[2])]
        new_s[axis] *= factor
        if isinstance(old, Gf.Vec3f):
            attr.default = Gf.Vec3f(*new_s)
        else:
            attr.default = Gf.Vec3d(*new_s)
    else:
        prim_spec = layer.GetPrimAtPath(prim_path)
        new_attr = Sdf.AttributeSpec(
            prim_spec, "xformOp:scale", Sdf.ValueTypeNames.Double3
        )
        s = [1.0, 1.0, 1.0]
        s[axis] = factor
        new_attr.default = Gf.Vec3d(*s)
        # Ensure xformOpOrder includes scale
        order_path = prim_path.AppendProperty("xformOpOrder")
        order_attr = layer.GetAttributeAtPath(order_path)
        if order_attr and order_attr.default is not None:
            ops = list(order_attr.default)
            if "xformOp:scale" not in ops:
                ops.append("xformOp:scale")
                order_attr.default = Vt.TokenArray(ops)
        else:
            order_spec = Sdf.AttributeSpec(
                prim_spec, "xformOpOrder", Sdf.ValueTypeNames.TokenArray
            )
            order_spec.default = Vt.TokenArray(["xformOp:scale"])


async def simplify_warehouse() -> None:
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    if stage is None:
        carb.log_error("[simplify_warehouse] No stage loaded.")
        return

    warehouse = stage.GetPrimAtPath(WAREHOUSE_PRIM_PATH)
    if not warehouse.IsValid():
        carb.log_error(f"[simplify_warehouse] Prim not found: {WAREHOUSE_PRIM_PATH}")
        return

    root_layer = stage.GetRootLayer()
    scenes_dir = os.path.dirname(root_layer.realPath)
    local_wh_path = os.path.join(scenes_dir, LOCAL_WAREHOUSE_FILENAME)
    stage_url = ctx.get_stage_url()

    # ── 1. Flatten the composed stage ─────────────────────────────
    carb.log_info("[simplify_warehouse] Flattening composed stage …")
    flat = stage.Flatten()
    await app.next_update_async()

    # ── 2. Copy warehouse subtree to a clean local layer ──────────
    if os.path.exists(local_wh_path):
        os.remove(local_wh_path)

    local_layer = Sdf.Layer.CreateNew(local_wh_path)
    Sdf.CopySpec(flat, Sdf.Path(WAREHOUSE_PRIM_PATH),
                 local_layer, Sdf.Path("/warehouse"))
    local_layer.defaultPrim = "warehouse"

    wh_local = local_layer.GetPrimAtPath("/warehouse")

    # Strip scene-assembly-level xform ops from the local copy so
    # they won't be doubled when scene_assembly applies its own.
    with Sdf.ChangeBlock():
        for prop in list(wh_local.properties.keys()):
            if "xformOp" in prop:
                del wh_local.properties[prop]

    # ── 3. Delete unwanted prims from the local copy ──────────────
    removed = 0
    with Sdf.ChangeBlock():
        for name in list(wh_local.nameChildren.keys()):
            if _should_remove(name):
                del wh_local.nameChildren[name]
                removed += 1

    # ── 3b. Compute warehouse extent along expansion axis ─────────
    tile_names = [n for n in wh_local.nameChildren.keys() if _should_tile(n)]

    if not tile_names:
        carb.log_error("[simplify_warehouse] No tileable structural prims found.")
        return

    positions = []
    for name in tile_names:
        child_path = wh_local.path.AppendChild(name)
        t = _get_translate(local_layer, child_path)
        positions.append(t[EXPAND_AXIS])

    axis_min = min(positions)
    axis_max = max(positions)
    width = axis_max - axis_min

    if width < 1e-6:
        carb.log_error(
            f"[simplify_warehouse] Warehouse extent along axis {EXPAND_AXIS} "
            f"is ~0 — cannot tile. Check EXPAND_AXIS setting."
        )
        return

    carb.log_info(
        f"[simplify_warehouse] Warehouse extent along axis {EXPAND_AXIS}: "
        f"{axis_min:.2f} → {axis_max:.2f}  (width {width:.2f})"
    )

    # ── 3c. Duplicate NON-WALL structural prims along expansion axis ─
    #
    #   Walls need special handling (3d) so exclude them here.
    #
    WALL_PREFIXES = ("SM_WallA_", "SM_WallWire_")

    def _is_wall(name: str) -> bool:
        return any(name.startswith(wp) for wp in WALL_PREFIXES)

    duped = 0
    with Sdf.ChangeBlock():
        for name in tile_names:
            if _is_wall(name):
                continue  # handled in 3d
            src_path = wh_local.path.AppendChild(name)
            for i in range(1, FLOOR_MULTIPLIER):
                dst_name = f"{name}_tile{i}"
                dst_path = wh_local.path.AppendChild(dst_name)
                Sdf.CopySpec(local_layer, src_path, local_layer, dst_path)
                _offset_translate(local_layer, dst_path, EXPAND_AXIS, width * i)
                duped += 1

    # ── 3d. Handle walls — continuous perimeter, no partitions ────
    #
    #   Classify every wall prim using TWO axes:
    #     • Side walls  – at the perpendicular-axis extremes (the long
    #       sides of the warehouse).  Tile normally so they cover the
    #       full expanded length.
    #     • End walls   – at the expansion-axis extremes but NOT at
    #       perpendicular extremes (the short cross-walls).
    #       Near end walls: keep originals only.
    #       Far end walls:  move originals to the final far position.
    #     • Interior / other walls: tile normally.
    #
    wall_names = [n for n in tile_names if _is_wall(n)]
    wall_duped = 0
    wall_moved = 0

    if wall_names:
        # Gather full 3-D positions
        wall_data = {}
        for name in wall_names:
            child_path = wh_local.path.AppendChild(name)
            wall_data[name] = _get_translate(local_layer, child_path)

        # Pick the perpendicular horizontal axis (largest wall spread
        # among the non-expansion axes).
        other_axes = [a for a in range(3) if a != EXPAND_AXIS]
        spreads = {}
        for ax in other_axes:
            vals = [t[ax] for t in wall_data.values()]
            spreads[ax] = max(vals) - min(vals) if vals else 0
        perp_axis = max(spreads, key=spreads.get)

        # Ranges
        expand_vals = [t[EXPAND_AXIS] for t in wall_data.values()]
        perp_vals   = [t[perp_axis]   for t in wall_data.values()]

        expand_min, expand_max = min(expand_vals), max(expand_vals)
        perp_min,   perp_max   = min(perp_vals),   max(perp_vals)

        expand_tol = max((expand_max - expand_min) * 0.01, 0.5)
        perp_tol   = max((perp_max   - perp_min)   * 0.01, 0.5)

        side_walls = []
        end_walls_near = []
        end_walls_far  = []
        other_walls = []

        for name, t in wall_data.items():
            at_perp_edge = (abs(t[perp_axis] - perp_min) <= perp_tol or
                            abs(t[perp_axis] - perp_max) <= perp_tol)
            at_expand_min = abs(t[EXPAND_AXIS] - expand_min) <= expand_tol
            at_expand_max = abs(t[EXPAND_AXIS] - expand_max) <= expand_tol

            if at_perp_edge:
                # On the long side of the warehouse → tile normally
                side_walls.append(name)
            elif at_expand_min:
                end_walls_near.append(name)
            elif at_expand_max:
                end_walls_far.append(name)
            else:
                other_walls.append(name)

        # ── 3d-ii. Pull corner pieces out of side walls ───────────
        #   Corner pieces (InnerCorner, OuterCorner, …) at expansion
        #   edges must NOT be tiled — they create false corners at
        #   internal seam points.  Keep near corners, move far corners.
        corner_walls_near = []
        corner_walls_far  = []
        non_corner_side   = []

        for name in side_walls:
            if _is_corner_wall(name):
                t = wall_data[name]
                if abs(t[EXPAND_AXIS] - expand_max) <= expand_tol:
                    corner_walls_far.append(name)
                else:
                    corner_walls_near.append(name)
            else:
                non_corner_side.append(name)

        side_walls = non_corner_side

        # Also pull corners out of other_walls
        non_corner_other = []
        for name in other_walls:
            if _is_corner_wall(name):
                t = wall_data[name]
                if abs(t[EXPAND_AXIS] - expand_max) <= expand_tol:
                    corner_walls_far.append(name)
                else:
                    corner_walls_near.append(name)
            else:
                non_corner_other.append(name)
        other_walls = non_corner_other

        # Also pull corners out of end walls
        for name in list(end_walls_near):
            if _is_corner_wall(name):
                end_walls_near.remove(name)
                corner_walls_near.append(name)
        for name in list(end_walls_far):
            if _is_corner_wall(name):
                end_walls_far.remove(name)
                corner_walls_far.append(name)

        carb.log_info(
            f"[simplify_warehouse] Wall classification — "
            f"side: {len(side_walls)}, end-near: {len(end_walls_near)}, "
            f"end-far: {len(end_walls_far)}, other: {len(other_walls)}, "
            f"corner-near: {len(corner_walls_near)}, "
            f"corner-far: {len(corner_walls_far)}  "
            f"(perp axis {perp_axis})"
        )

        with Sdf.ChangeBlock():
            # Side walls + other interior walls → tile normally
            for name in side_walls + other_walls:
                src_path = wh_local.path.AppendChild(name)
                for i in range(1, FLOOR_MULTIPLIER):
                    dst_name = f"{name}_tile{i}"
                    dst_path = wh_local.path.AppendChild(dst_name)
                    Sdf.CopySpec(local_layer, src_path, local_layer, dst_path)
                    _offset_translate(local_layer, dst_path, EXPAND_AXIS,
                                      width * i)
                    wall_duped += 1

            # Near end walls → keep originals, no copies (stay at start)
            # (nothing to do)

            # Far end walls → move originals to final far position
            far_offset = width * (FLOOR_MULTIPLIER - 1)
            for name in end_walls_far:
                child_path = wh_local.path.AppendChild(name)
                _offset_translate(local_layer, child_path, EXPAND_AXIS,
                                  far_offset)
                wall_moved += 1

            # ── Corner walls: keep near, move far, never tile ─────
            for name in corner_walls_far:
                child_path = wh_local.path.AppendChild(name)
                _offset_translate(local_layer, child_path, EXPAND_AXIS,
                                  far_offset)
                wall_moved += 1
            # corner_walls_near stay at their original positions

            # ── Scale one nearby side wall on Y to fill the gap ───
            #   For each corner, find the closest non-corner side wall
            #   at a similar expand position and apply CORNER_FILL_SCALE_Y.
            scaled_walls = set()
            corner_scaled = 0
            all_corners = corner_walls_near + corner_walls_far
            for cname in all_corners:
                ct = wall_data[cname]
                best_wall = None
                best_dist = float("inf")
                for sw in side_walls:
                    if sw in scaled_walls:
                        continue
                    sw_t = wall_data[sw]
                    # Must be at similar expand position
                    if abs(sw_t[EXPAND_AXIS] - ct[EXPAND_AXIS]) > expand_tol:
                        continue
                    dist = abs(sw_t[perp_axis] - ct[perp_axis])
                    if dist < best_dist:
                        best_dist = dist
                        best_wall = sw
                if best_wall and best_wall not in scaled_walls:
                    scaled_walls.add(best_wall)
                    # Scale the original
                    wall_path = wh_local.path.AppendChild(best_wall)
                    _set_scale(local_layer, wall_path, 1, CORNER_FILL_SCALE_Y)
                    # Scale all its tiled copies too
                    for i in range(1, FLOOR_MULTIPLIER):
                        tile_name = f"{best_wall}_tile{i}"
                        tile_path = wh_local.path.AppendChild(tile_name)
                        if local_layer.GetPrimAtPath(tile_path):
                            _set_scale(local_layer, tile_path, 1,
                                       CORNER_FILL_SCALE_Y)
                    corner_scaled += 1
                    carb.log_info(
                        f"[simplify_warehouse] Scaled {best_wall} "
                        f"(Y × {CORNER_FILL_SCALE_Y}) to fill gap "
                        f"from corner {cname}"
                    )

    remaining = len(wh_local.nameChildren)
    local_layer.Save()
    carb.log_info(
        f"[simplify_warehouse] {LOCAL_WAREHOUSE_FILENAME}: "
        f"removed {removed}, tiled {len(tile_names)} structural prims "
        f"(+{duped} non-wall copies, +{wall_duped} wall copies, "
        f"{wall_moved} end-walls moved, "
        f"{len(corner_walls_near) + len(corner_walls_far) if wall_names else 0} "
        f"corners kept (not tiled), "
        f"{corner_scaled if wall_names else 0} walls Y-scaled), "
        f"{remaining} total remain."
    )

    # ── 4. Rewire scene_assembly.usd ──────────────────────────────
    wh_sdf = root_layer.GetPrimAtPath(Sdf.Path(WAREHOUSE_PRIM_PATH))

    # 4a. Remove ALL over-specs for unwanted prims (active=false ghosts)
    cleaned = 0
    with Sdf.ChangeBlock():
        for name in list(wh_sdf.nameChildren.keys()):
            if _should_remove(name):
                del wh_sdf.nameChildren[name]
                cleaned += 1

    # 4b. Replace payload → local clean file
    pl = wh_sdf.payloadList
    pl.prependedItems = [Sdf.Payload(f"./{LOCAL_WAREHOUSE_FILENAME}")]
    wh_sdf.payloadList = pl

    root_layer.Save()
    carb.log_info(
        f"[simplify_warehouse] scene_assembly: "
        f"cleaned {cleaned} overs, "
        f"payload → ./{LOCAL_WAREHOUSE_FILENAME}"
    )

    # ── 5. Reload stage ──────────────────────────────────────────
    carb.log_info("[simplify_warehouse] Reloading stage …")
    await app.next_update_async()
    await ctx.open_stage_async(stage_url)
    await app.next_update_async()

    # ── 6. Verify ─────────────────────────────────────────────────
    new_stage = ctx.get_stage()
    wh = new_stage.GetPrimAtPath(WAREHOUSE_PRIM_PATH) if new_stage else None
    if wh and wh.IsValid():
        final_count = sum(1 for _ in wh.GetChildren())
        carb.log_info(
            f"[simplify_warehouse] Done — {final_count} prims in warehouse. "
            f"Building tiled ×{FLOOR_MULTIPLIER} along axis {EXPAND_AXIS}, "
            f"assets at original size, beams/brackets/pillars removed."
        )
    else:
        carb.log_warn(
            "[simplify_warehouse] Could not verify — "
            "warehouse prim not found after reload."
        )


asyncio.ensure_future(simplify_warehouse())
