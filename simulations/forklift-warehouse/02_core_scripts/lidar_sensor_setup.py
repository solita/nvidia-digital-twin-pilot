# lidar_sensor_setup.py - Create and attach a PhysX LIDAR sensor to the forklift.
#
# Based on the official NVIDIA guide for Adding Sensors in Isaac Sim:
#   Create > Isaac > Sensors > PhysX Lidar > Rotating
#
# Uses PhysX raycasting (not RTX ray tracing) with built-in Draw Lines
# visualisation:
#   - Gray beams = no object detected (empty space)
#   - Red beams  = object hit by a beam (walls, obstacles, etc.)
#
# Spatial data (from 04_helper_scripts):
#   Warehouse navigable bounds: X(-10.49 -> 9.48), Y(-17.69 -> 18.99)
#   Floor Z: 0.0
#   Forklift size: 3.031 m (X) x 1.130 m (Y) x 2.935 m (Z)
#
# Run via VS Code: open this file -> Ctrl+Shift+P -> Isaac Sim: Run File Remotely
# Scene must already be open in Isaac Sim (scene_assembly.usd).

import asyncio
import traceback

import carb
import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom

# -- Configuration -------------------------------------------------------------

FORKLIFT_PRIM_PATH = "/World/forklift_b"
FORKLIFT_BODY_PRIM_PATH = f"{FORKLIFT_PRIM_PATH}/body"
LIDAR_PRIM_NAME = "lidar_sensor"
LIDAR_PRIM_PATH = f"{FORKLIFT_BODY_PRIM_PATH}/{LIDAR_PRIM_NAME}"
LEGACY_LIDAR_PRIM_PATH = f"{FORKLIFT_PRIM_PATH}/{LIDAR_PRIM_NAME}"

# Position offset relative to forklift origin (top of mast)
LIDAR_POSITION = Gf.Vec3d(0.0, 0.0, 2.98)  # forklift height ~ 2.935 m

# PhysX Lidar configuration (Rotating type)
ROTATION_RATE = 10.0            # Hz - rotation speed
MIN_RANGE = 0.4                 # metres
MAX_RANGE = 40.0                # metres (warehouse is ~20 x 35 m)
HORIZONTAL_FOV = 360.0          # degrees - full rotation
VERTICAL_FOV = 30.0             # degrees
HORIZONTAL_RESOLUTION = 0.4     # degrees per beam
VERTICAL_RESOLUTION = 4.0       # degrees per beam
DRAW_LINES = True               # visualise beams (gray = miss, red = hit)
HIGH_LOD = False                 # high level of detail mode

# Logging
LOG_INTERVAL_FRAMES = 60        # print summary every ~1 s at 60 Hz


# -- Helpers -------------------------------------------------------------------

def _log(msg):
    print(f"[lidar_sensor_setup] {msg}", flush=True)


def _get_stage():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("USD stage is not available -- is a scene open?")
    return stage


async def _settle(n=1):
    app = omni.kit.app.get_app()
    for _ in range(max(1, n)):
        await app.next_update_async()


# -- Main ----------------------------------------------------------------------

async def run_lidar_sensor_setup():
    try:
        _log("=== SCRIPT STARTING ===")
        timeline = omni.timeline.get_timeline_interface()

        # -- 1. Stop timeline for safe scene-graph edits -----------------------
        if timeline.is_playing():
            timeline.stop()
            await _settle(10)
            _log("Stopped timeline for safe scene editing")

        stage = _get_stage()

        # Validate forklift prim
        forklift = stage.GetPrimAtPath(FORKLIFT_PRIM_PATH)
        if not forklift.IsValid():
            _log(f"ERROR: forklift prim not found at {FORKLIFT_PRIM_PATH}")
            return
        _log(f"Forklift prim OK: {FORKLIFT_PRIM_PATH}")

        # Validate forklift body prim (lidar attachment parent)
        forklift_body = stage.GetPrimAtPath(FORKLIFT_BODY_PRIM_PATH)
        if not forklift_body.IsValid():
            raise RuntimeError(
                f"forklift body prim not found at {FORKLIFT_BODY_PRIM_PATH}"
            )
        _log(f"Forklift body prim OK: {FORKLIFT_BODY_PRIM_PATH}")

        # -- 2. Remove old LIDAR prim if it exists -----------------------------
        old = stage.GetPrimAtPath(LEGACY_LIDAR_PRIM_PATH)
        if old.IsValid():
            omni.kit.commands.execute("DeletePrims", paths=[LEGACY_LIDAR_PRIM_PATH])
            await _settle(5)
            _log(f"Removed stale legacy LIDAR prim: {LEGACY_LIDAR_PRIM_PATH}")

        old = stage.GetPrimAtPath(LIDAR_PRIM_PATH)
        if old.IsValid():
            omni.kit.commands.execute("DeletePrims", paths=[LIDAR_PRIM_PATH])
            await _settle(5)
            _log(f"Removed stale canonical LIDAR prim: {LIDAR_PRIM_PATH}")

        # -- 3. Create PhysX Lidar (Rotating) as child of forklift body --------
        #    Equivalent to menu: Create > Isaac > Sensors > PhysX Lidar > Rotating
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path=LIDAR_PRIM_NAME,
            parent=FORKLIFT_BODY_PRIM_PATH,
            min_range=MIN_RANGE,
            max_range=MAX_RANGE,
            draw_points=False,
            draw_lines=DRAW_LINES,
            horizontal_fov=HORIZONTAL_FOV,
            vertical_fov=VERTICAL_FOV,
            horizontal_resolution=HORIZONTAL_RESOLUTION,
            vertical_resolution=VERTICAL_RESOLUTION,
            rotation_rate=ROTATION_RATE,
            high_lod=HIGH_LOD,
            yaw_offset=0.0,
            enable_semantics=False,
        )

        lidar_prim = stage.GetPrimAtPath(LIDAR_PRIM_PATH)
        if not lidar_prim.IsValid():
            _log("ERROR: LIDAR prim not found after creation")
            return
        _log(f"Created PhysX LIDAR: {LIDAR_PRIM_PATH} (type={lidar_prim.GetTypeName()})")

        # -- 4. Position the lidar on top of the forklift ----------------------
        #    Per the guide: move the sensor to the robot's top.
        xformable = UsdGeom.Xformable(lidar_prim)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(LIDAR_POSITION)
        _log(f"Positioned LIDAR at offset {LIDAR_POSITION}")
        await _settle(5)

        # -- 5. Start the simulation -------------------------------------------
        timeline.play()
        await _settle(20)
        _log("Timeline playing")

        # -- 6. Acquire the PhysX lidar sensor interface -----------------------
        from omni.isaac.range_sensor import _range_sensor
        lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
        _log("Acquired LidarSensorInterface")

        # Warm up so physics and sensor pipeline are ready
        await _settle(30)
        _log("Entering main loop")

        # -- 7. Main loop: read depth data, log summaries ----------------------
        frame = 0

        while True:
            await _settle()

            if not timeline.is_playing():
                _log("Timeline stopped -- exiting")
                break

            frame += 1

            # Log early frames for diagnostics, then every LOG_INTERVAL_FRAMES
            if frame > 5 and frame % LOG_INTERVAL_FRAMES != 0:
                continue

            # Read linear depth data from the PhysX lidar
            depth = lidar_interface.get_linear_depth_data(LIDAR_PRIM_PATH)

            if depth is not None:
                depth_arr = np.asarray(depth).ravel()
                valid = depth_arr[
                    np.isfinite(depth_arr) & (depth_arr > 0.0) & (depth_arr <= MAX_RANGE)
                ]

                if valid.size > 0:
                    _log(
                        f"frame={frame} returns={valid.size} "
                        f"min={float(np.min(valid)):.3f}m "
                        f"mean={float(np.mean(valid)):.3f}m "
                        f"max={float(np.max(valid)):.3f}m"
                    )
                else:
                    _log(f"frame={frame} returns=0 (no valid depth data)")
            else:
                _log(f"frame={frame} depth=None (sensor not ready)")

    except Exception:
        _log(f"FATAL ERROR:\n{traceback.format_exc()}")


asyncio.ensure_future(run_lidar_sensor_setup())
