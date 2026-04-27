"""
ros2_cmd_bridge.py — Bridge ROS 2 cmd_vel topics → forklift USD joint drives.

Replaces the broken OmniGraph DifferentialController→ArticulationController
chain with a direct Python bridge that:
  1. Subscribes to /{forklift_id}/cmd_vel via OmniGraph ROS 2 twist_sub nodes
  2. Reads the Twist values each frame using the OmniGraph Python API
  3. Applies drive velocity and steer angle via UsdPhysics.DriveAPI

Also publishes odometry on /{forklift_id}/odom via OmniGraph.

Run via VS Code:  Ctrl+Shift+P → Isaac Sim: Run File Remotely
Stop by running:  ros2_cmd_bridge_stop.py  (or restart the simulation)
"""
from __future__ import annotations

import asyncio
import math

import carb
import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import UsdPhysics

# ── Configuration ─────────────────────────────────────────────────────────────

FORKLIFT_IDS = [f"forklift_{i}" for i in range(4)]

WHEEL_RADIUS = 0.15   # metres (matches warehouse_scene_streaming.py)

# linear.x  (m/s)   → drive joint target velocity (deg/s)
# Negative sign because DRIVE_VELOCITY < 0 means forks-forward in this USD asset.
DRIVE_SCALE = -(180.0 / (math.pi * WHEEL_RADIUS))  # ≈ −382 deg/s per m/s

# angular.z (rad/s)  → steer joint target position (degrees)
STEER_SCALE = 20.0    # degrees per rad/s
STEER_MAX   = 30.0    # clamp

# Joint physics (must match values from forklift_controller.py)
STEER_STIFFNESS = 40_000.0
STEER_DAMPING   = 10_000.0
DRIVE_DAMPING   = 15_000.0

# Global stop flag — set to True to exit the bridge loop
_stop = False


def _log(msg: str) -> None:
    tagged = f"[ros2_cmd_bridge] {msg}"
    carb.log_info(tagged)
    print(tagged, flush=True)


# ── OmniGraph creation ───────────────────────────────────────────────────────

def _create_bridge_graph(stage, fid: str):
    """Create a minimal OmniGraph: twist subscriber + odometry publisher."""
    graph_path = f"/World/ROS2CmdBridge_{fid}"

    # Remove any leftover graph from a previous run
    existing = stage.GetPrimAtPath(graph_path)
    if existing.IsValid():
        stage.RemovePrim(graph_path)

    keys = og.Controller.Keys
    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                ("twist_sub",       "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("compute_odom",    "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("odom_pub",        "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("read_sim_time",   "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("clock_pub",       "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                # Tick → subscribe + odom + clock
                ("on_playback_tick.outputs:tick", "twist_sub.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "compute_odom.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "clock_pub.inputs:execIn"),
                # Sim-time → publishers
                ("read_sim_time.outputs:simulationTime", "odom_pub.inputs:timeStamp"),
                ("read_sim_time.outputs:simulationTime", "clock_pub.inputs:timeStamp"),
                # Odom pipeline
                ("compute_odom.outputs:execOut",          "odom_pub.inputs:execIn"),
                ("compute_odom.outputs:position",         "odom_pub.inputs:position"),
                ("compute_odom.outputs:orientation",       "odom_pub.inputs:orientation"),
                ("compute_odom.outputs:linearVelocity",    "odom_pub.inputs:linearVelocity"),
                ("compute_odom.outputs:angularVelocity",   "odom_pub.inputs:angularVelocity"),
            ],
            keys.SET_VALUES: [
                ("twist_sub.inputs:topicName",   f"/{fid}/cmd_vel"),
                ("compute_odom.inputs:chassisPrim", f"/World/{fid}/body"),
                ("odom_pub.inputs:topicName",    f"/{fid}/odom"),
                ("odom_pub.inputs:chassisFrameId", f"{fid}/base_link"),
                ("odom_pub.inputs:odomFrameId",  "odom"),
                ("clock_pub.inputs:topicName",   "/clock"),
            ],
        },
    )
    # Return the twist_sub node reference (index 1 in CREATE_NODES order)
    return nodes[1]


def _remove_old_bridge_graphs(stage):
    """Remove the broken ROS2BridgeGraph_* graphs from warehouse_scene_streaming."""
    for fid in FORKLIFT_IDS:
        old_path = f"/World/ROS2BridgeGraph_{fid}"
        prim = stage.GetPrimAtPath(old_path)
        if prim.IsValid():
            stage.RemovePrim(old_path)
            _log(f"Removed old graph: {old_path}")


# ── Joint physics setup ──────────────────────────────────────────────────────

def _setup_joints(stage):
    """Configure drive and steer joint DriveAPI parameters for all forklifts."""
    for fid in FORKLIFT_IDS:
        drive_path = f"/World/{fid}/back_wheel_joints/back_wheel_drive"
        steer_path = f"/World/{fid}/back_wheel_joints/back_wheel_swivel"

        dp = stage.GetPrimAtPath(drive_path)
        sp = stage.GetPrimAtPath(steer_path)

        if dp.IsValid():
            api = UsdPhysics.DriveAPI(dp, "angular")
            api.GetDampingAttr().Set(DRIVE_DAMPING)
            api.GetTargetVelocityAttr().Set(0.0)

        if sp.IsValid():
            api = UsdPhysics.DriveAPI(sp, "angular")
            api.GetStiffnessAttr().Set(STEER_STIFFNESS)
            api.GetDampingAttr().Set(STEER_DAMPING)
            api.GetTargetPositionAttr().Set(0.0)

        _log(f"Joints configured for {fid}")


# ── Bridge loop ──────────────────────────────────────────────────────────────

async def _bridge_loop(twist_nodes: dict, stage):
    """Per-frame: read cmd_vel from OG twist_sub → set joint drive targets."""
    global _stop
    app = omni.kit.app.get_app()

    # Cache joint prim references
    drive_prims = {}
    steer_prims = {}
    for fid in FORKLIFT_IDS:
        dp = stage.GetPrimAtPath(f"/World/{fid}/back_wheel_joints/back_wheel_drive")
        sp = stage.GetPrimAtPath(f"/World/{fid}/back_wheel_joints/back_wheel_swivel")
        if dp.IsValid():
            drive_prims[fid] = dp
        if sp.IsValid():
            steer_prims[fid] = sp

    _log(f"Bridge loop running — {len(twist_nodes)} subscribers, "
         f"{len(drive_prims)} drive joints, {len(steer_prims)} steer joints")

    frame = 0
    while not _stop:
        for fid, node in twist_nodes.items():
            try:
                lin = og.Controller.attribute("outputs:linearVelocity", node).get()
                ang = og.Controller.attribute("outputs:angularVelocity", node).get()
                if lin is None or ang is None:
                    continue

                linear_x  = float(lin[0])   # m/s  (forward positive)
                angular_z = float(ang[2])   # rad/s (CCW positive)

                # ── Drive ────────────────────────────────────────────
                drive_vel = linear_x * DRIVE_SCALE
                if fid in drive_prims:
                    api = UsdPhysics.DriveAPI(drive_prims[fid], "angular")
                    api.GetTargetVelocityAttr().Set(drive_vel)

                # ── Steer ────────────────────────────────────────────
                steer_deg = angular_z * STEER_SCALE
                steer_deg = max(-STEER_MAX, min(STEER_MAX, steer_deg))
                if fid in steer_prims:
                    api = UsdPhysics.DriveAPI(steer_prims[fid], "angular")
                    api.GetTargetPositionAttr().Set(steer_deg)

                # Periodic diagnostics
                if frame % 600 == 0 and (abs(linear_x) > 0.01 or abs(angular_z) > 0.01):
                    _log(f"{fid}: lin={linear_x:.2f} m/s  ang={angular_z:.2f} rad/s "
                         f"→ drive={drive_vel:.0f}°/s  steer={steer_deg:.1f}°")

            except Exception:
                pass  # suppress per-frame noise

        frame += 1
        await app.next_update_async()

    _log("Bridge loop stopped")


# ── Entry point ──────────────────────────────────────────────────────────────

def _main():
    global _stop
    _stop = False

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        _log("ERROR: No stage open")
        return

    # Ensure timeline is playing
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
        _log("Started timeline")

    # Remove broken old graphs (from warehouse_scene_streaming.py)
    _remove_old_bridge_graphs(stage)

    # Create new minimal bridge graphs and collect twist_sub node references
    twist_nodes = {}
    for fid in FORKLIFT_IDS:
        fl_prim = stage.GetPrimAtPath(f"/World/{fid}")
        if not fl_prim.IsValid():
            _log(f"WARNING: {fid} prim not found — skipping")
            continue
        node = _create_bridge_graph(stage, fid)
        twist_nodes[fid] = node
        _log(f"Bridge graph created for {fid}")

    if not twist_nodes:
        _log("ERROR: No forklifts found — run populate_scene.py first")
        return

    # Configure joint physics
    _setup_joints(stage)

    # Launch the async bridge loop
    asyncio.ensure_future(_bridge_loop(twist_nodes, stage))

    _log(f"ROS 2 → joint bridge ACTIVE for {len(twist_nodes)} forklifts")
    _log("Publish geometry_msgs/Twist to /{forklift_id}/cmd_vel")
    _log("Use BEST_EFFORT or RELIABLE QoS")


try:
    _main()
except Exception as e:
    _log(f"EXCEPTION: {e}")
    import traceback
    traceback.print_exc()
