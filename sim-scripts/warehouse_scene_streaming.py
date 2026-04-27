"""
warehouse_scene_streaming.py — Kit startup script for streaming container.

This script is executed inside the running Isaac Sim Kit app via --exec.
It loads the warehouse scene, configures physics and the ROS 2 bridge,
and starts the timeline. Unlike warehouse_scene.py, it does NOT create
its own SimulationApp (the streaming container already has one).

Usage (in docker compose command):
  --exec "/sim-scripts/warehouse_scene_streaming.py"
"""
from __future__ import annotations

import asyncio
import math
import os
import sys

import carb
import numpy as np
import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.usd
import usdrt
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

# ---------------------------------------------------------------------------
# Fix FastDDS shared-memory transport (must happen before ROS 2 bridge loads)
# Isaac Sim container runs with ipc=private, so SHM segments are invisible
# to other containers. Force UDP-only transport for ROS 2 DDS data.
# ---------------------------------------------------------------------------
_fastdds_xml = "/tmp/fastdds_no_shm.xml"
if not os.path.exists(_fastdds_xml):
    with open(_fastdds_xml, "w") as _f:
        _f.write('<?xml version="1.0" encoding="UTF-8" ?>\n'
                 '<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">\n'
                 '  <transport_descriptors>\n'
                 '    <transport_descriptor>\n'
                 '      <transport_id>udp_only</transport_id>\n'
                 '      <type>UDPv4</type>\n'
                 '      <sendBufferSize>1048576</sendBufferSize>\n'
                 '      <receiveBufferSize>1048576</receiveBufferSize>\n'
                 '    </transport_descriptor>\n'
                 '  </transport_descriptors>\n'
                 '  <participant profile_name="participant_profile" is_default_profile="true">\n'
                 '    <rtps>\n'
                 '      <useBuiltinTransports>false</useBuiltinTransports>\n'
                 '      <userTransports>\n'
                 '        <transport_id>udp_only</transport_id>\n'
                 '      </userTransports>\n'
                 '    </rtps>\n'
                 '  </participant>\n'
                 '</profiles>\n')
if not os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = _fastdds_xml
    print(f"[warehouse_scene_streaming] FastDDS SHM disabled via {_fastdds_xml}", flush=True)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SCENE_USD = os.environ.get(
    "SCENE_USD",
    "/sim-scripts/scenes/scene_assembly.usd",
)

FORKLIFT_IDS = [f"forklift_{i}" for i in range(4)]

# Default paths (forklift_0) used for single-forklift verification
FORKLIFT_PRIM_PATH = "/World/forklift_0"
FORKLIFT_BODY_PATH = "/World/forklift_0/body"
DRIVE_JOINT_PATH = "/World/forklift_0/back_wheel_joints/back_wheel_drive"
STEER_JOINT_PATH = "/World/forklift_0/back_wheel_joints/back_wheel_swivel"

WHEEL_RADIUS = 0.15
WHEEL_DISTANCE = 1.0

# Joint drive scales — ArticulationController works in SI units (rad/s, rad)
# linear.x (m/s) → drive joint angular velocity (rad/s)
# Negative because DRIVE_VELOCITY < 0 means forks-forward in this USD asset.
DRIVE_SCALE = -1.0 / WHEEL_RADIUS  # ≈ −6.667 rad/s per m/s
# angular.z (rad/s) → steer joint position (radians)
STEER_SCALE = math.radians(20.0)    # rad of steer per rad/s of angular cmd
STEER_MAX   = math.radians(30.0)    # max steer angle (rad)
# Joint physics parameters (DriveAPI stiffness/damping — set before sim starts)
STEER_STIFFNESS = 40_000.0
STEER_DAMPING   = 10_000.0
DRIVE_DAMPING   = 15_000.0

ROS_DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "42"))
PHYSICS_DT = 1.0 / 60.0


def log(msg: str) -> None:
    carb.log_info(f"[warehouse_scene_streaming] {msg}")
    print(f"[warehouse_scene_streaming] {msg}", flush=True)


# ---------------------------------------------------------------------------
# Scene setup (same logic as warehouse_scene.py)
# ---------------------------------------------------------------------------

def setup_physics(stage: Usd.Stage) -> None:
    physics_scene_path = "/physicsScene"
    scene_prim = stage.GetPrimAtPath(physics_scene_path)

    if not scene_prim.IsValid():
        UsdPhysics.Scene.Define(stage, physics_scene_path)
        scene_prim = stage.GetPrimAtPath(physics_scene_path)

    physics_scene = UsdPhysics.Scene(scene_prim)
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)

    physx_api = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    physx_api.CreateEnableCCDAttr(True)
    physx_api.CreateEnableStabilizationAttr(True)
    physx_api.CreateTimeStepsPerSecondAttr(int(1.0 / PHYSICS_DT))
    physx_api.CreateEnableGPUDynamicsAttr(True)
    physx_api.CreateBroadphaseTypeAttr("GPU")
    physx_api.CreateSolverTypeAttr("TGS")

    log("PhysX configured: TGS solver, GPU dynamics, 60Hz, Z-up gravity")


def create_ros2_bridge_graph(stage: Usd.Stage) -> dict:
    """Create one OmniGraph per forklift for ROS 2 bridge.

    Returns dict of {fid: (twist_sub_node, drive_ctrl_node, steer_ctrl_node)}.
    """
    forklift_nodes = {}
    for fid in FORKLIFT_IDS:
        nodes_tuple = _create_ros2_bridge_graph_for(stage, fid)
        forklift_nodes[fid] = nodes_tuple
    log(f"ROS 2 bridge OmniGraphs created for {len(FORKLIFT_IDS)} forklifts")
    return forklift_nodes


def _create_ros2_bridge_graph_for(stage: Usd.Stage, fid: str):
    """Create OmniGraph for ROS 2 bridge with ArticulationController.

    Uses IsaacArticulationController nodes (the proven approach from
    nvidia-simu-test) instead of DriveAPI attribute setting, which does
    not propagate to PhysX at runtime with GPU dynamics.

    Returns (twist_sub_node, drive_ctrl_node, steer_ctrl_node).
    """
    fl_prim  = f"/World/{fid}"
    fl_body  = f"/World/{fid}/body"
    graph_path = f"/World/ROS2BridgeGraph_{fid}"

    existing = stage.GetPrimAtPath(graph_path)
    if existing.IsValid():
        stage.RemovePrim(graph_path)

    keys = og.Controller.Keys
    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                ("clock_pub", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("twist_sub", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("compute_odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("odom_pub", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("read_sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                # ArticulationController nodes — proven pattern from reference
                ("drive_controller", "isaacsim.core.nodes.IsaacArticulationController"),
                ("steer_controller", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.CONNECT: [
                ("on_playback_tick.outputs:tick", "clock_pub.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "twist_sub.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "compute_odom.inputs:execIn"),
                ("read_sim_time.outputs:simulationTime", "clock_pub.inputs:timeStamp"),
                ("read_sim_time.outputs:simulationTime", "odom_pub.inputs:timeStamp"),
                ("compute_odom.outputs:execOut", "odom_pub.inputs:execIn"),
                ("compute_odom.outputs:position", "odom_pub.inputs:position"),
                ("compute_odom.outputs:orientation", "odom_pub.inputs:orientation"),
                ("compute_odom.outputs:linearVelocity", "odom_pub.inputs:linearVelocity"),
                ("compute_odom.outputs:angularVelocity", "odom_pub.inputs:angularVelocity"),
                # Tick drives articulation controllers every frame
                ("on_playback_tick.outputs:tick", "drive_controller.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "steer_controller.inputs:execIn"),
            ],
            keys.SET_VALUES: [
                ("twist_sub.inputs:topicName", f"/{fid}/cmd_vel"),
                ("compute_odom.inputs:chassisPrim", [usdrt.Sdf.Path(fl_body)]),
                ("odom_pub.inputs:topicName", f"/{fid}/odom"),
                ("odom_pub.inputs:chassisFrameId", f"{fid}/base_link"),
                ("odom_pub.inputs:odomFrameId", "odom"),
                ("clock_pub.inputs:topicName", "/clock"),
                # Drive controller — velocity control on rear drive wheel
                ("drive_controller.inputs:targetPrim", [usdrt.Sdf.Path(fl_prim)]),
                ("drive_controller.inputs:jointNames", ["back_wheel_drive"]),
                # Steer controller — position control on rear steer pivot
                ("steer_controller.inputs:targetPrim", [usdrt.Sdf.Path(fl_prim)]),
                ("steer_controller.inputs:jointNames", ["back_wheel_swivel"]),
            ],
        },
    )

    log(f"ROS 2 bridge OmniGraph created for {fid}")
    # Node indices in CREATE_NODES: 0=tick 1=clock 2=twist 3=odom 4=odom_pub
    #   5=sim_time 6=drive_ctrl 7=steer_ctrl
    return nodes[2], nodes[6], nodes[7]


def create_pick_item(stage: Usd.Stage) -> None:
    box_path = "/World/pick_box_0"
    existing = stage.GetPrimAtPath(box_path)
    if existing.IsValid():
        return

    cube = UsdGeom.Cube.Define(stage, box_path)
    cube.CreateSizeAttr(0.4)

    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(5.0, 10.0, 0.6))

    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(cube.GetPrim())
    mass_api.CreateMassAttr(5.0)

    log("Created pick box at (5.0, 10.0, 0.6)")


# ---------------------------------------------------------------------------
# cmd_vel → joint drive bridge (replaces ros2_cmd_bridge.py)
# ---------------------------------------------------------------------------

def setup_joint_physics(stage: Usd.Stage) -> None:
    """Apply DriveAPI with stiffness/damping on drive and steer joints.

    These USD-level parameters are read by PhysX at sim init and define
    HOW the joints respond to targets.  Runtime targets are set by the
    IsaacArticulationController OG nodes, not here.
    """
    for fid in FORKLIFT_IDS:
        drive_path = f"/World/{fid}/back_wheel_joints/back_wheel_drive"
        steer_path = f"/World/{fid}/back_wheel_joints/back_wheel_swivel"

        dp = stage.GetPrimAtPath(drive_path)
        sp = stage.GetPrimAtPath(steer_path)

        if dp.IsValid():
            api = UsdPhysics.DriveAPI.Apply(dp, "angular")
            api.GetStiffnessAttr().Set(0.0)        # pure velocity control
            api.GetDampingAttr().Set(DRIVE_DAMPING)

        if sp.IsValid():
            api = UsdPhysics.DriveAPI.Apply(sp, "angular")
            api.GetStiffnessAttr().Set(STEER_STIFFNESS)
            api.GetDampingAttr().Set(STEER_DAMPING)

    log(f"Joint DriveAPI configured for {len(FORKLIFT_IDS)} forklifts")


async def cmd_vel_bridge_loop(forklift_nodes: dict, stage: Usd.Stage) -> None:
    """Per-frame: read cmd_vel from OG twist_sub → set ArticulationController targets."""
    app = omni.kit.app.get_app()

    log(f"cmd_vel bridge loop running — {len(forklift_nodes)} forklifts")

    frame = 0
    while True:
        for fid, (twist_node, drive_node, steer_node) in forklift_nodes.items():
            try:
                lin = og.Controller.attribute("outputs:linearVelocity", twist_node).get()
                ang = og.Controller.attribute("outputs:angularVelocity", twist_node).get()
                if lin is None or ang is None:
                    continue

                linear_x  = float(lin[0])
                angular_z = float(ang[2])

                # Drive: set velocity target (rad/s) on ArticulationController
                drive_vel = linear_x * DRIVE_SCALE
                og.Controller.attribute("inputs:velocityCommand", drive_node).set([drive_vel])

                # Steer: set position target (rad) on ArticulationController
                steer_rad = angular_z * STEER_SCALE
                steer_rad = max(-STEER_MAX, min(STEER_MAX, steer_rad))
                og.Controller.attribute("inputs:positionCommand", steer_node).set([steer_rad])

                if frame % 600 == 0 and (abs(linear_x) > 0.01 or abs(angular_z) > 0.01):
                    log(f"{fid}: lin={linear_x:.2f} m/s  ang={angular_z:.2f} rad/s "
                        f"→ drive={drive_vel:.2f} rad/s  steer={math.degrees(steer_rad):.1f}°")

            except Exception:
                pass

        frame += 1
        await app.next_update_async()


# ---------------------------------------------------------------------------
# Async main — runs inside the Kit event loop
# ---------------------------------------------------------------------------

async def setup_scene():
    log(f"Loading scene: {SCENE_USD}")

    if not os.path.exists(SCENE_USD):
        log(f"ERROR: Scene file not found: {SCENE_USD}")
        return

    # Open the scene
    usd_context = omni.usd.get_context()
    result, error = await usd_context.open_stage_async(SCENE_USD)
    if not result:
        log(f"ERROR: Failed to open stage: {error}")
        return

    # Let the stage settle
    app = omni.kit.app.get_app()
    for _ in range(120):
        await app.next_update_async()

    stage = usd_context.get_stage()
    if stage is None:
        log("ERROR: Stage is None after opening")
        return

    # Auto-enable VS Code integration so users don't have to do it manually
    # via Window → Extensions each time Isaac Sim restarts.
    try:
        ext_mgr = omni.kit.app.get_app().get_extension_manager()
        ext_mgr.set_extension_enabled_immediate("omni.isaac.vscode", True)
        log("Enabled omni.isaac.vscode extension (port 8226)")
    except Exception as e:
        log(f"WARNING: Could not enable omni.isaac.vscode: {e}")

    # Verify key prims — check first forklift
    forklift_prim = stage.GetPrimAtPath(FORKLIFT_PRIM_PATH)
    if not forklift_prim.IsValid():
        log(f"WARNING: Forklift prim not found at {FORKLIFT_PRIM_PATH}")
        log("  → Run populate_scene.py remotely first to create forklift_0..3")

    # Configure physics
    setup_physics(stage)

    # Enumerate joints on the forklift for diagnostics
    forklift_prim = stage.GetPrimAtPath(FORKLIFT_PRIM_PATH)
    if forklift_prim.IsValid():
        from pxr import UsdPhysics as _UsdPhysics
        joint_names = []
        for prim in Usd.PrimRange(forklift_prim):
            if prim.IsA(_UsdPhysics.Joint):
                joint_names.append(str(prim.GetPath()))
                log(f"  Joint found: {prim.GetPath()} type={prim.GetTypeName()}")
        log(f"Total joints on forklift: {len(joint_names)}")
        # Also check articulation root
        art_api = _UsdPhysics.ArticulationRootAPI(forklift_prim)
        if art_api:
            log(f"Articulation root found at {FORKLIFT_PRIM_PATH}")
        # Try the body as articulation root
        body_prim = stage.GetPrimAtPath(FORKLIFT_BODY_PATH)
        if body_prim.IsValid():
            art_api_body = _UsdPhysics.ArticulationRootAPI(body_prim)
            if art_api_body:
                log(f"Articulation root also at {FORKLIFT_BODY_PATH}")

    # Configure joint DriveAPI (stiffness/damping) before sim starts
    setup_joint_physics(stage)

    # Create ROS 2 bridge with ArticulationController nodes
    forklift_nodes = create_ros2_bridge_graph(stage)

    # Create pick item
    create_pick_item(stage)

    log("Scene setup complete — starting timeline")

    # Start the timeline (play the simulation)
    omni.timeline.get_timeline_interface().play()

    log("Timeline started — simulation running")

    # Start the cmd_vel → ArticulationController bridge loop
    asyncio.ensure_future(cmd_vel_bridge_loop(forklift_nodes, stage))
    log("cmd_vel bridge loop started")

    # Write sentinel file for Docker healthcheck
    with open("/tmp/isaac-sim-ready", "w") as f:
        f.write("ready\n")

    log("Isaac Sim Full Streaming App is loaded.")


# ---------------------------------------------------------------------------
# Entry point — schedule the async setup on the Kit event loop
# ---------------------------------------------------------------------------
log("Startup script loaded — scheduling scene setup...")
asyncio.ensure_future(setup_scene())
