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
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SCENE_USD = os.environ.get(
    "SCENE_USD",
    "/sim-scripts/scenes/scene_assembly.usd",
)

FORKLIFT_PRIM_PATH = "/World/forklift_b"
FORKLIFT_BODY_PATH = "/World/forklift_b/body"
DRIVE_JOINT_PATH = "/World/forklift_b/back_wheel_joints/back_wheel_drive"
STEER_JOINT_PATH = "/World/forklift_b/back_wheel_joints/back_wheel_swivel"

WHEEL_RADIUS = 0.15
WHEEL_DISTANCE = 1.0

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


def create_ros2_bridge_graph(stage: Usd.Stage) -> None:
    graph_path = "/World/ROS2BridgeGraph"

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
                ("break_linear", "omni.graph.nodes.BreakVector3"),
                ("break_angular", "omni.graph.nodes.BreakVector3"),
                ("diff_controller", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("art_controller", "isaacsim.core.nodes.IsaacArticulationController"),
                ("compute_odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("odom_pub", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("joint_state_pub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("tf_pub", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("read_sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            keys.CONNECT: [
                ("on_playback_tick.outputs:tick", "clock_pub.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "twist_sub.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "compute_odom.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "joint_state_pub.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "tf_pub.inputs:execIn"),
                ("twist_sub.outputs:execOut", "diff_controller.inputs:execIn"),
                ("twist_sub.outputs:linearVelocity", "break_linear.inputs:tuple"),
                ("twist_sub.outputs:angularVelocity", "break_angular.inputs:tuple"),
                ("break_linear.outputs:x", "diff_controller.inputs:linearVelocity"),
                ("break_angular.outputs:z", "diff_controller.inputs:angularVelocity"),
                ("on_playback_tick.outputs:tick", "art_controller.inputs:execIn"),
                ("diff_controller.outputs:velocityCommand", "art_controller.inputs:velocityCommand"),
                ("read_sim_time.outputs:simulationTime", "clock_pub.inputs:timeStamp"),
                ("read_sim_time.outputs:simulationTime", "odom_pub.inputs:timeStamp"),
                ("read_sim_time.outputs:simulationTime", "joint_state_pub.inputs:timeStamp"),
                ("read_sim_time.outputs:simulationTime", "tf_pub.inputs:timeStamp"),
                ("compute_odom.outputs:execOut", "odom_pub.inputs:execIn"),
                ("compute_odom.outputs:position", "odom_pub.inputs:position"),
                ("compute_odom.outputs:orientation", "odom_pub.inputs:orientation"),
                ("compute_odom.outputs:linearVelocity", "odom_pub.inputs:linearVelocity"),
                ("compute_odom.outputs:angularVelocity", "odom_pub.inputs:angularVelocity"),
            ],
            keys.SET_VALUES: [
                ("twist_sub.inputs:topicName", "/forklift_0/cmd_vel"),
                ("diff_controller.inputs:wheelRadius", WHEEL_RADIUS),
                ("diff_controller.inputs:wheelDistance", WHEEL_DISTANCE),
                ("diff_controller.inputs:maxLinearSpeed", 0.5),
                ("diff_controller.inputs:maxAngularSpeed", 1.0),
                ("art_controller.inputs:targetPrim", FORKLIFT_PRIM_PATH),
                ("compute_odom.inputs:chassisPrim", FORKLIFT_BODY_PATH),
                ("odom_pub.inputs:topicName", "/forklift_0/odom"),
                ("odom_pub.inputs:chassisFrameId", "forklift_0/base_link"),
                ("odom_pub.inputs:odomFrameId", "odom"),
                ("joint_state_pub.inputs:topicName", "/forklift_0/joint_states"),
                ("joint_state_pub.inputs:targetPrim", FORKLIFT_PRIM_PATH),
                ("tf_pub.inputs:topicName", "/tf"),
                ("tf_pub.inputs:targetPrims", [FORKLIFT_PRIM_PATH]),
                ("clock_pub.inputs:topicName", "/clock"),
            ],
        },
    )

    log("ROS 2 bridge OmniGraph created")
    return graph


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

    # Verify key prims
    forklift_prim = stage.GetPrimAtPath(FORKLIFT_PRIM_PATH)
    if not forklift_prim.IsValid():
        log(f"WARNING: Forklift prim not found at {FORKLIFT_PRIM_PATH}")

    # Configure physics
    setup_physics(stage)

    # Create ROS 2 bridge
    create_ros2_bridge_graph(stage)

    # Create pick item
    create_pick_item(stage)

    log("Scene setup complete — starting timeline")

    # Start the timeline (play the simulation)
    omni.timeline.get_timeline_interface().play()

    log("Timeline started — simulation running")


# ---------------------------------------------------------------------------
# Entry point — schedule the async setup on the Kit event loop
# ---------------------------------------------------------------------------
log("Startup script loaded — scheduling scene setup...")
asyncio.ensure_future(setup_scene())
