"""
warehouse_scene.py — Isaac Sim standalone script for the warehouse digital twin.

Loads the existing scene_assembly.usd (warehouse + forklift), configures PhysX,
sets up the ROS 2 bridge (clock, cmd_vel, odom, fork_cmd, joint_states),
and runs the simulation headless or with livestream.

Usage (inside Isaac Sim container):
  ./python.sh /sim-scripts/warehouse_scene.py --headless
  ./python.sh /sim-scripts/warehouse_scene.py --livestream
"""
from __future__ import annotations

import argparse
import math
import os
import sys

# ---------------------------------------------------------------------------
# Isaac Sim standalone application bootstrap
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp

# Parse args before SimulationApp init (it consumes some args too)
parser = argparse.ArgumentParser(description="Warehouse Digital Twin Scene")
parser.add_argument("--headless", action="store_true", help="Run without GUI")
parser.add_argument("--livestream", action="store_true", help="Enable WebSocket livestream")
args, _unknown = parser.parse_known_args()

CONFIG = {
    "headless": args.headless or args.livestream,
    "width": 1280,
    "height": 720,
}
if args.livestream:
    CONFIG["enable_livestream"] = True
    CONFIG["livestream_port"] = 49100

simulation_app = SimulationApp(CONFIG)

# ---------------------------------------------------------------------------
# Imports available only after SimulationApp is created
# ---------------------------------------------------------------------------
import carb
import numpy as np
import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Path to the existing warehouse + forklift scene (mounted into container)
SCENE_USD = os.environ.get(
    "SCENE_USD",
    "/sim-scripts/scenes/scene_assembly.usd",
)

# Existing prim paths in scene_assembly.usd
FORKLIFT_PRIM_PATH = "/World/forklift_b"
FORKLIFT_BODY_PATH = "/World/forklift_b/body"
DRIVE_JOINT_PATH = "/World/forklift_b/back_wheel_joints/back_wheel_drive"
STEER_JOINT_PATH = "/World/forklift_b/back_wheel_joints/back_wheel_swivel"

# Forklift physical parameters (from existing controller)
WHEEL_RADIUS = 0.15  # metres (estimated from forklift model)
WHEEL_DISTANCE = 1.0  # metres between left/right wheel sets

# ROS 2 config
ROS_DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "42"))

# PhysX settings
PHYSICS_DT = 1.0 / 60.0  # 60 Hz


def log(msg: str) -> None:
    carb.log_info(f"[warehouse_scene] {msg}")
    print(f"[warehouse_scene] {msg}")


# ---------------------------------------------------------------------------
# Scene setup
# ---------------------------------------------------------------------------

def setup_physics(stage: Usd.Stage) -> None:
    """Configure PhysX scene settings for warehouse simulation."""
    physics_scene_path = "/physicsScene"
    scene_prim = stage.GetPrimAtPath(physics_scene_path)

    if not scene_prim.IsValid():
        UsdPhysics.Scene.Define(stage, physics_scene_path)
        scene_prim = stage.GetPrimAtPath(physics_scene_path)

    physics_scene = UsdPhysics.Scene(scene_prim)
    # Z-up, gravity pointing down
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)

    # PhysX solver settings
    physx_api = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    physx_api.CreateEnableCCDAttr(True)
    physx_api.CreateEnableStabilizationAttr(True)
    physx_api.CreateTimeStepsPerSecondAttr(int(1.0 / PHYSICS_DT))
    physx_api.CreateEnableGPUDynamicsAttr(True)
    physx_api.CreateBroadphaseTypeAttr("GPU")
    physx_api.CreateSolverTypeAttr("TGS")

    log("PhysX configured: TGS solver, GPU dynamics, 60Hz, Z-up gravity")


def create_ros2_bridge_graph(stage: Usd.Stage) -> None:
    """Create OmniGraph action graph for the ROS 2 bridge.

    Sets up:
    - Clock publisher (/clock)
    - Twist subscriber (/forklift_0/cmd_vel → differential drive)
    - Odometry publisher (/forklift_0/odom)
    - Float64 subscriber (/forklift_0/fork_cmd → fork prismatic joint)
    - JointState publisher (/forklift_0/joint_states)
    """
    # We use OmniGraph to wire up the ROS 2 bridge nodes.
    # Isaac Sim 5.x provides built-in ROS 2 OG nodes.

    graph_path = "/World/ROS2BridgeGraph"

    # Remove existing graph if present (for re-runs)
    existing = stage.GetPrimAtPath(graph_path)
    if existing.IsValid():
        stage.RemovePrim(graph_path)

    # Create the action graph
    keys = og.Controller.Keys
    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                # Tick source
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),

                # -- Clock Publisher --
                ("clock_pub", "omni.isaac.ros2_bridge.ROS2PublishClock"),

                # -- Twist Subscriber (cmd_vel) --
                ("twist_sub", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),

                # -- Differential Controller --
                ("diff_controller", "omni.isaac.wheeled_robots.DifferentialController"),

                # -- Articulation Controller (apply drive commands) --
                ("art_controller", "omni.isaac.core_nodes.IsaacArticulationController"),

                # -- Odometry Publisher --
                ("compute_odom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                ("odom_pub", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),

                # -- Joint State Publisher --
                ("joint_state_pub", "omni.isaac.ros2_bridge.ROS2PublishJointState"),

                # -- TF Publisher --
                ("tf_pub", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),

                # -- Isaac Read Sim Time --
                ("read_sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
            ],
            keys.CONNECT: [
                # Tick drives everything
                ("on_playback_tick.outputs:tick", "clock_pub.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "twist_sub.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "compute_odom.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "joint_state_pub.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "tf_pub.inputs:execIn"),
                ("on_playback_tick.outputs:tick", "read_sim_time.inputs:execIn"),

                # Twist → Differential Controller → Articulation Controller
                ("twist_sub.outputs:execOut", "diff_controller.inputs:execIn"),
                ("twist_sub.outputs:linearVelocity", "diff_controller.inputs:linearVelocity"),
                ("twist_sub.outputs:angularVelocity", "diff_controller.inputs:angularVelocity"),
                ("diff_controller.outputs:execOut", "art_controller.inputs:execIn"),
                ("diff_controller.outputs:velocityCommand", "art_controller.inputs:velocityCommand"),

                # Sim time for publishers
                ("read_sim_time.outputs:simulationTime", "clock_pub.inputs:timeStamp"),
                ("read_sim_time.outputs:simulationTime", "odom_pub.inputs:timeStamp"),
                ("read_sim_time.outputs:simulationTime", "joint_state_pub.inputs:timeStamp"),
                ("read_sim_time.outputs:simulationTime", "tf_pub.inputs:timeStamp"),

                # Odom compute → publish
                ("compute_odom.outputs:execOut", "odom_pub.inputs:execIn"),
                ("compute_odom.outputs:position", "odom_pub.inputs:position"),
                ("compute_odom.outputs:orientation", "odom_pub.inputs:orientation"),
                ("compute_odom.outputs:linearVelocity", "odom_pub.inputs:linearVelocity"),
                ("compute_odom.outputs:angularVelocity", "odom_pub.inputs:angularVelocity"),
            ],
            keys.SET_VALUES: [
                # Twist subscriber topic
                ("twist_sub.inputs:topicName", "/forklift_0/cmd_vel"),

                # Differential controller params
                ("diff_controller.inputs:wheelRadius", WHEEL_RADIUS),
                ("diff_controller.inputs:wheelDistance", WHEEL_DISTANCE),
                ("diff_controller.inputs:maxLinearSpeed", 0.5),
                ("diff_controller.inputs:maxAngularSpeed", 1.0),

                # Articulation controller target
                ("art_controller.inputs:targetPrim", FORKLIFT_PRIM_PATH),
                ("art_controller.inputs:robotPath", FORKLIFT_PRIM_PATH),
                ("art_controller.inputs:usePath", True),

                # Odometry source
                ("compute_odom.inputs:chassisPrim", FORKLIFT_BODY_PATH),

                # Odometry publisher topic
                ("odom_pub.inputs:topicName", "/forklift_0/odom"),
                ("odom_pub.inputs:chassisFrameId", "forklift_0/base_link"),
                ("odom_pub.inputs:odomFrameId", "odom"),

                # Joint state publisher
                ("joint_state_pub.inputs:topicName", "/forklift_0/joint_states"),
                ("joint_state_pub.inputs:targetPrim", FORKLIFT_PRIM_PATH),

                # TF publisher
                ("tf_pub.inputs:topicName", "/tf"),
                ("tf_pub.inputs:targetPrims", [FORKLIFT_PRIM_PATH]),

                # Clock publisher
                ("clock_pub.inputs:topicName", "/clock"),
            ],
        },
    )

    log("ROS 2 bridge OmniGraph created with clock, cmd_vel, odom, joint_states, tf")
    return graph


def create_pick_item(stage: Usd.Stage) -> None:
    """Create a pick-able box on the first shelf slot for the Phase 1 demo."""
    box_path = "/World/pick_box_0"
    existing = stage.GetPrimAtPath(box_path)
    if existing.IsValid():
        return  # Already exists

    # Create a cube
    cube = UsdGeom.Cube.Define(stage, box_path)
    cube.CreateSizeAttr(0.4)

    # Position on a shelf (approximate from warehouse spatial data)
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(5.0, 10.0, 0.6))  # On shelf, slightly raised

    # Add rigid body physics
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(cube.GetPrim())
    mass_api.CreateMassAttr(5.0)  # 5 kg

    log("Created pick box at (5.0, 10.0, 0.6)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    log(f"Loading scene: {SCENE_USD}")

    if not os.path.exists(SCENE_USD):
        log(f"ERROR: Scene file not found: {SCENE_USD}")
        log("Make sure scene_assembly.usd is mounted at the expected path")
        simulation_app.close()
        sys.exit(1)

    # Create the Isaac Sim World
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=PHYSICS_DT,
        rendering_dt=PHYSICS_DT,
    )

    # Open the existing scene
    omni.usd.get_context().open_stage(SCENE_USD)

    # Wait for stage to load
    app = omni.kit.app.get_app()
    for _ in range(120):  # Up to 2 seconds at 60fps
        app.update()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        log("ERROR: Failed to load stage")
        simulation_app.close()
        sys.exit(1)

    # Verify key prims exist
    forklift_prim = stage.GetPrimAtPath(FORKLIFT_PRIM_PATH)
    if not forklift_prim.IsValid():
        log(f"WARNING: Forklift prim not found at {FORKLIFT_PRIM_PATH}")
        log("The scene may use different prim paths — check scene_assembly.usd")

    # Configure physics
    setup_physics(stage)

    # Create ROS 2 bridge
    create_ros2_bridge_graph(stage)

    # Create pick item for Phase 1 demo
    create_pick_item(stage)

    log("Scene setup complete — starting simulation")

    # Start the timeline
    omni.timeline.get_timeline_interface().play()

    # Main simulation loop
    frame = 0
    while simulation_app.is_running():
        world.step(render=not args.headless)
        frame += 1

        if frame % 600 == 0:
            log(f"Simulation running — frame {frame}")

    log("Simulation ended")
    simulation_app.close()


if __name__ == "__main__":
    main()
