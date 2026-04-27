"""
diag_dds.py — Test whether OmniGraph twist_sub can receive from a local OG publisher.
Isolates OmniGraph vs DDS cross-process issue.

Run via: Ctrl+Shift+P → Isaac Sim: Run File Remotely
"""
from __future__ import annotations
import asyncio
import carb
import omni.graph.core as og
import omni.kit.app
import omni.usd

def _log(msg):
    tagged = f"[diag_dds] {msg}"
    carb.log_warn(tagged)
    print(tagged, flush=True)

async def _test():
    stage = omni.usd.get_context().get_stage()
    app = omni.kit.app.get_app()

    # --- Test 1: Check if rclpy is available ---
    try:
        import rclpy
        _log(f"rclpy available: True (path: {rclpy.__file__})")
    except ImportError:
        _log("rclpy available: False")

    # --- Test 2: Check FastDDS env vars as seen by the process ---
    import os
    _log(f"ROS_DOMAIN_ID = {os.environ.get('ROS_DOMAIN_ID', 'NOT SET')}")
    _log(f"RMW_IMPLEMENTATION = {os.environ.get('RMW_IMPLEMENTATION', 'NOT SET')}")
    _log(f"ROS_DISCOVERY_SERVER = {os.environ.get('ROS_DISCOVERY_SERVER', 'NOT SET')}")
    _log(f"FASTRTPS_DEFAULT_PROFILES_FILE = {os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', 'NOT SET')}")

    # --- Test 3: Create OG publisher on /forklift_0/cmd_vel, publish, read sub ---
    _log("--- Internal pub/sub test ---")
    test_graph_path = "/World/_DiagDDS_TestGraph"
    existing = stage.GetPrimAtPath(test_graph_path)
    if existing.IsValid():
        stage.RemovePrim(test_graph_path)

    keys = og.Controller.Keys
    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": test_graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnPlaybackTick"),
                ("twist_pub", "isaacsim.ros2.bridge.ROS2PublishTwist"),
            ],
            keys.CONNECT: [
                ("on_tick.outputs:tick", "twist_pub.inputs:execIn"),
            ],
            keys.SET_VALUES: [
                ("twist_pub.inputs:topicName", "/forklift_0/cmd_vel"),
                ("twist_pub.inputs:linearVelocity", [1.0, 0.0, 0.0]),
                ("twist_pub.inputs:angularVelocity", [0.0, 0.0, 0.0]),
            ],
        },
    )
    _log("Created internal OG twist publisher on /forklift_0/cmd_vel")

    # Wait a few frames for messages to flow
    for i in range(30):
        await app.next_update_async()

    # Read the twist_sub in the bridge graph
    twist_sub_path = "/World/ROS2CmdBridge_forklift_0/twist_sub"
    try:
        node = og.get_node_by_path(twist_sub_path)
        lin = og.Controller.attribute("outputs:linearVelocity", node).get()
        ang = og.Controller.attribute("outputs:angularVelocity", node).get()
        _log(f"After internal pub — twist_sub linearVelocity = {lin}")
        _log(f"After internal pub — twist_sub angularVelocity = {ang}")
        if lin is not None and abs(float(lin[0])) > 0.01:
            _log("RESULT: Internal OG pub → OG sub WORKS. Issue is cross-container DDS.")
        else:
            _log("RESULT: Internal OG pub → OG sub FAILED. Issue is OmniGraph config.")
    except Exception as e:
        _log(f"ERROR reading twist_sub: {e}")

    # Clean up test graph and stop publishing
    stage.RemovePrim(test_graph_path)
    _log("Cleaned up test graph")

    # --- Test 4: Try rclpy subscriber to check cross-process connectivity ---
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        rclpy.init()
        test_node = rclpy.create_node("_diag_dds_test")
        # Check discovered topics
        topics = test_node.get_topic_names_and_types()
        _log(f"rclpy discovered {len(topics)} topics:")
        for name, types in topics:
            _log(f"  {name} [{', '.join(types)}]")
        test_node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        _log(f"rclpy test skipped/failed: {e}")

    _log("=== DDS diagnostic complete ===")

asyncio.ensure_future(_test())
