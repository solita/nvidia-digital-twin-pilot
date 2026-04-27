"""
diag_dds2.py — Test DDS connectivity using rclpy inside Isaac Sim.
Subscribes to /forklift_0/cmd_vel for 5 seconds, reports what arrives.

Run via: Ctrl+Shift+P → Isaac Sim: Run File Remotely
THEN IMMEDIATELY run from terminal:
  docker exec nvidia-digital-twin-pilot-vehicle-controller-1 bash -c \
    "source /opt/ros/humble/setup.bash && \
     ros2 topic pub -r 10 /forklift_0/cmd_vel geometry_msgs/msg/Twist \
     '{linear: {x: 1.0}, angular: {z: 0.0}}'"
"""
from __future__ import annotations
import asyncio
import carb
import omni.kit.app

def _log(msg):
    tagged = f"[diag_dds2] {msg}"
    carb.log_warn(tagged)
    print(tagged, flush=True)

async def _test():
    import os
    app = omni.kit.app.get_app()

    _log(f"PID = {os.getpid()}")
    _log(f"ROS_DOMAIN_ID = {os.environ.get('ROS_DOMAIN_ID')}")
    _log(f"RMW_IMPLEMENTATION = {os.environ.get('RMW_IMPLEMENTATION')}")
    _log(f"ROS_DISCOVERY_SERVER = {os.environ.get('ROS_DISCOVERY_SERVER')}")

    # Use rclpy directly to test DDS
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

    rclpy.init()
    node = rclpy.create_node("_diag_dds2_listener")

    # List discovered topics
    _log("Discovered topics at start:")
    for name, types in node.get_topic_names_and_types():
        _log(f"  {name} [{', '.join(types)}]")

    # Subscribe with BEST_EFFORT (matches most Isaac Sim defaults)
    msg_count = [0]
    last_msg = [None]

    from geometry_msgs.msg import Twist
    qos_best_effort = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    qos_reliable = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    def _cb_be(msg):
        msg_count[0] += 1
        last_msg[0] = msg
        if msg_count[0] <= 3:
            _log(f"BEST_EFFORT got msg #{msg_count[0]}: lin.x={msg.linear.x:.2f}")

    def _cb_rel(msg):
        _log(f"RELIABLE got msg: lin.x={msg.linear.x:.2f}")

    node.create_subscription(Twist, "/forklift_0/cmd_vel", _cb_be, qos_best_effort)
    node.create_subscription(Twist, "/forklift_0/cmd_vel", _cb_rel, qos_reliable)
    _log("Subscribed to /forklift_0/cmd_vel (BEST_EFFORT + RELIABLE)")
    _log("Waiting 5 seconds for messages... Publish now!")

    # Spin for ~5 seconds, yielding to Isaac Sim
    for i in range(150):  # ~5s at 30fps
        rclpy.spin_once(node, timeout_sec=0.01)
        await app.next_update_async()
        if i == 75:
            _log(f"  ...halfway: {msg_count[0]} messages received so far")

    _log(f"RESULT: Received {msg_count[0]} messages in 5 seconds")
    if msg_count[0] == 0:
        _log("DIAGNOSIS: DDS messages NOT reaching Isaac Sim process")
        _log("  → Check if discovery server sees Isaac Sim participant")
        # Try listing topics again
        topics_after = node.get_topic_names_and_types()
        _log(f"  Topics after wait ({len(topics_after)}):")
        for name, types in topics_after:
            _log(f"    {name} [{', '.join(types)}]")
    else:
        _log("DIAGNOSIS: DDS works! Issue is in OmniGraph twist_sub node processing")

    node.destroy_node()
    rclpy.shutdown()
    _log("=== DDS2 diagnostic complete ===")

asyncio.ensure_future(_test())
