"""
diag_dds3.py — Test if rclpy inside Isaac Sim can receive from Jazzy containers.
Publishes AND subscribes to /test_jazzy_check to verify DDS round-trip,
then subscribes to /forklift_0/cmd_vel to check external messages.

Run via: Ctrl+Shift+P → Isaac Sim: Run File Remotely
"""
import sys
import time
import threading

# Isaac Sim bundles its own rclpy
sys.path.insert(0, "/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy")
sys.path.insert(0, "/isaac-sim/exts/isaacsim.ros2.bridge/jazzy")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist

TAG = "[diag_dds3]"

class DiagNode(Node):
    def __init__(self):
        super().__init__("diag_dds3_node")
        self.cmd_vel_count = 0
        self.self_test_count = 0

        qos_best = QoSProfile(depth=10,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        qos_rel = QoSProfile(depth=10,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)

        # Subscribe to forklift cmd_vel (external)
        self.create_subscription(Twist, "/forklift_0/cmd_vel", self._cmd_vel_cb, qos_best)
        self.create_subscription(Twist, "/forklift_0/cmd_vel", self._cmd_vel_cb_rel, qos_rel)

        # Self-test: publish and subscribe on a test topic
        self.self_pub = self.create_publisher(Twist, "/diag_dds3_test", qos_rel)
        self.create_subscription(Twist, "/diag_dds3_test", self._self_cb, qos_rel)
        self.self_timer = self.create_timer(0.1, self._self_pub_cb)

    def _cmd_vel_cb(self, msg):
        self.cmd_vel_count += 1
        if self.cmd_vel_count <= 3:
            print(f"{TAG} cmd_vel BEST_EFFORT: lin.x={msg.linear.x:.2f} #{self.cmd_vel_count}", flush=True)

    def _cmd_vel_cb_rel(self, msg):
        self.cmd_vel_count += 1
        if self.cmd_vel_count <= 3:
            print(f"{TAG} cmd_vel RELIABLE: lin.x={msg.linear.x:.2f} #{self.cmd_vel_count}", flush=True)

    def _self_cb(self, msg):
        self.self_test_count += 1

    def _self_pub_cb(self):
        t = Twist()
        t.linear.x = 42.0
        self.self_pub.publish(t)


def run():
    rclpy.init()
    node = DiagNode()

    print(f"{TAG} Node created, spinning for 6 seconds...", flush=True)
    print(f"{TAG} Discovery server: checking env...", flush=True)

    import os
    for k in ["ROS_DISCOVERY_SERVER", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE"]:
        print(f"{TAG}   {k}={os.environ.get(k, '<not set>')}", flush=True)

    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    time.sleep(6)

    print(f"{TAG} Results after 6s:", flush=True)
    print(f"{TAG}   Self-test messages received: {node.self_test_count}", flush=True)
    print(f"{TAG}   /forklift_0/cmd_vel messages: {node.cmd_vel_count}", flush=True)

    if node.self_test_count > 0:
        print(f"{TAG}   ✓ rclpy self-pub/sub works (DDS participant alive)", flush=True)
    else:
        print(f"{TAG}   ✗ rclpy self-pub/sub FAILED (DDS participant broken!)", flush=True)

    if node.cmd_vel_count > 0:
        print(f"{TAG}   ✓ External cmd_vel messages arriving!", flush=True)
    else:
        print(f"{TAG}   ✗ No external cmd_vel messages (DDS cross-container issue)", flush=True)

    node.destroy_node()
    rclpy.shutdown()
    print(f"{TAG} Done", flush=True)


run()
