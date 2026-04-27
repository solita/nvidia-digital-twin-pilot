#!/usr/bin/env python3
"""
ros2_test_maneuver.py — Drive all 4 forklifts: backward → turn → forward 5 m → reverse.

Proves that forklifts are operated via ROS 2 cmd_vel topics.
Each forklift receives identical Twist commands simultaneously.

Sequence:
  Phase 1  Backward  ~1 m       (3 s  @ −0.3 m/s)
  Phase 2  Turn      ~90°       (3 s  @ linear 0.2 m/s + angular 0.5 rad/s)
  Phase 3  Forward   ~5 m       (10 s @ 0.5 m/s)
  -- pause --
  Phase 4  Backward  ~5 m       (10 s @ −0.5 m/s)          ← undo Phase 3
  Phase 5  Turn back ~90°       (3 s  @ linear 0.2 m/s + angular −0.5 rad/s)  ← undo Phase 2
  Phase 6  Forward   ~1 m       (3 s  @ 0.3 m/s)           ← undo Phase 1

Usage (inside vehicle-controller container):
  python3 /tmp/ros2_test_maneuver.py

Or from the host:
  docker exec nvidia-digital-twin-pilot-vehicle-controller-1 \
    bash -c "source /opt/ros/jazzy/setup.bash && python3 /sim-scripts/../simulations/forklift-warehouse/04_helper_scripts/ros2_test_maneuver.py"
"""
from __future__ import annotations

import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist

FORKLIFT_IDS = [f"forklift_{i}" for i in range(4)]
RATE_HZ = 20  # publish rate


class ManeuverTest(Node):
    def __init__(self):
        super().__init__("maneuver_test")
        # Use default (RELIABLE) QoS — compatible with both RELIABLE and BEST_EFFORT subscribers
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pubs: dict[str, any] = {}
        for fid in FORKLIFT_IDS:
            self.pubs[fid] = self.create_publisher(Twist, f"/{fid}/cmd_vel", qos)
        self.get_logger().info(
            f"Publishers created for {len(self.pubs)} forklifts: {', '.join(FORKLIFT_IDS)}"
        )

    # ── helpers ──────────────────────────────────────────────────────────────

    def _twist(self, linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def publish_all(self, linear_x: float, angular_z: float) -> None:
        msg = self._twist(linear_x, angular_z)
        for pub in self.pubs.values():
            pub.publish(msg)

    def stop_all(self) -> None:
        self.publish_all(0.0, 0.0)

    def run_phase(self, name: str, linear_x: float, angular_z: float, duration_s: float) -> None:
        self.get_logger().info(
            f"{name}: linear={linear_x:.2f} m/s  angular={angular_z:.2f} rad/s  "
            f"duration={duration_s:.1f} s"
        )
        ticks = int(duration_s * RATE_HZ)
        dt = 1.0 / RATE_HZ
        for _ in range(ticks):
            self.publish_all(linear_x, angular_z)
            time.sleep(dt)
        # Brief stop between phases
        self.stop_all()
        time.sleep(0.5)

    # ── maneuver ─────────────────────────────────────────────────────────────

    def run_maneuver(self) -> None:
        self.get_logger().info("=" * 50)
        self.get_logger().info("STARTING ROS 2 FORKLIFT MANEUVER TEST")
        self.get_logger().info("=" * 50)

        # Allow publishers to be discovered
        self.get_logger().info("Waiting 2 s for publisher discovery...")
        time.sleep(2.0)

        # ── Forward sequence ─────────────────────────────────────────────────
        self.run_phase("Phase 1 — BACKWARD",   linear_x=-0.3, angular_z=0.0,  duration_s=3.0)
        self.run_phase("Phase 2 — TURN",        linear_x=0.2,  angular_z=0.5,  duration_s=3.0)
        self.run_phase("Phase 3 — FORWARD 5m",  linear_x=0.5,  angular_z=0.0,  duration_s=10.0)

        self.get_logger().info("--- Pause 2 s ---")
        self.stop_all()
        time.sleep(2.0)

        # ── Reverse sequence ─────────────────────────────────────────────────
        self.get_logger().info("=== REVERSING SEQUENCE ===")
        self.run_phase("Phase 4 — BACKWARD 5m", linear_x=-0.5, angular_z=0.0,  duration_s=10.0)
        self.run_phase("Phase 5 — TURN BACK",   linear_x=0.2,  angular_z=-0.5, duration_s=3.0)
        self.run_phase("Phase 6 — FORWARD",     linear_x=0.3,  angular_z=0.0,  duration_s=3.0)

        self.stop_all()
        self.get_logger().info("=" * 50)
        self.get_logger().info("MANEUVER COMPLETE — all forklifts stopped")
        self.get_logger().info("=" * 50)


def main() -> None:
    rclpy.init()
    node = ManeuverTest()
    try:
        node.run_maneuver()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted — stopping all forklifts")
    finally:
        node.stop_all()
        # Publish a few extra stop messages to ensure they're received
        for _ in range(10):
            node.stop_all()
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
