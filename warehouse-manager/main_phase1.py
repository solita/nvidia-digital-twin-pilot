"""
main_phase1.py — Phase 1 Warehouse Manager: Hardcoded single order.

Publishes a single TaskAssignment after a 3-second startup delay,
then monitors task status and forklift state.

Usage:
  source /opt/ros/humble/setup.bash
  source ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=42
  python3 warehouse-manager/main_phase1.py --ros-args -p use_sim_time:=true
"""
from __future__ import annotations

import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point

try:
    from warehouse_msgs.msg import ForkliftStatus, TaskAssignment, TaskStatus
    HAS_WAREHOUSE_MSGS = True
except ImportError:
    HAS_WAREHOUSE_MSGS = False

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10,
)


class WarehouseManagerNode(Node):
    def __init__(self):
        super().__init__("warehouse_manager")

        if not HAS_WAREHOUSE_MSGS:
            self.get_logger().error(
                "warehouse_msgs not available — build them first!"
            )
            return

        # Publisher
        self.task_pub = self.create_publisher(
            TaskAssignment, "/warehouse/task_assignment", RELIABLE_QOS
        )

        # Subscribers
        self.task_status_sub = self.create_subscription(
            TaskStatus,
            "/warehouse/task_status",
            self._task_status_cb,
            RELIABLE_QOS,
        )
        self.forklift_status_sub = self.create_subscription(
            ForkliftStatus,
            "/forklift_0/status",
            self._forklift_status_cb,
            RELIABLE_QOS,
        )

        # Publish first task after 3-second startup delay
        self._task_published = False
        self._startup_timer = self.create_timer(3.0, self._publish_initial_task)

        self.get_logger().info("Phase 1 Warehouse Manager started — waiting 3s before task dispatch")

    def _publish_initial_task(self) -> None:
        """Publish a single hardcoded task after startup delay."""
        # One-shot: cancel the timer
        self._startup_timer.cancel()

        if self._task_published:
            return

        msg = TaskAssignment()
        msg.task_id = str(uuid.uuid4())
        msg.order_id = "order_001"
        msg.forklift_id = "forklift_0"
        msg.task_type = TaskAssignment.PICK
        msg.source = Point(x=5.0, y=10.0, z=0.0)       # shelf position
        msg.destination = Point(x=5.0, y=2.0, z=0.0)    # dock position
        msg.priority = 1

        self.task_pub.publish(msg)
        self._task_published = True
        self.get_logger().info(
            f"Published task {msg.task_id}: "
            f"PICK from ({msg.source.x},{msg.source.y}) "
            f"to ({msg.destination.x},{msg.destination.y})"
        )

    def _task_status_cb(self, msg: TaskStatus) -> None:
        status_names = {0: "PENDING", 1: "IN_PROGRESS", 2: "COMPLETED", 3: "FAILED"}
        name = status_names.get(msg.status, f"UNKNOWN({msg.status})")
        self.get_logger().info(
            f"Task {msg.task_id} [{msg.forklift_id}]: {name}"
        )
        if msg.status == TaskStatus.COMPLETED:
            self.get_logger().info("Order fulfilled successfully!")
        elif msg.status == TaskStatus.FAILED:
            self.get_logger().warn(f"Task failed: {msg.error_message}")

    def _forklift_status_cb(self, msg: ForkliftStatus) -> None:
        state_names = {
            0: "IDLE", 1: "NAV_TO_SHELF", 2: "PICKING",
            3: "NAV_TO_DOCK", 4: "DROPPING", 5: "ERROR", 6: "RECOVERING",
        }
        name = state_names.get(msg.state, f"UNKNOWN({msg.state})")
        self.get_logger().info(
            f"Forklift {msg.forklift_id}: {name} "
            f"pos=({msg.pose.position.x:.1f},{msg.pose.position.y:.1f}) "
            f"battery={msg.battery_level:.0f}%",
            throttle_duration_sec=5.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
