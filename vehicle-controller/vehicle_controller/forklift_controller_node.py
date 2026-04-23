"""
forklift_controller_node.py — ROS 2 node implementing the forklift state machine.

Phase 1: Proportional control (P-controller) for navigation.
Phase 3: Will be replaced with Nav2 action client.

State machine:
  IDLE → NAVIGATING_TO_SHELF → PICKING → NAVIGATING_TO_DOCK → DROPPING → IDLE

Subscribes:
  /{forklift_id}/odom           (nav_msgs/Odometry)
  /warehouse/task_assignment    (warehouse_msgs/TaskAssignment)

Publishes:
  /{forklift_id}/cmd_vel        (geometry_msgs/Twist)
  /{forklift_id}/fork_cmd       (std_msgs/Float64)
  /warehouse/task_status        (warehouse_msgs/TaskStatus)
  /{forklift_id}/status         (warehouse_msgs/ForkliftStatus)
"""
from __future__ import annotations

import enum
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from builtin_interfaces.msg import Time as TimeMsg

# These imports will work after warehouse_msgs is built
try:
    from warehouse_msgs.msg import ForkliftStatus, TaskAssignment, TaskStatus
    HAS_WAREHOUSE_MSGS = True
except ImportError:
    HAS_WAREHOUSE_MSGS = False


# ---------------------------------------------------------------------------
# QoS profiles
# ---------------------------------------------------------------------------

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10,
)

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1,
)


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class ForkliftState(enum.IntEnum):
    IDLE = 0
    NAVIGATING_TO_SHELF = 1
    PICKING = 2
    NAVIGATING_TO_DOCK = 3
    DROPPING = 4
    ERROR = 5
    RECOVERING = 6


# ---------------------------------------------------------------------------
# Proportional controller parameters (Phase 1)
# ---------------------------------------------------------------------------

KP_LINEAR = 0.5       # linear velocity gain
KP_ANGULAR = 2.0      # angular velocity gain
MAX_LINEAR_VEL = 0.5   # m/s
MAX_ANGULAR_VEL = 1.0  # rad/s
GOAL_REACHED_DIST = 0.3  # metres

# Fork positions
FORK_GROUND = 0.0     # Fork lowered to ground
FORK_TRAVEL = 0.5     # Fork at travel height
FORK_LIFTED = 0.8     # Fork raised with item

# Timing for pick/drop sequences (seconds)
FORK_MOVE_WAIT = 2.0
APPROACH_WAIT = 2.0
APPROACH_DIST = 0.3   # metres to drive into shelf


# ---------------------------------------------------------------------------
# ForkliftControllerNode
# ---------------------------------------------------------------------------

class ForkliftControllerNode(Node):
    def __init__(self):
        super().__init__("forklift_controller")

        # Parameters
        self.declare_parameter("forklift_id", "forklift_0")

        self.forklift_id = self.get_parameter("forklift_id").value
        ns = self.forklift_id

        self.get_logger().info(f"Starting controller for {self.forklift_id}")

        # State
        self.state = ForkliftState.IDLE
        self.current_task_id = ""
        self.current_order_id = ""
        self.source = Point()
        self.destination = Point()
        self.current_pose = Pose()
        self.battery_level = 100.0

        # Pick/drop sequence tracking
        self._sequence_start_time: float | None = None
        self._sequence_step = 0

        # -- Publishers --
        self.cmd_vel_pub = self.create_publisher(
            Twist, f"/{ns}/cmd_vel", BEST_EFFORT_QOS
        )
        self.fork_cmd_pub = self.create_publisher(
            Float64, f"/{ns}/fork_cmd", BEST_EFFORT_QOS
        )

        if HAS_WAREHOUSE_MSGS:
            self.task_status_pub = self.create_publisher(
                TaskStatus, "/warehouse/task_status", RELIABLE_QOS
            )
            self.forklift_status_pub = self.create_publisher(
                ForkliftStatus, f"/{ns}/status", RELIABLE_QOS
            )
        else:
            self.task_status_pub = None
            self.forklift_status_pub = None
            self.get_logger().warn(
                "warehouse_msgs not available — status topics disabled"
            )

        # -- Subscribers --
        self.odom_sub = self.create_subscription(
            Odometry, f"/{ns}/odom", self._odom_cb, BEST_EFFORT_QOS
        )

        if HAS_WAREHOUSE_MSGS:
            self.task_sub = self.create_subscription(
                TaskAssignment,
                "/warehouse/task_assignment",
                self._task_assignment_cb,
                RELIABLE_QOS,
            )

        # -- Control timer at 10 Hz --
        self.control_timer = self.create_timer(0.1, self._control_loop)

        # -- Status publish timer at 2 Hz --
        self.status_timer = self.create_timer(0.5, self._publish_status)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose

    def _task_assignment_cb(self, msg) -> None:
        if msg.forklift_id != self.forklift_id:
            return

        if self.state != ForkliftState.IDLE:
            self.get_logger().warn(
                f"Received task {msg.task_id} but state is {self.state.name} — ignoring"
            )
            return

        self.get_logger().info(
            f"Task assigned: {msg.task_id} type={msg.task_type} "
            f"src=({msg.source.x:.1f},{msg.source.y:.1f}) "
            f"dst=({msg.destination.x:.1f},{msg.destination.y:.1f})"
        )

        self.current_task_id = msg.task_id
        self.current_order_id = msg.order_id
        self.source = msg.source
        self.destination = msg.destination

        # Transition to navigation
        self._set_state(ForkliftState.NAVIGATING_TO_SHELF)
        self._publish_task_status(TaskStatus.IN_PROGRESS if HAS_WAREHOUSE_MSGS else 1)

    # ------------------------------------------------------------------
    # Control loop (10 Hz)
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        if self.state == ForkliftState.IDLE:
            self._stop()
            return

        if self.state == ForkliftState.NAVIGATING_TO_SHELF:
            self._navigate_to(self.source, ForkliftState.PICKING)

        elif self.state == ForkliftState.PICKING:
            self._run_pick_sequence()

        elif self.state == ForkliftState.NAVIGATING_TO_DOCK:
            self._navigate_to(self.destination, ForkliftState.DROPPING)

        elif self.state == ForkliftState.DROPPING:
            self._run_drop_sequence()

        elif self.state == ForkliftState.ERROR:
            self._stop()

    # ------------------------------------------------------------------
    # Navigation (Phase 1 — proportional control)
    # ------------------------------------------------------------------

    def _navigate_to(self, goal: Point, next_state: ForkliftState) -> None:
        dx = goal.x - self.current_pose.position.x
        dy = goal.y - self.current_pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < GOAL_REACHED_DIST:
            self._stop()
            self.get_logger().info(
                f"Reached goal ({goal.x:.1f},{goal.y:.1f}) — transitioning to {next_state.name}"
            )
            self._set_state(next_state)
            self._sequence_start_time = None
            self._sequence_step = 0
            return

        # Desired heading
        desired_yaw = math.atan2(dy, dx)

        # Current yaw from quaternion
        current_yaw = self._quaternion_to_yaw(self.current_pose.orientation)

        # Angle error (wrapped to [-pi, pi])
        angle_error = desired_yaw - current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Proportional control
        linear_vel = KP_LINEAR * distance
        angular_vel = KP_ANGULAR * angle_error

        # Clamp
        linear_vel = max(0.0, min(linear_vel, MAX_LINEAR_VEL))
        angular_vel = max(-MAX_ANGULAR_VEL, min(angular_vel, MAX_ANGULAR_VEL))

        # Reduce linear speed when turning sharply
        if abs(angle_error) > 0.5:  # ~30 degrees
            linear_vel *= 0.3

        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)

    # ------------------------------------------------------------------
    # Pick sequence (timed, not sensor-based in Phase 1)
    # ------------------------------------------------------------------

    def _run_pick_sequence(self) -> None:
        now = time.monotonic()
        if self._sequence_start_time is None:
            self._sequence_start_time = now
            self._sequence_step = 0
            self.get_logger().info("Starting pick sequence")

        elapsed = now - self._sequence_start_time

        if self._sequence_step == 0:
            # Step 1: Lower fork to ground
            self._publish_fork_cmd(FORK_GROUND)
            if elapsed > FORK_MOVE_WAIT:
                self._sequence_step = 1
                self._sequence_start_time = now

        elif self._sequence_step == 1:
            # Step 2: Drive forward into shelf
            cmd = Twist()
            cmd.linear.x = 0.1  # slow approach
            self.cmd_vel_pub.publish(cmd)
            if elapsed > APPROACH_WAIT:
                self._stop()
                self._sequence_step = 2
                self._sequence_start_time = now

        elif self._sequence_step == 2:
            # Step 3: Raise fork with item
            self._publish_fork_cmd(FORK_LIFTED)
            if elapsed > FORK_MOVE_WAIT:
                self.get_logger().info("Pick complete — navigating to dock")
                self._set_state(ForkliftState.NAVIGATING_TO_DOCK)
                self._sequence_start_time = None
                self._sequence_step = 0

    # ------------------------------------------------------------------
    # Drop sequence
    # ------------------------------------------------------------------

    def _run_drop_sequence(self) -> None:
        now = time.monotonic()
        if self._sequence_start_time is None:
            self._sequence_start_time = now
            self._sequence_step = 0
            self.get_logger().info("Starting drop sequence")

        elapsed = now - self._sequence_start_time

        if self._sequence_step == 0:
            # Step 1: Lower fork
            self._publish_fork_cmd(FORK_GROUND)
            if elapsed > FORK_MOVE_WAIT:
                self._sequence_step = 1
                self._sequence_start_time = now

        elif self._sequence_step == 1:
            # Step 2: Back up
            cmd = Twist()
            cmd.linear.x = -0.1
            self.cmd_vel_pub.publish(cmd)
            if elapsed > APPROACH_WAIT:
                self._stop()
                self._sequence_step = 2
                self._sequence_start_time = now

        elif self._sequence_step == 2:
            # Step 3: Raise fork to travel height
            self._publish_fork_cmd(FORK_TRAVEL)
            if elapsed > 1.0:
                self.get_logger().info("Drop complete — task finished")
                self._publish_task_status(
                    TaskStatus.COMPLETED if HAS_WAREHOUSE_MSGS else 2
                )
                self.current_task_id = ""
                self.current_order_id = ""
                self._set_state(ForkliftState.IDLE)
                self._sequence_start_time = None
                self._sequence_step = 0

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _stop(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _publish_fork_cmd(self, position: float) -> None:
        msg = Float64()
        msg.data = position
        self.fork_cmd_pub.publish(msg)

    def _set_state(self, new_state: ForkliftState) -> None:
        if new_state != self.state:
            self.get_logger().info(f"State: {self.state.name} → {new_state.name}")
            self.state = new_state

    def _publish_task_status(self, status_code: int) -> None:
        if not HAS_WAREHOUSE_MSGS or self.task_status_pub is None:
            return
        msg = TaskStatus()
        msg.task_id = self.current_task_id
        msg.forklift_id = self.forklift_id
        msg.status = status_code
        now = self.get_clock().now().to_msg()
        msg.timestamp = now
        self.task_status_pub.publish(msg)

    def _publish_status(self) -> None:
        if not HAS_WAREHOUSE_MSGS or self.forklift_status_pub is None:
            return
        msg = ForkliftStatus()
        msg.forklift_id = self.forklift_id
        msg.state = int(self.state)
        msg.pose = self.current_pose
        msg.battery_level = self.battery_level
        msg.current_task_id = self.current_task_id
        self.forklift_status_pub.publish(msg)

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        """Convert geometry_msgs/Quaternion to yaw angle in radians."""
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ForkliftControllerNode()
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
