"""
multi_forklift.launch.py — Launch file for multiple forklift controllers.

Phase 1: Just the custom controller nodes (no Nav2 yet).
Phase 3: Will add Nav2 planner, controller, BT navigator per forklift.
"""
import os

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    num_forklifts = int(os.environ.get("NUM_FORKLIFTS", "1"))

    ld = LaunchDescription()

    for i in range(num_forklifts):
        ns = f"forklift_{i}"
        group = GroupAction(
            [
                PushRosNamespace(ns),
                Node(
                    package="vehicle_controller",
                    executable="forklift_controller_node",
                    name="controller",
                    parameters=[
                        {"forklift_id": ns, "use_sim_time": True},
                    ],
                    output="screen",
                ),
            ]
        )
        ld.add_action(group)

    return ld
