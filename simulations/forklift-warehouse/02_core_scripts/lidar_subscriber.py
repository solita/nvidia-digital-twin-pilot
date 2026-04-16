"""
lidar_subscriber.py — Reads LIDAR data from Isaac Sim and forwards it to the dashboard.

Runs as a separate process alongside Isaac Sim. Connects to the ROS 2 bridge
(or Isaac Sim's native sensor API) and publishes processed stats (obstacle
distances, forklift position, speed) to the dashboard.

"""
from __future__ import annotations

# TODO: choose data transport — ROS 2 topic or Isaac Sim ZMQ bridge
# TODO: subscribe to /lidar/point_cloud topic
# TODO: compute min obstacle distance, publish to dashboard


def main() -> None:
    raise NotImplementedError("lidar_subscriber not yet implemented — LIDAR sensor rig needed first")
