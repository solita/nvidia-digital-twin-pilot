#!/usr/bin/env python3
"""
obstacle_avoidance_node.py — ROS2 node for Jetbot obstacle avoidance.

Subscribes to /scan (LaserScan), publishes /cmd_vel (Twist).
Uses the same gap-based steering logic as hello_robot.py but over ROS2 topics.

Run inside the ROS2 sidecar:
  source /opt/ros/humble/setup.bash
  python3 /ros2_ws/obstacle_avoidance_node.py
"""

import math
import random
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('ray_range', 3.0)
        self.declare_parameter('clear_fraction', 0.7)
        self.declare_parameter('slow_fraction', 0.3)
        self.declare_parameter('steer_lp', 0.15)
        self.declare_parameter('control_hz', 10.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.ray_range = self.get_parameter('ray_range').value
        self.clear_frac = self.get_parameter('clear_fraction').value
        self.slow_frac = self.get_parameter('slow_fraction').value
        self.steer_lp_coeff = self.get_parameter('steer_lp').value
        hz = self.get_parameter('control_hz').value

        # State
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.steer_filtered = 0.0
        self.t0 = time.monotonic()

        # Randomised gain oscillation — makes steering sensitivity vary over time
        # so the robot doesn't converge to the same route every run
        self._gain_osc_phase = random.uniform(0, 2 * math.pi)
        self._gain_osc_freq1 = random.uniform(0.04, 0.09)
        self._gain_osc_freq2 = random.uniform(0.11, 0.22)

        # Hysteresis on clear-threshold: once the robot decides "clear ahead"
        # it takes a slightly higher obstacle reading to switch back to steering,
        # and vice-versa.  The band width drifts slowly over time.
        self._steering_engaged = False

        # ROS2 I/O
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0 / hz, self.control_loop)

        self.get_logger().info(
            f'Obstacle avoidance started: speed={self.linear_speed}, '
            f'range={self.ray_range}, hz={hz}')

    def scan_cb(self, msg: LaserScan):
        self.ranges = list(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def control_loop(self):
        cmd = Twist()

        if not self.ranges:
            # No scan data yet — stop
            self.pub_cmd.publish(cmd)
            return

        n = len(self.ranges)
        # Clamp ranges to ray_range (replace inf/nan)
        dists = []
        for r in self.ranges:
            if math.isnan(r) or math.isinf(r) or r > self.ray_range:
                dists.append(self.ray_range)
            else:
                dists.append(r)

        # Only use forward 180° of scan
        # Find indices for -90° to +90°
        fwd_indices = []
        for i in range(n):
            angle = self.angle_min + i * self.angle_increment
            if -math.pi / 2 <= angle <= math.pi / 2:
                fwd_indices.append(i)

        if not fwd_indices:
            fwd_indices = list(range(n))

        fwd_dists = [dists[i] for i in fwd_indices]
        fwd_n = len(fwd_dists)
        center_idx = fwd_n // 2

        # Front distance (centre ray)
        front_dist = fwd_dists[center_idx] if fwd_dists else self.ray_range

        # Find best gap (widest contiguous clear arc)
        clear_thresh = self.ray_range * self.clear_frac
        best_start, best_len = 0, 0
        cur_start, cur_len = 0, 0
        for i in range(fwd_n):
            if fwd_dists[i] > clear_thresh:
                if cur_len == 0:
                    cur_start = i
                cur_len += 1
                if cur_len > best_len:
                    best_len = cur_len
                    best_start = cur_start
            else:
                cur_len = 0

        gap_centre = best_start + best_len / 2.0

        # Steering angle: gap centre relative to forward
        steer_angle = (gap_centre - center_idx) / max(fwd_n / 2.0, 1.0)

        # Urgency based on closest obstacle
        min_dist = min(fwd_dists) if fwd_dists else self.ray_range
        urgency = max(0.0, 1.0 - min_dist / self.ray_range)

        # Variable gain: gentle when clear, aggressive when close
        # Oscillating gain offset so sensitivity drifts over time (±0.25)
        t = time.monotonic() - self.t0
        gain_oscillation = (0.15 * math.sin(t * self._gain_osc_freq1 + self._gain_osc_phase)
                            + 0.10 * math.sin(t * self._gain_osc_freq2))
        gain = 0.15 + urgency * 1.85 + gain_oscillation
        steer_cmd = max(-1.0, min(1.0, steer_angle * gain))

        # Hysteresis on clear-threshold: once 'steering engaged' the robot
        # keeps steering until the path is clearly open (higher bar).
        # Conversely, once cruising straight, a slightly closer obstacle
        # is needed to trigger steering.  The hysteresis band drifts slowly
        # so the switch-over points change across runs.
        hyst_band = 0.08 + 0.05 * math.sin(t * 0.03 + 1.0)
        if self._steering_engaged:
            # Need a wider gap to disengage
            if min_dist > self.ray_range * (self.clear_frac + hyst_band):
                self._steering_engaged = False
        else:
            # Need to get close enough to engage
            if min_dist < self.ray_range * (self.clear_frac - hyst_band):
                self._steering_engaged = True

        # Dampen steering when hysteresis says "cruising straight"
        if not self._steering_engaged:
            steer_cmd *= 0.4

        # Slow sinusoidal heading drift so paths vary over time
        drift = 0.20 * math.sin(t * 0.07) + 0.12 * math.sin(t * 0.17)
        steer_cmd = max(-1.0, min(1.0, steer_cmd + drift))

        # Random jitter on turns — stronger when already steering
        jitter_scale = 0.03 + 0.08 * abs(steer_cmd)
        steer_cmd += random.gauss(0.0, jitter_scale)

        # Low-pass filter
        lp = self.steer_lp_coeff + urgency * 0.25
        self.steer_filtered += lp * (steer_cmd - self.steer_filtered)

        # Output velocities
        cmd.angular.z = float(self.steer_filtered * self.max_angular)

        # Speed: full when clear, slow when close
        if front_dist < self.ray_range * self.slow_frac:
            speed_factor = max(0.3, front_dist / (self.ray_range * self.slow_frac))
        else:
            speed_factor = 1.0
        cmd.linear.x = float(self.linear_speed * speed_factor)

        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
