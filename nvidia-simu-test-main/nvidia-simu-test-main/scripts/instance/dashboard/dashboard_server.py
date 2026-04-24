#!/usr/bin/env python3
"""
dashboard_server.py — Real-time web dashboard for Isaac Sim + ROS2 system.
Monitors Docker containers, system resources, and ROS2 robot data.
Serves a WebSocket-powered UI on port 8080.
"""

import asyncio
import base64
import json
import math
import os
import subprocess
import threading
import time
from pathlib import Path

from aiohttp import web

# ROS2 imports (graceful fallback if not available)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, CompressedImage
    from geometry_msgs.msg import Twist
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String as StringMsg
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

# ── Shared state ───────────────────────────────────────────────────────────────
_lock = threading.Lock()
_state = {
    "system": {},
    "containers": [],
    "robot": {
        "position": [0, 0, 0],
        "orientation": [0, 0, 0, 1],
        "linear_vel": [0, 0, 0],
        "angular_vel": [0, 0, 0],
        "cmd_linear": 0.0,
        "cmd_angular": 0.0,
        "scan_ranges": [],
        "scan_angle_min": 0.0,
        "scan_angle_increment": 0.0,
        "odom_age": 999,
        "scan_age": 999,
        "cmd_age": 999,
        "clock_age": 999,
        "sim_time": 0.0,
    },
    "topics": {},
}
_last_odom_t = 0.0
_last_scan_t = 0.0
_last_cmd_t = 0.0
_last_clock_t = 0.0
_camera_b64 = ""       # latest camera JPEG as base64 string
_last_camera_t = 0.0
_detections_ml = {}    # latest parsed ML detections dict
_last_detections_ml_t = 0.0

# Per-topic message counters and timestamps for rate calculation
_topic_stats: dict[str, dict] = {}
_topic_stats_lock = threading.Lock()

def _track_topic(name: str, msg_type: str):
    now = time.time()
    with _topic_stats_lock:
        if name not in _topic_stats:
            _topic_stats[name] = {"type": msg_type, "count": 0, "first_t": now, "last_t": now, "hz": 0.0}
        s = _topic_stats[name]
        s["count"] += 1
        s["last_t"] = now
        elapsed = now - s["first_t"]
        if elapsed > 1.0:
            s["hz"] = round(s["count"] / elapsed, 1)
_clients: set[web.WebSocketResponse] = set()


# ── System monitoring helpers ──────────────────────────────────────────────────
def _get_containers():
    try:
        r = subprocess.run(
            ["docker", "ps", "-a", "--format",
             "{{.Names}}\t{{.Status}}\t{{.Image}}\t{{.Ports}}\t{{.CreatedAt}}"],
            capture_output=True, text=True, timeout=5,
        )
        out = []
        for line in r.stdout.strip().splitlines():
            parts = line.split("\t")
            if len(parts) >= 3:
                out.append({
                    "name": parts[0],
                    "status": parts[1] if len(parts) > 1 else "",
                    "image": parts[2] if len(parts) > 2 else "",
                    "ports": parts[3] if len(parts) > 3 else "",
                    "created": parts[4] if len(parts) > 4 else "",
                })
        return out
    except Exception as exc:
        return [{"name": "error", "status": str(exc), "image": "", "ports": "", "created": ""}]


def _get_system():
    info = {}
    try:
        r = subprocess.run(
            ["nvidia-smi",
             "--query-gpu=name,temperature.gpu,utilization.gpu,memory.used,memory.total",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=5,
        )
        if r.returncode == 0:
            p = [x.strip() for x in r.stdout.strip().split(",")]
            if len(p) >= 5:
                info["gpu"] = {
                    "name": p[0], "temp": int(p[1]), "util": int(p[2]),
                    "mem_used": int(p[3]), "mem_total": int(p[4]),
                }
    except Exception:
        pass
    try:
        with open("/proc/loadavg") as f:
            info["load"] = f.read().strip().split()[:3]
    except Exception:
        pass
    try:
        r = subprocess.run(["free", "-m"], capture_output=True, text=True, timeout=5)
        for line in r.stdout.splitlines():
            if line.startswith("Mem:"):
                p = line.split()
                info["mem"] = {"total": int(p[1]), "used": int(p[2])}
    except Exception:
        pass
    try:
        r = subprocess.run(["df", "-h", "/"], capture_output=True, text=True, timeout=5)
        lines = r.stdout.strip().splitlines()
        if len(lines) > 1:
            p = lines[1].split()
            info["disk"] = {"total": p[1], "used": p[2], "pct": p[4]}
    except Exception:
        pass
    try:
        r = subprocess.run(["uptime", "-s"], capture_output=True, text=True, timeout=5)
        info["uptime"] = r.stdout.strip()
    except Exception:
        pass
    return info


def _get_logs(container: str, tail: int = 100) -> str:
    try:
        r = subprocess.run(
            ["docker", "logs", "--tail", str(tail), "--timestamps", container],
            capture_output=True, text=True, timeout=5,
        )
        text = r.stdout + r.stderr
        return text[-8000:]
    except Exception as exc:
        return f"Error fetching logs: {exc}"


# ── ROS2 subscriber ───────────────────────────────────────────────────────────
if HAS_ROS2:
    class _ROS2Node(Node):
        def __init__(self):
            super().__init__("dashboard")
            # Isaac Sim ROS2 bridge publishes BEST_EFFORT — subscribers must match
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.create_subscription(Odometry, "/odom", self._odom, sensor_qos)
            self.create_subscription(LaserScan, "/scan", self._scan, sensor_qos)
            self.create_subscription(Twist, "/cmd_vel", self._cmd, sensor_qos)
            self.create_subscription(Clock, "/clock", self._on_clock, sensor_qos)
            self.create_subscription(
                CompressedImage, "/camera/image/compressed", self._camera,
                QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                ),
            )
            self.create_subscription(
                StringMsg, "/camera/detections_ml", self._detections_ml,
                QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                ),
            )
            self.get_logger().info("Dashboard ROS2 subscriptions active (BEST_EFFORT QoS)")

        def _odom(self, m):
            global _last_odom_t
            if not _last_odom_t:
                self.get_logger().info("First /odom received")
            _track_topic("/odom", "nav_msgs/Odometry")
            p = m.pose.pose.position
            o = m.pose.pose.orientation
            lv = m.twist.twist.linear
            av = m.twist.twist.angular
            with _lock:
                _state["robot"]["position"] = [round(p.x, 4), round(p.y, 4), round(p.z, 4)]
                _state["robot"]["orientation"] = [round(o.x, 4), round(o.y, 4), round(o.z, 4), round(o.w, 4)]
                _state["robot"]["linear_vel"] = [round(lv.x, 3), round(lv.y, 3), round(lv.z, 3)]
                _state["robot"]["angular_vel"] = [round(av.x, 3), round(av.y, 3), round(av.z, 3)]
            _last_odom_t = time.time()

        def _scan(self, m):
            global _last_scan_t
            if not _last_scan_t:
                self.get_logger().info(f"First /scan received ({len(m.ranges)} rays)")
            _track_topic("/scan", "sensor_msgs/LaserScan")
            ranges = list(m.ranges)
            step = max(1, len(ranges) // 180)
            sampled = [
                round(r, 3) if not (math.isnan(r) or math.isinf(r)) else 0.0
                for r in ranges[::step]
            ]
            with _lock:
                _state["robot"]["scan_ranges"] = sampled
                _state["robot"]["scan_angle_min"] = round(m.angle_min, 4)
                _state["robot"]["scan_angle_increment"] = round(m.angle_increment * step, 4)
            _last_scan_t = time.time()

        def _cmd(self, m):
            global _last_cmd_t
            _track_topic("/cmd_vel", "geometry_msgs/Twist")
            with _lock:
                _state["robot"]["cmd_linear"] = round(m.linear.x, 3)
                _state["robot"]["cmd_angular"] = round(m.angular.z, 3)
            _last_cmd_t = time.time()

        def _on_clock(self, m):
            global _last_clock_t
            if not _last_clock_t:
                self.get_logger().info("First /clock received")
            _track_topic("/clock", "rosgraph_msgs/Clock")
            sim_t = m.clock.sec + m.clock.nanosec * 1e-9
            with _lock:
                _state["robot"]["sim_time"] = round(sim_t, 3)
            _last_clock_t = time.time()

        def _camera(self, m):
            global _camera_b64, _last_camera_t
            if not _last_camera_t:
                self.get_logger().info(f"First /camera/image/compressed received ({len(m.data)} bytes)")
            _track_topic("/camera/image/compressed", "sensor_msgs/CompressedImage")
            _camera_b64 = base64.b64encode(bytes(m.data)).decode("ascii")
            _last_camera_t = time.time()

        def _detections_ml(self, m):
            global _detections_ml, _last_detections_ml_t
            if not _last_detections_ml_t:
                self.get_logger().info("First /camera/detections_ml received")
            _track_topic("/camera/detections_ml", "std_msgs/String")
            try:
                _detections_ml = json.loads(m.data)
            except (json.JSONDecodeError, TypeError):
                pass
            _last_detections_ml_t = time.time()

    def _ros2_spin():
        rclpy.init()
        node = _ROS2Node()
        try:
            rclpy.spin(node)
        except Exception as exc:
            print(f"[ros2_spin] error: {exc}", flush=True)
            import traceback; traceback.print_exc()
        finally:
            node.destroy_node()
            rclpy.shutdown()


# ── Web handlers ───────────────────────────────────────────────────────────────
async def _index(request):
    return web.FileResponse(Path(__file__).parent / "index.html")


async def _ws_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    _clients.add(ws)
    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                try:
                    data = json.loads(msg.data)
                    if data.get("action") == "get_logs":
                        name = data.get("container", "")
                        if name and all(c.isalnum() or c in "-_." for c in name):
                            logs = await asyncio.get_event_loop().run_in_executor(
                                None, _get_logs, name
                            )
                            await ws.send_json({
                                "type": "logs", "container": name, "data": logs,
                            })
                except (json.JSONDecodeError, TypeError):
                    pass
            elif msg.type in (web.WSMsgType.ERROR, web.WSMsgType.CLOSE):
                break
    finally:
        _clients.discard(ws)
    return ws


async def _broadcaster(app):
    """Push robot data fast (~4 Hz), system/container info slow (~0.3 Hz)."""
    global _clients
    print("[broadcaster] started", flush=True)
    FAST_INTERVAL = 0.25       # robot data every 250 ms
    SLOW_EVERY = 12            # system/containers every 12 fast ticks (~3 s)
    tick = 0
    containers = []
    system = {}
    try:
        while True:
            try:
                loop = asyncio.get_event_loop()

                # Refresh heavy subprocess data infrequently
                if tick % SLOW_EVERY == 0:
                    containers = await loop.run_in_executor(None, _get_containers)
                    system = await loop.run_in_executor(None, _get_system)
                tick += 1

                now = time.time()
                with _lock:
                    _state["containers"] = containers
                    _state["system"] = system
                    _state["robot"]["odom_age"] = round(now - _last_odom_t, 1) if _last_odom_t else 999
                    _state["robot"]["scan_age"] = round(now - _last_scan_t, 1) if _last_scan_t else 999
                    _state["robot"]["cmd_age"] = round(now - _last_cmd_t, 1) if _last_cmd_t else 999
                    _state["robot"]["clock_age"] = round(now - _last_clock_t, 1) if _last_clock_t else 999
                with _topic_stats_lock:
                    topics_snap = {}
                    for name, s in _topic_stats.items():
                        age = round(now - s["last_t"], 1)
                        topics_snap[name] = {
                            "type": s["type"], "count": s["count"],
                            "hz": s["hz"], "age": age,
                        }
                with _lock:
                    _state["topics"] = topics_snap
                    payload_dict = {"type": "state", **_state}
                    # Include camera image every 2nd tick (~2 FPS) to limit bandwidth
                    if tick % 2 == 0 and _camera_b64:
                        payload_dict["camera_rgb"] = _camera_b64
                    if _detections_ml:
                        payload_dict["detections_ml"] = _detections_ml
                    payload = json.dumps(payload_dict)

                dead = set()
                for ws in list(_clients):
                    try:
                        await ws.send_str(payload)
                    except Exception:
                        dead.add(ws)
                _clients -= dead
            except Exception as exc:
                print(f"[broadcaster] error: {exc}", flush=True)
                import traceback; traceback.print_exc()
            await asyncio.sleep(FAST_INTERVAL)
    except asyncio.CancelledError:
        pass


async def _on_startup(app):
    app["_task"] = asyncio.create_task(_broadcaster(app))


async def _on_cleanup(app):
    app["_task"].cancel()
    try:
        await app["_task"]
    except asyncio.CancelledError:
        pass


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    if HAS_ROS2:
        threading.Thread(target=_ros2_spin, daemon=True).start()
        print("[dashboard] ROS2 monitoring enabled")
    else:
        print("[dashboard] ROS2 not available — robot data disabled")

    app = web.Application()
    app.router.add_get("/", _index)
    app.router.add_get("/ws", _ws_handler)
    app.on_startup.append(_on_startup)
    app.on_cleanup.append(_on_cleanup)

    port = int(os.environ.get("DASHBOARD_PORT", "8080"))
    print(f"[dashboard] http://0.0.0.0:{port}")
    web.run_app(app, host="0.0.0.0", port=port)


if __name__ == "__main__":
    main()
