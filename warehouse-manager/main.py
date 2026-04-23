"""
main.py — Phase 2 Warehouse Manager: FastAPI + ROS 2 service.

Architecture:
  - FastAPI runs in the main asyncio event loop
  - rclpy runs in a dedicated background thread with MultiThreadedExecutor
  - Events are bridged via asyncio.Queue (ROS → FastAPI) and thread-safe calls

Endpoints:
  POST /orders          — Create a new order
  GET  /orders          — List all orders
  GET  /forklifts       — List forklift statuses
  GET  /metrics         — Get current metrics
  POST /reset           — Reset simulation
  WS   /ws/live         — WebSocket for real-time updates
"""
from __future__ import annotations

import asyncio
import json
import math
import sqlite3
import threading
import time
import uuid
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from geometry_msgs.msg import Point

try:
    from warehouse_msgs.msg import ForkliftStatus, TaskAssignment, TaskStatus
    HAS_WAREHOUSE_MSGS = True
except ImportError:
    HAS_WAREHOUSE_MSGS = False


# ---------------------------------------------------------------------------
# QoS
# ---------------------------------------------------------------------------

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10,
)

# ---------------------------------------------------------------------------
# Database
# ---------------------------------------------------------------------------

DB_PATH = Path(__file__).parent / "warehouse.db"

_DB_SCHEMA = """
CREATE TABLE IF NOT EXISTS orders (
    id TEXT PRIMARY KEY,
    items TEXT NOT NULL,
    priority INTEGER DEFAULT 1,
    status TEXT DEFAULT 'pending',
    created_at TEXT DEFAULT (datetime('now')),
    completed_at TEXT
);
CREATE TABLE IF NOT EXISTS tasks (
    id TEXT PRIMARY KEY,
    order_id TEXT REFERENCES orders(id),
    forklift_id TEXT,
    task_type INTEGER,
    source_x REAL, source_y REAL, source_z REAL,
    dest_x REAL, dest_y REAL, dest_z REAL,
    status TEXT DEFAULT 'pending',
    created_at TEXT DEFAULT (datetime('now')),
    completed_at TEXT
);
CREATE TABLE IF NOT EXISTS task_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT REFERENCES tasks(id),
    event_type TEXT NOT NULL,
    details TEXT,
    timestamp TEXT DEFAULT (datetime('now'))
);
CREATE TABLE IF NOT EXISTS metrics_snapshots (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    orders_completed INTEGER,
    avg_fulfillment_secs REAL,
    fleet_utilization REAL,
    collision_count INTEGER,
    snapshot_at TEXT DEFAULT (datetime('now'))
);
"""


def _init_db() -> sqlite3.Connection:
    conn = sqlite3.connect(str(DB_PATH), check_same_thread=False)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA busy_timeout=5000")
    conn.row_factory = sqlite3.Row
    conn.executescript(_DB_SCHEMA)
    return conn


# ---------------------------------------------------------------------------
# Shelf / dock layout (hardcoded for Phase 2, matches scene)
# ---------------------------------------------------------------------------

SHELVES = [
    {"id": "shelf_0", "position": Point(x=5.0, y=10.0, z=0.0)},
]
DOCKS = [
    {"id": "dock_0", "position": Point(x=5.0, y=2.0, z=0.0)},
]


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class WarehouseManagerROS(Node):
    def __init__(self, event_queue: asyncio.Queue):
        super().__init__("warehouse_manager")
        self._event_queue = event_queue
        self._lock = threading.Lock()

        # Forklift state cache
        self.forklift_states: dict[str, dict[str, Any]] = {}

        if not HAS_WAREHOUSE_MSGS:
            self.get_logger().error("warehouse_msgs not available")
            return

        self.task_pub = self.create_publisher(
            TaskAssignment, "/warehouse/task_assignment", RELIABLE_QOS
        )
        self.create_subscription(
            TaskStatus, "/warehouse/task_status",
            self._task_status_cb, RELIABLE_QOS,
        )
        self.create_subscription(
            ForkliftStatus, "/forklift_0/status",
            self._forklift_status_cb, RELIABLE_QOS,
        )
        self.get_logger().info("WarehouseManagerROS node started")

    def publish_task(self, task_id: str, order_id: str, forklift_id: str,
                     task_type: int, source: Point, dest: Point,
                     priority: int) -> None:
        if not HAS_WAREHOUSE_MSGS:
            return
        msg = TaskAssignment()
        msg.task_id = task_id
        msg.order_id = order_id
        msg.forklift_id = forklift_id
        msg.task_type = task_type
        msg.source = source
        msg.destination = dest
        msg.priority = priority
        self.task_pub.publish(msg)

    def _task_status_cb(self, msg: TaskStatus) -> None:
        status_names = {0: "pending", 1: "in_progress", 2: "completed", 3: "failed"}
        event = {
            "type": "task_status",
            "data": {
                "task_id": msg.task_id,
                "forklift_id": msg.forklift_id,
                "status": status_names.get(msg.status, "unknown"),
                "status_code": msg.status,
            },
        }
        try:
            self._event_queue.put_nowait(event)
        except asyncio.QueueFull:
            pass

    def _forklift_status_cb(self, msg: ForkliftStatus) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        state_data = {
            "forklift_id": msg.forklift_id,
            "state": msg.state,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "yaw": yaw,
            "battery": msg.battery_level,
            "task_id": msg.current_task_id,
        }
        with self._lock:
            self.forklift_states[msg.forklift_id] = state_data
        event = {"type": "forklift_update", "data": state_data}
        try:
            self._event_queue.put_nowait(event)
        except asyncio.QueueFull:
            pass

    def get_forklift_states(self) -> list[dict]:
        with self._lock:
            return list(self.forklift_states.values())

    def get_idle_forklift(self) -> str | None:
        with self._lock:
            for fid, state in self.forklift_states.items():
                if state.get("state") == 0:  # IDLE
                    return fid
        return "forklift_0"  # fallback for Phase 1


# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------

ros_node: WarehouseManagerROS | None = None
event_queue: asyncio.Queue = asyncio.Queue(maxsize=1000)
db: sqlite3.Connection | None = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global ros_node, db

    db = _init_db()

    if not rclpy.ok():
        rclpy.init()
    ros_node = WarehouseManagerROS(event_queue)
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    yield

    ros_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if db:
        db.close()


app = FastAPI(title="Warehouse Digital Twin", version="0.2.0", lifespan=lifespan)


# ---------------------------------------------------------------------------
# Pydantic models
# ---------------------------------------------------------------------------

class OrderCreate(BaseModel):
    items: list[str]
    priority: int = 1


# ---------------------------------------------------------------------------
# REST endpoints
# ---------------------------------------------------------------------------

@app.post("/orders")
async def create_order(order: OrderCreate):
    order_id = str(uuid.uuid4())
    items_json = json.dumps(order.items)

    db.execute(
        "INSERT INTO orders (id, items, priority) VALUES (?, ?, ?)",
        (order_id, items_json, order.priority),
    )
    db.commit()

    # Simple dispatcher: pick from first shelf, deliver to first dock
    task_id = str(uuid.uuid4())
    source = SHELVES[0]["position"]
    dest = DOCKS[0]["position"]
    forklift_id = ros_node.get_idle_forklift() if ros_node else "forklift_0"

    db.execute(
        "INSERT INTO tasks (id, order_id, forklift_id, task_type, "
        "source_x, source_y, source_z, dest_x, dest_y, dest_z) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
        (task_id, order_id, forklift_id, 0,
         source.x, source.y, source.z, dest.x, dest.y, dest.z),
    )
    db.commit()

    # Publish via ROS 2
    if ros_node:
        ros_node.publish_task(
            task_id=task_id,
            order_id=order_id,
            forklift_id=forklift_id,
            task_type=0,
            source=source,
            dest=dest,
            priority=order.priority,
        )

    return JSONResponse(
        {"order_id": order_id, "task_id": task_id, "forklift_id": forklift_id},
        status_code=201,
    )


@app.get("/orders")
async def list_orders():
    rows = db.execute("SELECT * FROM orders ORDER BY created_at DESC").fetchall()
    return [dict(r) for r in rows]


@app.get("/forklifts")
async def list_forklifts():
    if ros_node:
        return ros_node.get_forklift_states()
    return []


@app.get("/metrics")
async def get_metrics():
    completed = db.execute(
        "SELECT COUNT(*) as c FROM orders WHERE status='completed'"
    ).fetchone()["c"]
    total = db.execute("SELECT COUNT(*) as c FROM orders").fetchone()["c"]
    return {
        "orders_completed": completed,
        "orders_total": total,
        "fleet_utilization": 0.0,  # TODO: compute from forklift states
    }


@app.post("/reset")
async def reset_simulation():
    db.execute("UPDATE tasks SET status='cancelled' WHERE status IN ('pending','in_progress')")
    db.execute("UPDATE orders SET status='cancelled' WHERE status IN ('pending','in_progress')")
    db.commit()
    return {"success": True, "message": "Reset complete"}


@app.websocket("/ws/live")
async def websocket_live(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            try:
                event = await asyncio.wait_for(event_queue.get(), timeout=1.0)
                await websocket.send_json(event)
            except asyncio.TimeoutError:
                # Send heartbeat
                await websocket.send_json({"type": "heartbeat"})
    except WebSocketDisconnect:
        pass


# ---------------------------------------------------------------------------
# Event processor: update DB based on ROS events
# ---------------------------------------------------------------------------

async def _process_events():
    """Background task that drains the event queue and updates the DB."""
    while True:
        event = await event_queue.get()
        if event["type"] == "task_status":
            data = event["data"]
            status = data["status"]
            task_id = data["task_id"]
            completed_at = (
                "datetime('now')" if status == "completed" else None
            )
            db.execute(
                "UPDATE tasks SET status=? WHERE id=?",
                (status, task_id),
            )
            if status == "completed":
                db.execute(
                    "UPDATE tasks SET completed_at=datetime('now') WHERE id=?",
                    (task_id,),
                )
                # Also update the parent order
                row = db.execute(
                    "SELECT order_id FROM tasks WHERE id=?", (task_id,)
                ).fetchone()
                if row:
                    db.execute(
                        "UPDATE orders SET status='completed', "
                        "completed_at=datetime('now') WHERE id=?",
                        (row["order_id"],),
                    )
            db.commit()
