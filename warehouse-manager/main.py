"""Warehouse Manager — FastAPI server with ROS 2 bridge.

Provides REST + WebSocket API for the dashboard and connects to
the Isaac Sim forklift fleet via ROS 2 DDS (FastDDS discovery server).
"""

from __future__ import annotations

import asyncio
import json
import os
import uuid
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from enum import IntEnum
from typing import Any

import redis.asyncio as aioredis
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, Pose
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware

from models import init_db, get_db
from dispatcher import nearest_assign, batch_assign
from scenario import ScenarioRecorder, save_scenario, load_scenario

# ── Lazy-import warehouse_msgs (built at Docker image build time) ─────────────
# fmt: off
try:
    from warehouse_msgs.msg import ForkliftStatus as ForkliftStatusMsg
    from warehouse_msgs.msg import TaskAssignment as TaskAssignmentMsg
    from warehouse_msgs.msg import TaskStatus as TaskStatusMsg
    HAS_MSGS = True
except ImportError:
    HAS_MSGS = False
# fmt: on


# ── Enums / config ───────────────────────────────────────────────────────────

class ForkliftState(IntEnum):
    IDLE = 0
    NAVIGATING_TO_SHELF = 1
    PICKING = 2
    NAVIGATING_TO_DOCK = 3
    DROPPING = 4
    ERROR = 5
    RECOVERING = 6


FORKLIFT_IDS = [f"forklift_{i}" for i in range(4)]
REDIS_URL = os.environ.get("REDIS_URL", "redis://localhost:6379")
DISPATCH_STRATEGY = os.environ.get("DISPATCH_STRATEGY", "nearest")  # "nearest" | "batched"


# ── In-memory state ──────────────────────────────────────────────────────────

forklifts: dict[str, dict[str, Any]] = {
    fid: {
        "forklift_id": fid,
        "state": ForkliftState.IDLE,
        "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
        "battery_level": 100.0,
        "current_task_id": None,
        "paused": False,
    }
    for fid in FORKLIFT_IDS
}

ws_clients: set[WebSocket] = set()
current_strategy: str = DISPATCH_STRATEGY
recorder: ScenarioRecorder | None = None


# ── ROS 2 Node ───────────────────────────────────────────────────────────────

class WarehouseManagerNode(Node):
    def __init__(self):
        super().__init__(
            "warehouse_manager",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        if HAS_MSGS:
            # Subscribe to each forklift's status
            for fid in FORKLIFT_IDS:
                self.create_subscription(
                    ForkliftStatusMsg,
                    f"/{fid}/status",
                    lambda msg, _fid=fid: self._on_forklift_status(msg, _fid),
                    qos,
                )

            # Subscribe to task status feedback
            self.create_subscription(TaskStatusMsg, "/warehouse/task_status", self._on_task_status, qos)

            # Publisher for task assignments
            self.task_pub = self.create_publisher(TaskAssignmentMsg, "/warehouse/task_assignment", qos)
        else:
            self.get_logger().warn("warehouse_msgs not found — running without ROS 2 message types")
            self.task_pub = None

        self.get_logger().info("WarehouseManagerNode started")

    def _on_forklift_status(self, msg: Any, forklift_id: str):
        forklifts[forklift_id].update(
            {
                "state": int(msg.state),
                "pose": {
                    "x": msg.pose.position.x,
                    "y": msg.pose.position.y,
                    "z": msg.pose.position.z,
                    "yaw": 0.0,
                },
                "battery_level": msg.battery_level,
                "current_task_id": msg.current_task_id or None,
            }
        )
        asyncio.get_event_loop().call_soon_threadsafe(
            asyncio.ensure_future,
            broadcast({"type": "forklift_update", "data": forklifts[forklift_id]}),
        )

    def _on_task_status(self, msg: Any):
        asyncio.get_event_loop().call_soon_threadsafe(
            asyncio.ensure_future,
            _handle_task_status(msg.task_id, msg.status, msg.forklift_id),
        )

    def publish_task(self, task_id: str, forklift_id: str, order_id: str, shelf: tuple, dock: tuple, priority: int = 1):
        if not self.task_pub:
            return
        msg = TaskAssignmentMsg()
        msg.task_id = task_id
        msg.forklift_id = forklift_id
        msg.order_id = order_id
        msg.shelf_position = Point(x=shelf[0], y=shelf[1], z=shelf[2])
        msg.dock_position = Point(x=dock[0], y=dock[1], z=dock[2])
        msg.priority = priority
        self.task_pub.publish(msg)
        self.get_logger().info(f"Published task {task_id} → {forklift_id}")


ros_node: WarehouseManagerNode | None = None
redis_client: aioredis.Redis | None = None


# ── Helpers ──────────────────────────────────────────────────────────────────

async def broadcast(message: dict):
    """Send a JSON message to all connected WebSocket clients."""
    data = json.dumps(message, default=str)
    dead: list[WebSocket] = []
    for ws in ws_clients:
        try:
            await ws.send_text(data)
        except Exception:
            dead.append(ws)
    for ws in dead:
        ws_clients.discard(ws)


async def _handle_task_status(task_id: str, status: str, forklift_id: str):
    db = await get_db()
    try:
        completed_at = datetime.now(timezone.utc).isoformat() if status in ("completed", "failed", "cancelled") else None
        await db.execute(
            "UPDATE tasks SET status=?, completed_at=COALESCE(?, completed_at) WHERE id=?",
            (status, completed_at, task_id),
        )

        # If task completed, update order status
        if status == "completed":
            row = await db.execute("SELECT order_id FROM tasks WHERE id=?", (task_id,))
            task_row = await row.fetchone()
            if task_row:
                await db.execute(
                    "UPDATE orders SET status='completed', completed_at=? WHERE id=?",
                    (completed_at, task_row["order_id"]),
                )

        await db.commit()
    finally:
        await db.close()

    await broadcast({"type": "task_status", "data": {"task_id": task_id, "status": status, "forklift_id": forklift_id}})


def _idle_forklifts() -> list[tuple[str, tuple[float, float]]]:
    return [
        (fid, (f["pose"]["x"], f["pose"]["y"]))
        for fid, f in forklifts.items()
        if f["state"] == ForkliftState.IDLE and not f["paused"]
    ]


# ── Lifespan ─────────────────────────────────────────────────────────────────

def _spin_ros(node: Node):
    """Blocking ROS 2 spin in a background thread."""
    rclpy.spin(node)


@asynccontextmanager
async def lifespan(_app: FastAPI):
    global ros_node, redis_client

    # Init DB
    await init_db()

    # Init Redis
    redis_client = aioredis.from_url(REDIS_URL, decode_responses=True)

    # Init ROS 2
    rclpy.init()
    ros_node = WarehouseManagerNode()

    # Spin ROS 2 in a background thread
    loop = asyncio.get_event_loop()
    ros_task = loop.run_in_executor(None, _spin_ros, ros_node)

    yield

    # Cleanup
    ros_node.destroy_node()
    rclpy.shutdown()
    if redis_client:
        await redis_client.aclose()


# ── FastAPI App ──────────────────────────────────────────────────────────────

app = FastAPI(title="Warehouse Manager", version="0.1.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ── REST Endpoints ───────────────────────────────────────────────────────────

@app.get("/forklifts")
async def list_forklifts():
    return list(forklifts.values())


@app.get("/forklifts/{forklift_id}")
async def get_forklift(forklift_id: str):
    if forklift_id not in forklifts:
        raise HTTPException(404, "Forklift not found")
    return forklifts[forklift_id]


@app.put("/forklifts/{forklift_id}/pause")
async def pause_forklift(forklift_id: str):
    if forklift_id not in forklifts:
        raise HTTPException(404, "Forklift not found")
    forklifts[forklift_id]["paused"] = True
    await broadcast({"type": "forklift_update", "data": forklifts[forklift_id]})
    return {"status": "paused", "forklift_id": forklift_id}


@app.put("/forklifts/{forklift_id}/resume")
async def resume_forklift(forklift_id: str):
    if forklift_id not in forklifts:
        raise HTTPException(404, "Forklift not found")
    forklifts[forklift_id]["paused"] = False
    await broadcast({"type": "forklift_update", "data": forklifts[forklift_id]})
    return {"status": "resumed", "forklift_id": forklift_id}


@app.post("/orders")
async def create_order(body: dict):
    items = body.get("items", [])
    priority = body.get("priority", 1)
    if not items:
        raise HTTPException(400, "items required")

    order_id = str(uuid.uuid4())
    now = datetime.now(timezone.utc).isoformat()

    db = await get_db()
    try:
        await db.execute(
            "INSERT INTO orders (id, items, priority, status, created_at) VALUES (?, ?, ?, 'pending', ?)",
            (order_id, json.dumps(items), priority, now),
        )
        await db.commit()
    finally:
        await db.close()

    order = {
        "id": order_id,
        "items": items,
        "priority": priority,
        "status": "pending",
        "assigned_forklift": None,
        "created_at": now,
        "completed_at": None,
    }

    # Record if scenario recording is active
    if recorder and recorder.is_recording:
        recorder.record_order({"items": items, "priority": priority})

    await broadcast({"type": "order_update", "data": order})

    # Attempt immediate dispatch
    await _try_dispatch(order_id, items, priority)

    return order


@app.get("/orders")
async def list_orders(status: str | None = None):
    db = await get_db()
    try:
        if status:
            cursor = await db.execute("SELECT * FROM orders WHERE status=? ORDER BY created_at DESC", (status,))
        else:
            cursor = await db.execute("SELECT * FROM orders ORDER BY created_at DESC")
        rows = await cursor.fetchall()
        return [
            {
                "id": r["id"],
                "items": json.loads(r["items"]),
                "priority": r["priority"],
                "status": r["status"],
                "assigned_forklift": r["assigned_forklift"],
                "created_at": r["created_at"],
                "completed_at": r["completed_at"],
            }
            for r in rows
        ]
    finally:
        await db.close()


@app.delete("/tasks/{task_id}")
async def cancel_task(task_id: str):
    db = await get_db()
    try:
        await db.execute("UPDATE tasks SET status='cancelled', completed_at=? WHERE id=?", (datetime.now(timezone.utc).isoformat(), task_id))
        await db.commit()
    finally:
        await db.close()
    await broadcast({"type": "task_status", "data": {"task_id": task_id, "status": "cancelled", "forklift_id": ""}})
    return {"status": "cancelled", "task_id": task_id}


@app.get("/metrics")
async def get_metrics():
    db = await get_db()
    try:
        total_cur = await db.execute("SELECT COUNT(*) as cnt FROM orders")
        total = (await total_cur.fetchone())["cnt"]

        completed_cur = await db.execute("SELECT COUNT(*) as cnt FROM orders WHERE status='completed'")
        completed = (await completed_cur.fetchone())["cnt"]

        in_progress_cur = await db.execute("SELECT COUNT(*) as cnt FROM orders WHERE status='in_progress'")
        in_progress = (await in_progress_cur.fetchone())["cnt"]

        pending_cur = await db.execute("SELECT COUNT(*) as cnt FROM orders WHERE status='pending'")
        pending = (await pending_cur.fetchone())["cnt"]

        idle_count = sum(1 for f in forklifts.values() if f["state"] == ForkliftState.IDLE)

        return {
            "total_orders": total,
            "completed_orders": completed,
            "in_progress_orders": in_progress,
            "pending_orders": pending,
            "fleet_size": len(forklifts),
            "idle_forklifts": idle_count,
            "active_forklifts": len(forklifts) - idle_count,
            "dispatch_strategy": current_strategy,
        }
    finally:
        await db.close()


@app.post("/config")
async def update_config(body: dict):
    global current_strategy
    strategy = body.get("dispatch_strategy")
    if strategy and strategy in ("nearest", "batched"):
        current_strategy = strategy
    return {"dispatch_strategy": current_strategy}


@app.post("/reset")
async def reset_simulation():
    db = await get_db()
    try:
        await db.execute("UPDATE orders SET status='cancelled' WHERE status IN ('pending', 'in_progress')")
        await db.execute("UPDATE tasks SET status='cancelled' WHERE status IN ('pending', 'in_progress')")
        await db.commit()
    finally:
        await db.close()

    for fid in forklifts:
        forklifts[fid]["state"] = ForkliftState.IDLE
        forklifts[fid]["current_task_id"] = None
        forklifts[fid]["paused"] = False

    await broadcast({"type": "reset", "data": {}})
    return {"status": "reset"}


# ── Scenario replay ──────────────────────────────────────────────────────────

@app.post("/replay/start")
async def replay_start(body: dict):
    scenario_id = body.get("scenario")
    if not scenario_id:
        raise HTTPException(400, "scenario required")
    try:
        scenario = load_scenario(scenario_id)
    except FileNotFoundError:
        raise HTTPException(404, f"Scenario '{scenario_id}' not found")

    # Schedule order injection at recorded offsets
    for event in scenario.events:
        if event.type == "order":
            asyncio.get_event_loop().call_later(
                event.t_offset_ms / 1000.0,
                lambda data=event.data: asyncio.ensure_future(create_order(data)),
            )

    return {"status": "replaying", "scenario_id": scenario_id, "events": len(scenario.events)}


@app.post("/replay/stop")
async def replay_stop():
    # No active scheduled events tracking — future improvement
    return {"status": "stopped"}


@app.post("/record/start")
async def record_start(body: dict):
    global recorder
    scenario_id = body.get("scenario_id", str(uuid.uuid4()))
    recorder = ScenarioRecorder(scenario_id)
    recorder.start()
    return {"status": "recording", "scenario_id": scenario_id}


@app.post("/record/stop")
async def record_stop():
    global recorder
    if not recorder or not recorder.is_recording:
        raise HTTPException(400, "No active recording")
    scenario = recorder.stop()
    filepath = save_scenario(scenario)
    recorder = None
    return {"status": "saved", "scenario_id": scenario.scenario_id, "path": str(filepath)}


# ── Dispatch logic ───────────────────────────────────────────────────────────

# Default shelf/dock positions (will be replaced by warehouse layout config)
DEFAULT_SHELF_POS = (10.0, 5.0, 0.0)
DEFAULT_DOCK_POS = (2.0, 25.0, 0.0)


async def _try_dispatch(order_id: str, items: list, priority: int):
    """Attempt to dispatch a pending order to an idle forklift."""
    idle = _idle_forklifts()
    if not idle:
        return

    # For single-order immediate dispatch, use nearest strategy
    order_pos = DEFAULT_SHELF_POS[:2]
    assignments = nearest_assign([(order_id, order_pos)], idle)

    for forklift_id, assigned_order_id in assignments.items():
        if assigned_order_id != order_id:
            continue

        task_id = str(uuid.uuid4())
        now = datetime.now(timezone.utc).isoformat()

        db = await get_db()
        try:
            await db.execute(
                "INSERT INTO tasks (id, order_id, forklift_id, status, shelf_x, shelf_y, shelf_z, dock_x, dock_y, dock_z, created_at) "
                "VALUES (?, ?, ?, 'in_progress', ?, ?, ?, ?, ?, ?, ?)",
                (task_id, order_id, forklift_id, *DEFAULT_SHELF_POS, *DEFAULT_DOCK_POS, now),
            )
            await db.execute(
                "UPDATE orders SET status='in_progress', assigned_forklift=? WHERE id=?",
                (forklift_id, order_id),
            )
            await db.commit()
        finally:
            await db.close()

        # Publish to ROS 2
        if ros_node:
            ros_node.publish_task(task_id, forklift_id, order_id, DEFAULT_SHELF_POS, DEFAULT_DOCK_POS, priority)

        await broadcast({
            "type": "order_update",
            "data": {
                "id": order_id,
                "status": "in_progress",
                "assigned_forklift": forklift_id,
                "items": items,
                "priority": priority,
                "created_at": now,
                "completed_at": None,
            },
        })
        break


# ── WebSocket ────────────────────────────────────────────────────────────────

@app.websocket("/ws/live")
async def websocket_live(ws: WebSocket):
    await ws.accept()
    ws_clients.add(ws)
    try:
        # Send current state snapshot on connect
        await ws.send_text(json.dumps({"type": "snapshot", "data": {"forklifts": list(forklifts.values())}}, default=str))
        # Keep connection alive, handle incoming commands
        while True:
            data = await ws.receive_text()
            # WebSocket is server→client only for now; ignore client messages
    except WebSocketDisconnect:
        pass
    finally:
        ws_clients.discard(ws)
