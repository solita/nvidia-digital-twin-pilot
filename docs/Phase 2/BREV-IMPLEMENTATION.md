# Brev Instance — Implementation Plan

> **Scope**: Everything that runs on the Brev GPU instance.
> **Phases 1–3**: All components (Isaac Sim, vehicle controller, warehouse manager, Redis) run here on one machine.
> **Phase 4+**: Only Isaac Sim, vehicle controller, FastDDS discovery server. Warehouse manager moves to local machine.

---

## 0. Prerequisites & Environment Setup

### 0.1: Provision Brev Instance

- GPU: A10G (24 GB VRAM) minimum, L40 (48 GB) preferred for multi-forklift scenes
- OS: Ubuntu 22.04 (matches ROS 2 Humble)
- Disk: 100 GB minimum (Isaac Sim images are ~20 GB, shader cache ~5 GB)
- NVIDIA driver: **550+** required for Isaac Sim 5.x containers

Verify GPU access:

```bash
nvidia-smi
# Must show driver version ≥550 and GPU listed
```

### 0.2: Install Docker + NVIDIA Container Toolkit

```bash
# Docker
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
newgrp docker

# NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Verify:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
# Must show the GPU inside the container
```

### 0.3: Install Docker Compose v2

```bash
sudo apt-get install -y docker-compose-plugin
docker compose version
# Must show v2.x
```

### 0.4: Install ROS 2 Humble (host — for debugging)

Needed for `ros2 topic list`, `ros2 topic echo`, etc. during development. Production runs inside containers.

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 0.5: Install Tailscale (needed from Phase 4)

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
# Sign in with the SAME Tailscale account used on your local machine
# Note the Tailscale IP — local machine will need this
tailscale ip -4
```

Save the Brev Tailscale IP in your project `.env` file (create it if it doesn't exist):

```bash
# In the repo root
echo "BREV_TAILSCALE_IP=$(tailscale ip -4)" >> .env
```

Verify connectivity from your **local machine**:

```bash
ping <brev-tailscale-ip>
```

### 0.6: Clone Monorepo

```bash
cd ~
git clone <REPO_URL> warehouse-sim
cd warehouse-sim
```

### 0.7: Pull Isaac Sim Container Image (slow — do early)

```bash
# Authenticate with NGC
docker login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key from https://ngc.nvidia.com/setup>

docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

**⚠ GOTCHA**: First pull is ~20 GB. First run compiles shaders (~2-5 min). To pre-warm:

```bash
docker run --rm --gpus all --network host \
  -e ACCEPT_EULA=Y \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  ./runheadless.sh -v
# Wait until you see "Isaac Sim Ready" then Ctrl+C
# Shader cache is now in the Docker layer cache — won't help across runs
# For persistent cache, mount a volume (see compose file below)
```

### 0.8: Verify Isaac Sim ROS 2 Bridge

Isaac Sim 5.x has a built-in ROS 2 bridge (Humble). Verify it works:

```bash
# Terminal 1: Run Isaac Sim
docker run --rm --gpus all --network host \
  -e ACCEPT_EULA=Y \
  -e ROS_DOMAIN_ID=42 \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  ./runheadless.sh

# Terminal 2: Check topics from host
export ROS_DOMAIN_ID=42
ros2 topic list
# Should show /clock at minimum once a scene with ROS bridge is loaded
```

---

## 1. Phase 1 — Minimal Loop with ROS 2 (Everything on Brev)

### 1.1: Project Scaffolding

Create the monorepo structure (from the cloned repo root):

```bash
cd ~/warehouse-sim
mkdir -p warehouse_msgs/{msg,srv}
mkdir -p sim-scripts
mkdir -p vehicle-controller
mkdir -p warehouse-manager
mkdir -p dashboard  # placeholder for Phase 5
```

Create `warehouse_msgs/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>warehouse_msgs</name>
  <version>0.1.0</version>
  <description>Warehouse Digital Twin message definitions</description>
  <maintainer email="dev@example.com">dev</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

Create `warehouse_msgs/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(warehouse_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskAssignment.msg"
  "msg/TaskStatus.msg"
  "msg/ForkliftStatus.msg"
  "srv/ResetSimulation.srv"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_package()
```

### 1.2: Define Messages

`warehouse_msgs/msg/TaskAssignment.msg`:

```
string task_id
string order_id
string forklift_id
uint8 task_type
geometry_msgs/Point source
geometry_msgs/Point destination
uint8 priority

uint8 PICK=0
uint8 DELIVER=1
```

`warehouse_msgs/msg/TaskStatus.msg`:

```
string task_id
string forklift_id
uint8 status
string error_message
builtin_interfaces/Time timestamp

uint8 PENDING=0
uint8 IN_PROGRESS=1
uint8 COMPLETED=2
uint8 FAILED=3
```

`warehouse_msgs/msg/ForkliftStatus.msg`:

```
string forklift_id
uint8 state
geometry_msgs/Pose pose
float32 battery_level
string current_task_id

uint8 IDLE=0
uint8 NAVIGATING_TO_SHELF=1
uint8 PICKING=2
uint8 NAVIGATING_TO_DOCK=3
uint8 DROPPING=4
uint8 ERROR=5
uint8 RECOVERING=6
```

`warehouse_msgs/srv/ResetSimulation.srv`:

```
bool clear_orders
---
bool success
string message
```

### 1.3: Build warehouse_msgs

Create colcon workspace structure:

```bash
cd ~/warehouse-sim
mkdir -p ros2_ws/src
ln -s ~/warehouse-sim/warehouse_msgs ros2_ws/src/warehouse_msgs

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select warehouse_msgs
source install/setup.bash

# Verify
ros2 interface show warehouse_msgs/msg/TaskAssignment
```

**⚠ GOTCHA**: colcon must be run from `ros2_ws/`, not from the monorepo root. The symlink approach keeps the source in the monorepo while colcon finds it.

### 1.4: Isaac Sim Scene Setup

Create `sim-scripts/warehouse_scene.py`. This is an Isaac Sim standalone script.

**Key implementation details**:

- **Coordinate system**: Isaac Sim uses meters, Y-up by default but robotics convention is Z-up. Force Z-up in the USD stage settings.
- **Floor**: Use `isaacsim.core.prims.create_prim("/World/Floor", "Plane")` scaled to 50m × 30m, or use a cube with `scale=(50, 30, 0.01)`.
- **Shelf rack**: Create as a rigid body with colliders. Simple box geometry (2m wide × 0.5m deep × 2m tall per slot, 3 slots = 6m wide). Position at a known coordinate (e.g., `x=5, y=10, z=0`).
- **Loading dock**: Floor-level marker at `x=5, y=2, z=0`. Use a visual prim (no physics needed — just a waypoint).
- **Forklift**:
  - Articulated body with chassis, 4 wheel joints (revolute, axis=Y for forward wheels), 1 fork prismatic joint (axis=Z, range 0–1.5m)
  - `isaacsim.robot.surface_gripper` on fork tips — set `grip_threshold=0.02`, `force_limit=1000.0`
  - Differential drive: apply velocity to left/right wheel pairs from `cmd_vel` Twist messages
- **Box (pick item)**: Rigid body, 0.4m cube, mass=5kg, placed on shelf slot 0 at height matching fork level

**ROS 2 Bridge setup** (in the script, using Isaac Sim's OmniGraph or extension API):

```python
# Key bridge components to configure:
# 1. Clock publisher: /clock (publishes sim time)
# 2. Twist subscriber: /forklift_0/cmd_vel → differential drive
# 3. Float64 subscriber: /forklift_0/fork_cmd → prismatic joint target
# 4. Odometry publisher: /forklift_0/odom (from chassis position/orientation)
# 5. JointState publisher: /forklift_0/joint_states (fork position feedback)
```

**PhysX settings**:

```python
# In the USD scene or via script:
physics_scene = UsdPhysics.Scene.Get(stage, "/physicsScene")
# Solver: TGS (Temporal Gauss-Seidel) — better for articulations
# Position iterations: 8
# Velocity iterations: 4
# Timestep: 1/60s (0.01667)
# Gravity: (0, 0, -9.81) for Z-up
```

**Launch headless**:

```bash
cd ~/warehouse-sim
docker run --rm --gpus all --network host \
  -e ACCEPT_EULA=Y \
  -e ROS_DOMAIN_ID=42 \
  -v $(pwd)/sim-scripts:/sim-scripts \
  -v isaac-sim-cache:/isaac-sim/kit/cache \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  ./python.sh /sim-scripts/warehouse_scene.py --headless
```

**⚠ GOTCHA**: The `--headless` flag disables the GUI. For debugging, use livestream instead:

```bash
# With livestream (view from browser at http://<brev-ip>:49100)
docker run --rm --gpus all --network host \
  -e ACCEPT_EULA=Y \
  -e ROS_DOMAIN_ID=42 \
  -v $(pwd)/sim-scripts:/sim-scripts \
  -v isaac-sim-cache:/isaac-sim/kit/cache \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  ./python.sh /sim-scripts/warehouse_scene.py --livestream
```

**⚠ GOTCHA**: Isaac Sim's `python.sh` uses its own embedded Python, NOT the system Python. All imports must be available in Isaac Sim's environment. `rclpy` is available because Isaac Sim 5.x ships with ROS 2 Humble support.

### 1.5: Vehicle Controller Node

Create `vehicle-controller/forklift_controller_node.py`:

**Key implementation details**:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
# After building warehouse_msgs:
from warehouse_msgs.msg import TaskAssignment, TaskStatus, ForkliftStatus
import math
import enum

class ForkliftState(enum.IntEnum):
    IDLE = 0
    NAVIGATING_TO_SHELF = 1
    PICKING = 2
    NAVIGATING_TO_DOCK = 3
    DROPPING = 4
    ERROR = 5
    RECOVERING = 6
```

**State machine critical details**:

- **Proportional control** (Phase 1 only, replaced by Nav2 in Phase 3):
  - Linear velocity: `v = Kp_lin * distance_to_goal` clamped to `[0, 0.5]` m/s
  - Angular velocity: `w = Kp_ang * angle_error` clamped to `[-1.0, 1.0]` rad/s
  - Goal reached threshold: distance < 0.3m
  - **⚠ GOTCHA**: Publish `cmd_vel` at a fixed rate (10 Hz timer), NOT in the odom callback. Odom callback rate depends on sim performance and can cause jerky motion.

- **Pick sequence** (timed, not sensor-based in Phase 1):
  1. Publish `fork_cmd = 0.0` (lower fork to ground level) — wait 2s
  2. Drive forward 0.3m (into shelf) — wait 2s
  3. Publish `fork_cmd = 0.8` (raise fork with item) — wait 2s
  4. Verify: item should be on forks (no sensor check in Phase 1 — trust the physics)

- **Drop sequence**: reverse of pick
  1. Publish `fork_cmd = 0.0` (lower fork) — wait 2s
  2. Drive backward 0.3m — wait 2s
  3. Publish `fork_cmd = 0.5` (raise fork to travel height) — wait 1s

**QoS profiles**:

```python
# For task assignments — must not lose messages
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

# For cmd_vel, odom — latest value matters, not history
BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1
)
```

**⚠ GOTCHA**: The `TRANSIENT_LOCAL` durability on task assignment means late-joining subscribers get the last published message. This is intentional — if the controller restarts, it picks up the last task. But it also means you must explicitly clear/overwrite old tasks on reset.

**Parameter**: `use_sim_time=true` — set via `--ros-args` on the command line or in the launch file `parameters=` list:

```python
# In launch file:
Node(
    ...
    parameters=[{'use_sim_time': True}],
)

# Or on command line:
# ros2 run vehicle_controller forklift_controller_node --ros-args -p use_sim_time:=true
```

**⚠ GOTCHA**: Do NOT call `self.declare_parameter('use_sim_time', True)` in the node constructor. The `use_sim_time` parameter is automatically declared by `rclpy.Node.__init__()`. Declaring it again raises `ParameterAlreadyDeclaredException`. Only declare your own custom parameters (e.g., `forklift_id`).

### 1.6: Warehouse Manager (Phase 1 — Simple Script)

Create `warehouse-manager/main_phase1.py`:

```python
"""Phase 1: Hardcoded single order. No FastAPI yet."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from warehouse_msgs.msg import TaskAssignment, TaskStatus, ForkliftStatus
from geometry_msgs.msg import Point
import uuid

class WarehouseManagerNode(Node):
    def __init__(self):
        super().__init__('warehouse_manager')
        # NOTE: Do NOT declare or override 'use_sim_time' here.
        # It is auto-declared by rclpy.Node.__init__().
        # Pass it via --ros-args -p use_sim_time:=true on the command line.
        # ... publisher for /warehouse/task_assignment (RELIABLE/TRANSIENT_LOCAL)
        # ... subscriber for /warehouse/task_status
        # ... subscriber for /forklift_0/status
        # On startup timer (once, after 3s delay):
        #   Publish TaskAssignment:
        #     task_id = str(uuid.uuid4())
        #     order_id = "order_001"
        #     forklift_id = "forklift_0"
        #     task_type = TaskAssignment.PICK
        #     source = Point(x=5.0, y=10.0, z=0.0)   # shelf_0 position
        #     destination = Point(x=5.0, y=2.0, z=0.0) # dock_0 position
        #     priority = 1
```

**⚠ GOTCHA**: Use a 3-second startup delay timer before publishing the first task. Isaac Sim and the vehicle controller need time to initialize. If you publish immediately, the message may arrive before subscribers are ready (even with `TRANSIENT_LOCAL`, the publisher needs to be alive when the subscriber joins).

### 1.7: Phase 1 Run & Validation

Run all three components **on Brev** (no Docker Compose yet — just terminals):

```bash
# Terminal 1: Isaac Sim
cd ~/warehouse-sim
docker run --rm --gpus all --network host \
  -e ACCEPT_EULA=Y \
  -e ROS_DOMAIN_ID=42 \
  -v $(pwd)/sim-scripts:/sim-scripts \
  -v isaac-sim-cache:/isaac-sim/kit/cache \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  ./python.sh /sim-scripts/warehouse_scene.py --headless

# Terminal 2: Vehicle Controller
cd ~/warehouse-sim/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run vehicle_controller forklift_controller_node --ros-args -p use_sim_time:=true
# OR if not a ROS 2 package yet, just run:
python3 ../vehicle-controller/forklift_controller_node.py --ros-args -p use_sim_time:=true

# Terminal 3: Warehouse Manager
cd ~/warehouse-sim/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
python3 ../warehouse-manager/main_phase1.py --ros-args -p use_sim_time:=true

# Terminal 4: Monitoring
export ROS_DOMAIN_ID=42
ros2 topic echo /warehouse/task_status
ros2 topic echo /forklift_0/status
ros2 topic echo /clock
```

**Validation checklist**:

| Check              | Command                                             | Expected                                  |
| ------------------ | --------------------------------------------------- | ----------------------------------------- |
| Clock topic exists | `ros2 topic list \| grep clock`                     | `/clock`                                  |
| Clock is ticking   | `ros2 topic hz /clock`                              | ~60 Hz                                    |
| Task assigned      | `ros2 topic echo /warehouse/task_assignment --once` | TaskAssignment message                    |
| Forklift moves     | `ros2 topic echo /forklift_0/odom`                  | Changing position                         |
| Task completes     | `ros2 topic echo /warehouse/task_status`            | Status goes PENDING→IN_PROGRESS→COMPLETED |
| No PhysX errors    | Isaac Sim logs                                      | No "PhysX error" or "NaN" warnings        |

**⚠ GOTCHA**: If the forklift doesn't move, check:

1. `cmd_vel` topic is being published: `ros2 topic hz /forklift_0/cmd_vel`
2. The Isaac Sim Twist subscriber is connected to the right joint drives
3. The differential drive gains are set (Isaac Sim OmniGraph differential drive controller needs `wheel_radius` and `wheel_distance` parameters)

---

## 2. Phase 2 — Warehouse Manager as Service (Still on Brev)

### 2.1: Refactor to FastAPI + ROS 2

Create `warehouse-manager/main.py` replacing `main_phase1.py`:

**Critical architecture**: FastAPI runs in the main asyncio event loop. `rclpy` runs in a dedicated background thread with its own executor.

```python
import asyncio
import threading
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket
import rclpy
from rclpy.executors import MultiThreadedExecutor

# Bridge pattern:
# ROS 2 callbacks put events into an asyncio.Queue
# FastAPI WebSocket handler reads from the queue
# REST endpoints call thread-safe methods on the ROS node

ros_node = None
event_queue = asyncio.Queue()

@asynccontextmanager
async def lifespan(app: FastAPI):
    global ros_node
    rclpy.init()
    ros_node = WarehouseManagerNode(event_queue)
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    yield
    ros_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

app = FastAPI(lifespan=lifespan)
```

**⚠ GOTCHA**: Thread safety. The ROS 2 executor runs callbacks in its thread. FastAPI runs in the asyncio thread. Shared state (orders dict, forklift states) must use `threading.Lock` or be passed through thread-safe queues. Do NOT access ROS 2 publishers directly from FastAPI handlers — use a queue or `call_soon_threadsafe`.

**⚠ GOTCHA**: `rclpy.init()` must be called ONCE. If it's called twice (e.g., module reloaded by uvicorn), it crashes. Guard with `if not rclpy.ok(): rclpy.init()`.

**⚠ GOTCHA**: `rclpy.shutdown()` must be guarded with `if rclpy.ok():` in the lifespan teardown. If the process is killed (e.g., by `timeout`, Docker stop, or Ctrl+C), rclpy may already be shut down. Calling `rclpy.shutdown()` a second time raises `RCLError: rcl_shutdown already called`.

### 2.2: SQLite Persistence

Create `warehouse-manager/models.py`:

```sql
-- Schema
CREATE TABLE IF NOT EXISTS orders (
    id TEXT PRIMARY KEY,
    items TEXT NOT NULL,           -- JSON array of item SKUs
    priority INTEGER DEFAULT 1,
    status TEXT DEFAULT 'pending', -- pending, in_progress, completed, cancelled
    created_at TEXT DEFAULT (datetime('now')),
    completed_at TEXT
);

CREATE TABLE IF NOT EXISTS tasks (
    id TEXT PRIMARY KEY,
    order_id TEXT REFERENCES orders(id),
    forklift_id TEXT,
    task_type INTEGER,            -- 0=PICK, 1=DELIVER
    source_x REAL, source_y REAL, source_z REAL,
    dest_x REAL, dest_y REAL, dest_z REAL,
    status TEXT DEFAULT 'pending',
    created_at TEXT DEFAULT (datetime('now')),
    completed_at TEXT
);

CREATE TABLE IF NOT EXISTS task_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT REFERENCES tasks(id),
    event_type TEXT NOT NULL,      -- assigned, started, completed, failed, reassigned
    details TEXT,                  -- JSON payload
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
```

**⚠ GOTCHA**: Enable WAL mode immediately after opening:

```python
import sqlite3
conn = sqlite3.connect('warehouse.db', check_same_thread=False)
conn.execute("PRAGMA journal_mode=WAL")
conn.execute("PRAGMA busy_timeout=5000")
```

`check_same_thread=False` is required because FastAPI and ROS 2 threads both access the DB. WAL mode allows concurrent reads with one writer. `busy_timeout` prevents immediate SQLITE_BUSY errors.

### 2.3: Redis Setup

```bash
# On Brev (Phase 1-3, Redis runs locally)
docker run -d --name redis --network host redis:7-alpine
redis-cli ping  # PONG
```

Redis usage:

```python
import os, redis

redis_url = os.getenv('REDIS_URL', 'redis://localhost:6379')
r = redis.Redis.from_url(redis_url, decode_responses=True)

# Forklift positions (hot state, updated at 10Hz)
r.hset("forklift:forklift_0", mapping={"x": "5.0", "y": "10.0", "yaw": "1.57", "state": "1"})

# Task queue (priority queue)
r.zadd("task_queue", {"task_abc": priority_score})

# Pub/sub for WebSocket fan-out — 5 message types:
# forklift_update — real-time forklift position/state (10Hz)
# order_update    — order lifecycle changes
# task_status     — task state transitions
# metric_update   — periodic KPI snapshots
# collision       — collision events from contact sensors
r.publish("ws:events", json.dumps({"type": "forklift_update", "data": {...}}))
r.publish("ws:events", json.dumps({"type": "task_status", "data": {...}}))
```

**Quaternion → Yaw conversion** (needed when bridging ROS Pose to WebSocket/REST):

```python
import math
def quaternion_to_yaw(q):
    """Convert geometry_msgs/Quaternion to yaw angle in radians."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y**2 + q.z**2))
```

### 2.4: Order Generation Engine

Create `warehouse-manager/order_generator.py`:

- Poisson process: `next_arrival = -math.log(1 - random.random()) / lambda_rate`
- Runs as an asyncio background task in FastAPI (not in the ROS thread)
- Picks random shelf with available inventory
- Creates order in SQLite → dispatches to task dispatcher

### 2.5: Task Dispatcher

Create `warehouse-manager/dispatcher.py`:

- `assign_task(order)`: find idle forklift closest to shelf → publish TaskAssignment
- `enqueue_order(order)`: add to Redis sorted set if no idle forklift
- `on_forklift_idle(forklift_id)`: pop from queue, assign
- Distance calculation: Euclidean on 2D positions from Redis `forklift:{id}` hash

### 2.6: Reset Service

- ROS 2 service server `/warehouse/reset` in the warehouse manager node
- REST `POST /reset` handler calls the ROS 2 service internally
- Reset sequence:
  1. Cancel all in-flight tasks (SQLite: `UPDATE tasks SET status='cancelled'`)
  2. Flush Redis keys: `forklift:*`, `task_queue`
  3. Publish TaskAssignment with `task_type=RESET` (or use a dedicated topic `/warehouse/reset_cmd`)
  4. Wait for all forklifts to report IDLE status (timeout 30s)
  5. Isaac Sim: reset script triggered by a ROS 2 subscriber in the sim script

### 2.7: Phase 2 Validation

```bash
# Start all services on Brev
# Terminal 1: Isaac Sim (same as Phase 1)
# Terminal 2: Redis
docker run -d --name redis --network host redis:7-alpine
# Terminal 3: Warehouse Manager
cd ~/warehouse-sim
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
uvicorn warehouse-manager.main:app --host 0.0.0.0 --port 8000
# Terminal 4: Vehicle Controller (same as Phase 1)
```

Validation:

```bash
# Create order via API
curl -X POST http://localhost:8000/orders \
  -H "Content-Type: application/json" \
  -d '{"items": ["box_001"], "priority": 1}'

# Check order status
curl http://localhost:8000/orders

# Check forklift statuses
curl http://localhost:8000/forklifts

# Check metrics
curl http://localhost:8000/metrics

# Test WebSocket (install websocat)
websocat ws://localhost:8000/ws/live

# Test persistence: kill uvicorn, restart, check orders still exist
curl http://localhost:8000/orders  # Should show previous orders

# Test reset
curl -X POST http://localhost:8000/reset
```

---

## 3. Phase 3 — Multi-Forklift + Nav2 (Still on Brev)

### 3.1: Expand Isaac Sim Scene

Update `sim-scripts/warehouse_scene.py`:

- **4 forklifts**: Duplicate the forklift USD prim. Each forklift gets a unique prim path (`/World/forklift_0`, `/World/forklift_1`, etc.) and unique spawn position.
- **Spawn positions** (avoid overlap):
  ```
  forklift_0: (2, 2, 0)
  forklift_1: (4, 2, 0)
  forklift_2: (6, 2, 0)
  forklift_3: (8, 2, 0)
  ```
- **20 shelves**: 4 rows × 5 racks. Grid layout:
  ```
  Row 0: shelves at y=8,  x = 5, 10, 15, 20, 25
  Row 1: shelves at y=14, x = 5, 10, 15, 20, 25
  Row 2: shelves at y=20, x = 5, 10, 15, 20, 25
  Row 3: shelves at y=26, x = 5, 10, 15, 20, 25
  ```
- **2 docks**: At `(12, 1, 0)` and `(22, 1, 0)`
- **Aisles**: 3m wide between rows (enough for forklift to pass)
- **ROS 2 bridge per forklift**: Each forklift gets namespaced topics:
  - `/forklift_{n}/cmd_vel`, `/forklift_{n}/odom`, `/forklift_{n}/fork_cmd`, `/forklift_{n}/lidar`
- **Contact sensors**: Add `PhysicsContactSensor` to each forklift chassis. Publish on `/forklift_{n}/contact`

**⚠ GOTCHA**: When duplicating forklift USD prims, ensure each prim's articulation root and joint names are unique. Isaac Sim may share references if you just copy/paste in the USD stage. Use `Sdf.Path` operations to make deep copies.

### 3.2: Occupancy Map Generation

Create `sim-scripts/occupancy_map_gen.py`:

```python
# Uses isaacsim.asset.gen.omap extension
# 1. Load the warehouse scene
# 2. Call OccupancyMapGenerator with parameters:
#    - cell_size: 0.1m (10cm resolution)
#    - height_range: (0.1, 2.0) — captures shelves but not ceiling
#    - origin: (0, 0) — bottom-left of warehouse
# 3. Publish on /map as nav_msgs/OccupancyGrid with:
#    - QoS: RELIABLE + TRANSIENT_LOCAL (latched — Nav2 gets it whenever it subscribes)
#    - frame_id: "map"
#    - resolution: 0.1
# 4. Also save as .pgm/.yaml for Nav2 map_server fallback
```

**⚠ GOTCHA**: The occupancy map must match the physical scene exactly. If shelves are moved after map generation, Nav2 will plan through shelves. Regenerate on layout changes.

### 3.3: Vehicle Controller with Nav2

Update `vehicle-controller/` to a proper ROS 2 package:

```
vehicle-controller/
├── package.xml
├── setup.py                   # ament_python build type
├── setup.cfg                  # REQUIRED — see gotcha below
├── resource/
│   └── vehicle_controller     # empty marker file for ament index
├── vehicle_controller/
│   ├── __init__.py
│   ├── forklift_controller_node.py
│   ├── zone_reservation.py
│   ├── config/
│   │   └── nav2_params.yaml
│   └── launch/
│       └── multi_forklift.launch.py
└── Dockerfile
```

**⚠ GOTCHA**: For `ament_python` packages, you MUST create a `setup.cfg` with script installation directories, otherwise `ros2 run` won't find the entry point:

```ini
[develop]
script_dir=$base/lib/vehicle_controller

[install]
install_scripts=$base/lib/vehicle_controller
```

Without this, setuptools installs the entry point script to `$base/bin/` instead of `$base/lib/vehicle_controller/`, and `ros2 run` can't find it.

You also need a `resource/vehicle_controller` empty marker file for the ament index.

**Nav2 parameters** (`config/nav2_params.yaml`) — critical tuning for warehouse:

```yaml
# MUST OVERRIDE DEFAULTS — Nav2 defaults are for outdoor/large robots
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.3 # Goal tolerance in meters
      use_astar: true # A* faster than Dijkstra for point-to-point
      allow_unknown: false

controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5 # Warehouse forklift speed
      min_vel_x: 0.0
      max_vel_theta: 1.0
      min_speed_theta: 0.0
      acc_lim_x: 0.5
      acc_lim_theta: 1.0
      decel_lim_x: -0.5
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.15
      transform_tolerance: 0.5

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      width: 5 # 5m local window
      height: 5
      resolution: 0.05 # 5cm for local obstacle avoidance
      robot_radius: 0.8 # Forklift footprint radius
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        inflation_radius: 1.0 # Keep 1m from obstacles
        cost_scaling_factor: 3.0 # Aggressive falloff

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      robot_radius: 0.8
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        map_subscribe_transient_local: true # Get the latched /map
      inflation_layer:
        inflation_radius: 0.8 # Narrower for global planning (fit in aisles)
        cost_scaling_factor: 5.0
```

**⚠ GOTCHA**: If `inflation_radius` in global costmap is too large, Nav2 will consider aisles as blocked and fail to find paths. 0.8m works for 3m-wide aisles with 0.8m robot radius. Test by visualizing costmap in RViz.

**Launch file** (`launch/multi_forklift.launch.py`):

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    num_forklifts = int(os.environ.get('NUM_FORKLIFTS', '4'))

    ld = LaunchDescription()

    # Shared map server (no namespace)
    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': '', 'topic_name': '/map',
                      'use_sim_time': True}],
    ))

    for i in range(num_forklifts):
        ns = f'forklift_{i}'
        group = GroupAction([
            PushRosNamespace(ns),
            # Nav2 planner
            Node(package='nav2_planner', executable='planner_server',
                 name='planner_server',
                 parameters=['config/nav2_params.yaml', {'use_sim_time': True}]),
            # Nav2 controller
            Node(package='nav2_controller', executable='controller_server',
                 name='controller_server',
                 parameters=['config/nav2_params.yaml', {'use_sim_time': True}]),
            # Nav2 BT navigator
            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 name='bt_navigator',
                 parameters=[{'use_sim_time': True}]),
            # Lifecycle manager for this namespace
            Node(package='nav2_lifecycle_manager',
                 executable='lifecycle_manager',
                 name='lifecycle_manager',
                 parameters=[{
                     'autostart': True,
                     'use_sim_time': True,
                     'node_names': ['planner_server', 'controller_server', 'bt_navigator']
                 }]),
            # Forklift controller (our custom node)
            Node(package='vehicle_controller',
                 executable='forklift_controller_node',
                 name='controller',
                 parameters=[{'forklift_id': ns, 'use_sim_time': True}]),
        ])
        ld.add_action(group)

    return ld
```

**⚠ GOTCHA**: Nav2 lifecycle management is critical. All Nav2 nodes start in an unconfigured state. The `lifecycle_manager` must activate them in order. If any node fails to activate, check logs for missing transforms (tf). The forklift controller must publish `odom` → `base_link` transform.

### 3.4: Zone Reservation

Create `vehicle-controller/vehicle_controller/zone_reservation.py`:

```python
# Zone-based reservation system
# Warehouse divided into rectangular zones:
#   - Each aisle segment = 1 zone
#   - Each intersection = 1 zone
# Protocol:
#   1. Forklift publishes ZoneRequest on /zone_requests (forklift_id, zone_id, priority)
#   2. ZoneManager node (singleton) grants/denies via /zone_grants
#   3. Forklift waits for grant before entering zone
#   4. On exit, forklift publishes ZoneRelease
# Priority: forklift with cargo > empty forklift > lower task priority
# Deadlock: if wait > 5s, ZoneManager forces lower-priority forklift to back out
```

**⚠ GOTCHA**: This is the most complex part of Phase 3. Start with a simple version:

1. Phase 3.4 MVP: global mutex per aisle (only 1 forklift per aisle at a time)
2. Refine later: zone-level granularity

### 3.5: Error Recovery

Add to `forklift_controller_node.py`:

```python
# RECOVERING state transitions:
# 1. Contact sensor triggers (/forklift_n/contact has data):
#    - Immediately publish cmd_vel = (0, 0) — STOP
#    - Transition to RECOVERING
#    - Publish cmd_vel = (-0.2, 0) for 2s — back up
#    - Clear local costmap (call /forklift_n/local_costmap/clear_entirely_local_costmap service)
#    - Re-send NavigateToPose goal — resume task
#    - If NavigateToPose fails 3 times → ERROR
#
# 2. NavigateToPose action returns ABORTED:
#    - Retry 1: same goal, clear costmap first
#    - Retry 2: goal with 0.5m offset (try adjacent approach)
#    - Retry 3: report ERROR
#
# 3. ERROR state:
#    - Publish TaskStatus(status=FAILED)
#    - Stay in ERROR until new TaskAssignment or reset
#    - Warehouse Manager will reassign the task after 10s timeout
```

### 3.6: Phase 3 Validation

```bash
# Run with 4 forklifts
export NUM_FORKLIFTS=4
export ROS_DOMAIN_ID=42

# Terminal 1: Isaac Sim (updated scene with 4 forklifts)
# Terminal 2: Redis
# Terminal 3: Warehouse Manager (update config for 4 forklifts, 2 orders/min)
# Terminal 4: Vehicle Controller (launch file starts all 4)
ros2 launch vehicle_controller multi_forklift.launch.py

# Monitor for 10 minutes
ros2 topic echo /warehouse/task_status  # Watch for COMPLETED and FAILED
ros2 topic echo /forklift_0/contact     # Watch for collisions

# Count completed vs failed
# Expected: most orders complete, failed orders get reassigned
```

Validation:

| Check                  | How                                      | Expected                       |
| ---------------------- | ---------------------------------------- | ------------------------------ |
| All forklifts navigate | `ros2 topic hz /forklift_{0..3}/cmd_vel` | All ~10 Hz                     |
| No permanent deadlocks | Watch task_status for 10 min             | No task stuck >60s             |
| Collision recovery     | Manually push a forklift in Isaac Sim    | Backs up, replans              |
| Task reassignment      | Kill forklift_0's controller node        | Its task reassigned to another |
| Occupancy map          | `ros2 topic echo /map --once \| head`    | Valid OccupancyGrid            |

---

## 4. Phase 4 — Split Deployment (Brev Side)

From this phase, Brev runs **only** sim + vehicle controller. Warehouse manager moves to local machine.

### 4.1: FastDDS Discovery Server

Create `fastdds_discovery.xml`:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <participant profile_name="discovery_server" is_default_profile="true">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>SERVER</discoveryProtocol>
                        <discoveryServersList>
                            <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
                                <metatrafficUnicastLocatorList>
                                    <locator>
                                        <udpv4>
                                            <address>0.0.0.0</address>
                                            <port>11811</port>
                                        </udpv4>
                                    </locator>
                                </metatrafficUnicastLocatorList>
                            </RemoteServer>
                        </discoveryServersList>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

### 4.2: Brev Docker Compose

Create `brev-compose.yml`:

```yaml
services:
  # --- FastDDS Discovery Server ---
  discovery-server:
    image: ros:humble
    network_mode: host
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        fast-discovery-server -i 0 -p 11811
      "
    restart: unless-stopped
    healthcheck:
      test: ["CMD-SHELL", "ss -uln | grep -q 11811"]
      interval: 5s
      timeout: 3s
      retries: 5

  # --- Isaac Sim ---
  isaac-sim:
    image: nvcr.io/nvidia/isaac-sim:5.1.0
    network_mode: host
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
    environment:
      - ACCEPT_EULA=Y
      - ROS_DOMAIN_ID=42
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - ROS_DISCOVERY_SERVER=localhost:11811
      - SCENE_USD=/sim-scripts/scenes/scene_assembly.usd
      # NOTE: Do NOT set FASTRTPS_DEFAULT_PROFILES_FILE here.
      # ROS_DISCOVERY_SERVER env var auto-configures this container as a
      # DDS super-client. Loading the server XML would conflict with the
      # dedicated discovery-server container.
    volumes:
      - ./sim-scripts:/sim-scripts:ro
      - ./simulations/forklift-warehouse/01_scenes:/sim-scripts/scenes:ro
      - isaac-sim-cache:/isaac-sim/kit/cache
    command: ./python.sh /sim-scripts/warehouse_scene.py --headless
    depends_on:
      discovery-server:
        condition: service_healthy
    restart: unless-stopped

  # --- Vehicle Controller (single container, all forklifts) ---
  vehicle-controller:
    build:
      context: .
      dockerfile: vehicle-controller/Dockerfile
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=42
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - ROS_DISCOVERY_SERVER=localhost:11811
      # NOTE: Same as isaac-sim — do NOT load the server XML profile.
      # ROS_DISCOVERY_SERVER is sufficient for client discovery.
      - NUM_FORKLIFTS=4
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /ros2_ws/install/setup.bash &&
        ros2 launch vehicle_controller multi_forklift.launch.py
      "
    depends_on:
      discovery-server:
        condition: service_healthy
    restart: unless-stopped

volumes:
  isaac-sim-cache:
```

**⚠ GOTCHA**: The discovery server command is `fast-discovery-server -i 0 -p 11811`, NOT `fastdds discovery --server-id 0 --port 11811`. The `fastdds` CLI subcommand doesn't work in the `ros:humble` Docker image — it reports `fast-discovery-server tool not found!` and the `--port` argument is unrecognized. The standalone binary `fast-discovery-server` (at `/opt/ros/humble/bin/fast-discovery-server`) works correctly.

**⚠ GOTCHA**: Use `depends_on` with `condition: service_healthy` (not just `depends_on:`) and add a `healthcheck` on the discovery server. Without this, Isaac Sim and the vehicle controller may start before the discovery server is ready, causing DDS discovery failures.

**⚠ GOTCHA**: The scene USD file lives in `simulations/forklift-warehouse/01_scenes/scene_assembly.usd`. Mount it into the Isaac Sim container via `./simulations/forklift-warehouse/01_scenes:/sim-scripts/scenes:ro` and set `SCENE_USD=/sim-scripts/scenes/scene_assembly.usd` in the environment.

### 4.3: Vehicle Controller Dockerfile

Create `vehicle-controller/Dockerfile`:

```dockerfile
FROM ros:humble

# Install Nav2
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rmw-fastrtps-cpp \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Build warehouse_msgs
WORKDIR /ros2_ws/src
COPY warehouse_msgs/ warehouse_msgs/
COPY vehicle-controller/ vehicle_controller/

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    echo 'source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc"

# Install Python dependencies
RUN pip3 install --no-cache-dir redis

WORKDIR /ros2_ws
```

**⚠ GOTCHA**: The `colcon build` step will fail if `vehicle-controller/` is not a valid ROS 2 package (needs `package.xml` and either `CMakeLists.txt` or `setup.py`). For a Python-only package, use `setup.py` with `ament_python`.

**⚠ GOTCHA**: The `package.xml` must declare `<build_type>ament_python</build_type>` inside `<export>` for Python packages. And the `setup.cfg` file is mandatory — see the vehicle controller section above for the required `[develop]` and `[install]` sections.

### 4.4: Tailscale Verification

```bash
# On Brev
tailscale status
# Should show both Brev and local machine connected

# Test connectivity
ping <local-tailscale-ip>

# Verify discovery server is reachable from local
# (run from local machine)
nc -zv <brev-tailscale-ip> 11811
```

### 4.5: Phase 4 Run

```bash
cd ~/warehouse-sim
git pull  # Get latest code
docker compose -f brev-compose.yml up --build -d

# Verify containers are running
docker compose -f brev-compose.yml ps

# Check logs
docker compose -f brev-compose.yml logs -f isaac-sim
docker compose -f brev-compose.yml logs -f vehicle-controller

# Verify ROS 2 topics are published
export ROS_DOMAIN_ID=42
export ROS_DISCOVERY_SERVER=localhost:11811
ros2 topic list
```

---

## 5. Phase 6 — Optimization (Brev Side)

### 5.1: Fleet Routing (runs on local warehouse-manager, but affects Brev)

No Brev-side changes for fleet routing optimization — the warehouse manager handles dispatching.

### 5.2: Domain Randomization

Update `sim-scripts/warehouse_scene.py` to accept randomization parameters:

```bash
# Launch with randomization
docker run ... nvcr.io/nvidia/isaac-sim:5.1.0 \
  ./python.sh /sim-scripts/warehouse_scene.py --headless \
  --randomize-items --randomize-layout --seed 42
```

### 5.3: Layout A/B Testing

Create `sim-scripts/layout_generator.py`:

- Reads layout config (aisle width, shelf spacing, dock count)
- Generates USD scene procedurally
- Publishes matching occupancy map

---

## Troubleshooting

### Isaac Sim won't start

```bash
# Check GPU
nvidia-smi  # Must show GPU with enough free VRAM

# Check Docker GPU access
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi

# Check Isaac Sim logs
docker compose -f brev-compose.yml logs isaac-sim | grep -i error

# Common: EULA not accepted
# Fix: ensure ACCEPT_EULA=Y in environment

# Common: shader compilation OOM
# Fix: close other GPU processes, or use A10G with 24GB VRAM
```

### ROS 2 topics not visible

```bash
# Check discovery server is running
docker compose -f brev-compose.yml logs discovery-server

# Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID  # Must be 42

# Check ROS_DISCOVERY_SERVER
echo $ROS_DISCOVERY_SERVER  # Must be localhost:11811 on Brev

# List participants
ros2 daemon stop && ros2 daemon start
ros2 topic list
```

### Nav2 nodes fail to activate

```bash
# Check lifecycle status
ros2 lifecycle list /forklift_0/planner_server

# Common: missing /map topic
# Fix: ensure occupancy map is published before Nav2 starts

# Common: missing odom → base_link transform
# Fix: ensure Isaac Sim publishes tf or the controller node publishes it

# Common: planner fails to find path
# Fix: check costmap inflation radius — too large blocks aisles
```

### Forklift doesn't pick up item

```bash
# Check fork position
ros2 topic echo /forklift_0/joint_states

# Common: surface gripper threshold too tight
# Fix: increase grip_threshold to 0.05

# Common: fork doesn't align with item height
# Fix: adjust fork target position to match shelf slot height

# Common: item mass too high for gripper force
# Fix: increase force_limit on surface_gripper
```
