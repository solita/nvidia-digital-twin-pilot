# Warehouse Digital Twin — Architecture Reference

> **Last updated:** 2026-04-27
>
> A full-stack warehouse automation digital twin powered by NVIDIA Isaac Sim, ROS 2 Jazzy, FastAPI, and React.
> Simulates 4 autonomous forklifts performing pick-and-place tasks in a virtual warehouse.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Deployment Modes](#deployment-modes)
3. [Docker Services](#docker-services)
   - [discovery-server](#discovery-server)
   - [isaac-sim-init](#isaac-sim-init)
   - [isaac-sim](#isaac-sim)
   - [web-viewer](#web-viewer)
   - [vehicle-controller](#vehicle-controller)
   - [warehouse-manager](#warehouse-manager)
   - [redis](#redis)
   - [dashboard](#dashboard)
4. [ROS 2 Communication Layer](#ros-2-communication-layer)
   - [Custom Messages (warehouse_msgs)](#custom-messages-warehouse_msgs)
   - [Topic Map](#topic-map)
   - [QoS Profiles](#qos-profiles)
5. [Isaac Sim Scene & Physics](#isaac-sim-scene--physics)
   - [Scene Layout (scene_assembly.usd)](#scene-layout)
   - [Physics Configuration](#physics-configuration)
   - [OmniGraph ROS 2 Bridge](#omnigraph-ros-2-bridge)
   - [cmd_vel → Joint Bridge Loop](#cmd_vel--joint-bridge-loop)
6. [Vehicle Controller](#vehicle-controller-1)
   - [Forklift State Machine](#forklift-state-machine)
   - [Proportional Navigation (Phase 1)](#proportional-navigation)
   - [Pick & Drop Sequences](#pick--drop-sequences)
7. [Warehouse Manager](#warehouse-manager-1)
   - [REST API](#rest-api)
   - [WebSocket Interface](#websocket-interface)
   - [ROS 2 Node](#ros-2-node)
   - [Task Dispatch System](#task-dispatch-system)
   - [Scenario Recording & Replay](#scenario-recording--replay)
   - [Database Schema](#database-schema)
8. [Dashboard (React UI)](#dashboard-react-ui)
   - [Components](#components)
   - [State Management](#state-management)
   - [Data Flow](#data-flow)
9. [DDS / FastDDS Configuration](#dds--fastdds-configuration)
10. [Environment Variables](#environment-variables)
11. [Makefile Commands](#makefile-commands)
12. [Directory Structure](#directory-structure)

---

## System Overview

The Warehouse Digital Twin is a monorepo project that simulates an autonomous warehouse with 4 forklifts. The system coordinates physics simulation (Isaac Sim), robot control logic (ROS 2 nodes), task orchestration (FastAPI server), and real-time visualization (React dashboard).

**Core data flow:**

```
Dashboard (React) ──HTTP/WS──▶ Warehouse Manager (FastAPI+ROS2)
                                       │
                              ┌────────┼────────┐
                    ROS 2 DDS │        │        │ ROS 2 DDS
                              ▼        ▼        ▼
                       Vehicle Controller (×4 forklift nodes)
                              │
                    ROS 2 DDS │ cmd_vel / odom
                              ▼
                    Isaac Sim (PhysX + OmniGraph ROS2 Bridge)
```

**Key technology choices:**

| Layer | Technology |
|-------|-----------|
| Physics simulation | NVIDIA Isaac Sim 5.1.0 (Kit-based, PhysX TGS) |
| Robot middleware | ROS 2 Jazzy with FastDDS (rmw_fastrtps_cpp) |
| Orchestration server | Python FastAPI + rclpy hybrid |
| Persistent storage | SQLite (WAL mode) + Redis 7 |
| Browser UI | React 18 + TypeScript + Vite + Zustand |
| Containerization | Docker Compose (all network_mode: host) |
| Compute | Brev cloud GPU instance (L40S, Ubuntu 22.04) |

---

## Deployment Modes

The project supports two deployment modes:

### All-in-One (brev-compose.yml)

All services run on the same Brev GPU host. This is the primary development mode.

```
brev-compose.yml
├── discovery-server
├── isaac-sim-init → isaac-sim
├── web-viewer
├── vehicle-controller
├── warehouse-manager
├── redis
└── dashboard (optional profile)
```

**Usage:**
```bash
make brev-up       # Start all services
make brev-logs     # Tail logs
make brev-down     # Stop
```

### Split Deployment (brev-compose-phase4.yml + local-compose.yml)

GPU-bound services run on Brev; orchestration and UI run locally. Services communicate over Tailscale VPN using FastDDS CLIENT/SERVER discovery.

**Brev side** (brev-compose-phase4.yml):
- discovery-server, isaac-sim, vehicle-controller

**Local side** (local-compose.yml):
- warehouse-manager, redis, dashboard

**Usage:**
```bash
# On Brev:
docker compose -f brev-compose-phase4.yml up --build -d

# On local machine:
make up             # envsubst + docker compose up
make dashboard      # Optional dashboard
```

The local side uses `envsubst` to substitute `${BREV_TAILSCALE_IP}` into `fastdds_client.xml`, producing `fastdds_client_resolved.xml` which configures the DDS CLIENT discovery protocol.

---

## Docker Services

### discovery-server

| Property | Value |
|----------|-------|
| Image | `ros:jazzy` |
| Network | host |
| Command | `fast-discovery-server -i 0 -p 11811` |
| Port | UDP 11811 |
| Healthcheck | `grep -q 2E23 /proc/net/udp` |

The FastDDS discovery server provides centralized DDS participant discovery for all ROS 2 nodes in the system. All other ROS 2 containers register with this server.

**Important:** The binary is `fast-discovery-server`, NOT the `fastdds discovery` subcommand (different binary, different behavior).

In the all-in-one compose, all containers use `network_mode: host` so discovery happens via localhost. In split deployment, the local side connects via Tailscale IP using the CLIENT discovery protocol.

---

### isaac-sim-init

| Property | Value |
|----------|-------|
| Image | `nvcr.io/nvidia/isaac-sim:5.1.0` |
| User | `0:0` (root) |
| Purpose | One-shot volume permission fix |

A transient init container that `chown`s named Docker volumes (cache, logs, data, config) to UID 1234 (the non-root user Isaac Sim runs as). Runs once and exits before `isaac-sim` starts (`service_completed_successfully` dependency).

---

### isaac-sim

| Property | Value |
|----------|-------|
| Image | `nvcr.io/nvidia/isaac-sim:5.1.0` |
| Network | host |
| GPU | All (NVIDIA L40S) |
| Command | `--exec "/sim-scripts/warehouse_scene_streaming.py"` |
| Healthcheck | `test -f /tmp/isaac-sim-ready` (start_period: 300s) |
| Streaming | WebRTC on ports 49100 (signal) / 47998 (stream) |

The physics simulation engine and rendering server. Isaac Sim 5.1.0 is a Kit-based streaming container that runs headless with WebRTC output.

**Entrypoint chain:** `runheadless.sh` → `isaac-sim.streaming.sh` → Kit app loads, then executes the `--exec` script inside the running Kit event loop.

**Key behavior:**
- The `--exec` script does NOT create a `SimulationApp` (the streaming container already has one)
- Instead, it schedules an async coroutine via `asyncio.ensure_future(setup_scene())`
- The script loads `scene_assembly.usd`, configures physics, creates OmniGraph ROS 2 bridge nodes, and starts the timeline
- A sentinel file `/tmp/isaac-sim-ready` is written when setup completes (used by Docker healthcheck)
- First startup includes shader compilation (up to 5 minutes, hence 300s start_period)
- Subsequent starts use the cached shaders from `isaac-sim-cache` volume

**Volume mounts:**
| Mount | Source | Purpose |
|-------|--------|---------|
| `/sim-scripts` | `./sim-scripts` | Python scripts |
| `/sim-scripts/scenes` | `./simulations/forklift-warehouse/01_scenes` | USD scene files |
| `/sim-scripts/core_scripts` | `./simulations/forklift-warehouse/02_core_scripts` | Additional scripts |
| `/isaac-sim/kit/cache` | `isaac-sim-cache` volume | Shader + PhysX cache |
| Logs, data, config | Named volumes | Runtime state |

**Isaac Sim 5.1.0 breaking changes:**
All `omni.isaac.*` OmniGraph node types were renamed to `isaacsim.*`:
- `omni.isaac.ros2_bridge.*` → `isaacsim.ros2.bridge.*`
- `omni.isaac.core_nodes.*` → `isaacsim.core.nodes.*`
- `omni.isaac.wheeled_robots.*` → `isaacsim.robot.wheeled_robots.*`

---

### web-viewer

| Property | Value |
|----------|-------|
| Image | Custom (Vite static site) |
| Network | host |
| Port | 8210 |
| Depends on | isaac-sim (healthy) |

A static WebRTC client that connects to Isaac Sim's streaming output. Serves a browser-based 3D viewport at `http://localhost:8210`. Built with Vite and served via `npx vite preview`.

Connects to Isaac Sim via:
- Signal port: `ISAACSIM_SIGNAL_PORT` (default 49100)
- Stream port: `ISAACSIM_STREAM_PORT` (default 47998)

---

### vehicle-controller

| Property | Value |
|----------|-------|
| Image | Custom (ros:jazzy base) |
| Network | host |
| Command | `ros2 launch vehicle_controller multi_forklift.launch.py` |
| Build context | Repository root (needs `warehouse_msgs/` for message build) |

Runs 1–4 instances of `ForkliftControllerNode` (one per simulated forklift). The `NUM_FORKLIFTS` environment variable controls how many nodes the launch file spawns.

Each node implements a complete forklift state machine with proportional navigation control. See [Vehicle Controller](#vehicle-controller-1) section for detailed behavior.

**Dockerfile highlights:**
- Based on `ros:jazzy`
- Builds `warehouse_msgs` CMake package first (colcon build)
- Then builds `vehicle_controller` Python package (ament_python)
- Sources both workspaces at runtime

---

### warehouse-manager

| Property | Value |
|----------|-------|
| Image | Custom (ros:jazzy + FastAPI) |
| Network | host |
| Port | 8000 (uvicorn) |
| Depends on | redis (healthy) |

A hybrid FastAPI web server and ROS 2 node. Runs `uvicorn` on the main thread with the ROS 2 node spinning in a background executor thread.

See [Warehouse Manager](#warehouse-manager-1) section for API, WebSocket, and dispatch details.

**Dockerfile highlights:**
- Based on `ros:jazzy`
- Installs Python dependencies: `fastapi`, `uvicorn`, `aiosqlite`, `redis[hiredis]`, `scipy`, `numpy`
- Builds `warehouse_msgs` for ROS 2 message types
- `HAS_MSGS` flag allows running without built messages (degraded mode)

---

### redis

| Property | Value |
|----------|-------|
| Image | `redis:7-alpine` |
| Port | 6379 |
| Persistence | AOF (appendonly yes) |
| Volume | `redis-data` |
| Healthcheck | `redis-cli ping` |

In-memory data store used by the warehouse manager for:
- Hot state caching of forklift positions and status
- Pub/Sub event relay between components
- Fast lookup for real-time dashboard updates

---

### dashboard

| Property | Value |
|----------|-------|
| Image | Custom (Node.js build → Nginx) |
| Port | 3000 (mapped to container port 80) |
| Profile | `dashboard` (only starts when explicitly requested) |
| Depends on | warehouse-manager |

React single-page application providing real-time visualization of the warehouse fleet. See [Dashboard](#dashboard-react-ui) section for component details.

**Build pipeline:** `npm install` → `vite build` → static files served by Nginx.

**Configuration:** `VITE_API_URL` and `VITE_WS_URL` environment variables (build-time, baked into bundle).

---

## ROS 2 Communication Layer

All ROS 2 communication uses **ROS 2 Jazzy** with the **FastDDS** middleware (`rmw_fastrtps_cpp`). Domain ID is **42** across all containers.

### Custom Messages (warehouse_msgs)

The `warehouse_msgs` package is a ROS 2 CMake message package that defines the custom message and service types used by the system.

#### ForkliftStatus.msg

Published by each vehicle controller at 2 Hz. Consumed by the warehouse manager.

```
string forklift_id          # "forklift_0" .. "forklift_3"
uint8 state                 # 0=IDLE, 1=NAV_TO_SHELF, 2=PICKING, 3=NAV_TO_DOCK, 4=DROPPING, 5=ERROR, 6=RECOVERING
geometry_msgs/Pose pose     # Current pose from odometry
float32 battery_level       # Always 100.0 in Phase 1 (simulation only)
string current_task_id      # UUID of active task, or empty
builtin_interfaces/Time stamp
```

#### TaskAssignment.msg

Published by the warehouse manager when dispatching an order. Consumed by vehicle controllers (each filters by `forklift_id`).

```
string task_id              # UUID
string forklift_id          # Target forklift
string order_id             # Source order UUID
geometry_msgs/Point shelf_position   # Pick location (x, y, z)
geometry_msgs/Point dock_position    # Drop location (x, y, z)
uint8 priority              # 1 = normal (higher = more urgent)
builtin_interfaces/Time stamp
```

#### TaskStatus.msg

Published by the vehicle controller on task lifecycle events. Consumed by the warehouse manager to update order/task state.

```
string task_id              # UUID matching TaskAssignment
string forklift_id
string status               # "pending", "in_progress", "completed", "cancelled", "failed"
string detail               # Human-readable detail
builtin_interfaces/Time stamp
```

#### OrderStatus.msg

Status broadcast for order lifecycle events (reserved for future use).

```
string order_id
string status               # "pending", "in_progress", "completed", "cancelled"
string[] items
string assigned_forklift
builtin_interfaces/Time stamp
```

#### ResetSimulation.srv

Service for resetting the simulation state.

```
# Request
bool clear_orders
---
# Response
bool success
string message
```

### Topic Map

| Topic | Message Type | QoS | Rate | Publisher → Subscriber |
|-------|-------------|-----|------|----------------------|
| `/warehouse/task_assignment` | `warehouse_msgs/TaskAssignment` | RELIABLE / VOLATILE | On dispatch | Manager → Controller |
| `/{id}/status` | `warehouse_msgs/ForkliftStatus` | RELIABLE | 2 Hz | Controller → Manager |
| `/warehouse/task_status` | `warehouse_msgs/TaskStatus` | RELIABLE | On event | Controller → Manager |
| `/{id}/cmd_vel` | `geometry_msgs/Twist` | RELIABLE | 10 Hz | Controller → Isaac Sim |
| `/{id}/fork_cmd` | `std_msgs/Float64` | BEST_EFFORT | On event | Controller → Isaac Sim |
| `/{id}/odom` | `nav_msgs/Odometry` | BEST_EFFORT | 60 Hz | Isaac Sim → Controller |
| `/clock` | `rosgraph_msgs/Clock` | Default | 60 Hz | Isaac Sim → All |

Where `{id}` is `forklift_0`, `forklift_1`, `forklift_2`, or `forklift_3`.

### QoS Profiles

Two QoS profiles are used throughout the system:

**RELIABLE (task orchestration):**
```python
QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)
```
Used for task assignments, status messages, and cmd_vel — where message delivery must be guaranteed.

**BEST_EFFORT (telemetry):**
```python
QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=1,
)
```
Used for high-frequency sensor data (odometry, fork commands) where latest-value semantics are preferred over guaranteed delivery.

---

## Isaac Sim Scene & Physics

### Scene Layout

The warehouse scene is defined in `scene_assembly.usd` (located in `simulations/forklift-warehouse/01_scenes/`). It was created using `populate_scene.py` which clones and positions prims from a template.

**USD Prim Hierarchy:**

| Prim Path | Description |
|-----------|-------------|
| `/World/warehouse` | Warehouse environment mesh (walls, floor, columns) |
| `/World/forklift_b` | Template forklift (deactivated after cloning) |
| `/World/forklift_0` | Active forklift 0, spawn position (-15, -25) |
| `/World/forklift_1` | Active forklift 1, spawn position (-5, -25) |
| `/World/forklift_2` | Active forklift 2, spawn position (5, -25) |
| `/World/forklift_3` | Active forklift 3, spawn position (15, -25) |
| `/World/Rack_A` .. `/World/Rack_D` | 4 shelf racks at Y≈10m (box collision prims) |
| `/World/Dock_1`, `/World/Dock_2` | 2 loading docks at Y≈-32m |
| `/World/pick_box_0` | Test rigid body cube (0.4m) at (5, 10, 0.6) |

**Forklift prim structure** (per forklift):
```
/World/forklift_N/
├── body                              # Rigid body (articulation root)
└── back_wheel_joints/
    ├── back_wheel_drive              # RevoluteJoint — rear drive (velocity control)
    └── back_wheel_swivel             # RevoluteJoint — rear steering (position control)
```

**Coordinate system:**
- X range: approximately -36 to +33 meters
- Y range: approximately -36 to +52 meters
- Z: up (gravity direction: (0, 0, -9.81))
- Dashboard abstracts this to a 50×30m view

### Physics Configuration

Configured by `setup_physics()` in `warehouse_scene_streaming.py`:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| Solver | TGS (Temporal Gauss-Seidel) | More stable for articulated bodies |
| GPU Dynamics | Enabled | Accelerated physics on GPU |
| GPU Broadphase | Enabled | GPU-accelerated collision detection |
| CCD | Enabled | Continuous collision detection |
| Stabilization | Enabled | Prevents jitter at rest |
| Timestep | 60 Hz (16.67ms) | Physics update rate |
| Gravity | (0, 0, -9.81) | Z-up world |

### Joint Drive Configuration

Set by `setup_joint_physics()` before simulation starts:

| Joint | Type | Stiffness | Damping | Control Mode |
|-------|------|-----------|---------|--------------|
| `back_wheel_drive` | Angular | 0.0 | 15,000 | Pure velocity |
| `back_wheel_swivel` | Angular | 40,000 | 10,000 | Position + damping |

These USD-level `DriveAPI` parameters define HOW the joints respond to targets. The actual runtime targets are set per-frame by the OmniGraph `IsaacArticulationController` nodes, not by direct DriveAPI attribute writes.

**Important:** Direct DriveAPI attribute setting does NOT propagate to PhysX at runtime when GPU dynamics are enabled. The ArticulationController OG node pathway is the only verified working approach.

### OmniGraph ROS 2 Bridge

One OmniGraph is created per forklift by `_create_ros2_bridge_graph_for()`. Each graph contains:

| Node Index | Node Type | Purpose |
|-----------|-----------|---------|
| 0 | `omni.graph.action.OnPlaybackTick` | 60 Hz tick source |
| 1 | `isaacsim.ros2.bridge.ROS2PublishClock` | Publishes `/clock` |
| 2 | `isaacsim.ros2.bridge.ROS2SubscribeTwist` | Subscribes `/{id}/cmd_vel` |
| 3 | `isaacsim.core.nodes.IsaacComputeOdometry` | Computes odometry from chassis |
| 4 | `isaacsim.ros2.bridge.ROS2PublishOdometry` | Publishes `/{id}/odom` |
| 5 | `isaacsim.core.nodes.IsaacReadSimulationTime` | Reads sim time for timestamps |
| 6 | `isaacsim.core.nodes.IsaacArticulationController` | **Drive** wheel velocity control |
| 7 | `isaacsim.core.nodes.IsaacArticulationController` | **Steer** wheel position control |

**Execution flow per tick:**
1. `OnPlaybackTick` fires at 60 Hz
2. Triggers clock publisher, twist subscriber, odometry compute, and both articulation controllers
3. Sim time node feeds timestamps to clock and odometry publishers
4. Odometry compute reads chassis prim, passes to odometry publisher

### cmd_vel → Joint Bridge Loop

The `cmd_vel_bridge_loop()` is an async Python coroutine running inside the Kit event loop (every physics frame). It bridges ROS 2 Twist messages to joint targets:

**Per forklift, per frame:**
1. Read `outputs:linearVelocity` and `outputs:angularVelocity` from the twist subscriber OG node
2. Extract `linear.x` (m/s) and `angular.z` (rad/s)
3. Convert to joint targets:
   - **Drive velocity:** `linear.x × DRIVE_SCALE` where `DRIVE_SCALE = -1.0 / WHEEL_RADIUS ≈ -6.667 rad/s per m/s`
   - **Steer position:** `angular.z × STEER_SCALE` where `STEER_SCALE = radians(20°) ≈ 0.349 rad per rad/s`
   - Steer clamped to `±STEER_MAX = ±30°`
4. Set `inputs:velocityCommand` on drive ArticulationController
5. Set `inputs:positionCommand` on steer ArticulationController

The negative drive scale is because the USD asset's joint direction convention requires negative angular velocity for the "forks-forward" direction.

---

## Vehicle Controller

**Source:** `vehicle-controller/vehicle_controller/forklift_controller_node.py`

The vehicle controller is a ROS 2 Python package (`ament_python`) containing the `ForkliftControllerNode`. The launch file `multi_forklift.launch.py` spawns `NUM_FORKLIFTS` instances, each parameterized with a unique `forklift_id`.

### Forklift State Machine

Each forklift runs an independent state machine with 7 possible states:

```
┌───────┐  TaskAssignment  ┌─────────────────────┐  dist<0.3m  ┌─────────┐
│ IDLE  │ ───────────────▶ │ NAVIGATING_TO_SHELF │ ──────────▶ │ PICKING │
│  (0)  │                  │        (1)          │             │   (2)   │
└───────┘                  └─────────────────────┘             └────┬────┘
    ▲                                                               │
    │ TaskStatus=completed                                 ~6s pick sequence
    │                                                               │
┌───┴─────┐  dist<0.3m  ┌────────────────────┐                     │
│DROPPING │ ◀────────── │ NAVIGATING_TO_DOCK │ ◀───────────────────┘
│   (4)   │             │       (3)          │
└─────────┘             └────────────────────┘

Additional states: ERROR (5), RECOVERING (6) — for future fault handling
```

**State transitions:**

| From | To | Trigger |
|------|----|---------|
| IDLE | NAVIGATING_TO_SHELF | Received TaskAssignment matching this forklift_id |
| NAVIGATING_TO_SHELF | PICKING | Distance to shelf position < 0.3m |
| PICKING | NAVIGATING_TO_DOCK | Pick sequence completed (~6 seconds) |
| NAVIGATING_TO_DOCK | DROPPING | Distance to dock position < 0.3m |
| DROPPING | IDLE | Drop sequence completed (~5 seconds), TaskStatus=completed published |

### Proportional Navigation

Phase 1 uses a simple P-controller for point-to-point navigation (no path planning, no obstacle avoidance):

```python
# Desired heading to goal
desired_yaw = atan2(dy, dx)
angle_error = wrap_to_pi(desired_yaw - current_yaw)

# Proportional control
linear_vel  = clamp(KP_LINEAR * distance, 0, MAX_LINEAR_VEL)
angular_vel = clamp(KP_ANGULAR * angle_error, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)

# Reduce speed when turning sharply (> 30°)
if abs(angle_error) > 0.5:
    linear_vel *= 0.3
```

**Parameters:**

| Parameter | Value |
|-----------|-------|
| `KP_LINEAR` | 0.5 |
| `KP_ANGULAR` | 2.0 |
| `MAX_LINEAR_VEL` | 0.5 m/s |
| `MAX_ANGULAR_VEL` | 1.0 rad/s |
| `GOAL_REACHED_DIST` | 0.3 m |

### Pick & Drop Sequences

Both sequences are **timed** (not sensor-based) — this is a Phase 1 simplification.

**Pick sequence (~6 seconds):**

| Step | Duration | Action |
|------|----------|--------|
| 1 | 2.0s | Lower fork to GROUND (0.0m) |
| 2 | 2.0s | Drive forward at 0.1 m/s (into shelf) |
| 3 | 2.0s | Raise fork to LIFTED (0.8m) |

**Drop sequence (~5 seconds):**

| Step | Duration | Action |
|------|----------|--------|
| 1 | 2.0s | Lower fork to GROUND (0.0m) |
| 2 | 2.0s | Reverse at -0.1 m/s (back away) |
| 3 | 1.0s | Raise fork to TRAVEL (0.5m) |

**Fork height positions:**

| Constant | Value | Usage |
|----------|-------|-------|
| `FORK_GROUND` | 0.0 m | Fork lowered to ground (pick/drop start) |
| `FORK_TRAVEL` | 0.5 m | Fork at travel height (idle, post-drop) |
| `FORK_LIFTED` | 0.8 m | Fork raised with item (transport) |

**Timer architecture:**
- Control loop runs at **10 Hz** (`create_timer(0.1)`)
- Status broadcast runs at **2 Hz** (`create_timer(0.5)`)
- Battery level is hardcoded to **100.0** (no simulation of depletion)

---

## Warehouse Manager

**Source:** `warehouse-manager/main.py`, `dispatcher.py`, `models.py`, `scenario.py`

The warehouse manager is a hybrid application combining a FastAPI web server with a ROS 2 node. It serves as the central orchestrator: receiving orders via REST, dispatching tasks to forklifts via ROS 2, and streaming real-time updates to the dashboard via WebSocket.

### REST API

All endpoints are served by uvicorn on port **8000**.

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/forklifts` | List all 4 forklift states (from in-memory dict) |
| `GET` | `/forklifts/{id}` | Get specific forklift state |
| `PUT` | `/forklifts/{id}/pause` | Pause a forklift (prevents dispatch) |
| `PUT` | `/forklifts/{id}/resume` | Resume a paused forklift |
| `POST` | `/orders` | Create order `{items: [...], priority: N}` |
| `GET` | `/orders` | List orders (optional `?status=` filter) |
| `DELETE` | `/tasks/{task_id}` | Cancel a task |
| `GET` | `/metrics` | Fleet KPIs (counts, active/idle stats) |
| `POST` | `/config` | Update dispatch strategy `{dispatch_strategy: "nearest"\|"batched"}` |
| `POST` | `/reset` | Cancel all pending orders/tasks, reset forklifts to IDLE |
| `POST` | `/record/start` | Start scenario recording `{scenario_id: "..."}` |
| `POST` | `/record/stop` | Stop recording, save to JSON file |
| `POST` | `/replay/start` | Replay recorded scenario `{scenario: "scenario_id"}` |
| `POST` | `/replay/stop` | Stop replay |

**CORS:** All origins allowed (`allow_origins=["*"]`) for development.

**Metrics response format:**
```json
{
  "total_orders": 10,
  "completed_orders": 7,
  "in_progress_orders": 2,
  "pending_orders": 1,
  "fleet_size": 4,
  "idle_forklifts": 2,
  "active_forklifts": 2,
  "dispatch_strategy": "nearest"
}
```

### WebSocket Interface

**Endpoint:** `ws://localhost:8000/ws/live`

On connection, the server sends a **snapshot** of current forklift states:
```json
{"type": "snapshot", "data": {"forklifts": [...]}}
```

Subsequent real-time events:
```json
{"type": "forklift_update", "data": {"forklift_id": "forklift_0", "state": 1, ...}}
{"type": "order_update", "data": {"id": "uuid", "status": "in_progress", ...}}
{"type": "task_status", "data": {"task_id": "uuid", "status": "completed", "forklift_id": "forklift_0"}}
{"type": "reset", "data": {}}
```

The WebSocket is server→client only (client messages are ignored).

### ROS 2 Node

`WarehouseManagerNode` extends `rclpy.Node` and runs in a background thread via `loop.run_in_executor(None, rclpy.spin, node)`.

**Subscriptions:**
- `/{forklift_id}/status` (ForkliftStatus) × 4 — updates in-memory forklift dict, broadcasts to WebSocket
- `/warehouse/task_status` (TaskStatus) — updates task/order in SQLite, broadcasts to WebSocket

**Publishers:**
- `/warehouse/task_assignment` (TaskAssignment) — sends dispatch commands to forklift controllers

**Parameter:** `use_sim_time=True` (uses simulation clock from Isaac Sim `/clock` topic)

### Task Dispatch System

When an order is created via `POST /orders`:

1. Order is persisted to SQLite with status `pending`
2. If scenario recording is active, the order event is recorded
3. The order is broadcast to all WebSocket clients
4. `_try_dispatch()` is called immediately

**Dispatch logic (`_try_dispatch`):**
1. Query idle, non-paused forklifts
2. If none available, return (order stays pending)
3. Use the current dispatch strategy to assign:
   - **nearest** (default): Greedy assignment — closest idle forklift to shelf position
   - **batched**: Hungarian algorithm — optimal assignment via `scipy.optimize.linear_sum_assignment`
4. Create task in SQLite, publish `TaskAssignment` via ROS 2
5. Update order status to `in_progress`

**Default positions (Phase 1 hardcoded):**
- Shelf position: `(10.0, 5.0, 0.0)`
- Dock position: `(2.0, 25.0, 0.0)`

#### Dispatcher Algorithms

**`nearest_assign()`** — Greedy nearest-available:
- For each order, find the closest idle forklift by Euclidean distance
- Remove assigned forklift from pool
- O(n×m) complexity

**`batch_assign()`** — Hungarian optimal:
- Build n×m cost matrix of Euclidean distances
- Pad to square matrix with high-cost dummy entries
- Solve with `scipy.optimize.linear_sum_assignment`
- Returns globally optimal minimum-cost assignment
- O(n³) complexity

### Scenario Recording & Replay

The scenario system allows recording order sequences and replaying them for reproducible testing.

**Recording:**
1. `POST /record/start {scenario_id: "test1"}` — starts recording
2. Each `POST /orders` call records the order data with millisecond offset from recording start
3. `POST /record/stop` — saves to `scenarios/test1.json`

**Replay:**
1. `POST /replay/start {scenario: "test1"}` — loads scenario file
2. For each recorded event, schedules `call_later()` at the original time offset
3. Orders are injected as if submitted in real time

**Scenario file format:**
```json
{
  "scenario_id": "test1",
  "start_time": "2026-04-27T12:00:00+00:00",
  "events": [
    {"t_offset_ms": 0, "type": "order", "data": {"items": ["widget"], "priority": 1}},
    {"t_offset_ms": 5000, "type": "order", "data": {"items": ["gadget"], "priority": 2}}
  ]
}
```

### Database Schema

**SQLite** with WAL mode and 5000ms busy timeout. Located at `DATABASE_PATH` (default: `warehouse.db`).

#### orders

| Column | Type | Description |
|--------|------|-------------|
| `id` | TEXT PK | UUID |
| `items` | TEXT | JSON array of item names |
| `priority` | INTEGER | 1 = normal, higher = more urgent |
| `status` | TEXT | `pending`, `in_progress`, `completed`, `cancelled` |
| `assigned_forklift` | TEXT | Forklift ID or NULL |
| `created_at` | TEXT | ISO 8601 timestamp |
| `completed_at` | TEXT | ISO 8601 timestamp or NULL |

#### tasks

| Column | Type | Description |
|--------|------|-------------|
| `id` | TEXT PK | UUID |
| `order_id` | TEXT FK | References orders.id |
| `forklift_id` | TEXT | Assigned forklift ID |
| `status` | TEXT | `pending`, `in_progress`, `completed`, `cancelled`, `failed` |
| `shelf_x/y/z` | REAL | Pick position |
| `dock_x/y/z` | REAL | Drop position |
| `created_at` | TEXT | ISO 8601 |
| `completed_at` | TEXT | ISO 8601 or NULL |

#### metrics_log

| Column | Type | Description |
|--------|------|-------------|
| `id` | INTEGER PK | Auto-increment |
| `timestamp` | TEXT | ISO 8601 |
| `metric_name` | TEXT | Metric identifier |
| `metric_value` | REAL | Numeric value |
| `strategy` | TEXT | Dispatch strategy at time of logging |

---

## Dashboard (React UI)

**Source:** `dashboard/src/`

A React 18 single-page application built with TypeScript and Vite. Uses Zustand for state management, Konva for 2D canvas rendering, and Recharts for charts.

### Components

#### App (`App.tsx`)

Root component. Loads initial state via REST on mount, starts WebSocket connection.

**Layout:**
```
┌──────────────────────────────────────────────────┐
│ Header: "Warehouse Digital Twin | Phase 5"       │
├────────────────────────────┬─────────────────────┤
│ WarehouseMap               │ FleetStatus         │
│ (Konva 2D canvas)          │ (2×2 forklift cards)│
│                            ├─────────────────────┤
│                            │ ConfigPanel         │
├────────────────────────────┤ (strategy toggle)   │
│ MetricsPanel               │                     │
│ (KPI cards + charts)       │                     │
├────────────────────────────┴─────────────────────┤
│ OrderQueue (full-width sortable table)           │
└──────────────────────────────────────────────────┘
```

Max width: 1440px. Dark theme (`#0f0f23` background).

#### WarehouseMap

Konva canvas rendering a 2D top-down view of the warehouse:
- Gray rectangle shelf racks (4 total)
- Blue rectangle loading docks (2 total)
- Colored triangles for forklifts (color encodes state)
- Grid background for scale reference
- Abstract 50×30m coordinate space

#### FleetStatus

2×2 grid of forklift status cards, one per forklift:
- Current position (x, y)
- Battery level
- Current task ID
- State badge (colored by state: green=IDLE, blue=NAV, yellow=PICK/DROP, red=ERROR)
- Pause/Resume buttons

#### OrderQueue

Full-width sortable table of all orders:
- Columns: ID (short UUID), Items, Priority, Status, Assigned Forklift
- Row background color by status: yellow=pending, blue=in_progress, green=completed, gray=cancelled
- Sorted by: status → priority → creation time

#### MetricsPanel

Fleet KPI dashboard:
- **KPI Cards:** total orders, completed orders, active forklifts, dispatch strategy
- **Pie Chart:** Order status breakdown (pending/in_progress/completed/cancelled)
- **Bar Chart:** Idle vs active forklift count
- Polls `GET /metrics` every 2 seconds

#### ConfigPanel

Dispatch strategy configuration:
- Radio buttons: "nearest" (greedy) or "batched" (Hungarian)
- Calls `POST /config` on change

### State Management

Three Zustand stores:

| Store | Key | Description |
|-------|-----|-------------|
| `warehouseStore` | `{[forklift_id]: ForkliftStatus}` | In-memory forklift states |
| `orderStore` | `{[order_id]: Order}` | All orders |
| `metricsStore` | `{current: Metrics, history: Metrics[]}` | KPI data (max 120 samples) |

### Data Flow

1. **On mount:** REST calls `GET /forklifts` and `GET /orders` → populate stores
2. **WebSocket connection:** `useWebSocket()` hook connects to `/ws/live`
   - Auto-reconnects with exponential backoff (1s → 10s)
   - On `snapshot`: bulk-update warehouse store
   - On `forklift_update`: update single forklift in store
   - On `order_update`: upsert order in store
   - On `task_status`: update matching order
   - On `reset`: clear stores
3. **Polling:** MetricsPanel polls `GET /metrics` every 2 seconds
4. **User actions:** ConfigPanel and OrderQueue call REST endpoints via `useApi()` hook

### TypeScript Types

```typescript
interface ForkliftStatus {
  forklift_id: string;
  state: number;           // ForkliftState enum
  pose: { x: number; y: number; z: number; yaw: number };
  battery_level: number;
  current_task_id: string | null;
  paused: boolean;
}

interface Order {
  id: string;
  items: string[];
  priority: number;
  status: "pending" | "in_progress" | "completed" | "cancelled";
  assigned_forklift: string | null;
  created_at: string;
  completed_at: string | null;
}

interface Metrics {
  total_orders: number;
  completed_orders: number;
  in_progress_orders: number;
  pending_orders: number;
  fleet_size: number;
  idle_forklifts: number;
  active_forklifts: number;
  dispatch_strategy: string;
}
```

---

## DDS / FastDDS Configuration

All ROS 2 communication uses FastDDS with shared memory **disabled**. Three XML profiles are provided:

### fastdds_no_shm.xml (Primary — used everywhere)

Forces UDP-only transport. Required because Isaac Sim runs with `ipc=private`, making SHM segments invisible to other containers.

```xml
<transport_descriptor>
  <transport_id>udp_only</transport_id>
  <type>UDPv4</type>
  <sendBufferSize>1048576</sendBufferSize>
  <receiveBufferSize>1048576</receiveBufferSize>
</transport_descriptor>
<participant>
  <rtps>
    <useBuiltinTransports>false</useBuiltinTransports>
    <userTransports><transport_id>udp_only</transport_id></userTransports>
  </rtps>
</participant>
```

### fastdds_discovery.xml (Discovery server config)

Configures the discovery server (SERVER role) on the Brev host.

### fastdds_client.xml (Client config — for split deployment)

Configures CLIENT discovery protocol pointing to the Brev host. Contains `${BREV_TAILSCALE_IP}` placeholder that gets resolved via `envsubst` before use.

**Note:** The Isaac Sim streaming script also generates its own `fastdds_no_shm.xml` at `/tmp/` as a fallback if the volume-mounted version isn't available.

---

## Environment Variables

| Variable | Default | Used By | Description |
|----------|---------|---------|-------------|
| `ROS_DOMAIN_ID` | `42` | All ROS 2 | DDS domain isolation |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | All ROS 2 | FastDDS middleware selection |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | `/fastdds_no_shm.xml` | All ROS 2 | FastDDS transport config |
| `ROS_DISCOVERY_SERVER` | `localhost:11811` | Split deploy | Discovery server address |
| `SCENE_USD` | `/sim-scripts/scenes/scene_assembly.usd` | isaac-sim | USD scene file path |
| `NUM_FORKLIFTS` | `1` (compose) / `4` | vehicle-controller | Number of forklift nodes to launch |
| `REDIS_URL` | `redis://localhost:6379` | warehouse-manager | Redis connection string |
| `DATABASE_PATH` | `warehouse.db` | warehouse-manager | SQLite file path |
| `DISPATCH_STRATEGY` | `nearest` | warehouse-manager | `"nearest"` or `"batched"` |
| `BREV_TAILSCALE_IP` | (required) | local-compose | Brev host Tailscale IP for split deploy |
| `VITE_API_URL` | `http://localhost:8000` | dashboard | REST API base URL |
| `VITE_WS_URL` | `ws://localhost:8000/ws/live` | dashboard | WebSocket endpoint |
| `ACCEPT_EULA` | `Y` | isaac-sim | NVIDIA license acceptance |
| `PRIVACY_CONSENT` | `Y` | isaac-sim | NVIDIA telemetry consent |
| `ISAACSIM_HOST` | `127.0.0.1` | isaac-sim / web-viewer | Streaming bind address |
| `ISAACSIM_SIGNAL_PORT` | `49100` | isaac-sim / web-viewer | WebRTC signaling port |
| `ISAACSIM_STREAM_PORT` | `47998` | isaac-sim / web-viewer | WebRTC stream port |
| `WEB_VIEWER_PORT` | `8210` | web-viewer | HTTP server port |

---

## Makefile Commands

The `Makefile` provides shortcuts for both local and Brev operations:

### Local Commands (local-compose.yml)

| Command | Description |
|---------|-------------|
| `make up` | Start core services (redis + warehouse-manager). Runs `envsubst` on FastDDS client XML. |
| `make down` | Stop all services (including dashboard profile) |
| `make dashboard` | Start dashboard on port 3000 (docker profile) |
| `make dashboard-dev` | Start dashboard in Vite dev mode on port 5173 |
| `make logs` | Tail logs for core services |
| `make logs-dashboard` | Tail dashboard logs |
| `make status` | Show running containers |
| `make restart` | Restart all running services |
| `make clean` | Stop everything and remove volumes |

### Brev Commands (brev-compose.yml)

| Command | Description |
|---------|-------------|
| `make brev-up` | Start all Brev services, wait for Isaac Sim ready signal |
| `make brev-down` | Stop all Brev services |
| `make brev-restart` | Restart all Brev services |
| `make brev-restart-sim` | Restart only Isaac Sim container |
| `make brev-logs` | Tail all Brev service logs |
| `make brev-logs-sim` | Tail Isaac Sim logs only |
| `make brev-status` | Show Brev container status |
| `make brev-clean` | Stop and remove volumes |

---

## Directory Structure

```
nvidia-digital-twin-pilot/
├── brev-compose.yml                    # All-in-one Docker Compose (primary)
├── brev-compose-phase4.yml             # Brev-only compose (split deployment)
├── local-compose.yml                   # Local-only compose (split deployment)
├── local-start.sh                      # Local startup helper script
├── Makefile                            # Command shortcuts
├── fastdds_no_shm.xml                  # FastDDS UDP-only transport
├── fastdds_client.xml                  # FastDDS client discovery (template)
├── fastdds_discovery.xml               # FastDDS server discovery
├── pyproject.toml                      # Python project config
│
├── warehouse_msgs/                     # ROS 2 custom message package (CMake)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── msg/
│   │   ├── ForkliftStatus.msg
│   │   ├── TaskAssignment.msg
│   │   ├── TaskStatus.msg
│   │   └── OrderStatus.msg
│   └── srv/
│       └── ResetSimulation.srv
│
├── vehicle-controller/                 # ROS 2 forklift controller (ament_python)
│   ├── Dockerfile
│   ├── package.xml
│   ├── setup.py / setup.cfg
│   └── vehicle_controller/
│       ├── __init__.py
│       ├── forklift_controller_node.py # State machine + P-controller
│       ├── config/
│       │   └── nav2_params.yaml        # Nav2 config (Phase 3, not yet active)
│       └── launch/
│           └── multi_forklift.launch.py
│
├── warehouse-manager/                  # FastAPI + ROS 2 orchestration server
│   ├── Dockerfile
│   ├── requirements.txt
│   ├── main.py                         # FastAPI app + ROS 2 node
│   ├── main_phase1.py                  # Phase 1 standalone version
│   ├── models.py                       # SQLite schema + helpers
│   ├── dispatcher.py                   # Nearest + Hungarian dispatch
│   ├── scenario.py                     # Recording & replay engine
│   └── tests/
│       └── ...                         # Test suite
│
├── dashboard/                          # React + TypeScript web UI
│   ├── Dockerfile
│   ├── package.json
│   ├── vite.config.ts
│   ├── tsconfig.json
│   └── src/
│       ├── App.tsx                     # Root component
│       ├── config.ts                   # API_URL, WS_URL
│       ├── main.tsx                    # Entry point
│       ├── components/
│       │   ├── WarehouseMap/           # 2D Konva canvas
│       │   ├── FleetStatus/           # Forklift cards
│       │   ├── OrderQueue/            # Order table
│       │   ├── MetricsPanel/          # KPI dashboard
│       │   └── ConfigPanel/           # Strategy config
│       ├── hooks/
│       │   ├── useApi.ts              # REST client
│       │   └── useWebSocket.ts        # WebSocket with reconnect
│       ├── stores/
│       │   ├── warehouseStore.ts      # Forklift state
│       │   ├── orderStore.ts          # Order state
│       │   └── metricsStore.ts        # KPI state
│       └── types/
│           ├── forklift.ts
│           ├── order.ts
│           ├── metrics.ts
│           └── websocket.ts
│
├── sim-scripts/                        # Isaac Sim Python scripts
│   ├── warehouse_scene.py              # Standalone (SimulationApp) version
│   ├── warehouse_scene_streaming.py    # Streaming container (--exec) version
│   ├── core_scripts/                   # Shared utilities
│   └── scenes/                         # (symlinked from simulations/)
│
├── simulations/
│   └── forklift-warehouse/
│       ├── 01_scenes/                  # USD scene files (scene_assembly.usd)
│       ├── 02_core_scripts/            # populate_scene.py, forklift_controller.py
│       ├── 03_dashboard/               # Dashboard assets
│       ├── 04_helper_scripts/          # Diagnostic & test scripts
│       ├── 04_current_outputs/         # Runtime output files
│       ├── 05_reference_milestones/    # Reference implementations
│       └── 06_master_handoff/          # Handoff documentation
│
├── web-viewer/                         # WebRTC viewer static site
│   └── Dockerfile
│
├── smoke_test/                         # VS Code smoke test
│   └── vscode_smoke_test.py
│
└── docs/                               # Documentation
    ├── architecture.puml               # PlantUML system diagram
    ├── ARCHITECTURE.md                 # This file
    └── ...
```
