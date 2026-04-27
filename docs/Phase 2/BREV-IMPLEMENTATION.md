# Brev Instance — Implementation Guide

> **Scope**: Everything that runs on the Brev GPU instance.
> **Current state (2026-04-27)**: Phase 1 complete — single forklift drives via ROS 2 cmd_vel from the vehicle-controller container. All services run on Brev via `brev-compose.yml`.
> **Phase 4+**: Only Isaac Sim, vehicle controller, FastDDS discovery server. Warehouse manager moves to local machine.

---

## 0. Prerequisites & Environment Setup

### 0.1: Provision Brev Instance

| Requirement | Value |
|---|---|
| GPU | L40S (48 GB VRAM) — confirmed working. A10G (24 GB) minimum |
| OS | Ubuntu 22.04 |
| Disk | 100 GB minimum (Isaac Sim image ~20 GB, shader cache ~5 GB) |
| NVIDIA driver | **550+** required for Isaac Sim 5.x (current: 580, CUDA 13.0) |

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

### 0.4: ROS 2 on Host — NOT Needed

All ROS 2 runs inside containers (`ros:jazzy`). No host ROS 2 installation required. For debugging, exec into a container:

```bash
docker exec -it nvidia-digital-twin-pilot-vehicle-controller-1 \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source /ros2_ws/install/setup.bash && \
           ros2 topic list"
```

### 0.5: Clone Repository

```bash
cd ~/docker/isaac-sim/data
git clone <REPO_URL> nvidia-digital-twin-pilot
cd nvidia-digital-twin-pilot
```

### 0.6: Pull Isaac Sim Container Image (slow — do early)

```bash
# Authenticate with NGC
docker login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key from https://ngc.nvidia.com/setup>

docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

**IMPORTANT**: First pull is ~20 GB. First run compiles shaders (~2-5 min). The compose file handles shader cache persistence via named Docker volumes.

---

## 1. Architecture Overview

### 1.1: Tech Stack

| Component | Image / Runtime | ROS Distro | Python |
|---|---|---|---|
| Isaac Sim | `nvcr.io/nvidia/isaac-sim:5.1.0` | Jazzy (bundled) | 3.10 (Kit embedded) |
| Discovery Server | `ros:jazzy` | Jazzy | — |
| Vehicle Controller | Custom from `ros:jazzy` | Jazzy | 3.12 |
| Warehouse Manager | Custom from `ros:jazzy` | Jazzy | 3.12 |
| Redis | `redis:7-alpine` | — | — |
| Web Viewer | Custom Node.js | — | — |

**CRITICAL**: Isaac Sim 5.1.0 uses ROS 2 Jazzy internally. All sidecar containers **MUST** use `ros:jazzy`, not `ros:humble`. Humble containers discover via DDS but fail to deserialize messages due to ABI mismatch.

### 1.2: DDS Configuration

The project uses FastDDS with **shared memory disabled** (`fastdds_no_shm.xml`) and **simple multicast discovery** (no discovery server needed for intra-host). All containers use `network_mode: host` so they share the host network stack.

**`fastdds_no_shm.xml`** — mounted into all containers:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_only</transport_id>
      <type>UDPv4</type>
      <sendBufferSize>1048576</sendBufferSize>
      <receiveBufferSize>1048576</receiveBufferSize>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="participant_profile" is_default_profile="true">
    <rtps>
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports>
        <transport_id>udp_only</transport_id>
      </userTransports>
    </rtps>
  </participant>
</profiles>
```

**Why SHM is disabled**: Isaac Sim container runs with `ipc=private` (Docker default), so SHM segments are invisible to other containers. UDP works cross-container with `network_mode: host`.

### 1.3: Environment Variables (all containers)

| Variable | Value | Notes |
|---|---|---|
| `ROS_DOMAIN_ID` | `42` | Must match everywhere |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | FastDDS — default in Jazzy |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | `/fastdds_no_shm.xml` | Disables SHM transport |
| `ACCEPT_EULA` | `Y` | Isaac Sim only |
| `PRIVACY_CONSENT` | `Y` | Isaac Sim only |

---

## 2. Repository Structure (Brev-relevant parts)

```
nvidia-digital-twin-pilot/
├── brev-compose.yml                 ← PRIMARY — all-in-one compose (current)
├── brev-compose-phase4.yml          ← Future split (sim + controller only, OUTDATED)
├── fastdds_no_shm.xml              ← DDS config, disables shared memory
├── fastdds_discovery.xml           ← Discovery server XML (Phase 4)
├── fastdds_client.xml              ← Discovery client XML (Phase 4, local side)
├── Makefile                        ← make brev-up, brev-down, brev-logs, etc.
├── run_phase1.sh                   ← Sequential startup script
│
├── warehouse_msgs/                 ← ROS 2 message definitions (ament_cmake)
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
├── sim-scripts/
│   ├── warehouse_scene_streaming.py ← Kit --exec script (loads scene + ROS 2 bridge)
│   └── warehouse_scene.py          ← Standalone script (unused in compose)
│
├── simulations/forklift-warehouse/
│   ├── 01_scenes/                  ← USD scene files (scene_assembly.usd)
│   ├── 02_core_scripts/            ← Sim helper scripts (populate_scene.py, etc.)
│   └── 04_helper_scripts/          ← Debug/test scripts (ros2_test_maneuver.py, etc.)
│
├── vehicle-controller/             ← ROS 2 ament_python package
│   ├── Dockerfile
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/vehicle_controller
│   └── vehicle_controller/
│       ├── __init__.py
│       ├── forklift_controller_node.py
│       ├── config/nav2_params.yaml
│       └── launch/multi_forklift.launch.py
│
├── warehouse-manager/              ← FastAPI + ROS 2 hybrid
│   ├── Dockerfile
│   ├── requirements.txt
│   ├── main.py
│   ├── main_phase1.py
│   ├── models.py
│   ├── dispatcher.py
│   ├── scenario.py
│   └── tests/test_dispatcher.py
│
├── dashboard/                      ← React + Vite + TypeScript
│   ├── Dockerfile
│   ├── package.json
│   └── src/ (components, hooks, stores, types)
│
└── web-viewer/                     ← WebRTC viewer proxy
    └── Dockerfile
```

---

## 3. Message Definitions

### 3.1: warehouse_msgs

Built as a standard `ament_cmake` package. Both the vehicle-controller and warehouse-manager Dockerfiles build it via `colcon build`.

**`warehouse_msgs/package.xml`** — dependencies:

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>builtin_interfaces</depend>
<exec_depend>rosidl_default_runtime</exec_depend>
```

**`warehouse_msgs/CMakeLists.txt`** — generated interfaces:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ForkliftStatus.msg"
  "msg/TaskAssignment.msg"
  "msg/TaskStatus.msg"
  "msg/OrderStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
```

**Message definitions**:

`msg/ForkliftStatus.msg`:
```
string forklift_id
uint8 state              # 0=IDLE 1=NAV_TO_SHELF 2=PICKING 3=NAV_TO_DOCK 4=DROPPING 5=ERROR 6=RECOVERING
geometry_msgs/Pose pose
float32 battery_level
string current_task_id
builtin_interfaces/Time stamp
```

`msg/TaskAssignment.msg`:
```
string task_id
string forklift_id
string order_id
geometry_msgs/Point shelf_position
geometry_msgs/Point dock_position
uint8 priority
builtin_interfaces/Time stamp
```

`msg/TaskStatus.msg`:
```
string task_id
string forklift_id
string status            # "pending", "in_progress", "completed", "cancelled", "failed"
string detail
builtin_interfaces/Time stamp
```

`msg/OrderStatus.msg`:
```
string order_id
string status            # "pending", "in_progress", "completed", "cancelled"
string[] items
string assigned_forklift
builtin_interfaces/Time stamp
```

`srv/ResetSimulation.srv`:
```
bool clear_orders
---
bool success
string message
```

---

## 4. Docker Compose — `brev-compose.yml`

This is the primary compose file. Start everything with:

```bash
make brev-up
# OR
docker compose -f brev-compose.yml up --build -d
```

### 4.1: Service Details

#### discovery-server

```yaml
image: ros:jazzy
network_mode: host
command: >
  bash -c "
    source /opt/ros/jazzy/setup.bash &&
    fast-discovery-server -i 0 -p 11811
  "
healthcheck:
  test: ["CMD-SHELL", "grep -q 2E23 /proc/net/udp"]
```

**GOTCHA**: The binary is `fast-discovery-server`, NOT `fastdds discovery` (which is a subcommand that may not exist in all distributions). The healthcheck looks for port 11811 (hex `2E23`) in `/proc/net/udp`.

#### isaac-sim-init (one-shot)

Fixes permissions on named Docker volumes. Isaac Sim runs as UID 1234, but named volumes start as root.

```yaml
image: nvcr.io/nvidia/isaac-sim:5.1.0
user: "0:0"
command: chown -R 1234:1234 /isaac-sim/.nvidia-omniverse/logs ...
```

#### isaac-sim

```yaml
image: nvcr.io/nvidia/isaac-sim:5.1.0
network_mode: host
command: --exec "/sim-scripts/warehouse_scene_streaming.py"
```

**Key volumes**:
- `./fastdds_no_shm.xml:/fastdds_no_shm.xml:ro` — DDS config
- `./sim-scripts:/sim-scripts` — Python scripts
- `./simulations/forklift-warehouse/01_scenes:/sim-scripts/scenes` — USD scene files
- `./simulations/forklift-warehouse/02_core_scripts:/sim-scripts/core_scripts:ro` — Helper scripts
- `isaac-sim-cache`, `isaac-sim-logs`, `isaac-sim-data`, `isaac-sim-config` — persistent volumes

**CRITICAL**: The entrypoint is the streaming container's built-in `isaac-sim.streaming.sh`. It IGNORES `./python.sh` arguments. The `command:` field passes `--exec "/path/to/script.py"` as a Kit CLI flag that runs the script inside the running Kit app event loop. The script must NOT create a `SimulationApp` — it must use `async def` + `asyncio.ensure_future()`.

**Environment**:
- `ISAACSIM_HOST` / `ISAACSIM_SIGNAL_PORT` / `ISAACSIM_STREAM_PORT` — WebRTC streaming config (defaults: 127.0.0.1, 49100, 47998)
- `SCENE_USD=/sim-scripts/scenes/scene_assembly.usd` — path to USD scene inside container

**Healthcheck**: Waits for `/tmp/isaac-sim-ready` sentinel file (written by `warehouse_scene_streaming.py` after scene setup). `start_period: 300s` allows 5 min for first-time shader compilation.

#### web-viewer

WebRTC browser client on port 8210. Connects to Isaac Sim's streaming ports.

```yaml
build: context: ./web-viewer
network_mode: host
command: ["npx", "vite", "preview", "--host", "--port", "${WEB_VIEWER_PORT:-8210}"]
```

Access at `http://<brev-ip>:8210` to view the live simulation.

#### vehicle-controller

```yaml
build:
  context: .
  dockerfile: vehicle-controller/Dockerfile
network_mode: host
command: >
  bash -c "
    source /opt/ros/jazzy/setup.bash &&
    source /ros2_ws/install/setup.bash &&
    ros2 launch vehicle_controller multi_forklift.launch.py
  "
environment:
  - NUM_FORKLIFTS=1     # Currently single forklift; set to 4 for multi
```

**Dockerfile** (`vehicle-controller/Dockerfile`):

```dockerfile
FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rmw-fastrtps-cpp \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src
COPY warehouse_msgs/ warehouse_msgs/
COPY vehicle-controller/ vehicle_controller/

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install && \
    echo 'source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc"

RUN pip3 install --no-cache-dir --break-system-packages redis
```

**GOTCHA**: `ros:jazzy` uses Python 3.12 — `pip install` requires `--break-system-packages` flag.

**GOTCHA**: `setup.cfg` MUST have `[develop]` and `[install]` sections pointing to `$base/lib/vehicle_controller`, otherwise `ros2 run` can't find the entry point:

```ini
[develop]
script_dir=$base/lib/vehicle_controller

[install]
install_scripts=$base/lib/vehicle_controller
```

#### warehouse-manager

```yaml
build:
  context: .
  dockerfile: warehouse-manager/Dockerfile
network_mode: host
command: >
  bash -c "
    source /opt/ros/jazzy/setup.bash &&
    source /ros2_ws/install/setup.bash &&
    uvicorn main:app --host 0.0.0.0 --port 8000
  "
```

**Dockerfile** (`warehouse-manager/Dockerfile`):

```dockerfile
FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-jazzy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY warehouse-manager/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt

WORKDIR /ros2_ws/src
COPY warehouse_msgs/ warehouse_msgs/

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select warehouse_msgs && \
    echo 'source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc"

COPY warehouse-manager/ /app/
WORKDIR /app
EXPOSE 8000
```

**NOTE**: The Dockerfile's CMD still references `ros:humble` — this is harmless because the compose `command:` overrides it. Fix it if rebuilding without compose:
```dockerfile
# WRONG (in Dockerfile CMD):
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && ..."]
# CORRECT:
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && ..."]
```

**`requirements.txt`**:
```
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
aiosqlite>=0.19.0
redis>=5.0.0
pyyaml>=6.0
websockets>=12.0
scipy>=1.11.0
```

#### redis

```yaml
image: redis:7-alpine
network_mode: host
```

No port mapping needed since all containers use host networking.

### 4.2: Named Volumes

```yaml
volumes:
  isaac-sim-cache:    # Shader cache (~5 GB after first run)
  isaac-sim-logs:     # Omniverse logs
  isaac-sim-data:     # Omniverse data
  isaac-sim-config:   # Omniverse config
```

---

## 5. Isaac Sim Streaming Script

### 5.1: `warehouse_scene_streaming.py` — How It Works

This is the heart of the simulation. It runs as a Kit `--exec` script inside the Isaac Sim streaming container.

**Startup sequence**:
1. Writes `fastdds_no_shm.xml` to `/tmp/` if not mounted (fallback)
2. Sets `FASTRTPS_DEFAULT_PROFILES_FILE` env var
3. Opens `SCENE_USD` (default: `/sim-scripts/scenes/scene_assembly.usd`)
4. Waits 120 frames for stage to settle
5. Enables `omni.isaac.vscode` extension (port 8226 for VS Code remote)
6. Configures PhysX: TGS solver, GPU dynamics, 60 Hz, Z-up gravity
7. Sets up joint DriveAPI (stiffness/damping) on all forklift drive/steer joints
8. Creates OmniGraph per forklift with ROS 2 bridge nodes
9. Creates a pick box at (5, 10, 0.6)
10. Starts timeline (plays simulation)
11. Starts `cmd_vel_bridge_loop` — reads twist_sub OG outputs → sets ArticulationController targets each frame
12. Writes `/tmp/isaac-sim-ready` sentinel for Docker healthcheck

### 5.2: OmniGraph Per Forklift

Each forklift gets an OmniGraph at `/World/ROS2BridgeGraph_{fid}` with these nodes:

| Index | Node | Type | Purpose |
|---|---|---|---|
| 0 | on_playback_tick | `omni.graph.action.OnPlaybackTick` | Tick source |
| 1 | clock_pub | `isaacsim.ros2.bridge.ROS2PublishClock` | Publishes `/clock` |
| 2 | twist_sub | `isaacsim.ros2.bridge.ROS2SubscribeTwist` | Subscribes `/{fid}/cmd_vel` |
| 3 | compute_odom | `isaacsim.core.nodes.IsaacComputeOdometry` | Computes odometry from chassis |
| 4 | odom_pub | `isaacsim.ros2.bridge.ROS2PublishOdometry` | Publishes `/{fid}/odom` |
| 5 | read_sim_time | `isaacsim.core.nodes.IsaacReadSimulationTime` | Simulation time source |
| 6 | drive_controller | `isaacsim.core.nodes.IsaacArticulationController` | Velocity control on `back_wheel_drive` |
| 7 | steer_controller | `isaacsim.core.nodes.IsaacArticulationController` | Position control on `back_wheel_swivel` |

**CRITICAL**: Isaac Sim 5.1.0 renamed all node types:
- `omni.isaac.ros2_bridge.*` → `isaacsim.ros2.bridge.*`
- `omni.isaac.core_nodes.*` → `isaacsim.core.nodes.*`
- `omni.isaac.wheeled_robots.*` → `isaacsim.robot.wheeled_robots.*`

### 5.3: cmd_vel Bridge Loop

The bridge runs every frame and converts ROS 2 Twist → ArticulationController targets:

```
linear_x  = twist.linear.x    # m/s forward
angular_z = twist.angular.z   # rad/s rotation

drive_vel = linear_x * DRIVE_SCALE    # DRIVE_SCALE = -1.0/WHEEL_RADIUS ≈ -6.667
steer_rad = angular_z * STEER_SCALE   # STEER_SCALE = radians(20) ≈ 0.349
steer_rad = clamp(steer_rad, -STEER_MAX, +STEER_MAX)  # STEER_MAX = radians(30)
```

**Why negative DRIVE_SCALE**: In this USD asset, negative angular velocity on `back_wheel_drive` means forks-forward motion.

### 5.4: Joint Physics Parameters

Set before simulation starts via `UsdPhysics.DriveAPI.Apply()`:

| Joint | Stiffness | Damping | Control Mode |
|---|---|---|---|
| `back_wheel_drive` | 0 | 15,000 | Pure velocity |
| `back_wheel_swivel` | 40,000 | 10,000 | Position (spring) |

**GOTCHA**: Must use `DriveAPI.Apply()` (not just constructor) to ensure the drive schema exists on joint prims.

### 5.5: Scene Prim Paths

```
/World/warehouse                              — Environment
/World/forklift_b                             — Template (deactivated)
/World/forklift_0 .. /World/forklift_3        — Active forklifts (cloned from forklift_b)
  /{id}/body                                  — Rigid body (articulation root)
  /{id}/back_wheel_joints/back_wheel_drive    — Rear drive joint
  /{id}/back_wheel_joints/back_wheel_swivel   — Rear steer joint
/World/Rack_A .. /World/Rack_D                — Shelf racks (box collision prims)
/World/Dock_1, /World/Dock_2                  — Loading docks
```

Spawn positions: `forklift_0 (-15,-25)`, `forklift_1 (-5,-25)`, `forklift_2 (5,-25)`, `forklift_3 (15,-25)`

Warehouse bounds: X ≈ -36..+33, Y ≈ -36..+52

---

## 6. Vehicle Controller

### 6.1: State Machine

```
IDLE → NAVIGATING_TO_SHELF → PICKING → NAVIGATING_TO_DOCK → DROPPING → IDLE
                                                                  ↓
                                                               ERROR → RECOVERING → (retry)
```

### 6.2: `forklift_controller_node.py`

**Subscribes**:
- `/{forklift_id}/odom` (`nav_msgs/Odometry`) — BEST_EFFORT QoS
- `/warehouse/task_assignment` (`warehouse_msgs/TaskAssignment`) — RELIABLE/TRANSIENT_LOCAL

**Publishes**:
- `/{forklift_id}/cmd_vel` (`geometry_msgs/Twist`) — at 10 Hz timer
- `/{forklift_id}/fork_cmd` (`std_msgs/Float64`)
- `/warehouse/task_status` (`warehouse_msgs/TaskStatus`)
- `/{forklift_id}/status` (`warehouse_msgs/ForkliftStatus`)

**Phase 1 proportional controller**:
```
KP_LINEAR  = 0.5      # m/s per metre error
KP_ANGULAR = 2.0      # rad/s per radian error
MAX_LINEAR_VEL  = 0.5  # m/s
MAX_ANGULAR_VEL = 1.0  # rad/s
GOAL_REACHED_DIST = 0.3  # metres
```

**GOTCHA**: Do NOT call `self.declare_parameter('use_sim_time', True)` — it's auto-declared by `rclpy.Node.__init__()`. Pass via `--ros-args -p use_sim_time:=true` or `parameter_overrides=`.

**GOTCHA**: Publish `cmd_vel` at a fixed rate (10 Hz timer), NOT in the odom callback. Odom callback rate depends on sim performance and can cause jerky motion.

### 6.3: Launch File

`vehicle_controller/launch/multi_forklift.launch.py`:
- Reads `NUM_FORKLIFTS` env var (default 4)
- Creates one `PushRosNamespace(forklift_{i})` group per forklift
- Each group runs one `forklift_controller_node` with `parameters=[{'forklift_id': ns, 'use_sim_time': True}]`

---

## 7. Warehouse Manager

### 7.1: FastAPI + ROS 2 Hybrid Architecture

- FastAPI runs in the main asyncio event loop (uvicorn)
- ROS 2 node runs in a background thread via `MultiThreadedExecutor`
- `use_sim_time` set via `parameter_overrides` in `WarehouseManagerNode.__init__()` (NOT via CLI `--ros-args`, so uvicorn launches normally)
- Shared state bridged through in-memory dicts with thread safety
- Redis for hot state (forklift positions at 10 Hz) and pub/sub for WebSocket fan-out

### 7.2: Key Files

| File | Purpose |
|---|---|
| `main.py` | FastAPI app + `WarehouseManagerNode` ROS 2 bridge |
| `models.py` | SQLite schema + helpers (aiosqlite, WAL mode) |
| `dispatcher.py` | Task assignment: `nearest_assign()` (greedy) + `batch_assign()` (Hungarian/scipy) |
| `scenario.py` | Record/replay order sequences |
| `requirements.txt` | Python deps: fastapi, uvicorn, aiosqlite, redis, scipy, etc. |

### 7.3: SQLite Schema

Tables: `orders`, `tasks`, `metrics_log`

```python
# WAL mode + busy_timeout for concurrent FastAPI + ROS thread access
await db.execute("PRAGMA journal_mode=WAL")
await db.execute("PRAGMA busy_timeout=5000")
```

### 7.4: `WarehouseManagerNode`

```python
class WarehouseManagerNode(Node):
    def __init__(self):
        super().__init__(
            "warehouse_manager",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
```

**Subscribes**: `/{fid}/status` for all 4 forklifts, `/warehouse/task_status`
**Publishes**: `/warehouse/task_assignment`
**Dispatch strategies**: `DISPATCH_STRATEGY` env var — `"nearest"` (default) or `"batched"`

### 7.5: API Endpoints (Phase 2)

| Method | Endpoint | Purpose |
|---|---|---|
| `GET` | `/forklifts` | All forklift states (from in-memory dict) |
| `GET` | `/orders` | All orders (from SQLite) |
| `POST` | `/orders` | Create new order `{"items": [...], "priority": N}` |
| `GET` | `/metrics` | KPI snapshots |
| `POST` | `/reset` | Cancel all tasks, reset forklifts |
| `WS` | `/ws/live` | Real-time events (forklift_update, order_update, task_status, etc.) |
| `GET` | `/docs` | Swagger UI |

---

## 8. Running the System

### 8.1: Quick Start (Makefile)

```bash
cd /home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot

# Start everything and wait for Isaac Sim to be ready
make brev-up

# Monitor logs
make brev-logs

# Isaac Sim logs only
make brev-logs-sim

# Check status
make brev-status

# Restart just Isaac Sim
make brev-restart-sim

# Stop everything
make brev-down

# Stop + remove volumes (clean slate)
make brev-clean
```

### 8.2: Manual Startup (run_phase1.sh)

```bash
./run_phase1.sh
```

This starts services sequentially: discovery-server → redis → isaac-sim → vehicle-controller → warehouse-manager.

### 8.3: Verification

```bash
# Check all containers are running
docker compose -f brev-compose.yml ps

# Expected:
# discovery-server     Up (healthy)
# isaac-sim            Up (healthy)     ← takes 2-5 min first time
# redis                Up (healthy)
# vehicle-controller   Up
# warehouse-manager    Up
# web-viewer           Up (healthy)

# Test ROS 2 topics (from inside vehicle-controller container)
docker exec nvidia-digital-twin-pilot-vehicle-controller-1 \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source /ros2_ws/install/setup.bash && \
           ros2 topic list"
# Expected topics:
#   /clock
#   /forklift_0/cmd_vel
#   /forklift_0/odom
#   /forklift_0/status
#   /warehouse/task_assignment
#   /warehouse/task_status

# Test API
curl http://localhost:8000/docs          # Swagger UI
curl http://localhost:8000/forklifts     # Forklift states
curl http://localhost:8000/orders        # Order list

# Test WebRTC viewer
# Open http://<brev-ip>:8210 in browser

# Test forklift movement via helper script
docker cp simulations/forklift-warehouse/04_helper_scripts/ros2_test_maneuver.py \
  nvidia-digital-twin-pilot-vehicle-controller-1:/tmp/ros2_test_maneuver.py
docker exec nvidia-digital-twin-pilot-vehicle-controller-1 \
  bash -c "source /opt/ros/jazzy/setup.bash && python3 /tmp/ros2_test_maneuver.py"
```

### 8.4: Validation Checklist

| Check | Command | Expected |
|---|---|---|
| Clock topic exists | `ros2 topic list \| grep clock` | `/clock` |
| Clock is ticking | `ros2 topic hz /clock` | ~60 Hz |
| Odom publishing | `ros2 topic hz /forklift_0/odom` | ~60 Hz |
| cmd_vel works | Send test twist, watch odom | Position changes |
| API responds | `curl localhost:8000/forklifts` | JSON with forklift states |
| WebRTC works | Open `:8210` in browser | See warehouse scene |

---

## 9. Key Gotchas & Lessons Learned

1. **`fast-discovery-server` binary**, not `fastdds discovery` subcommand — the latter may not exist
2. **Isaac Sim 5.1.0 uses Jazzy** — all sidecars must use `ros:jazzy` (not humble)
3. **`ros:jazzy` = Python 3.12** — pip needs `--break-system-packages`
4. **Isaac Sim ignores `./python.sh` in streaming mode** — use `--exec "/script.py"` Kit CLI flag
5. **`--exec` scripts must be async** — use `asyncio.ensure_future()`, NOT `SimulationApp()`
6. **OmniGraph node types renamed** in 5.1.0: `omni.isaac.*` → `isaacsim.*`
7. **ArticulationController** is the correct way to drive joints (not DriveAPI attribute setting at runtime — that doesn't propagate to PhysX with GPU dynamics)
8. **`targetPrim` format**: Must use `[usdrt.Sdf.Path("/World/forklift_0")]` (list of Sdf.Path)
9. **SHM must be disabled** — Docker containers have isolated IPC namespaces
10. **`rclpy.shutdown()` guard**: Always use `if rclpy.ok():` before shutdown to avoid double-shutdown errors
11. **`use_sim_time` auto-declared** by rclpy — don't declare it again or you get `ParameterAlreadyDeclaredException`
12. **`setup.cfg` required** for ament_python — must have `[develop]` and `[install]` sections
13. **FastDDS SHM disabled via XML**, not environment variable — the `fastdds_no_shm.xml` file is mounted into every container and also generated as fallback inside `warehouse_scene_streaming.py`
14. **Isaac Sim container UID**: Runs as UID 1234 — named volumes need `isaac-sim-init` one-shot to fix permissions
15. **DRIVE_SCALE is negative** (-6.667 rad/s per m/s) because negative angular velocity on `back_wheel_drive` means forks-forward in this USD asset

---

## 10. Phase 4 — Split Deployment (Brev Side)

> Not yet active. When ready, switch from `brev-compose.yml` to `brev-compose-phase4.yml`.

**IMPORTANT**: The existing `brev-compose-phase4.yml` is **outdated** — it still references `ros:humble` and `./python.sh`. Before using it, update to match the current `brev-compose.yml`:
- `ros:humble` → `ros:jazzy` for discovery-server and vehicle-controller
- Source `/opt/ros/jazzy/setup.bash` instead of humble
- Isaac Sim command to use `--exec` instead of `./python.sh`
- Add `fastdds_no_shm.xml` volume mount and env vars to all services
- Add `isaac-sim-init` service for volume permissions
- Add `web-viewer` service
- Add `PRIVACY_CONSENT=Y` env var to isaac-sim
- Copy volume mount pattern from current `brev-compose.yml`

The local machine will run warehouse-manager + Redis + dashboard, connecting to Brev via Tailscale. See `LOCAL-IMPLEMENTATION.md` for the local side.

### 10.1: Install Tailscale (needed from Phase 4)

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
# Sign in with the SAME Tailscale account used on your local machine
tailscale ip -4
# Note the IP — local machine will need this
```

Save the Brev Tailscale IP in your project `.env` file:

```bash
echo "BREV_TAILSCALE_IP=$(tailscale ip -4)" >> .env
```
