# NVIDIA Isaac Sim — GPU Instance Automation & ROS2 Integration

Automated provisioning, deployment, and operation of an **NVIDIA Isaac Sim 5.1.0** GPU instance on [brev.dev](https://brev.dev). A Jetbot robot navigates an 8×8 m walled arena with obstacle avoidance, streamed via WebRTC and monitored through a real-time web dashboard. All control runs over **ROS2 Humble** topics bridged from the simulator.

---

## Prerequisites

| Requirement | Setup |
|-------------|-------|
| **macOS** (or Linux) | Local development machine — all editing and deployment happens here |
| **brev CLI** | `brew install brevdev/homebrew-brev/brev` → `brev login` (creates `~/.brev/brev.pem` automatically) |
| **NGC API key** | Free account at [ngc.nvidia.com](https://ngc.nvidia.com) → Setup → Generate API Key → save to `~/.ngc/key` (`chmod 600`) |
| **GPU instance** | Provisioned via brev.dev — see `scripts/provision/create_instance.sh` |

**No secrets are stored in this repository.** The NGC key lives at `~/.ngc/key` on your Mac and is injected into scripts at deploy time (see [Secrets Management](#secrets-management)). The brev SSH key is managed automatically by the brev CLI.

### Quick Start

```bash
# 1. Store your NGC API key (one-time)
mkdir -p ~/.ngc && chmod 700 ~/.ngc
echo "nvapi-YOUR_KEY_HERE" > ~/.ngc/key && chmod 600 ~/.ngc/key

# 2. Set instance IP and deploy
export ISAAC_SIM_IP=<your-instance-ip>
bash scripts/deploy_to_instance.sh

# 3. SSH into instance and run first-time setup (~10 min)
brev shell isaac-sim-2
sudo bash ~/setup.sh

# 4. Start Isaac Sim + ROS2 stack
bash ~/launch_isaac_sim.sh          # in brev shell
bash ~/start_ros2_all.sh            # in a second brev shell tab

# 5. Open streaming viewer (on Mac)
bash scripts/launch_streaming.sh
# Dashboard at http://<instance-ip>:8080
```

---

## Concepts & Technology

### NVIDIA Isaac Sim

[Isaac Sim](https://developer.nvidia.com/isaac-sim) is NVIDIA's robotics simulation platform built on top of **NVIDIA Omniverse**. It provides physically accurate simulation of robots, sensors, and environments using:

- **PhysX 5** for rigid-body dynamics, joint articulation, and collision detection
- **RTX rendering** for photorealistic visuals and camera sensors
- **PhysX raycast sensors** for lidar, ultrasonic, and proximity sensing
- **USD (Universal Scene Description)** as the scene format — every object, material, light, and sensor is a USD prim on the stage

Isaac Sim runs as a **containerized application** (`nvcr.io/nvidia/isaac-sim:5.1.0`) requiring an NVIDIA GPU. It exposes a WebRTC streaming interface so the full 3D viewport can be viewed in a browser — no local GPU required on the developer machine. Scripts are executed inside the simulation through the **Script Editor** (Window → Script Editor → Ctrl+Enter), which runs Python code in the same process as the simulator, with direct access to the physics engine, USD stage, and all extensions.

In this project, Isaac Sim runs on a remote GPU instance (NVIDIA RTX A4000, 16 GB VRAM) provisioned through brev.dev. The developer edits scripts locally on a Mac and deploys them to the instance via SCP.

### OmniGraph

**OmniGraph** is the visual/programmatic dataflow graph system inside Omniverse. It replaces traditional scripting for real-time data pipelines by connecting **nodes** (typed compute units) through **attributes** (typed input/output ports).

In this project, OmniGraph is used exclusively for the **ROS2 bridge wiring** — a 14-node execution graph that runs every simulation frame:

```
OnPlaybackTick ─── fires once per frame at ~60 Hz
       │
       ├──→ IsaacReadLidar ──→ PublishLaserScan     (sensor → ROS2)
       ├──→ ComputeOdometry ──→ PublishOdometry      (physics → ROS2)  
       ├──→ PublishClock                              (sim time → ROS2)
       └──→ SubscribeTwist ──→ DiffController ──→ ArticulationController
                                                      (ROS2 → actuators)
```

Each node has a specific **type** (e.g. `isaacsim.ros2.bridge.ROS2PublishOdometry`) and **attributes** (e.g. `inputs:topicName`, `outputs:position`). The graph is built programmatically in Python using `og.Controller.edit()` with explicit node creation, value assignment, and connection wiring. The graph evaluator is set to `"execution"` mode, meaning nodes only compute when triggered by an execution signal (the tick).

Key OmniGraph API patterns used:
- **Two-argument attribute access:** `og.Controller.attribute("outputs:position", "/World/ROS2Graph/ComputeOdom")` — the attribute name and node prim path must be separate arguments
- **Target-type attributes** (prims, meshes): Must be set as `[usdrt.Sdf.Path("/World/Jetbot")]` — a list of `usdrt.Sdf.Path` objects
- **BreakVector3 decomposition:** `SubscribeTwist` outputs `double3` vectors, but `DifferentialController` expects scalar `double` inputs — `omni.graph.nodes.BreakVector3` extracts `.x` or `.z` components

> **Isaac Sim 5.1.0 naming change:** All node types changed from `omni.isaac.*` to `isaacsim.*`. For example, `omni.isaac.ros2_bridge.ROS2PublishOdometry` became `isaacsim.ros2.bridge.ROS2PublishOdometry`.

### OmniGraph ROS2 Bridge

The **ROS2 bridge** is an Isaac Sim extension (`isaacsim.ros2.bridge`) that provides OmniGraph node types for publishing and subscribing to ROS2 topics directly from the simulation loop. It links against a real ROS2 Humble DDS stack (FastDDS) compiled into the Isaac Sim container.

How the bridge works at runtime:

1. A **`ROS2Context`** node initializes a DDS participant inside the simulator process. This participant joins the DDS domain (domain ID 0) and discovers other participants on the network.
2. **Publisher nodes** (e.g. `ROS2PublishOdometry`) create DDS writers. On each tick, they serialize the data from their OmniGraph input attributes into the corresponding ROS2 message type and publish it.
3. **Subscriber nodes** (e.g. `ROS2SubscribeTwist`) create DDS readers. On each tick, they check for new messages and expose the deserialized fields as OmniGraph output attributes.
4. **Timestamps** must be explicitly wired: `ReadSimulationTime.outputs:simulationTime` connects to each publisher's `inputs:timeStamp`. Without this, messages have zero timestamps.

The bridge publishes with **BEST_EFFORT** QoS reliability. Any external subscriber (dashboard, sidecar) must also use BEST_EFFORT — a RELIABLE subscriber silently drops all messages from a BEST_EFFORT publisher.

Data flows at the simulation frame rate (~60 Hz for physics). The actual topic publish rate may be lower depending on the node type — the PhysX lidar rotates at 20 Hz, so `/scan` publishes at ~18 Hz; odometry computes every frame at ~50 Hz.

### ROS2 (Robot Operating System 2)

**ROS2** is the standard middleware for robotics software. It provides:

- **Topics** — publish/subscribe message channels (e.g. `/cmd_vel`, `/scan`, `/odom`)
- **Messages** — strongly-typed serializable data structures (e.g. `geometry_msgs/Twist`, `sensor_msgs/LaserScan`)
- **Nodes** — independent processes that communicate through topics
- **QoS policies** — configurable reliability, durability, and history for each subscription
- **DDS transport** — the underlying wire protocol (this project uses **FastDDS** / `rmw_fastrtps_cpp`)

This project uses **ROS2 Humble Hawksbill** (the LTS release). Three separate DDS participants exist in the system:

| Participant | Container | Role |
|-------------|-----------|------|
| Isaac Sim bridge | `isim-isaac-sim-1` | Publishes `/odom`, `/scan`, `/clock`; subscribes `/cmd_vel` |
| Obstacle avoidance | `ros2-sidecar` | Subscribes `/scan`; publishes `/cmd_vel` |
| Dashboard | `ros2-dashboard` | Subscribes `/odom`, `/scan`, `/cmd_vel`, `/clock` (read-only) |

All three use `--net=host` Docker networking, so DDS discovery (UDP multicast on port 7400) works automatically. The critical issue is that FastDDS defaults to **shared-memory (SHM) transport** for data, which fails across Docker IPC namespaces — hence the UDP-only XML profile applied to every container.

### The Sidecar Pattern

A **sidecar** is a companion container that runs alongside a primary container to provide supplementary functionality. In this project:

- **Primary container:** `isim-isaac-sim-1` — runs Isaac Sim with the ROS2 bridge. This container is large (~20 GB image), GPU-bound, and has its own IPC namespace. We cannot easily install additional ROS2 packages or run arbitrary ROS2 nodes inside it.
- **Sidecar container:** `ros2-sidecar` — a lightweight ROS2 Humble desktop image (`osrf/ros:humble-desktop`). It shares the host network namespace (`--net=host`) so DDS topics are mutually visible. It runs the obstacle avoidance node and provides a full ROS2 CLI (`ros2 topic list`, `ros2 topic echo`, etc.) for debugging.
- **Dashboard container:** `ros2-dashboard` — another sidecar specifically for monitoring. Subscribes to topics passively and serves a web UI.

The sidecar pattern decouples **simulation** (physics, rendering, sensor simulation) from **robot logic** (obstacle avoidance, path planning, SLAM). This mirrors real-world robotics where the control software runs on a separate computer from the sensor hardware.

### Simulation ↔ ROS2 Interaction Model

The following interactions are possible between external ROS2 code and the Isaac Sim simulation:

| Direction | Topic | What it does | Example |
|-----------|-------|-------------|---------|
| **Sim → ROS2** | `/odom` | Read robot pose, velocity | Navigation, localization |
| **Sim → ROS2** | `/scan` | Read 360° lidar distances | Obstacle detection, mapping |
| **Sim → ROS2** | `/clock` | Read simulation time | Synchronized timing |
| **ROS2 → Sim** | `/cmd_vel` | Command robot velocity | Teleoperation, autonomy |

**What you can do from a ROS2 sidecar:**

- **Teleoperate:** `ros2 run teleop_twist_keyboard teleop_twist_keyboard` — drive the robot with keyboard arrows
- **Autonomous control:** Run any node that subscribes to `/scan` and publishes `/cmd_vel` — the robot moves in simulation
- **Record data:** `ros2 bag record -a` — save all topics to a rosbag for replay
- **Visualize:** `rviz2` (if X11 forwarded) or build custom dashboards (like this project's web dashboard)
- **Inspect:** `ros2 topic hz /scan`, `ros2 topic echo /odom --once`, `ros2 topic info /cmd_vel`
- **Bridge to other systems:** Any ROS2 node on the same DDS domain can interact — SLAM stacks (Nav2), planners, ML inference nodes, etc.

The simulation is a **drop-in replacement for real hardware**. Code written against these ROS2 topics would work identically on a physical Jetbot with a real lidar, with no code changes — only the DDS discovery network would differ.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  Mac (local workstation)                                                     │
│                                                                              │
│  export ISAAC_SIM_IP=94.101.98.52                                            │
│                                                                              │
│  ┌─────────────────────────┐   ┌────────────────────────────┐                │
│  │ deploy_to_instance.sh   │   │ launch_streaming.sh        │                │
│  │ • Injects NGC key       │   │ • SSH tunnel :8210→:8210   │                │
│  │ • SCP all scripts       │   │ • Polls readiness endpoint │                │
│  │ • Copies into container │   │ • Opens Chrome             │                │
│  └──────────┬──────────────┘   └────────────────────────────┘                │
└─────────────┼────────────────────────────────────────────────────────────────┘
              │ SCP + SSH (-o ControlPath=none -i ~/.brev/brev.pem)
              ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│  GPU Instance — isaac-sim-2 (brev.dev / Hyperstack Oslo)                     │
│  NVIDIA RTX A4000 16 GB · 4 vCPU · Ubuntu · User: shadeform                 │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐     │
│  │  isim-isaac-sim-1  (nvcr.io/nvidia/isaac-sim:5.1.0)                 │     │
│  │  --net=host · ipc=private                                           │     │
│  │                                                                     │     │
│  │  ┌───────────────┐  ┌──────────────┐  ┌─────────────────────────┐   │     │
│  │  │ ros2_jetbot.py │  │hello_robot.py│  │ OmniGraph ROS2 Bridge   │   │     │
│  │  │ (Script Editor)│  │(Script Editor│  │ 14 nodes                │   │     │
│  │  │               │  │  standalone) │  │ Execution evaluator     │   │     │
│  │  └───────────────┘  └──────────────┘  └──────┬──────────────────┘   │     │
│  │                                               │ DDS (UDP :7400+)    │     │
│  │  Ports: 8210 (WebRTC UI), 8011 (API),         │                     │     │
│  │         49100 (stream), 47998/udp (stream)    │                     │     │
│  └───────────────────────────────────────────────┼─────────────────────┘     │
│                                                  │                           │
│  ┌───────────────────────────────────┐           │ FastDDS UDP-only           │
│  │  ros2-sidecar                     │           │ (SHM disabled)             │
│  │  osrf/ros:humble-desktop          │◄──────────┘                           │
│  │  --net=host · --ipc=host          │                                       │
│  │                                   │                                       │
│  │  obstacle_avoidance_node.py       │                                       │
│  │  /scan → gap steering → /cmd_vel  │                                       │
│  └───────────────────────────────────┘                                       │
│                                                                              │
│  ┌───────────────────────────────────┐                                       │
│  │  ros2-dashboard                   │                                       │
│  │  ros:humble-ros-base + aiohttp    │                                       │
│  │  --net=host · --gpus all          │                                       │
│  │                                   │                                       │
│  │  Port 8080 → WebSocket dashboard  │                                       │
│  │  ROS2 subs: /odom /scan /cmd_vel  │                                       │
│  │  System: nvidia-smi, docker ps    │                                       │
│  └───────────────────────────────────┘                                       │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## Network & Ports

| Port | Protocol | Service | Direction |
|------|----------|---------|-----------|
| 22 | TCP | SSH (system) | Inbound |
| 2222 | TCP | SSH (brev) | Inbound |
| 8011 | TCP | Isaac Sim REST API | Internal/tunnel |
| 8210 | TCP | Isaac Sim WebRTC UI | Tunneled to Mac |
| 49100 | TCP | Isaac Sim WebRTC stream | Inbound |
| 47998 | UDP | Isaac Sim WebRTC stream | Inbound |
| 8080 | TCP | Web dashboard | Inbound |
| 7400+ | UDP | DDS discovery & data (FastDDS) | localhost only |

UFW firewall is configured by `setup.sh` to allow only required ports.

---

## Docker Containers

Three containers run on the instance, all using `--net=host` for shared UDP namespace:

| Container | Image | Key Flags | Purpose |
|-----------|-------|-----------|---------|
| `isim-isaac-sim-1` | `nvcr.io/nvidia/isaac-sim:5.1.0` | `--gpus all`, `ipc=private` | Isaac Sim simulator + ROS2 bridge |
| `ros2-sidecar` | `osrf/ros:humble-desktop` | `--ipc=host` | ROS2 runtime for obstacle avoidance node |
| `ros2-dashboard` | `ros2-dashboard:latest` (custom) | `--gpus all`, `--pid=host` | Web monitoring + ROS2 subscriber |

### Critical: FastDDS Shared-Memory Fix

Isaac Sim's container runs with Docker default `ipc=private`, isolating its `/dev/shm` namespace. FastDDS uses shared memory for data transport by default. This means:

- **DDS discovery works** (UDP multicast) — topics are visible across containers
- **DDS data transport fails** (SHM segments are invisible across IPC namespaces)

**Fix:** All three containers use a FastDDS XML profile that disables SHM and forces UDP-only transport:

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

Written to `/tmp/fastdds_no_shm.xml` and activated via `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable. The 1 MB buffer sizes are necessary because a 360-ray LaserScan message exceeds the default UDP buffer.

---

## ROS2 Topics

All topics use **ROS2 Humble** with **rmw_fastrtps_cpp** middleware, `ROS_DOMAIN_ID=0`.

| Topic | Message Type | Publisher | Subscriber | Rate | QoS |
|-------|-------------|-----------|------------|------|-----|
| `/clock` | `rosgraph_msgs/Clock` | Isaac Sim (OmniGraph) | Dashboard | ~56 Hz | BEST_EFFORT |
| `/odom` | `nav_msgs/Odometry` | Isaac Sim (OmniGraph) | Dashboard | ~50 Hz | BEST_EFFORT |
| `/scan` | `sensor_msgs/LaserScan` | Isaac Sim (OmniGraph) | Obstacle avoidance, Dashboard | ~18 Hz | BEST_EFFORT |
| `/cmd_vel` | `geometry_msgs/Twist` | Obstacle avoidance node | Isaac Sim (OmniGraph) | ~10 Hz | BEST_EFFORT |

### Message Structures

**`/odom` — nav_msgs/Odometry**
```
header:
  stamp: {sec, nanosec}      # simulation time
  frame_id: "odom"
child_frame_id: "base_link"
pose.pose:
  position: {x, y, z}        # metres, world frame
  orientation: {x, y, z, w}  # quaternion
twist.twist:
  linear: {x, y, z}          # m/s
  angular: {x, y, z}         # rad/s
```

**`/scan` — sensor_msgs/LaserScan**
```
header:
  stamp: {sec, nanosec}      # simulation time
  frame_id: "lidar"
angle_min: -π                 # start angle (rad)
angle_max: +π                 # end angle (rad)
angle_increment: ~0.0175      # 1° resolution (360 rays)
range_min: 0.2                # metres
range_max: 10.0               # metres
ranges: float32[360]          # distance per ray
intensities: float32[360]     # signal intensity
```

**`/cmd_vel` — geometry_msgs/Twist**
```
linear:
  x: 0.0–0.3                 # forward speed (m/s)
  y: 0.0                     # unused
  z: 0.0                     # unused
angular:
  x: 0.0                     # unused
  y: 0.0                     # unused
  z: -0.8–0.8                # yaw rate (rad/s)
```

**`/clock` — rosgraph_msgs/Clock**
```
clock:
  sec: uint32                 # simulation seconds
  nanosec: uint32             # simulation nanoseconds
```

---

## OmniGraph — ROS2 Bridge Wiring

The `ros2_jetbot.py` script builds a 14-node execution graph at `/World/ROS2Graph`:

```
OnPlaybackTick ──┬──→ SubscribeTwist ──→ DiffController ──→ ArticController
(every frame)    │         │                    ▲
                 │    BreakLinVel.x ─────────────┘ linearVelocity
                 │    BreakAngVel.z ─────────────┘ angularVelocity
                 │
                 ├──→ ComputeOdom ──→ PublishOdom ──→ /odom
                 │
                 ├──→ PublishClock ──→ /clock
                 │
                 └──→ IsaacReadLidar ──→ PublishLaserScan ──→ /scan

ROS2Context ────────→ (provides context handle to all ROS2 nodes)
ReadSimTime ────────→ (timestamps for PublishOdom, PublishClock, PublishLaserScan)
```

### Node Types (Isaac Sim 5.1.0)

| Node | OmniGraph Type |
|------|----------------|
| OnPlaybackTick | `omni.graph.action.OnPlaybackTick` |
| ROS2Context | `isaacsim.ros2.bridge.ROS2Context` |
| SubscribeTwist | `isaacsim.ros2.bridge.ROS2SubscribeTwist` |
| BreakLinVel / BreakAngVel | `omni.graph.nodes.BreakVector3` |
| DiffController | `isaacsim.robot.wheeled_robots.DifferentialController` |
| ArticController | `isaacsim.core.nodes.IsaacArticulationController` |
| ComputeOdom | `isaacsim.core.nodes.IsaacComputeOdometry` |
| PublishOdom | `isaacsim.ros2.bridge.ROS2PublishOdometry` |
| ReadSimTime | `isaacsim.core.nodes.IsaacReadSimulationTime` |
| PublishClock | `isaacsim.ros2.bridge.ROS2PublishClock` |
| IsaacReadLidar | `isaacsim.sensors.physx.IsaacReadLidarBeams` |
| PublishLaserScan | `isaacsim.ros2.bridge.ROS2PublishLaserScan` |

> **Note:** Isaac Sim 5.1.0 renamed all node types from `omni.isaac.*` to `isaacsim.*`. The `BreakVector3` nodes are needed because `SubscribeTwist` outputs `double3` vectors but `DifferentialController` expects scalar `double` inputs.

### Differential Controller Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `wheelDistance` | 0.1125 m | Axle width |
| `wheelRadius` | 0.0325 m | Wheel radius |
| Joint names | `left_wheel_joint`, `right_wheel_joint` | Jetbot wheel DOFs |

---

## Simulation Scene

### Arena

- **Size:** 8×8 m (half-extent 4.0 m)
- **Wall thickness:** 0.15 m
- **Wall height:** 0.15 m (low enough for camera to see over, high enough for lidar rays at 0.08 m)

**Obstacles** (5 semi-transparent pillars, 35% opacity, blue-tinted):

| Path | Position (x, y) | Size (half-extents) |
|------|-----------------|---------------------|
| P1 | (2.0, 1.5) | 0.25 × 0.25 × 0.12 |
| P2 | (−2.0, −1.5) | 0.25 × 0.25 × 0.12 |
| P3 | (1.0, −2.5) | 0.25 × 0.25 × 0.12 |
| P4 | (−1.0, 2.5) | 0.25 × 0.25 × 0.12 |
| P5 | (0.0, 0.5) | 0.5 × 0.1 × 0.12 (bar) |

### Lighting

Default Isaac Sim lights (harsh directional) are removed and replaced with even illumination:

| Light | Type | Intensity | Color (RGB) | Position |
|-------|------|-----------|-------------|----------|
| DomeLight | Dome (ambient) | 800 | (0.95, 0.95, 1.0) | Everywhere |
| FillA | Sphere (r=2m) | 400 | (1.0, 0.98, 0.95) | (5, 5, 6) |
| FillB | Sphere (r=2m) | 400 | (1.0, 0.98, 0.95) | (−5, −5, 6) |

### Robot — NVIDIA Jetbot

- **Asset:** `Isaac/Robots/NVIDIA/Jetbot/jetbot.usd`
- **Prim path:** `/World/Jetbot`
- **Drive:** Differential (2-wheeled)
- **Wheel joints:** `left_wheel_joint`, `right_wheel_joint`
- **Spawn position:** Origin (0, 0, 0)

### PhysX 2D Lidar

Mounted on the Jetbot chassis at `/World/Jetbot/chassis/Lidar`, offset 10 cm above chassis:

| Parameter | Value |
|-----------|-------|
| Type | PhysX raycast (not RTX) |
| Min range | 0.2 m |
| Max range | 10.0 m |
| Horizontal FOV | 360° |
| Horizontal resolution | 1.0° (360 rays) |
| Vertical FOV | 10° |
| Vertical resolution | 10° (single plane) |
| Rotation rate | 20 Hz |
| Height above chassis | 0.1 m |

> **Why PhysX not RTX lidar?** RTX lidar crashes with `IsaacReadLidarBeams` in Isaac Sim 5.1.0. PhysX raycast lidar is stable and sufficient for 2D obstacle avoidance.

### Orbit Camera

An orbital camera follows the robot during simulation:

| Parameter | Value |
|-----------|-------|
| Prim path | `/World/FollowCam` |
| Focal length | 24 mm |
| Orbit radius | 2.0 m (horizontal) |
| Orbit height | 2.0 m (above robot) |
| Orbit speed | 0.3 rad/s (~21 s per revolution) |
| Look-at target | Robot position + 0.15 m Z offset |
| Update rate | Every physics step (~60 Hz) |

### Trail Line

An orange trail traces the robot's path on the ground:

| Parameter | Value |
|-----------|-------|
| Width | 0.04 m |
| Height | 0.02 m above ground |
| Color | Orange (1.0, 0.45, 0.0) |
| Max points | 2000 |
| Sample rate | Every 6 physics steps (~10 Hz) |

---

## Obstacle Avoidance

Two implementations exist — one standalone (hello_robot.py, PhysX raycasts) and one ROS2-based (obstacle_avoidance_node.py, LaserScan subscriber).

### ROS2 Obstacle Avoidance Node

Runs in the `ros2-sidecar` container. Subscribes to `/scan`, publishes `/cmd_vel` at 10 Hz.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `linear_speed` | 0.3 m/s | Base forward velocity |
| `max_angular_speed` | 0.8 rad/s | Maximum turning rate |
| `ray_range` | 3.0 m | Sensing horizon (clamp lidar data) |
| `clear_fraction` | 0.7 | Rays beyond 70% of range = "clear" |
| `slow_fraction` | 0.3 | Front distance below 30% of range triggers slowdown |
| `steer_lp` | 0.15 | Low-pass filter coefficient (0=frozen, 1=instant) |
| `control_hz` | 10.0 | Control loop frequency |

**Algorithm:**

1. **Filter forward arc:** Extract rays from −90° to +90° (forward 180°)
2. **Clamp distances:** Replace inf/NaN with `ray_range` (3.0 m)
3. **Find best gap:** Widest contiguous run of "clear" rays (distance > 2.1 m)
4. **Compute steering angle:** Gap center position relative to forward, normalized to [−1, +1]
5. **Urgency weighting:** `urgency = 1 − (min_distance / ray_range)`, 0.0 (clear) to 1.0 (touching)
6. **Variable gain:** `0.15 + urgency × 1.85` — gentle drift when clear, hard steer when close
7. **Sinusoidal heading drift:** Two overlapping sine waves create a slow, non-repeating bias:
   ```
   drift = 0.08 × sin(t × 0.05) + 0.05 × sin(t × 0.13)
   ```
   Periods: ~126 s and ~48 s. Prevents the robot from repeating the same path.
8. **Random jitter:** Gaussian noise σ=0.02 added to each steering command
9. **Low-pass filter:** `coeff = 0.15 + urgency × 0.25` — faster response when urgent
10. **Speed control:** Full speed when clear, linearly reduced to 30% when front obstacle < 0.9 m

### Standalone Obstacle Avoidance (hello_robot.py)

Runs inside Isaac Sim's Script Editor. Uses direct PhysX raycasts instead of ROS2:

| Parameter | Value |
|-----------|-------|
| Ray count | 19 |
| Arc | ±90° (180° total, 10° steps) |
| Range | 0.35 m |
| Ray height | 0.08 m |
| Drive speed | 8.0 rad/s (wheel velocity) |
| Max steer differential | 6.0 rad/s |

---

## Web Dashboard

Accessible at `http://<ISAAC_SIM_IP>:8080`. A single-page app served by aiohttp with real-time WebSocket updates.

### Server Architecture

```
┌─────────────────────────────────────────────────────────┐
│  dashboard_server.py (aiohttp)                          │
│                                                         │
│  Thread: ROS2 spin (daemon)                             │
│  ├─ /odom  callback → _state["robot"]["position"]       │
│  ├─ /scan  callback → _state["robot"]["scan_ranges"]    │
│  ├─ /cmd_vel callback → _state["robot"]["cmd_*"]        │
│  └─ /clock callback → _state["robot"]["sim_time"]       │
│                                                         │
│  Async: broadcaster coroutine                           │
│  ├─ Every  250 ms: push robot + topic data to WebSocket │
│  └─ Every ~3.0 s:  refresh system info + containers     │
│       ├─ nvidia-smi (GPU temp, util, memory)            │
│       ├─ /proc/loadavg (CPU load)                       │
│       ├─ free -m (memory)                               │
│       ├─ df -h / (disk)                                 │
│       ├─ uptime -s                                      │
│       └─ docker ps -a (container list)                  │
│                                                         │
│  HTTP: GET / → index.html                               │
│  WS:   /ws  → bidirectional state + log streaming       │
└─────────────────────────────────────────────────────────┘
```

### WebSocket Protocol

**Server → Client (every 250 ms):**
```json
{
  "type": "state",
  "system": {
    "gpu": {"name": "...", "temp": 65, "util": 80, "mem_used": 8000, "mem_total": 16384},
    "load": ["1.5", "1.2", "0.9"],
    "mem": {"total": 32000, "used": 15000},
    "disk": {"total": "100G", "used": "45G", "pct": "45%"},
    "uptime": "2026-03-19 10:00:00"
  },
  "containers": [
    {"name": "isim-isaac-sim-1", "status": "Up 2 hours", "image": "nvcr.io/nvidia/isaac-sim:5.1.0", "ports": "..."}
  ],
  "robot": {
    "position": [3.79, -1.44, 0.0],
    "orientation": [0.0, 0.0, 0.38, 0.92],
    "linear_vel": [0.28, 0.05, 0.0],
    "angular_vel": [0.0, 0.0, 0.15],
    "cmd_linear": 0.3,
    "cmd_angular": -0.12,
    "scan_ranges": [1.2, 1.5, ...],
    "scan_angle_min": -3.1415,
    "scan_angle_increment": 0.0349,
    "odom_age": 0.2,
    "scan_age": 0.3,
    "cmd_age": 0.1,
    "clock_age": 0.1,
    "sim_time": 1234.567
  },
  "topics": {
    "/clock": {"type": "rosgraph_msgs/Clock", "count": 50000, "hz": 56.0, "age": 0.1},
    "/odom": {"type": "nav_msgs/Odometry", "count": 45000, "hz": 50.0, "age": 0.2},
    "/scan": {"type": "sensor_msgs/LaserScan", "count": 16000, "hz": 18.0, "age": 0.3},
    "/cmd_vel": {"type": "geometry_msgs/Twist", "count": 9000, "hz": 10.0, "age": 0.1}
  }
}
```

**Client → Server (on-demand log request):**
```json
{"action": "get_logs", "container": "isim-isaac-sim-1"}
```

**Server → Client (log response):**
```json
{"type": "logs", "container": "isim-isaac-sim-1", "data": "...last 8000 chars of docker logs..."}
```

### Dashboard UI Panels

1. **System:** GPU (temp/util/VRAM), CPU load (1m/5m/15m), memory, disk — with progress bars
2. **Containers:** Table listing all Docker containers with status badges
3. **ROS2 Topics:** Table with topic name, message type, Hz, count, age, health indicator
4. **Robot Info:** Position (X/Y), speed, heading, cmd_vel values, sim time, scan ray count
5. **Lidar Visualization:** 2D polar canvas showing ray distances (red=close, green=far), range circles 1–5 m, heading indicator
6. **Container Logs:** Dropdown selector, fetch/auto-refresh, timestamped output

### ROS2 QoS Configuration

All dashboard subscriptions use `BEST_EFFORT` QoS to match Isaac Sim's publishers:

```python
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
```

> **Why BEST_EFFORT?** Isaac Sim's ROS2 bridge publishes with BEST_EFFORT reliability. A RELIABLE subscriber silently drops all data from a BEST_EFFORT publisher — this was the root cause of the dashboard showing stale data.

---

## File Structure

```
nvidia-simu-test/
├── .github/
│   └── copilot-instructions.md          # AI coding rules & project conventions
├── .gitignore                           # Excludes keys, PEMs, logs, startup.sh
├── README.md                            # This file
├── INSTALL.md                           # Installation guide
├── ROBOT_TUTORIAL.md                    # Jetbot tutorial walkthrough
├── ROS2_PLAN.md                         # ROS2 integration planning doc
│
└── scripts/
    ├── deploy_to_instance.sh            # Mac → instance file deployment
    ├── launch_streaming.sh              # SSH tunnel + Chrome opener
    ├── check_status.sh                  # Instance health check
    ├── run_tutorial_part3.sh            # Standalone headless run
    │
    ├── provision/
    │   └── startup.sh                   # (gitignored) brev startup with NGC key
    │
    └── instance/                        # Files deployed to GPU instance
        ├── setup.sh                     # Docker + NVIDIA CTK installer
        ├── launch_isaac_sim.sh          # Docker Compose launcher
        ├── ros2_jetbot.py               # Scene + OmniGraph ROS2 bridge
        ├── hello_robot.py               # Standalone obstacle avoidance
        ├── standalone_robot.py          # Headless square-path demo
        ├── run_standalone.sh            # Headless runner wrapper
        ├── ros2_sidecar.sh              # ROS2 Humble sidecar container
        ├── obstacle_avoidance_node.py   # ROS2 node: /scan → /cmd_vel
        ├── start_ros2_all.sh            # Orchestrator (sidecar + dash + avoidance)
        ├── dashboard_launch.sh          # Dashboard container builder
        ├── check_status_instance.sh     # Instance-side diagnostics
        │
        └── dashboard/
            ├── Dockerfile               # ros:humble-ros-base + aiohttp
            ├── dashboard_server.py      # Async server + ROS2 subscriber
            └── index.html               # Single-page monitoring UI
```

---

## Deployment Workflow

### First-Time Setup

```bash
# 1. Set instance IP
export ISAAC_SIM_IP=94.101.98.52

# 2. Deploy all files to instance
bash scripts/deploy_to_instance.sh

# 3. SSH into instance
brev shell isaac-sim-2

# 4. Install Docker + NVIDIA CTK + pull Isaac Sim image (~10 min)
sudo bash ~/setup.sh

# 5. Start Isaac Sim streaming
bash ~/launch_isaac_sim.sh
```

### Start Simulation + ROS2

```bash
# Mac — open streaming viewer
export ISAAC_SIM_IP=94.101.98.52
bash scripts/launch_streaming.sh

# Isaac Sim browser (http://localhost:8210) — Script Editor:
#   File > Open > /isaac-sim/ros2_jetbot.py → Ctrl+Enter

# Instance — start ROS2 stack
bash ~/start_ros2_all.sh
# Starts: ros2-sidecar + ros2-dashboard + obstacle_avoidance_node
```

### Iterative Development

```bash
# Edit scripts locally, then:
export ISAAC_SIM_IP=94.101.98.52
bash scripts/deploy_to_instance.sh

# Re-run script in Script Editor (Ctrl+Enter)
# Restart specific services as needed:
#   docker rm -f ros2-dashboard && bash ~/dashboard_launch.sh
#   docker exec ros2-sidecar pkill -f obstacle_avoidance
#   docker exec -d ros2-sidecar bash -c 'source /opt/ros/humble/setup.bash && python3 /ros2_ws/obstacle_avoidance_node.py'
```

---

## Secrets Management

| Secret | Location | Handling |
|--------|----------|----------|
| NGC API key | `~/.ngc/key` (Mac, chmod 600) | Injected into `setup.sh` at deploy time via `sed` |
| brev SSH key | `~/.brev/brev.pem` (Mac) | Used for all SCP/SSH to instance |
| NGC key placeholder | `scripts/instance/setup.sh` | Contains literal `NGC_KEY_PLACEHOLDER` (never the real key) |
| Patched setup.sh | `/tmp/setup.XXXXXX.sh` | Temporary; deleted after SCP |
| `startup.sh` | `scripts/provision/startup.sh` | Gitignored (contains NGC key for brev --startup-script) |

---

## SSH Configuration

brev.dev configures SSH with `RequestTTY yes` and `ControlMaster auto`, which causes non-interactive commands (`ssh user@host "cmd"`) to hang. All automated SSH/SCP commands use:

```bash
ssh -o ControlPath=none -i ~/.brev/brev.pem shadeform@$ISAAC_SIM_IP "command"
scp -o ControlPath=none -i ~/.brev/brev.pem file shadeform@$ISAAC_SIM_IP:dest
```

---

## Known Issues & Solutions

| Issue | Root Cause | Solution |
|-------|-----------|----------|
| Topics visible but no data flows | FastDDS SHM isolated by `ipc=private` | UDP-only XML profile in all containers |
| Dashboard stale (no live updates) | QoS mismatch: RELIABLE sub ↔ BEST_EFFORT pub | Use BEST_EFFORT QoS on all subscriptions |
| `ROSClock object not callable` | `_clock` callback shadows `rclpy.Node._clock` | Rename callback to `_on_clock` |
| LaserScan "sequence size exceeds buffer" | Default UDP buffer too small for 360-ray scan | 1 MB sendBufferSize / receiveBufferSize |
| RTX Lidar crashes | `IsaacReadLidarBeams` incompatible with RTX lidar in 5.1.0 | Use PhysX raycast lidar instead |
| Robot repeats same path | Deterministic gap-steering always picks same direction | Sinusoidal drift + Gaussian jitter |
| nvidia-smi unavailable in dashboard | Container missing `--gpus all` | Add `--gpus all` to docker run |
| Non-interactive SSH hangs | brev `ControlMaster auto` + `RequestTTY yes` | Use `-o ControlPath=none` |
