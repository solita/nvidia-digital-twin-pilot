# Phase 1 — Minimal Closed-Loop Demo with ROS2

## Goal

Replace the current physics-callback obstacle avoidance in `hello_robot.py` with a
ROS2-based control loop. The Jetbot's sensors and actuators are exposed as ROS2 topics
via the Isaac Sim ROS2 bridge. A separate ROS2 control node reads sensor data, decides
actions, and publishes velocity commands — closing the loop through the standard
robotics middleware.

This mirrors a real deployment: the same control node can later run on a Jetson
connected to a physical robot, with zero changes to the control logic.

---

## What we have today

| Component | Status |
|---|---|
| Isaac Sim 5.1.0 streaming via WebRTC | Working |
| 8×8m arena with walls + obstacles | Working |
| Jetbot with PhysX raycast obstacle avoidance | Working (`hello_robot.py`) |
| Deploy pipeline (Mac → instance → container) | Working |
| ROS2 Humble libraries inside Isaac Sim container | Built-in (not yet enabled) |

---

## Architecture

```
┌─────────────────────────────────────┐
│  Isaac Sim container (isim-isaac-sim-1)  │
│                                     │
│  Jetbot (Articulation)              │
│    ├─ RTX Lidar      ──► /scan      │
│    ├─ RGB Camera      ──► /image    │
│    ├─ IMU             ──► /imu      │
│    └─ Odometry        ──► /odom     │
│                                     │
│  ROS2 Bridge (isaacsim.ros2.bridge) │
│    └─ /cmd_vel subscriber ──► Diff  │
│       Controller ──► Articulation   │
│                                     │
│  DDS: FastDDS (default)             │
└───────────────┬─────────────────────┘
                │ localhost / --net=host
                │
┌───────────────┴─────────────────────┐
│  ROS2 sidecar container             │
│  (osrf/ros:humble-desktop)          │
│                                     │
│  ┌─ obstacle_avoidance_node ─┐      │
│  │  sub: /scan, /odom        │      │
│  │  pub: /cmd_vel (Twist)    │      │
│  └───────────────────────────┘      │
│                                     │
│  ┌─ rviz2 (optional) ───────┐      │
│  │  Visualize /scan, /odom,  │      │
│  │  /tf, /image              │      │
│  └───────────────────────────┘      │
└─────────────────────────────────────┘
```

---

## Step 1.1 — Enable ROS2 bridge in Isaac Sim

**What**: Configure the Isaac Sim container to start with the ROS2 bridge enabled.

**How**:
- Set environment variables in the docker-compose override or `launch_isaac_sim.sh`:
  ```
  ROS_DISTRO=humble
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ```
- Enable the extension at runtime in the simulation script:
  ```python
  import omni.kit.app
  ext_mgr = omni.kit.app.get_app().get_extension_manager()
  ext_mgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
  ```

**Verify**: In the Script Editor console, confirm no errors on enable. Run
`ros2 topic list` from the sidecar (Step 1.3) and see `/rosout` + `/clock`.

**Files to change**:
- `scripts/instance/launch_isaac_sim.sh` — add env vars to docker-compose up
- New: `scripts/instance/ros2_jetbot.py` — simulation script with bridge + OmniGraph

---

## Step 1.2 — Simulated scene with sensor topics

**What**: Keep the arena and Jetbot from `hello_robot.py`, but wire sensors and
actuators through OmniGraph → ROS2 topics instead of physics callbacks.

### OmniGraph wiring

Build the action graph programmatically in `ros2_jetbot.py`:

| OG Node | Role |
|---|---|
| `ROS2 Context` | Create a ROS2 context for the graph |
| `On Playback Tick` | Event source — fires every sim step |
| `ROS2 Subscribe Twist` | Listens on `/cmd_vel` → linear.x, angular.z |
| `Differential Controller` | Converts (v, ω) → left/right wheel velocities |
| `Articulation Controller` | Applies wheel velocities to Jetbot joints |
| `Isaac Compute Odometry` | Reads Jetbot pose → (position, orientation) |
| `ROS2 Publish Odometry` | Publishes on `/odom` |
| `ROS2 Publish LaserScan` | Publishes RTX Lidar data on `/scan` |
| `ROS2 Publish Clock` | Publishes sim clock on `/clock` |

### Sensors to add

| Sensor | Prim type | Topic | Message type |
|---|---|---|---|
| RTX Lidar | `RtxLidar` at `/World/Jetbot/chassis/lidar` | `/scan` | `sensor_msgs/LaserScan` |
| RGB Camera | `Camera` at `/World/Jetbot/chassis/camera` | `/image` | `sensor_msgs/Image` |
| IMU | `IsaacImuSensor` at `/World/Jetbot/chassis/imu` | `/imu` | `sensor_msgs/Imu` |
| Odometry | Computed from articulation root | `/odom` | `nav_msgs/Odometry` |

The RTX Lidar prim is created via `omni.isaac.sensor` API:
```python
from isaacsim.sensors.rtx import LidarRtx
lidar = LidarRtx(
    prim_path="/World/Jetbot/chassis/lidar",
    rotation_frequency=0,       # non-rotating (2D sweep)
    high_lod=False,
)
```

**Verify**: `ros2 topic echo /scan` from sidecar shows LaserScan data.
`ros2 topic echo /odom` shows position updating at sim rate.

**Files**:
- New: `scripts/instance/ros2_jetbot.py` — full simulation setup (arena + sensors + OG)

---

## Step 1.3 — ROS2 sidecar container

**What**: Run a ROS2 Humble container alongside Isaac Sim, sharing the DDS domain
via `--net=host`.

**How**: Add to docker-compose or run standalone:
```bash
docker run -d --name ros2-sidecar \
  --net=host \
  --pid=host \
  osrf/ros:humble-desktop \
  bash -c "sleep infinity"
```

Then exec into it to run nodes:
```bash
docker exec -it ros2-sidecar bash
# Inside:
ros2 topic list
ros2 topic echo /scan
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Verify**: `ros2 topic list` shows `/cmd_vel`, `/odom`, `/scan`, `/clock`, `/image`, `/imu`.

**Files**:
- New: `scripts/instance/ros2_sidecar.sh` — pull image, start sidecar container
- Update: `scripts/deploy_to_instance.sh` — deploy new files

---

## Step 1.4 — Obstacle avoidance ROS2 node

**What**: A Python ROS2 node that subscribes to `/scan`, makes decisions,
and publishes `/cmd_vel`. This replaces the current `on_step` physics callback
with a standard ROS2 control loop.

```
/scan (LaserScan) ──► obstacle_avoidance_node ──► /cmd_vel (Twist)
                              │
                     (optional: /odom)
```

**Behaviour** (same as current `hello_robot.py`):
1. Drive forward at constant speed (`linear.x = 0.3`)
2. When min range in `/scan` < threshold → stop, rotate randomly left/right
3. When path is clear → resume driving

```python
# Pseudocode — obstacle_avoidance_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.min_range = float('inf')

    def scan_cb(self, msg):
        self.min_range = min(msg.ranges)

    def control_loop(self):
        cmd = Twist()
        if self.min_range < 0.5:
            cmd.angular.z = 0.5  # turn
        else:
            cmd.linear.x = 0.3   # drive
        self.pub_cmd.publish(cmd)
```

**Verify**: Jetbot drives in arena, avoids obstacles, with all decision-making
happening externally via ROS2 topics. `ros2 topic hz /cmd_vel` shows ~10 Hz.

**Files**:
- New: `scripts/instance/obstacle_avoidance_node.py`
- Copy into sidecar via `docker cp` during deploy

---

## Step 1.5 — Closed-loop verification

**What**: Confirm the full loop works end-to-end and document the evidence.

**Checks**:
1. **Teleop**: From sidecar, run `teleop_twist_keyboard` → Jetbot moves in viewport ✓
2. **Sensor echo**: `ros2 topic echo /scan` → LaserScan populates with arena geometry ✓
3. **Autonomous**: Kill teleop, launch `obstacle_avoidance_node.py` → Jetbot navigates ✓
4. **Latency**: `ros2 topic delay /cmd_vel` → < 50ms ✓
5. **RViz2** (optional): Forward X11 or use VNC — visualize `/scan` overlaid on `/odom` ✓

---

## Implementation order

| # | Task | Est. complexity | Depends on |
|---|---|---|---|
| 1 | Enable ROS2 bridge + verify `/clock` | Small | — |
| 2 | Create `ros2_jetbot.py` with arena + OmniGraph wiring | Medium | #1 |
| 3 | Add RTX Lidar sensor + `/scan` publisher | Medium | #2 |
| 4 | Add `/odom` publisher via OmniGraph | Small | #2 |
| 5 | Start ROS2 sidecar, verify `ros2 topic list` | Small | #1 |
| 6 | Teleop test (`teleop_twist_keyboard`) | Small | #2, #5 |
| 7 | Add camera + IMU sensors | Small | #2 |
| 8 | Write `obstacle_avoidance_node.py` | Small | #3, #5 |
| 9 | End-to-end closed-loop verification | Small | #6, #8 |
| 10 | Update deploy script + docs | Small | all |

---

## Key technical references

- Isaac Sim 5.1.0 ships with **internal ROS2 Humble** libraries — no host ROS2 install needed
- Extension: `isaacsim.ros2.bridge` (enable at runtime or via `.kit` config)
- Default DDS: FastDDS (`rmw_fastrtps_cpp`) — works with `--net=host` between containers
- OmniGraph node catalog: `Window > Graph Editors > Action Graph > right-click > search`
- RTX Lidar: `omni.isaac.sensor` extension, `RtxLidar` prim
- TurtleBot ROS2 drive tutorial applies directly — Jetbot uses the same Differential Controller pattern

---

## Future phases (out of scope for Phase 1)

| Phase | Description |
|---|---|
| **Phase 2 — Nav2** | Add `nav2_bringup` stack, AMCL localisation, map server, autonomous waypoint navigation |
| **Phase 3 — Camera + perception** | Object detection on `/image`, feed results back to planner |
| **Phase 4 — Jetson deployment** | Run the same `obstacle_avoidance_node.py` on a Jetson Orin with a physical Jetbot, swapping Isaac Sim for real sensors |
| **Phase 5 — Sim-to-real** | Domain randomization, synthetic data training, transfer learning |
