# Designing and Testing a Simple Moving Robot in Isaac Sim
## With Visualization Guide

---

## Overview: Three Ways to Work

| Workflow | When to use |
|---|---|
| **GUI (interactive)** | Explore, prototype, drag-and-drop, visualize live |
| **Extension script** | Interactive development, hot-reload while Isaac Sim is open |
| **Standalone Python** | Headless cloud execution, training pipelines, CI |

All three can produce the exact same simulation result. Start with GUI to understand the layout, then move to Python scripting.

---

## Part 1 — GUI Quickstart: Add and Move a Robot

### Step 1: Launch Isaac Sim (see INSTALL.md §3e for full setup)

Isaac Sim runs via Docker Compose with WebRTC browser streaming. Follow INSTALL.md §3e for the complete setup. Summary:

```bash
# On the instance — start (warm cache, ~35 seconds to ready):
cd ~/isaacsim-docker
ISAACSIM_HOST=<instance-ip> \
ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0 \
docker compose -p isim -f tools/docker/docker-compose.yml up -d

# Wait for Isaac Sim to report ready:
watch -n 5 'curl -s http://127.0.0.1:8011/v1/streaming/ready'
# Wait for: {"statusMessage":"Status: Ready for connection"}
```

```bash
# On your Mac — SSH tunnel for the web viewer UI:
ssh -i ~/.ssh/id_ed25519 -L 8210:localhost:8210 shadeform@<instance-ip> -N -f

# Open in Chrome:
# http://localhost:8210
```

### Step 2: Create a Scene

1. `File > New Stage`
2. `Create > Physics > Ground Plane` — adds a flat floor
3. `Create > Lights > Distant Light` — illuminates the scene

### Step 3: Add a Pre-built Robot

Isaac Sim includes ready-to-use robot models:

1. `Create > Robots > Franka Emika Panda Arm` — robot arm
   - Or: `Create > Robots` — browse all built-in robots (Jetbot, Carter, etc.)
2. The robot appears in the **Viewport** and in the **Stage tree** on the right.

### Step 4: Inspect Joint Properties

1. `Tools > Physics > Physics Inspector`
2. Click the robot in the Stage tree
3. View joint limits, stiffness, damping — edit values to see the robot adjust in real-time

### Step 5: Add a Controller and Move the Robot

For a robot arm (Franka):
1. `Tools > Robotics > Omnigraph Controllers > Joint Position`
2. In the popup, click **Add** for "Robot Prim", select **Franka**, click **OK**
3. Press **Play** (triangle button, bottom-left of viewport)
4. In `Stage > Graph > Position_Controller > JointCommandArray`: drag joint values to move the arm

For a wheeled robot (Jetbot):
- Use velocity targets on wheel joints (see Python section below)

### Step 6: Visualize

| What to visualize | How |
|---|---|
| 3D robot in motion | Viewport (main window) — use orbit/pan/zoom |
| Joint values live | Physics Inspector (`Tools > Physics > Physics Inspector`) |
| OmniGraph (control logic) | `Window > Graph Editors > Action Graph` |
| Sensor data (camera, lidar) | Add a sensor prim, open `Window > Viewport > Viewport 2` |
| TF / ROS topics | Connect ROS 2 bridge, use `rviz2` externally |

---

## Part 2 — Python: Simple Differential Drive Robot (Jetbot)

This is the recommended way to programmatically design, control, and test a moving robot.

### File: `hello_robot.py`

```python
import carb
import numpy as np
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.storage.native import get_assets_root_path


class HelloRobot(BaseSample):
    """Simple Jetbot differential drive demo."""

    def __init__(self):
        super().__init__()
        self._physics_callback_id = None
        self._elapsed = 0.0

    def setup_scene(self):
        assets_root = get_assets_root_path()
        if assets_root is None:
            carb.log_error("Cannot reach Nucleus asset server")
            return

        # Ground plane
        stage_utils.add_reference_to_stage(
            usd_path=assets_root + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/Ground",
        )

        # Jetbot — NVIDIA's 2-wheeled differential drive robot
        stage_utils.add_reference_to_stage(
            usd_path=assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
            path="/World/Jetbot",
        )

    async def setup_post_load(self):
        # Wrap robot with Articulation for joint control
        self._robot = Articulation("/World/Jetbot")

        print("DOF names:", self._robot.dof_names)
        # Typically: ['left_wheel_joint', 'right_wheel_joint']

        # Get wheel joint indices by name (robust to model changes)
        self._wheel_idx = self._robot.get_dof_indices(
            ["left_wheel_joint", "right_wheel_joint"]
        ).numpy()

        # Register physics step callback
        self._physics_callback_id = SimulationManager.register_callback(
            self._on_physics_step, IsaacEvents.POST_PHYSICS_STEP
        )

    def _on_physics_step(self, dt, context):
        """Called every physics step (~60 Hz). Apply drive commands here."""
        self._elapsed += dt

        if self._elapsed < 3.0:
            # Drive forward: both wheels same speed
            v_left, v_right = 10.0, 10.0
        elif self._elapsed < 5.0:
            # Turn right: left wheel faster
            v_left, v_right = 10.0, -10.0
        elif self._elapsed < 8.0:
            # Drive forward again
            v_left, v_right = 10.0, 10.0
        else:
            # Stop
            v_left, v_right = 0.0, 0.0

        self._robot.set_dof_velocity_targets(
            np.array([[v_left, v_right]]),
            dof_indices=self._wheel_idx,
        )

    def physics_cleanup(self):
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None
```

### Run inside Isaac Sim container

```bash
# Inside the Docker container
./python.sh hello_robot.py
```

---

## Part 3 — Standalone Headless Script (Cloud / CI)

Use this pattern when running without a GUI (training pipelines, cloud instances).

### File: `standalone_robot.py`

```python
"""Headless standalone script: Jetbot drives a square path."""

from isaacsim import SimulationApp

# Must be created before any other Isaac Sim imports
simulation_app = SimulationApp({"headless": True})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.storage.native import get_assets_root_path
import isaacsim.core.experimental.utils.stage as stage_utils

# --- Build the world ---
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

assets_root = get_assets_root_path()
stage_utils.add_reference_to_stage(
    usd_path=assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
    path="/World/Jetbot",
)

world.reset()

robot = Robot("/World/Jetbot")

# Get wheel joint index
from isaacsim.core.experimental.prims import Articulation
artic = Articulation("/World/Jetbot")
wheel_idx = artic.get_dof_indices(["left_wheel_joint", "right_wheel_joint"]).numpy()

# --- Simulation loop ---
total_steps = 500
for step in range(total_steps):
    world.step(render=False)   # render=False = pure headless, much faster

    t = step * world.get_physics_dt()

    # Simple square path logic
    if t < 2.0:
        vl, vr = 10.0, 10.0      # forward
    elif t < 2.5:
        vl, vr = 10.0, -10.0     # turn
    elif t < 4.5:
        vl, vr = 10.0, 10.0      # forward
    elif t < 5.0:
        vl, vr = 10.0, -10.0     # turn
    else:
        vl, vr = 10.0, 10.0

    artic.set_dof_velocity_targets(
        np.array([[vl, vr]]), dof_indices=wheel_idx
    )

    pos = artic.get_local_poses()[0]
    if step % 60 == 0:
        print(f"t={t:.2f}s  pos={pos.numpy()[0][:2]}")  # print XY position

simulation_app.close()
```

```bash
# Run headless (no display needed)
./python.sh standalone_robot.py
```

---

## Part 4 — Visualization Options

### Option A: Live viewport (WebRTC streaming from cloud)

When using Docker Compose with `INSTALL.md §3e`:
- SSH tunnel on Mac: `ssh -i ~/.ssh/id_ed25519 -L 8210:localhost:8210 shadeform@<instance-ip> -N -f`
- Open `http://localhost:8210` in Chrome
- Full 3D viewport with interactive camera: **orbit** (LMB), **pan** (MMB), **zoom** (scroll)
- Press **Play** to start simulation, watch the robot move in real-time

### Option B: RViz2 (ROS 2 integration)

Connect Isaac Sim ROS 2 bridge and visualize with RViz on your local machine:

```bash
# In Isaac Sim container — enable ROS 2 bridge
# Add to your script or use the GUI: Robotics > ROS2 > Enable ROS2 Bridge

# On your local machine (with ROS 2 Humble):
source /opt/ros/humble/setup.bash
rviz2
# Add displays: TF, RobotModel, LaserScan, Image, etc.
```

### Option C: Plot joint data / position with matplotlib (headless)

```python
import matplotlib.pyplot as plt

positions = []  # collect (x, y) each step
# ... in loop:
pos = artic.get_local_poses()[0].numpy()[0]
positions.append(pos[:2])

# After loop:
positions = np.array(positions)
plt.plot(positions[:, 0], positions[:, 1])
plt.xlabel("X (m)"); plt.ylabel("Y (m)")
plt.title("Jetbot Path")
plt.savefig("robot_path.png")
```

### Option D: Isaac Sim built-in debug visualization

```python
# Draw robot velocity vectors, joint frames, collision meshes
# In GUI: Show > Physics > Collision Meshes (toggle)
#         Show > Physics > Rigid Body Frames
# Or via Python:
from omni.physx.scripts import utils
utils.setCollisionGroupSimulationFilteringEnabled(True)
```

### Option E: Sensor visualization (camera / lidar)

```python
# Add a camera to the robot in GUI:
# Stage tree > right-click /World/Jetbot > Add > Camera
# Then: Window > Viewport > New Viewport
# In the new viewport, select the camera from the dropdown

# For lidar: Create > Sensors > Lidar (select RTX or PhysX type)
# Visualize point cloud: Show > Lidar Point Cloud (in viewport menu)
```

---

## Part 5 — Import Your Own Robot (URDF)

If you have a custom URDF robot:

```bash
# In Isaac Sim GUI:
# File > Import > URDF
# Adjust joint drives, collision meshes, and physics properties in the inspector

# Or via Python:
from isaacsim.asset.importer.urdf import _urdf
import_config = _urdf.ImportConfig()
result, robot_prim_path = _urdf.import_robot(
    "path/to/your_robot.urdf",
    import_config
)
```

---

## Part 6 — Recommended Learning Path

```
1. GUI Basic Tutorial    → https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/quickstart_isaacsim.html
2. GUI Robot Tutorial   → https://docs.isaacsim.omniverse.nvidia.com/latest/introduction/quickstart_isaacsim_robot.html
3. Hello World (Python) → https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_hello_world.html
4. Hello Robot (Python) → https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_hello_robot.html
5. Isaac Lab (RL)        → https://isaac-sim.github.io/IsaacLab
```

---

## Quick Reference: Essential Python APIs

```python
from isaacsim.core.experimental.prims import Articulation

robot = Articulation("/World/MyRobot")

# Inspect
robot.num_dofs                         # number of joints
robot.dof_names                        # list of joint names
robot.get_dof_positions()              # current joint angles (rad)
robot.get_dof_velocities()             # current joint velocities

# Control
robot.set_dof_velocity_targets(vels)   # velocity control (wheels)
robot.set_dof_position_targets(pos)    # position control (arm)

# World pose
robot.get_local_poses()                # (translation, orientation) tensors
robot.get_world_poses()

# Physics step callback
SimulationManager.register_callback(fn, IsaacEvents.POST_PHYSICS_STEP)
```
