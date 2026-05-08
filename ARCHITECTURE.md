# Architecture Summary

**NVIDIA Digital Twin Pilot** — a cloud-based digital twin platform for running NVIDIA Isaac Sim simulations on cloud GPUs (AWS Brev instances) with remote development via VS Code.

---

## High-Level Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                    Developer Machine                           │
│                                                                │
│  ┌─────────────────────┐  ┌──────────────────────────────┐  │
│  │  VS Code Remote     │  │  Browser Dashboard           │  │
│  │  (SSH tunnel :22)   │  │  (http://<ip>:8080)          │  │
│  │                     │  │                              │  │
│  │  • Edit code live   │  │  • Forklift monitoring       │  │
│  │  • Run remotely     │  │  • Live sensor visualization │  │
│  └─────────────────────┘  └──────────────────────────────┘  │
│           │                            │                     │
│           └────────────────┬───────────┘                     │
│                            │ Brev CLI (start/stop)           │
└────────────────────────────┼─────────────────────────────────┘
                             │
                             ▼
┌────────────────────────────────────────────────────────────────┐
│            AWS Brev Instance (L40S GPU)                        │
│                                                                │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │   Isaac Sim 5.1.0 Container (--network=host)             │ │
│  │                                                          │ │
│  │   Simulation Execution Environment                       │ │
│  │   ├─ USD Scene Loading                                   │ │
│  │   ├─ Physics Simulation (PhysX)                          │ │
│  │   └─ Sensor Simulation (LIDAR, etc.)                     │ │
│  │                                                          │ │
│  │   ┌──────────────────────────────────────────────────┐  │ │
│  │   │  Forklift Simulation                             │  │ │
│  │   │  ├─ forklift_controller.py (asyncio loop)        │  │ │
│  │   │  ├─ lidar_processor.py (perception)              │  │ │
│  │   │  ├─ nav_controller.py (planning & control)       │  │ │
│  │   │  └─ HTTP server :8081 (command interface)        │  │ │
│  │   └──────────────────────────────────────────────────┘  │ │
│  │                     │                                   │ │
│  │                     ▼                                   │ │
│  │   ┌──────────────────────────────────────────────────┐  │ │
│  │   │  Dashboard Backend (FastAPI) :8080               │  │ │
│  │   │  ├─ Proxies commands to controller :8081         │  │ │
│  │   │  ├─ Streams simulation state via WebSocket       │  │ │
│  │   │  └─ Serves UI (canvas map + fleet cards)         │  │ │
│  │   └──────────────────────────────────────────────────┘  │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                                │
│  Persistent Storage: ~/docker/isaac-sim/data/<repo>/           │
└────────────────────────────────────────────────────────────────┘
```

---

## System Breakdown

### 1. Infrastructure Layer

#### 1.1 Brev Cloud Instance Management

| File                                       | Purpose                                                                                |
| ------------------------------------------ | -------------------------------------------------------------------------------------- | --- |
| [Makefile](Makefile)                       | Server-side: `make init` (provision), `make dev` (start Isaac Sim), `make check-ports` |     |
| [brev.code-workspace](brev.code-workspace) | VS Code multi-root workspace pointing at the Isaac Sim data volume                     |

**`make init`** performs one-time setup:

1. Creates Isaac Sim volume-mount directories (`cache`, `config`, `data`, `logs`)
2. Sets ownership to UID 1234 (Isaac Sim container user)
3. Pulls `nvcr.io/nvidia/isaac-sim:5.1.0` Docker image
4. Installs Python deps (`fastapi`, `uvicorn`) and the Isaac Sim VS Code extension
5. Enables the `isaacsim.code_editor.vscode` extension in Isaac Sim config
6. Verifies streaming ports (49100, 47998, 8080) are reachable

- `make start` — boots the Brev instance + schedules auto-stop after 7 hours (configurable via `HOURS=N`) to prevent runaway cloud costs
- `make stop` — stops instance + cancels the auto-stop timer
- Auto-stop runs as a background `nohup` process tracked via PID file

#### 1.2 Networking

| Port  | Service                      | Protocol |
| ----- | ---------------------------- | -------- |
| 22    | SSH (VS Code Remote)         | TCP      |
| 8080  | Forklift Dashboard           | HTTP/WS  |
| 8081  | Forklift Controller commands | HTTP     |
| 8226  | Isaac Sim VS Code extension  | TCP      |
| 49100 | WebRTC signalling            | TCP      |
| 47998 | WebRTC media stream          | UDP      |

The Isaac Sim container runs with `--network=host`, meaning all ports are directly accessible on the Brev instance IP.

#### 1.3 FastDDS Configuration

[fastdds_client_resolved.xml](fastdds_client_resolved.xml) configures eProsima FastDDS (DDS middleware) in **CLIENT** discovery mode for optional ROS 2 / DDS interoperability with external systems.

---

### 2. Simulation Framework

Simulations are organized in a standardized structure under `simulations/<name>/`:

```
simulations/<name>/
├── 01_scenes/           # USD scene files
├── 02_core_scripts/     # Python scripts (launcher.py is entry point)
├── 03_preview_donor_scripts/  # Supporting scripts
├── 04_current_outputs/  # Runtime outputs (gitignored)
└── run_sim.sh           # Launch script
```

The **launcher.py** entry point:

1. Opens the USD scene asynchronously
2. Waits for the stage to fully load (90s timeout)
3. Starts the simulation timeline
4. Executes the simulation logic

**Two execution paths**:

- **Shell**: `./run_sim.sh` runs the launcher inside Isaac Sim
- **VS Code**: Open launcher → `Ctrl+Shift+P` → "Isaac Sim: Run Remotely" (executes script remotely via port 8226 TCP)

---

### 3. Forklift Warehouse Simulation

An autonomous forklift navigating a warehouse environment with LIDAR-based obstacle avoidance, waypoint patrol, and a live web dashboard.

#### 3.1 Configuration — [fl_config.py](simulations/forklift-warehouse/02_core_scripts/fl_config.py)

Centralized tunable parameters:

| Category             | Key Parameters                                                             |
| -------------------- | -------------------------------------------------------------------------- |
| **Vehicle physics**  | `DRIVE_VELOCITY=-550°/s`, `STEER_STIFFNESS_SETTLE/DRIVE`, steering damping |
| **PD controller**    | `STEER_KP`, `STEER_KD`, `STEER_DEADBAND=2.5°`                              |
| **Waypoint patrol**  | 6-point rectangular loop, `ARRIVAL_RADIUS=2.5m`                            |
| **LIDAR thresholds** | Forward stop 5.5m, slow zone 8m, hard stop 2.5m, two-tier hit validation   |
| **APF repulsion**    | 1/d lateral steering (gain=10), ±130° arc, open-side bias 8°               |
| **Stuck detection**  | 180-frame window (~3s), 80-frame escape maneuver                           |
| **Command server**   | Port 8081                                                                  |

#### 3.2 Launcher — [forklift_launcher.py](simulations/forklift-warehouse/02_core_scripts/forklift_launcher.py)

Opens `01_scenes/scene_assembly.usd` (which references warehouse geometry, forklift model, and sensor rigs), starts the timeline, and hands off to the controller via `runpy.run_path()`. Uses hardcoded container paths to work around unreliable `__file__` in remote execution.

#### 3.3 Controller — [forklift_controller.py](simulations/forklift-warehouse/02_core_scripts/forklift_controller.py)

Main `asyncio` loop orchestrating perception → planning → actuation:

```
Per-frame loop:
  1. Read forklift world transform (X, Y, yaw) from USD prim
  2. Process LIDAR depth data → LidarResult
  3. Compute navigation command → DriveCommand
  4. Apply speed override if queued from dashboard
  5. Set drive/steer joint targets on USD ArticulationRoot
  6. Write forklift_state.json every 10 frames (atomic via temp file)
```

**State machine**: `DRIVING` → `DODGING` (LIDAR obstacle / APF active) → `STUCK` (no movement for ~3s)

**HTTP Command Server** (port 8081, stdlib `HTTPServer` in daemon thread):

- `POST /cmd` — accepts `{action, value}`: pause, resume, speed override, lidar_range, reset_location
- `GET /status` — returns live state JSON
- Socket stored in `sys.modules["__forklift_cmd_sock__"]` to survive Isaac Sim script re-runs

#### 3.4 LIDAR Processor — [lidar_processor.py](simulations/forklift-warehouse/02_core_scripts/lidar_processor.py)

Per-frame LIDAR depth data processing pipeline:

1. **Self-hit filtering** — discards rays inside forklift bounding box (computed per-ray minimum safe distance based on ray angle relative to box extent)
2. **Range gating** — halves detection range outside ±20° forward cone (side/back)
3. **Emergency close-range** — 2+ rays in tight ±6° cone at <2.5m = immediate hard stop
4. **Two-tier forward validation**:
   - Near zone (0.8–4.5m): ≥4 ray hits required
   - Far zone (4.5–5.5m): ≥10 ray hits required (filters out fork tine reflections ~8 rays)
5. **Hysteresis debounce** — count up fast, decay only after 5 consecutive clear frames
6. **APF lateral repulsion** — inverse-distance (1/d) per side, blended with open-side bias (8°)
7. **Dashboard slices** — 9-sector pie-chart boolean flags for UI visualization

Output: `LidarResult(forward_min, fwd_stop, fwd_slow, repulsion_steer, lidar_slices, lidar_state, debounce_count)`

#### 3.5 Navigation Controller — [nav_controller.py](simulations/forklift-warehouse/02_core_scripts/nav_controller.py)

Waypoint tracking with PD heading control:

- **PD controller**: `steer = KP × error + KD × d_error` (heading error to next waypoint)
- **Heading EMA filter**: α=0.4 smoothing for noisy orientation readings
- **Turn-speed limiter**: 50–100% speed scaled by |heading_error|
- **APF blending**: PD steering + LIDAR repulsion, clamped to ±30°
- **Stuck detection**: Position ring buffer (180 frames), triggers two-phase escape:
  - Phase A (40 frames): Reverse at 40% speed, zero steer
  - Phase B (40 frames): Forward full speed, ±30° steer toward open side
  - After 5 consecutive stuck events: skip to next waypoint
- **Heading-loss recovery**: If |error| > 90° for 10+ frames → creep at 20% speed with full steer until back within 90°

Output: `DriveCommand(target_velocity, steer_angle, speed_frac, heading data, waypoint data, escape flags)`

---

### 4. Dashboard System

A real-time web dashboard for monitoring and controlling the forklift simulation.

#### 4.1 Backend — [dashboard.py](simulations/forklift-warehouse/03_dashboard/dashboard.py)

FastAPI server (port 8080) that bridges the browser UI and the Isaac Sim controller:

```
Browser ──POST /api/cmd/*──→ Dashboard (:8080) ──POST──→ Controller (:8081)
                                  │
                   ┌──────────────┴──────────────┐
                   │                             │
            GET /api/state              WebSocket /ws
            (mtime-cached read          (50ms poll,
             of state JSON)              push on change
                                         or 2s heartbeat)
```

| Endpoint                | Method    | Description                                                                          |
| ----------------------- | --------- | ------------------------------------------------------------------------------------ |
| `/`                     | GET       | Serves `index.html`                                                                  |
| `/api/state`            | GET       | Returns `forklift_state.json` (mtime-cached to avoid redundant I/O)                  |
| `/api/controller-alive` | GET       | Health-check probe to controller port 8081                                           |
| `/api/cmd/{action}`     | POST      | Forwards pause/resume/reset_location to controller                                   |
| `/api/cmd/override`     | POST      | Forwards speed/lidar_range overrides                                                 |
| `/ws`                   | WebSocket | Streams live state updates (polls file every 50ms, pushes on change or 2s heartbeat) |

Port killer on startup ensures clean bind (kills any leftover process on 8080).

#### 4.2 Frontend — index.html + dashboard.js + styles.css

**Canvas map** (left 2/3 of viewport):

- Static layer: warehouse walls, rack positions, columns, obstacles, metric grid with axis labels, scale bar
- Dynamic layer: patrol path (polyline), position trail (dots), waypoints (numbered circles), forklift (to-scale 3.03m × 4.0m rectangle with cab/counterweight shading and fork tines)
- LIDAR overlay: 9-sector pie chart around forklift — full range (8m) for front 3 slices, half range (4m) for 6 side/back slices, color-coded per hit status

**Fleet cards** (right sidebar, tabbed interface):

- Position & heading readouts (X, Y, heading°, error°)
- Route info (waypoint index, lap count, distance to waypoint, frame counter)
- State badges: `CLEAR`/`SLOW`/`STOP` (LIDAR) + `DRIVING`/`DODGING`/`STUCK`/`PAUSED` (nav)
- Speed % with progress bar
- Control buttons: Pause, Resume, Reset Location
- Collapsible overrides: Speed slider (0–100%) + LIDAR range slider (1–25m)
- Optimistic UI: buttons disable for 200ms after command, revert if state doesn't change

**WebSocket loop**: Connects to `ws://<host>:8080/ws`, detects stale data (>1s without update → red pill), triggers canvas redraw + sidebar sync on each new state push.

**Styling**: Dark theme via CSS variables, responsive 2-column grid (collapses to single column < 700px).

---

## Data Flow: Forklift Simulation

```
USD Simulation Stage
  ├─ Forklift physics model (ArticulationRoot with joints)
  └─ LIDAR range sensor
      │
      ▼
forklift_controller.py (main async loop)
  ├─ Read: World transform (position, heading)
  ├─ Read: LIDAR depth data
      │
      ├─→ lidar_processor.py (obstacle detection & APF repulsion)
      ├─→ nav_controller.py (waypoint tracking & heading control)
      │
      ├─ Compute: Drive velocity & steering commands
      └─ Write: forklift_state.json
          │
          ▼
      Dashboard Backend (FastAPI)
        ├─ Read: forklift_state.json (periodic polling)
        └─ Broadcast: WebSocket state updates
            │
            ▼
      Browser UI
        ├─ Canvas: Real-time map visualization
        ├─ Cards: Fleet telemetry
        └─ Controls: Pause / Resume / Speed override
```
