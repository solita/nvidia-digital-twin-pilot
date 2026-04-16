# Forklift Warehouse — Autonomous Navigation POC

A digital twin POC of an autonomous forklift navigating a warehouse environment, built on NVIDIA Isaac Sim.

The forklift drives a pre-defined waypoint path, detects obstacles via a LIDAR sensor rig, and stops or reroutes when a hazard is detected. Live sensor stats are streamed to an external dashboard.

---

## Folder Structure

```
forklift-warehouse/
├── 01_scenes/                  # USD scene files
│   ├── scene_assembly.usd      ← open this in Isaac Sim (references warehouse + forklift + sensors)
│   ├── warehouse.usd           ← custom warehouse environment (if not using stock Isaac asset)
│   └── sensors.usd             ← LIDAR rig positioned on the forklift
├── 02_core_scripts/
│   ├── forklift_launcher.py    ← entry point: opens scene, wires up controller
│   ├── forklift_controller.py  ← waypoint path driving logic (Dev 2)
│   ├── obstacle_avoidance.py   ← LIDAR → stop/reroute logic (Dev 2)
│   └── lidar_subscriber.py     ← streams sensor data to dashboard (Dev 3)
├── 03_dashboard/
│   └── dashboard.py            ← real-time stats UI (Dev 3)
├── 04_current_outputs/         # Runtime outputs — gitignored
├── 05_reference_milestones/    # Archived good runs
├── 06_master_handoff/
└── run_sim.sh
```

---


## Prerequisites

- Brev instance running with Isaac Sim 5.1.0 container (see [Guide 01](../../docs/01_brev_setup.md))
- VS Code connected to Brev with read/write access (see [Guide 02](../../docs/02_vscode_access.md))
- Isaac Sim VS Code Edition extension configured (see [Guide 03](../../docs/03_vscode_isaacsim_smoke_test.md))

---

## Deploying to the Brev Machine

```bash
# Copy this simulation folder to the Isaac Sim data volume
cp -r /path/to/<simulation_name> /home/ubuntu/docker/isaac-sim/data/

# Fix ownership (Isaac Sim runs as UID 1234)
sudo chown -R 1234:1234 /home/ubuntu/docker/isaac-sim/data/<simulation_name>

# Fix permissions (VS Code runs as ubuntu / UID 1000)
sudo chmod -R o+rwx /home/ubuntu/docker/isaac-sim/data/<simulation_name>
```

Inside the container the simulation will be at:

```
/isaac-sim/.local/share/ov/data/<simulation_name>/
```

---

## Running the Simulation

### Option A — Shell script

```bash
cd /isaac-sim/.local/share/ov/data/<simulation_name>
./run_sim.sh
```

### Option B — VS Code Remote Execution

1. Open `02_core_scripts/launcher.py` in the remote VS Code window.
2. Press `Ctrl+Shift+P` → **Isaac Sim: Run Remotely**.

---

## Outputs

Runtime outputs are written to `04_current_outputs/` (gitignored). Document the
expected output format here once your simulation produces them.

---