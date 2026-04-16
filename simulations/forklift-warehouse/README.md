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

## One-time Setup on Your Brev Instance

Each developer uses their own Brev instance. Run these steps once after provisioning.

**Clone the repo directly into the Isaac Sim data volume:**
```bash
sudo mkdir -p /home/ubuntu/docker/isaac-sim/data
sudo chmod o+rwx /home/ubuntu/docker/isaac-sim/data

git clone https://github.com/<org>/nvidia-digital-twin-pilot.git \
  /home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot

sudo chown -R 1234:1234 /home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot
sudo chmod -R o+rwx /home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot
```

That's it — no copy-paste needed. Scene files are immediately visible to Isaac Sim. After that, `git pull` is all you need to pick up updates.

---

## Running the Simulation

**1. Open Isaac Sim in your browser:**
```
http://<your-brev-instance-ip>:8211/streaming/client/
```

**2. Load the scene in Isaac Sim:**
File → Open → `/isaac-sim/data/nvidia-digital-twin-pilot/simulations/forklift-warehouse/01_scenes/scene_assembly.usd`

**3. Run the controller from VS Code (Remote-SSH to your instance):**

Open `simulations/forklift-warehouse/02_core_scripts/forklift_controller.py` and press `Ctrl+Shift+P` → **Isaac Sim: Run File Remotely**.

The forklift will start driving the warehouse patrol route automatically.

---

## Outputs

Runtime outputs are written to `04_current_outputs/` (gitignored). Document the
expected output format here once your simulation produces them.

---