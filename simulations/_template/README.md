# [Simulation Name]

<!-- Replace this header and the description below with your simulation's details. -->

A brief description of what this simulation does, what physical process it models,
and how it fits into the broader digital twin project.

---

## Folder Structure

```
<simulation_name>/
├── 01_scenes/                  # USD scene files and textures
├── 02_core_scripts/            # Python scripts executed inside Isaac Sim
│   └── launcher.py             ← entry point (rename to match your sim)
├── 03_preview_donor_scripts/   # Supporting or donor scripts
├── 04_current_outputs/         # Runtime outputs — gitignored, generated at runtime
├── 05_reference_milestones/    # Archived snapshots of good known-good runs
├── 06_master_handoff/
│   └── session_logs/
└── run_sim.sh                  ← Linux launch script
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

## Notes

<!-- Add simulation-specific notes here: known limitations, config knobs, etc. -->
