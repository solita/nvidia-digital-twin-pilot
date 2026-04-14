# Concrete Spray (Shotcrete) Simulation

A real-time digital twin simulation of a **concrete spray (shotcrete) operation** inside a mining cave environment, built for NVIDIA Isaac Sim.

The simulation visualises shotcrete particle distribution across a cave surface, driven by a microburst proxy data pipeline and rendered using USD particle instancing.

---

## Folder Structure

```
concrete_spray/
├── 01_scenes/                  # USD scene files
│   ├── CAVE_clean_master.usdc  # Base cave geometry
│   ├── boolean Applied ver3.usdc
│   ├── boolean Applied ver3_preview_particles.usdc   ← used by the preview launcher
│   ├── boolean Applied ver3_calibration.usdc
│   └── textures/
├── 02_core_scripts/            # Python scripts run inside Isaac Sim
│   ├── shotcrete_direct_cave_preview_launcher_v1.py  ← entry point
│   ├── shotcrete_direct_cave_preview_v1.py
│   ├── shotcrete_direct_cave_preview_validate_v1.py
│   ├── shotcrete_microburst_v16_2_proxy_receiver_exact_field_export.py
│   ├── shotcrete_proxy_to_cave_map_v17_8b_exact_field_import_new_cave_autodetect.py
│   └── shotcrete_one_click_pipeline_v1.py
├── 03_preview_donor_scripts/   # Supporting/donor scripts
├── 04_current_outputs/         # Run output files (gitignored — generated at runtime)
├── 05_reference_milestones/    # Archived milestone snapshots
├── 06_master_handoff/          # Handoff session logs
├── run_direct_cave_preview_v1.sh       ← Linux launch script
├── run_direct_cave_preview_validate_v1.sh
└── STARTUP_PROMPT_UPDATED.txt
```

---

## Prerequisites

- Brev instance running with Isaac Sim 5.1.0 container (see [Guide 01](../../docs/01_brev_setup.md))
- VS Code connected to Brev with read/write access (see [Guide 02](../../docs/02_vscode_access.md))
- Isaac Sim VS Code Edition extension configured (see [Guide 03](../../docs/03_vscode_isaacsim_smoke_test.md))

---

## Deploying to the Brev Machine

With the repo cloned on the Brev instance (see [Guide 02](../../docs/02_vscode_access.md)), copy the simulation into the Isaac Sim data volume directly on the Brev host:

```bash
cp -r ~/nvidia-digital-twin-pilot/simulations/concrete_spray \
      /home/ubuntu/docker/isaac-sim/data/

# Fix ownership so Isaac Sim (UID 1234) can write outputs
sudo chown -R 1234:1234 /home/ubuntu/docker/isaac-sim/data/concrete_spray

# Fix permissions so VS Code (ubuntu user) also has access
sudo chmod -R o+rwx /home/ubuntu/docker/isaac-sim/data/concrete_spray
```

> For the full step-by-step walkthrough see [Guide 04 — Running the Concrete Spray Simulation](../../docs/04_concrete_spray.md).

Inside the container, the simulation will be accessible at:

```
/isaac-sim/.local/share/ov/data/concrete_spray/
```

---

## Running the Preview (Direct Cave Preview)

### Option A — Shell script (recommended)

From inside the Isaac Sim container (after `docker exec -it isaac-sim bash` or from within the running container session):

```bash
cd /isaac-sim/.local/share/ov/data/concrete_spray
./run_direct_cave_preview_v1.sh
```

This script calls `isaac-sim.sh --exec shotcrete_direct_cave_preview_launcher_v1.py` from the `02_core_scripts/` directory.

### Option B — VS Code Remote Execution

1. Open `02_core_scripts/shotcrete_direct_cave_preview_launcher_v1.py` in VS Code (remote window).
2. Press `Ctrl+Shift+P` → **Isaac Sim: Run Remotely**.

The launcher will:
1. Open the preview particles scene (`01_scenes/boolean Applied ver3_preview_particles.usdc`)
2. Wait for the stage to fully load
3. Start the timeline and execute the preview simulation script

---

## Running Validation

```bash
./run_direct_cave_preview_validate_v1.sh
```

Or run `shotcrete_direct_cave_preview_validate_v1.py` remotely from VS Code.

---

## Outputs

Run outputs (JSON data + text logs) are written to `04_current_outputs/` and are gitignored. Each run produces:

- `shotcrete_direct_cave_preview_v1_run_<YYYYMMDD_HHMMSS>.json`
- `shotcrete_direct_cave_preview_v1_run_<YYYYMMDD_HHMMSS>.txt`
- `shotcrete_direct_cave_preview_v1_latest.json` (always overwritten with the most recent run)
- `shotcrete_direct_cave_preview_v1_latest.txt`

---

## Notes

- The scene uses **USD particle instancing** — ensure the `preview_particles` variant scene is used for the live preview, not the base scene.
- The launcher has a configurable `STAGE_READY_TIMEOUT_S = 90.0` — on a cold cache this may need increasing.
- The `.bat` files in the root are Windows variants kept for reference; use the `.sh` scripts on Brev (Ubuntu).
