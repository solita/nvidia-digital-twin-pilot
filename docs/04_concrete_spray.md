# Guide 04 — Running the Concrete Spray Simulation

This guide walks through deploying the concrete spray (shotcrete) simulation onto the Brev instance and running it inside Isaac Sim. This serves as the first end-to-end test beyond the smoke test — real scene, real scripts, real output.

For a full reference of the simulation's folder structure, scripts, and outputs see the [simulation README](../simulations/concrete_spray/README.md).

---

## Prerequisites

- Guides 01–03 complete and smoke test passing
- Repo cloned on the Brev instance at `~/nvidia-digital-twin-pilot` (required for multi-root workspace — [Guide 02, Step 1](./02_vscode_access.md#step-1--clone-the-repo-and-open-the-multi-root-workspace))
- Isaac Sim container running (`isaac-sim` container active)

---

## Step 1 — Copy the Simulation to the Isaac Sim Data Volume

The Isaac Sim container reads from `/home/ubuntu/docker/isaac-sim/data/` (volume-mounted as `/isaac-sim/.local/share/ov/data/` inside the container). Copy the simulation there.

The `data/` directory is owned by UID 1234 (set in Guide 01, Step 6). Grant `ubuntu` write access to it first, then copy:

```bash
sudo chmod o+rwx /home/ubuntu/docker/isaac-sim/data
cp -r ~/nvidia-digital-twin-pilot/simulations/concrete_spray \
      /home/ubuntu/docker/isaac-sim/data/
```

---

## Step 2 — Fix Permissions

The container runs as UID 1234. The `ubuntu` user also needs access for VS Code to read/write the files.

```bash
sudo chown -R 1234:1234 /home/ubuntu/docker/isaac-sim/data/concrete_spray
sudo chmod -R o+rwx /home/ubuntu/docker/isaac-sim/data/concrete_spray
```

---

## Step 3 — Option A: Open Scene in Isaac Sim and Press Play

This is the simplest approach — load the scene manually in the WebRTC client and start the simulation from the UI.

In the **Isaac Sim WebRTC Streaming Client**:

1. Go to **File → Open**.
2. Navigate to:
   ```
   /isaac-sim/.local/share/ov/data/concrete_spray/01_scenes/boolean Applied ver3_preview_particles.usdc
   ```
3. Wait for the stage to fully load — the status bar should show the scene path.
4. Press the **Play** button (triangle) in the Isaac Sim toolbar to start the simulation.

> Use the `_preview_particles` variant of the scene (not the base scene). This is the one configured for the live preview renderer.

---

## Step 4 — Option B: Run Programmatically (no manual scene loading)

These alternatives use the launcher script, which opens the scene, waits for it to load, starts the timeline, and runs the simulation automatically — no WebRTC interaction needed.

### Via VS Code Remote Execution

1. In the `Isaac Sim Data` tree in VS Code, open:
   ```
   concrete_spray/02_core_scripts/shotcrete_direct_cave_preview_launcher_v1.py
   ```
2. Press `Ctrl+Shift+P` → **Isaac Sim: Run Remotely**.
3. Monitor output in `Ctrl+Shift+U` → Output → **"Isaac Sim"**.

The scene will appear in the WebRTC client automatically as the launcher opens it.

### Via shell script inside the container

```bash
# On the Brev host
docker exec -it isaac-sim bash

# Now inside the container
cd /isaac-sim/.local/share/ov/data/concrete_spray
./run_direct_cave_preview_v1.sh
```

---

## Step 5 — Verify Outputs

Run outputs are written to `04_current_outputs/` (gitignored). Check that two files were created:

```bash
ls /home/ubuntu/docker/isaac-sim/data/concrete_spray/04_current_outputs/
```

Expected files:

```
shotcrete_direct_cave_preview_v1_run_<YYYYMMDD_HHMMSS>.json
shotcrete_direct_cave_preview_v1_run_<YYYYMMDD_HHMMSS>.txt
shotcrete_direct_cave_preview_v1_latest.json
shotcrete_direct_cave_preview_v1_latest.txt
```

The `_latest.*` files are always overwritten with the most recent run — useful for quick inspection.

---

## Optional — Run Validation

A validation script checks the run output for correctness:

### Via VS Code

Open `02_core_scripts/shotcrete_direct_cave_preview_validate_v1.py` and run remotely.

### Via shell (inside container)

```bash
cd /isaac-sim/.local/share/ov/data/concrete_spray
./run_direct_cave_preview_validate_v1.sh
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Scene fails to open | Wrong scene variant | Use `boolean Applied ver3_preview_particles.usdc`, not the base scene |
| `Permission denied` on data directory | Permissions not set | Re-run Step 2 |
| Launcher times out waiting for stage | Cold cache / slow shader compile | Increase `STAGE_READY_TIMEOUT_S` in the launcher (default: `90.0`) |
| No output files after run | Script error mid-run | Check the Isaac Sim output panel for exceptions |
| `docker exec` fails | Container not running | Confirm container is up: `docker ps | grep isaac-sim` |

---

**Next:** [Guide 05 — Developer Workflow](./05_developer_workflow.md)
