# Forklift Dashboard (03_dashboard)

This folder contains a simple FastAPI dashboard for forklift LiDAR status. The app reads a shared JSON state file and serves a small browser UI plus a JSON API.

## Data Flow

1. Isaac Sim scene runs with a forklift LiDAR prim.
2. `02_core_scripts/lidar_sensor_setup.py` creates/attaches the PhysX LiDAR sensor at `/World/forklift_b/body/lidar_sensor`.
3. `02_core_scripts/lidar_subscriber.py` reads LiDAR depth data and continuously writes the latest snapshot to:
   - `/home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot/simulations/forklift-warehouse/03_dashboard/runtime/lidar_state.json`
   - (container candidate) `/isaac-sim/.local/share/ov/data/nvidia-digital-twin-pilot/simulations/forklift-warehouse/03_dashboard/runtime/lidar_state.json`
4. `03_dashboard/dashboard.py` reads `runtime/lidar_state.json` and exposes:
   - `GET /` (HTML dashboard)
   - `GET /api/state` (latest state JSON)
5. Browser polls `/api/state` every 500 ms and updates metrics/plot.

## Files In This Folder

- `dashboard.py`: FastAPI web app + embedded HTML/JS UI; reads the shared state file.
- `runtime/lidar_state.json`: latest LiDAR snapshot written by `lidar_subscriber.py`.
- `.gitignore`: ignores `runtime` runtime output directory content.

## Prerequisites

- Python 3.10+ environment.
- FastAPI and Uvicorn available (required by `dashboard.py` and its run command).
- If you want live data: Isaac Sim scene is open and LiDAR pipeline scripts are run from Isaac Sim.

## Run The Dashboard Web App

From this folder:

```bash
cd /home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot/simulations/forklift-warehouse/03_dashboard
uvicorn dashboard:app --host 0.0.0.0 --port 8080
```

Then open `http://<host>:8080`.

## Produce Live LiDAR Data (Isaac Sim)

1. Open the forklift scene in Isaac Sim.
2. Run `02_core_scripts/lidar_sensor_setup.py` first (required to create the LiDAR prim).
3. Run `02_core_scripts/lidar_subscriber.py` next (writes `runtime/lidar_state.json` continuously).

Both scripts are intended to be run via Isaac Sim remote execution in VS Code (Command Palette: **Isaac Sim: Run File Remotely**).

## Startup Without Live Data

The dashboard can start even when no live LiDAR stream is running.

- If `runtime/lidar_state.json` is missing/unreadable, the app returns default empty state.
- UI indicators will remain placeholders (for example `--` and sample count `0`) until valid state is available.

To detect stale data, watch the dashboard "Last update" time: if it is not advancing, `lidar_subscriber.py` is not actively updating the JSON.

## Troubleshooting

- Missing JSON file:
  - Run `lidar_sensor_setup.py` first, then `lidar_subscriber.py`.
  - Confirm `runtime/lidar_state.json` appears in this folder.
- Stale JSON (timestamp not changing):
  - Confirm Isaac Sim timeline is playing and `lidar_subscriber.py` is still running.
  - Check subscriber logs for sensor-not-ready or missing-prim messages.
- Path/permission mismatch:
  - `lidar_subscriber.py` writes to the first writable candidate path.
  - Ensure write access to dashboard `runtime/` under the same mounted workspace path used by both Isaac Sim and your host process.
