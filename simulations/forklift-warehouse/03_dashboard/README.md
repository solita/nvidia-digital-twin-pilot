# Forklift Patrol Dashboard

Live browser dashboard for the forklift warehouse simulation.  
Shows the forklift position on a 2D floor map, patrol route progress, LIDAR state, and speed — updated in real time from the running simulation.

---

## Quick Start

### 1 — Start the simulation (Isaac Sim)

In VS Code, open `02_core_scripts/forklift_controller.py` and run it remotely:

```
Ctrl+Shift+P → Isaac Sim: Run File Remotely
```

The controller writes `04_current_outputs/forklift_state.json` every 10 simulation frames.

### 2 — Start the dashboard server

Open a terminal on the host machine and run:

```bash
cd simulations/forklift-warehouse/03_dashboard
python3 -m uvicorn dashboard:app --host 0.0.0.0 --port 8080
```

### 3 — Open in your browser

```
http://<your-host-ip>:8080
```

To find your host IP:
```bash
curl -s ifconfig.me
```

---

## What You See

| Panel | Content |
|---|---|
| **Floor map** (left) | Top-down warehouse grid, dashed patrol route, numbered waypoint circles (active = blue), yellow forklift with heading arrow, green position trail |
| **Position & Heading** | X/Y coordinates, current heading, heading error |
| **Patrol Route** | Active waypoint, lap counter, distance to next waypoint, frame number |
| **LIDAR** | State badge (CLEAR / SLOW / STOP), forward clearance, repulsion angle |
| **Speed** | Current speed as % of max drive velocity with progress bar |
| **LIVE / STALE pill** | Green = sim is writing data; turns red if no update for ~1 second |

---

## Data Flow

```
forklift_controller.py (Isaac Sim)
        │  writes every 10 frames
        ▼
04_current_outputs/forklift_state.json
        │  read on each HTTP request
        ▼
dashboard.py  GET /api/state  ←── browser polls every 200 ms
        │
        ▼
http://<host>:8080
```

---

## Stopping the Dashboard

```bash
# If started in background (&):
pkill -f "uvicorn dashboard:app"

# Or press Ctrl+C in the terminal where it's running
```

---

## Prerequisites

```bash
pip install fastapi uvicorn
```

Both are already installed on the Brev instance.
