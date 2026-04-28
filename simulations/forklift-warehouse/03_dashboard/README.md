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

## Forklift Control Architecture

The controller exposes a lightweight HTTP server (stdlib `HTTPServer`, daemon
thread) that runs **inside Isaac Sim** on port `8081`.  The dashboard acts as a
thin proxy — browser clicks POST to the dashboard, which forwards them to the
controller.

```
Browser
  │
  │  POST http://<host>:8080/api/cmd/pause
  │  POST http://<host>:8080/api/cmd/resume
  ▼
dashboard.py  (FastAPI, port 8080)
  │
  │  POST http://localhost:8081/cmd  {"action": "pause"|"resume"}
  ▼
forklift_controller.py — stdlib HTTPServer thread (port 8081)
  │
  │  puts {"action": ..., "value": ...} into queue.SimpleQueue
  ▼
run_forklift() asyncio loop — drains queue every simulation frame
  │
  ├─ "pause"  → stops drive/steer joints, sets paused=True in live state
  └─ "resume" → resumes normal nav loop, sets paused=False in live state
```

### State feedback loop

`forklift_controller.py` writes `forklift_state.json` every 10 frames
(and on every frame while paused).  The `"paused"` field in that JSON is
read by the dashboard's state poll, which calls `updateButtons(paused)` in
the browser to flip the button between **⏸ Pause** and **▶ Resume**.

### Port binding across re-runs

The controller creates the port-8081 socket once on first run and stores it
in `sys.modules["__forklift_cmd_sock__"]`.  Every subsequent re-run of the
script (without restarting Isaac Sim) reuses the same socket — no rebind,
no "Address already in use" errors.

### Adding new commands

To add a new control action:

1. **Controller** (`forklift_controller.py`) — add an `elif action == "your_action":` branch in the command-drain loop.
2. **Dashboard backend** (`dashboard.py`) — add a `@app.post("/api/cmd/your_action")` endpoint that calls `_post_controller_cmd("your_action")`.
3. **Dashboard frontend** (the `<script>` block in `_HTML`) — add a button that calls `sendCmd('your_action')`.

No changes needed to the HTTP server, queue, or port binding.

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
