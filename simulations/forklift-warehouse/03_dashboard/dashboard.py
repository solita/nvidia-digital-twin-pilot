"""dashboard.py - Simple direct-file lidar dashboard.

This file intentionally chooses the simpler dashboard option from the original
stub: a small FastAPI app with polling JSON endpoints.

Why this option:
  - It avoids ROS 2 entirely.
  - The Isaac Sim script writes a shared JSON file and this app reads it.
  - A tiny FastAPI page keeps everything in one file and is easy to run.

Expected data file:
  runtime/lidar_state.json

Run:
  uvicorn dashboard:app --host 0.0.0.0 --port 8080
"""

import math
import os
import time

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse


SAFETY_DISTANCE_M = 2.0
STATE_FILE_PATH = "/home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot/simulations/forklift-warehouse/03_dashboard/runtime/lidar_state.json"

_state = {
    "updated_at": 0.0,
    "nearest_obstacle_m": None,
    "path_blocked": None,
    "scan": {
        "range_min": None,
        "range_max": None,
        "ranges": [],
        "count": 0,
    },
}


INDEX_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Forklift Lidar Dashboard</title>
  <style>
    :root {
      --bg: #f2efe8;
      --card: #fffaf2;
      --ink: #1d2a2f;
      --muted: #647076;
      --accent: #c75b39;
      --safe: #2e7d32;
      --danger: #c62828;
      --grid: #d8d0c2;
    }
    body {
      margin: 0;
      font-family: Georgia, "Times New Roman", serif;
      background: radial-gradient(circle at top left, #fff7e5, var(--bg) 55%);
      color: var(--ink);
    }
    main {
      max-width: 1100px;
      margin: 0 auto;
      padding: 24px;
    }
    h1 {
      margin: 0 0 8px;
      font-size: 2.1rem;
    }
    p {
      color: var(--muted);
      margin-top: 0;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 16px;
      margin: 24px 0;
    }
    .card {
      background: var(--card);
      border: 1px solid #eadfca;
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 10px 30px rgba(0, 0, 0, 0.06);
    }
    .label {
      font-size: 0.85rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--muted);
      margin-bottom: 8px;
    }
    .value {
      font-size: 1.8rem;
      font-weight: 700;
    }
    .status-safe {
      color: var(--safe);
    }
    .status-danger {
      color: var(--danger);
    }
    canvas {
      width: 100%;
      height: 320px;
      display: block;
      background: linear-gradient(180deg, #fffdf7, #f5efe3);
      border-radius: 18px;
      border: 1px solid #eadfca;
      box-shadow: inset 0 0 0 1px rgba(255, 255, 255, 0.3);
    }
    .footnote {
      margin-top: 12px;
      font-size: 0.9rem;
      color: var(--muted);
    }
  </style>
</head>
<body>
  <main>
    <h1>Forklift Lidar Dashboard</h1>
    <p>Simple option selected: FastAPI + polling UI. Data source is a shared JSON file.</p>
    <section class="grid">
      <article class="card"><div class="label">Nearest Obstacle</div><div class="value" id="nearest">--</div></article>
      <article class="card"><div class="label">Path State</div><div class="value" id="blocked">--</div></article>
      <article class="card"><div class="label">Scan Samples</div><div class="value" id="samples">0</div></article>
      <article class="card"><div class="label">Range Limits</div><div class="value" id="limits">--</div></article>
    </section>
    <canvas id="scanCanvas" width="1000" height="320"></canvas>
    <div class="footnote" id="updated">Waiting for ROS 2 messages...</div>
  </main>
  <script>
    const canvas = document.getElementById("scanCanvas");
    const ctx = canvas.getContext("2d");

    function drawScan(ranges, maxRange) {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.strokeStyle = "#d8d0c2";
      ctx.lineWidth = 1;
      for (let i = 1; i <= 4; i += 1) {
        const y = (canvas.height / 5) * i;
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
      }

      if (!ranges || ranges.length === 0 || !maxRange) {
        return;
      }

      ctx.strokeStyle = "#c75b39";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ranges.forEach((value, index) => {
        const x = (index / Math.max(ranges.length - 1, 1)) * canvas.width;
        const clipped = Math.min(value, maxRange);
        const y = canvas.height - (clipped / maxRange) * (canvas.height - 20) - 10;
        if (index === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();
    }

    async function refresh() {
      const response = await fetch("/api/state", { cache: "no-store" });
      const data = await response.json();

      document.getElementById("nearest").textContent =
        data.nearest_obstacle_m == null ? "--" : `${data.nearest_obstacle_m.toFixed(3)} m`;

      const blockedEl = document.getElementById("blocked");
      if (data.path_blocked == null) {
        blockedEl.textContent = "--";
        blockedEl.className = "value";
      } else if (data.path_blocked) {
        blockedEl.textContent = "BLOCKED";
        blockedEl.className = "value status-danger";
      } else {
        blockedEl.textContent = "CLEAR";
        blockedEl.className = "value status-safe";
      }

      document.getElementById("samples").textContent = String(data.scan.count || 0);
      document.getElementById("limits").textContent =
        data.scan.range_min == null ? "--" : `${data.scan.range_min.toFixed(1)}-${data.scan.range_max.toFixed(1)} m`;

      const updated = data.updated_at ? new Date(data.updated_at * 1000).toLocaleTimeString() : "--";
      document.getElementById("updated").textContent = `Last update: ${updated}`;
      drawScan(data.scan.ranges || [], data.scan.range_max || 1.0);
    }

    refresh();
    setInterval(refresh, 500);
  </script>
</body>
</html>
"""


def _touch_state():
    _state["updated_at"] = time.time()


def _coerce_state(payload):
  scan = payload.get("scan") or {}
  ranges = scan.get("ranges") or []
  valid = [value for value in ranges if math.isfinite(value) and value > 0.0]
  nearest = min(valid) if valid else None
  return {
    "updated_at": payload.get("updated_at", 0.0),
    "nearest_obstacle_m": nearest,
    "path_blocked": nearest is not None and nearest < SAFETY_DISTANCE_M,
    "scan": {
      "range_min": scan.get("range_min"),
      "range_max": scan.get("range_max"),
      "ranges": ranges,
      "count": scan.get("count", len(ranges)),
    },
  }


def _read_state():
  if not os.path.exists(STATE_FILE_PATH):
    return _state
  try:
    import json

    with open(STATE_FILE_PATH, "r", encoding="utf-8") as handle:
      payload = json.load(handle)
    return _coerce_state(payload)
  except Exception:
    return _state

app = FastAPI(title="Forklift Lidar Dashboard")


@app.get("/", response_class=HTMLResponse)
def index():
    return HTMLResponse(INDEX_HTML)


@app.get("/api/state", response_class=JSONResponse)
def get_state():
  return JSONResponse(_read_state())
