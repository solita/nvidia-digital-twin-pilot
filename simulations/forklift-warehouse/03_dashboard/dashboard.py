"""dashboard.py — Forklift patrol dashboard.

Reads forklift_state.json written by forklift_controller.py every 10 frames
and serves a live browser UI showing the 2D warehouse floor map, forklift
position/heading, waypoint route, and key metrics.

Run:
    cd simulations/forklift-warehouse/03_dashboard
    python3 -m uvicorn dashboard:app --host 0.0.0.0 --port 8080

Then open http://<host>:8080 in a browser.
"""

import json
import os

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse

STATE_FILE = (
    "/home/ubuntu/docker/isaac-sim/data/nvidia-digital-twin-pilot/"
    "simulations/forklift-warehouse/04_current_outputs/forklift_state.json"
)

_EMPTY_STATE: dict = {
    "frame": 0,
    "x": 0.0, "y": 0.0,
    "heading": 0.0, "target_hdg": 0.0, "heading_err": 0.0,
    "wp": 0, "lap": 0, "dist_to_wp": 0.0,
    "lidar_state": "CLEAR",
    "forward_min": 9.9, "repulsion": 0.0,
    "speed_frac": 0.0,
    "waypoints": [],
}


def _read_state() -> dict:
    try:
        with open(STATE_FILE, encoding="utf-8") as fh:
            return json.load(fh)
    except Exception:
        return _EMPTY_STATE


_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Forklift Dashboard</title>
<style>
  :root {
    --bg:#1a1f2e; --card:#242b3d; --border:#2e3650;
    --ink:#e8eaf0; --muted:#7a8299;
    --green:#4caf7d; --yellow:#f5c842; --red:#e05555; --blue:#4a9eff;
  }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--ink); font-family:system-ui,sans-serif; }
  header {
    display:flex; align-items:center; gap:12px;
    padding:14px 24px; border-bottom:1px solid var(--border);
  }
  header h1 { font-size:1.15rem; font-weight:600; }
  .pill {
    margin-left:auto; padding:4px 12px; border-radius:99px;
    font-size:0.78rem; font-weight:600; letter-spacing:.04em;
    background:var(--green); color:#000;
  }
  .pill.stale { background:var(--red); }
  .layout {
    display:grid; grid-template-columns:1fr 300px; gap:16px;
    padding:16px 24px; height:calc(100vh - 57px);
  }
  .map-wrap {
    background:var(--card); border:1px solid var(--border); border-radius:12px;
    display:flex; flex-direction:column; overflow:hidden;
  }
  .map-title { padding:10px 16px; font-size:0.8rem; color:var(--muted); border-bottom:1px solid var(--border); }
  canvas#map { flex:1; width:100%; display:block; }
  .sidebar { display:flex; flex-direction:column; gap:12px; overflow-y:auto; }
  .card { background:var(--card); border:1px solid var(--border); border-radius:12px; padding:14px 16px; }
  .card-title { font-size:0.72rem; text-transform:uppercase; letter-spacing:.07em; color:var(--muted); margin-bottom:10px; }
  .metric-grid { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
  .lbl { font-size:0.7rem; color:var(--muted); margin-bottom:2px; }
  .val { font-size:1.25rem; font-weight:700; }
  .badge { display:inline-block; padding:3px 10px; border-radius:6px; font-size:0.78rem; font-weight:700; }
  .badge.CLEAR { background:#1b3a2a; color:var(--green); }
  .badge.SLOW  { background:#3a3010; color:var(--yellow); }
  .badge.STOP  { background:#3a1010; color:var(--red); }
  .progress-bar { height:8px; background:var(--border); border-radius:4px; overflow:hidden; margin-top:6px; }
  .progress-bar .fill { height:100%; background:var(--blue); border-radius:4px; transition:width .3s; }
  .footnote { font-size:0.72rem; color:var(--muted); margin-top:8px; }
</style>
</head>
<body>
<header>
  <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
    <rect x="1" y="3" width="15" height="13" rx="1"/>
    <path d="M16 8h5l2 3v4h-7V8z"/>
    <circle cx="5.5" cy="18.5" r="2.5"/>
    <circle cx="18.5" cy="18.5" r="2.5"/>
  </svg>
  <h1>Forklift Patrol Dashboard</h1>
  <span class="pill" id="liveTag">LIVE</span>
</header>
<div class="layout">
  <div class="map-wrap">
    <div class="map-title">Warehouse Floor Map — top-down (X east, Y north)</div>
    <canvas id="map"></canvas>
  </div>
  <div class="sidebar">
    <div class="card">
      <div class="card-title">Position &amp; Heading</div>
      <div class="metric-grid">
        <div><div class="lbl">X</div><div class="val" id="vX">--</div></div>
        <div><div class="lbl">Y</div><div class="val" id="vY">--</div></div>
        <div><div class="lbl">Heading</div><div class="val" id="vHdg">--</div></div>
        <div><div class="lbl">Hdg Error</div><div class="val" id="vErr">--</div></div>
      </div>
    </div>
    <div class="card">
      <div class="card-title">Patrol Route</div>
      <div class="metric-grid">
        <div><div class="lbl">Waypoint</div><div class="val" id="vWp">--</div></div>
        <div><div class="lbl">Lap</div><div class="val" id="vLap">--</div></div>
        <div><div class="lbl">Dist to WP</div><div class="val" id="vDist">--</div></div>
        <div><div class="lbl">Frame</div><div class="val" id="vFrame">--</div></div>
      </div>
    </div>
    <div class="card">
      <div class="card-title">LIDAR</div>
      <div style="margin-bottom:8px"><span class="badge CLEAR" id="lidarBadge">CLEAR</span></div>
      <div class="metric-grid">
        <div><div class="lbl">Fwd Min</div><div class="val" id="vFwd">--</div></div>
        <div><div class="lbl">Repulsion</div><div class="val" id="vRep">--</div></div>
      </div>
    </div>
    <div class="card">
      <div class="card-title">Speed</div>
      <div class="val" id="vSpeed">--%</div>
      <div class="progress-bar"><div class="fill" id="speedBar" style="width:0%"></div></div>
    </div>
    <div class="footnote" id="updated">Waiting for simulation data...</div>
  </div>
</div>
<script>
const WORLD = { xMin:-35, xMax:35, yMin:-45, yMax:60 };
const canvas = document.getElementById("map");
const ctx    = canvas.getContext("2d");
const trail  = [];
const TRAIL_MAX = 600;

function toCanvas(wx, wy) {
  const W = canvas.width, H = canvas.height;
  return [
    (wx - WORLD.xMin) / (WORLD.xMax - WORLD.xMin) * W,
    H - (wy - WORLD.yMin) / (WORLD.yMax - WORLD.yMin) * H
  ];
}

function drawMap(data) {
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);

  // Background
  ctx.fillStyle = "#1a1f2e";
  ctx.fillRect(0, 0, W, H);

  // Grid
  ctx.strokeStyle = "#2e3650"; ctx.lineWidth = 1;
  for (let x = Math.ceil(WORLD.xMin/10)*10; x <= WORLD.xMax; x += 10) {
    const [cx] = toCanvas(x, 0);
    ctx.beginPath(); ctx.moveTo(cx,0); ctx.lineTo(cx,H); ctx.stroke();
  }
  for (let y = Math.ceil(WORLD.yMin/10)*10; y <= WORLD.yMax; y += 10) {
    const [,cy] = toCanvas(0, y);
    ctx.beginPath(); ctx.moveTo(0,cy); ctx.lineTo(W,cy); ctx.stroke();
  }

  // Axis labels
  ctx.fillStyle = "#3a4260"; ctx.font = "10px system-ui";
  ctx.textAlign = "center";
  for (let x = -30; x <= 30; x += 10) {
    const [cx,cy] = toCanvas(x, WORLD.yMin);
    ctx.fillText(x, cx, cy - 4);
  }
  ctx.textAlign = "right";
  for (let y = -40; y <= 55; y += 10) {
    const [cx,cy] = toCanvas(WORLD.xMin, y);
    ctx.fillText(y, cx+18, cy+4);
  }

  const wps = data.waypoints || [];
  const sc  = canvas.width / (WORLD.xMax - WORLD.xMin);

  // Patrol path dashed loop
  if (wps.length > 1) {
    ctx.strokeStyle = "#2e3a50"; ctx.lineWidth = 2; ctx.setLineDash([6,4]);
    ctx.beginPath();
    wps.forEach(([wx,wy], i) => {
      const [px,py] = toCanvas(wx, wy);
      i === 0 ? ctx.moveTo(px,py) : ctx.lineTo(px,py);
    });
    ctx.closePath(); ctx.stroke(); ctx.setLineDash([]);
  }

  // Trail
  if (trail.length > 1) {
    ctx.strokeStyle = "#4caf7d55"; ctx.lineWidth = 2;
    ctx.beginPath();
    trail.forEach(([tx,ty], i) => {
      const [cx,cy] = toCanvas(tx,ty);
      i === 0 ? ctx.moveTo(cx,cy) : ctx.lineTo(cx,cy);
    });
    ctx.stroke();
  }

  // Waypoint markers
  wps.forEach(([wx,wy], idx) => {
    const [cx,cy] = toCanvas(wx, wy);
    const active  = idx === data.wp;
    ctx.beginPath();
    ctx.arc(cx, cy, active ? 8 : 5, 0, Math.PI*2);
    ctx.fillStyle   = active ? "#4a9eff" : "#2e3a50";
    ctx.strokeStyle = active ? "#4a9eff" : "#3e4f6a";
    ctx.lineWidth = 2; ctx.fill(); ctx.stroke();
    ctx.fillStyle = active ? "#fff" : "#7a8299";
    ctx.font = `bold ${active?11:9}px system-ui`;
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText(idx, cx, cy);
  });

  // Forklift rectangle + heading arrow
  const [fx, fy] = toCanvas(data.x, data.y);
  const hdgRad   = (data.heading * Math.PI) / 180;
  const bW = 3.03 * sc, bL = 4.0 * sc;

  ctx.save();
  ctx.translate(fx, fy);
  ctx.rotate(-hdgRad);   // canvas Y is flipped → negate
  ctx.fillStyle   = "#c8a832";
  ctx.strokeStyle = "#f5c842"; ctx.lineWidth = 2;
  ctx.beginPath();
  if (ctx.roundRect) ctx.roundRect(-bL/2, -bW/2, bL, bW, 3);
  else ctx.rect(-bL/2, -bW/2, bL, bW);
  ctx.fill(); ctx.stroke();
  // Arrow
  ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(bL/2+6,0); ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(bL/2+6,0); ctx.lineTo(bL/2+1,-4);
  ctx.moveTo(bL/2+6,0); ctx.lineTo(bL/2+1,+4);
  ctx.stroke();
  ctx.restore();
}

function updateSidebar(data) {
  document.getElementById("vX").textContent     = data.x     != null ? data.x.toFixed(1)           + " m" : "--";
  document.getElementById("vY").textContent     = data.y     != null ? data.y.toFixed(1)           + " m" : "--";
  document.getElementById("vHdg").textContent   = data.heading    != null ? data.heading.toFixed(1)+ "°"  : "--";
  document.getElementById("vErr").textContent   = data.heading_err!= null ? (data.heading_err>=0?"+":"")+data.heading_err.toFixed(1)+"°" : "--";
  document.getElementById("vWp").textContent    = data.wp    != null ? "WP " + data.wp             : "--";
  document.getElementById("vLap").textContent   = data.lap   != null ? data.lap                    : "--";
  document.getElementById("vDist").textContent  = data.dist_to_wp != null ? data.dist_to_wp.toFixed(1) + " m" : "--";
  document.getElementById("vFrame").textContent = data.frame != null ? data.frame                  : "--";

  const ls = data.lidar_state || "CLEAR";
  const badge = document.getElementById("lidarBadge");
  badge.textContent = ls; badge.className = "badge " + ls;

  document.getElementById("vFwd").textContent = data.forward_min != null
    ? (data.forward_min >= 9.8 ? "Clear" : data.forward_min.toFixed(1)+" m") : "--";
  document.getElementById("vRep").textContent = data.repulsion != null
    ? (data.repulsion>=0?"+":"")+data.repulsion.toFixed(1)+"°" : "--";

  const sp = Math.round((data.speed_frac || 0) * 100);
  document.getElementById("vSpeed").textContent   = sp + "%";
  document.getElementById("speedBar").style.width = sp + "%";
}

let lastFrame = -1, staleCount = 0;

function resizeCanvas() {
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width  = Math.floor(rect.width)  || 600;
  canvas.height = Math.floor(rect.height) - 36 || 400;
}

async function refresh() {
  try {
    const res  = await fetch("/api/state", { cache:"no-store" });
    const data = await res.json();
    resizeCanvas();
    if (data.x != null) { trail.push([data.x, data.y]); if (trail.length > TRAIL_MAX) trail.shift(); }
    drawMap(data);
    updateSidebar(data);
    if (data.frame !== lastFrame) { staleCount = 0; lastFrame = data.frame; } else staleCount++;
    const tag = document.getElementById("liveTag");
    if (staleCount > 4) { tag.textContent="STALE"; tag.className="pill stale"; }
    else                { tag.textContent="LIVE";  tag.className="pill"; }
    document.getElementById("updated").textContent = "Frame " + (data.frame || 0);
  } catch(e) {
    document.getElementById("updated").textContent = "Connection error — retrying...";
  }
}

window.addEventListener("resize", resizeCanvas);
resizeCanvas();
refresh();
setInterval(refresh, 200);
</script>
</body>
</html>"""


app = FastAPI(title="Forklift Dashboard")


@app.get("/", response_class=HTMLResponse)
def index():
    return HTMLResponse(_HTML)


@app.get("/api/state", response_class=JSONResponse)
def get_state():
    return JSONResponse(_read_state())
