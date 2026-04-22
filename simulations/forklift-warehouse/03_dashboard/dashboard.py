"""dashboard.py — Forklift patrol dashboard (top-down warehouse view).

Reads forklift_state.json written by forklift_controller.py every 10 frames
and serves a live browser UI showing a to-scale top-down view of every asset
in scene_assembly.usd: warehouse walls, rack shelving, structural columns,
obstacle cubes, the forklift (live), waypoint route, and key metrics.

Asset geometry is extracted from scene_assembly.usd and the controller's
spatial calibration data so the map matches the physical simulation exactly.

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
    --bg:#0f1219; --card:#1a1f2e; --border:#2e3650;
    --ink:#e8eaf0; --muted:#7a8299;
    --green:#4caf7d; --yellow:#f5c842; --red:#e05555; --blue:#4a9eff;
    --floor:#1e2433; --wall:#3a4260; --rack:#5c3d1a; --column:#6b7a99;
    --cube:#c05050; --forklift:#f5c842;
  }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--ink); font-family:'Inter',system-ui,sans-serif; }
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
    display:flex; flex-direction:column; overflow:hidden; position:relative;
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
  .legend { display:flex; flex-wrap:wrap; gap:10px; }
  .legend-item { display:flex; align-items:center; gap:5px; font-size:0.72rem; color:var(--muted); }
  .legend-swatch { width:14px; height:10px; border-radius:2px; }
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
  <h1>Forklift Warehouse — Top-Down View</h1>
  <span class="pill" id="liveTag">LIVE</span>
</header>
<div class="layout">
  <div class="map-wrap">
    <div class="map-title">scene_assembly.usd — All assets to scale (1 grid = 10 m) &nbsp;|&nbsp; X → east, Y ↑ north</div>
    <canvas id="map"></canvas>
  </div>
  <div class="sidebar">
    <div class="card">
      <div class="card-title">Scene Legend</div>
      <div class="legend">
        <div class="legend-item"><div class="legend-swatch" style="background:#2a3348;border:1px solid #3a4260"></div>Warehouse floor</div>
        <div class="legend-item"><div class="legend-swatch" style="background:#3a4260"></div>Walls</div>
        <div class="legend-item"><div class="legend-swatch" style="background:#5c3d1a"></div>Rack shelving</div>
        <div class="legend-item"><div class="legend-swatch" style="background:#6b7a99"></div>Columns</div>
        <div class="legend-item"><div class="legend-swatch" style="background:#c05050"></div>Obstacle cubes</div>
        <div class="legend-item"><div class="legend-swatch" style="background:#f5c842"></div>Forklift body</div>
        <div class="legend-item"><div class="legend-swatch" style="background:#8b9ab0"></div>Fork tines →</div>
        <div class="legend-item"><div class="legend-swatch" style="background:#4a9eff"></div>Waypoints</div>
        <div class="legend-item"><div class="legend-swatch" style="background:#4caf7d55"></div>Trail</div>
      </div>
    </div>
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
    <div class="card">
      <div class="card-title">Scene Assets</div>
      <div style="font-size:0.75rem; color:var(--muted); line-height:1.6">
        <div>/World/warehouse <span style="color:#5c8a5c">(payload)</span></div>
        <div>/World/forklift_b <span style="color:#5c8a5c">(payload)</span></div>
        <div>/World/Cube .. Cube_06 <span style="color:#c05050">×6 obstacles</span></div>
        <div>/World/PhysicsScene</div>
        <div>/World/PhysicsGround <span style="color:#555">(invisible)</span></div>
      </div>
    </div>
    <div class="footnote" id="updated">Waiting for simulation data...</div>
  </div>
</div>
<script>
// ═══════════════════════════════════════════════════════════════════════
// SCENE DATA — extracted from scene_assembly.usd & controller spatial
// calibration.  All coordinates in world-space metres.
//
// Cardinal directions: +Y = north, -Y = south, +X = east, -X = west.
// Warehouse long axis runs north–south (Y).
// Long walls are the east (X) and west (-X) walls.
// ═══════════════════════════════════════════════════════════════════════

// /World/warehouse — Simple_Warehouse payload, translate (-0.50, -0.76),
// scale (3.019, 3.004, 3.169).
// Bounds derived from controller calibration:
//   south floor boundary Y ≈ -36.4,  north wall Y ≈ 52.3
//   west rack at X = -30.27 → west wall just behind ≈ X = -31.5
//   easternmost column X = 26.52,  ~9 m buffer comment → east wall ≈ X = +28
const WAREHOUSE = {
  xMin: -31.5, xMax: 28.0,     // ~59.5 m east–west (short dimension)
  yMin: -36.4, yMax: 52.3,     // ~88.7 m north–south (long dimension)
  wallThickness: 0.8,
};

// Viewport — add padding around the warehouse so everything fits
const VPAD = 5;
const VIEW = {
  xMin: WAREHOUSE.xMin - VPAD,
  xMax: WAREHOUSE.xMax + VPAD,
  yMin: WAREHOUSE.yMin - VPAD,
  yMax: WAREHOUSE.yMax + VPAD,
};

// Interior rack shelving (from controller spatial calibration)
// West rack: solid collision block against the west long wall
const RACKS = [
  { label:"West Rack", xMin:-30.27, xMax:-27.02, yMin:-12.06, yMax:36.38 },
];

// Structural columns — 0.60 m × 0.60 m pillars (full height Z).
// 4 column rows at known X positions (from controller obstacle data).
// Columns span Y from -27.80 (south end) to +45.12 (north end).
// Individual pillar Y positions estimated from even spacing within that range.
const COLUMN_ROWS_X = [-27.16, -4.37, 8.41, 26.52];
const COLUMN_Y_MIN  = -27.80;
const COLUMN_Y_MAX  =  45.12;
const COLUMN_SPACING = 14.5;   // approximate Y spacing between pillars
const COLUMN_SIZE    =  0.60;  // metres each side

// Build individual column positions
const COLUMNS = [];
COLUMN_ROWS_X.forEach(cx => {
  for (let y = COLUMN_Y_MIN; y <= COLUMN_Y_MAX + 0.1; y += COLUMN_SPACING) {
    COLUMNS.push({ x: cx, y: y, w: COLUMN_SIZE, h: COLUMN_SIZE });
  }
});

// /World/Cube .. /World/Cube_06 — 1 m³ obstacle mesh cubes (from USD)
const CUBES = [
  { label:"Cube",    x:-15.01, y:-23.87, size:1.0 },
  { label:"Cube_01", x:-15.01, y:-26.38, size:1.0 },
  { label:"Cube_02", x: -6.80, y:-26.54, size:1.0 },
  { label:"Cube_03", x: -6.80, y:-27.56, size:1.0 },
  { label:"Cube_04", x:-14.14, y:-29.98, size:1.0 },
  { label:"Cube_06", x: -1.24, y:-33.05, size:1.0 },
];

// Forklift dimensions (metres) — from USD ForkliftB bbox
const FL_WIDTH  = 3.03;   // X extent
const FL_LENGTH = 4.00;   // Y extent (front-to-back)

// ═══════════════════════════════════════════════════════════════════════
// RENDERING
// ═══════════════════════════════════════════════════════════════════════

const canvas = document.getElementById("map");
const ctx    = canvas.getContext("2d");
const trail  = [];
const TRAIL_MAX = 800;

// ── Uniform-scale coordinate transform ───────────────────────────────
// Uses a single px-per-metre scale so X and Y are never distorted.
// The map is centred in the canvas; unused margins stay as background.
let _scale = 1, _ox = 0, _oy = 0;  // computed each frame in resizeCanvas

function computeTransform() {
  const W = canvas.width, H = canvas.height;
  const spanX = VIEW.xMax - VIEW.xMin;
  const spanY = VIEW.yMax - VIEW.yMin;
  _scale = Math.min(W / spanX, H / spanY);       // uniform px/m
  _ox = (W - spanX * _scale) / 2;                 // centre horizontally
  _oy = (H - spanY * _scale) / 2;                 // centre vertically
}

function toCanvas(wx, wy) {
  return [
    _ox + (wx - VIEW.xMin) * _scale,
    _oy + (VIEW.yMax - wy) * _scale,              // Y flipped: north = up
  ];
}
function worldToPixel(metres) {
  return metres * _scale;
}

// Draw a world-space axis-aligned rect
function drawRect(xMin, yMin, xMax, yMax, fill, stroke, lw) {
  const [x1,y1] = toCanvas(xMin, yMax);  // top-left in canvas
  const [x2,y2] = toCanvas(xMax, yMin);  // bottom-right in canvas
  if (fill) { ctx.fillStyle = fill; ctx.fillRect(x1,y1,x2-x1,y2-y1); }
  if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lw||1; ctx.strokeRect(x1,y1,x2-x1,y2-y1); }
}

// ── Static scene layer (redrawn every frame for simplicity) ──────────

function drawScene() {
  const W = canvas.width, H = canvas.height;
  const sc = worldToPixel(1);  // pixels per metre

  // Sky / outside
  ctx.fillStyle = "#0f1219";
  ctx.fillRect(0, 0, W, H);

  // Warehouse floor
  drawRect(WAREHOUSE.xMin, WAREHOUSE.yMin, WAREHOUSE.xMax, WAREHOUSE.yMax, "#1e2433", null);

  // Grid lines (every 10 m)
  ctx.strokeStyle = "#252d40"; ctx.lineWidth = 1;
  for (let x = Math.ceil(VIEW.xMin/10)*10; x <= VIEW.xMax; x += 10) {
    const [cx] = toCanvas(x, 0);
    ctx.beginPath(); ctx.moveTo(cx,0); ctx.lineTo(cx,H); ctx.stroke();
  }
  for (let y = Math.ceil(VIEW.yMin/10)*10; y <= VIEW.yMax; y += 10) {
    const [,cy] = toCanvas(0, y);
    ctx.beginPath(); ctx.moveTo(0,cy); ctx.lineTo(W,cy); ctx.stroke();
  }

  // Axis labels
  ctx.fillStyle = "#3a4260"; ctx.font = "10px system-ui"; ctx.textBaseline = "alphabetic";
  ctx.textAlign = "center";
  for (let x = Math.ceil(VIEW.xMin/10)*10; x <= VIEW.xMax; x += 10) {
    const [cx,cy] = toCanvas(x, VIEW.yMin);
    ctx.fillText(x+"m", cx, cy - 4);
  }
  ctx.textAlign = "right";
  for (let y = Math.ceil(VIEW.yMin/10)*10; y <= VIEW.yMax; y += 10) {
    const [cx,cy] = toCanvas(VIEW.xMin, y);
    ctx.fillText(y+"m", cx+22, cy+4);
  }

  // Warehouse walls (thick outline)
  const wt = WAREHOUSE.wallThickness;
  ctx.fillStyle = "#3a4260";
  // bottom wall
  drawRect(WAREHOUSE.xMin, WAREHOUSE.yMin, WAREHOUSE.xMax, WAREHOUSE.yMin+wt, "#3a4260", null);
  // top wall
  drawRect(WAREHOUSE.xMin, WAREHOUSE.yMax-wt, WAREHOUSE.xMax, WAREHOUSE.yMax, "#3a4260", null);
  // left wall
  drawRect(WAREHOUSE.xMin, WAREHOUSE.yMin, WAREHOUSE.xMin+wt, WAREHOUSE.yMax, "#3a4260", null);
  // right wall
  drawRect(WAREHOUSE.xMax-wt, WAREHOUSE.yMin, WAREHOUSE.xMax, WAREHOUSE.yMax, "#3a4260", null);

  // Rack shelving
  RACKS.forEach(r => {
    drawRect(r.xMin, r.yMin, r.xMax, r.yMax, "#3d2810", "#5c3d1a", 1.5);
    // Hatching for shelves (horizontal lines every 4m)
    ctx.strokeStyle = "#5c3d1a55"; ctx.lineWidth = 1;
    for (let y = r.yMin+4; y < r.yMax; y += 4) {
      const [lx,ly] = toCanvas(r.xMin, y);
      const [rx]    = toCanvas(r.xMax, y);
      ctx.beginPath(); ctx.moveTo(lx,ly); ctx.lineTo(rx,ly); ctx.stroke();
    }
    // Label
    const [lx,ly] = toCanvas((r.xMin+r.xMax)/2, (r.yMin+r.yMax)/2);
    ctx.save();
    ctx.translate(lx,ly); ctx.rotate(-Math.PI/2);
    ctx.fillStyle = "#8b6030"; ctx.font = "bold 11px system-ui";
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText(r.label, 0, 0);
    ctx.restore();
  });

  // Columns
  COLUMNS.forEach(c => {
    const pw = Math.max(worldToPixel(c.w), 4);
    const ph = Math.max(worldToPixel(c.h), 4);
    const [cx,cy] = toCanvas(c.x, c.y);
    ctx.fillStyle = "#6b7a99";
    ctx.fillRect(cx-pw/2, cy-ph/2, pw, ph);
    ctx.strokeStyle = "#8b9ab9"; ctx.lineWidth = 1;
    ctx.strokeRect(cx-pw/2, cy-ph/2, pw, ph);
  });

  // Obstacle cubes
  CUBES.forEach(c => {
    const ps = Math.max(worldToPixel(c.size), 4);
    const [cx,cy] = toCanvas(c.x, c.y);
    ctx.fillStyle = "#c0505088";
    ctx.fillRect(cx-ps/2, cy-ps/2, ps, ps);
    ctx.strokeStyle = "#e06060"; ctx.lineWidth = 1.5;
    ctx.strokeRect(cx-ps/2, cy-ps/2, ps, ps);
    // Label
    ctx.fillStyle = "#e8a0a0"; ctx.font = "9px system-ui";
    ctx.textAlign = "center"; ctx.textBaseline = "bottom";
    ctx.fillText(c.label, cx, cy - ps/2 - 2);
  });

  // Scale bar (bottom-right of warehouse floor area)
  const barM = 10;  // 10 metre scale bar
  const barPx = worldToPixel(barM);
  const bx = W - 30 - barPx;
  const by = H - 20;
  ctx.strokeStyle = "#7a8299"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(bx,by); ctx.lineTo(bx+barPx,by); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(bx,by-4); ctx.lineTo(bx,by+4); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(bx+barPx,by-4); ctx.lineTo(bx+barPx,by+4); ctx.stroke();
  ctx.fillStyle = "#7a8299"; ctx.font = "11px system-ui";
  ctx.textAlign = "center"; ctx.textBaseline = "bottom";
  ctx.fillText(barM+" m", bx+barPx/2, by-6);
}

// ── Dynamic overlay ──────────────────────────────────────────────────

function drawDynamic(data) {
  const sc = worldToPixel(1);
  const wps = data.waypoints || [];

  // Patrol path dashed loop
  if (wps.length > 1) {
    ctx.strokeStyle = "#2e4a60"; ctx.lineWidth = 2; ctx.setLineDash([6,4]);
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

  // Forklift — to-scale rectangle + heading arrow
  if (data.x == null || data.y == null) return;
  const [fx, fy] = toCanvas(data.x, data.y);
  const hdgRad   = (data.heading * Math.PI) / 180;
  const bW = FL_WIDTH  * sc;
  const bL = FL_LENGTH * sc;

  ctx.save();
  ctx.translate(fx, fy);
  // body +X = cab/counterweight end; body -X = forks/tines end (driving direction).
  // ctx.rotate(-hdgRad) → local +X = body +X (cab), local -X = body -X (forks).
  // Canvas Y is flipped (south = canvas +Y), so negating hdgRad is the only correction needed.
  ctx.rotate(-hdgRad);

  // Shadow
  ctx.fillStyle = "rgba(0,0,0,0.35)";
  ctx.fillRect(-bL/2+2, -bW/2+2, bL, bW);

  // Body
  ctx.fillStyle   = "#c8a832";
  ctx.strokeStyle = "#f5c842"; ctx.lineWidth = 2;
  ctx.beginPath();
  if (ctx.roundRect) ctx.roundRect(-bL/2, -bW/2, bL, bW, 3);
  else ctx.rect(-bL/2, -bW/2, bL, bW);
  ctx.fill(); ctx.stroke();

  // Cab / counterweight area (rear 30%) — body +X end = local +X = right side
  ctx.fillStyle = "#a08520";
  ctx.fillRect(bL/2 - bL*0.30, -bW/2, bL*0.30, bW);

  // Fork tines — two thin rects extending from forks end (body -X = local -X = left side)
  const tineL = bL * 0.55, tineW = bW * 0.11, tineGap = bW * 0.22;
  ctx.fillStyle = "#8b9ab0"; ctx.strokeStyle = "#aabbd0"; ctx.lineWidth = 1;
  [-tineGap, tineGap].forEach(offset => {
    ctx.beginPath();
    ctx.rect(-bL/2 - tineL, offset - tineW/2, tineL, tineW);
    ctx.fill(); ctx.stroke();
  });

  // Direction arrow — points forks-forward (local -X = body -X = driving direction)
  ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(-bL/2 - tineL - 5, 0); ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(-bL/2 - tineL - 5, 0); ctx.lineTo(-bL/2 - tineL + 1, -4);
  ctx.moveTo(-bL/2 - tineL - 5, 0); ctx.lineTo(-bL/2 - tineL + 1, +4);
  ctx.stroke();

  // Label
  ctx.fillStyle = "#fff"; ctx.font = "bold 9px system-ui";
  ctx.textAlign = "center"; ctx.textBaseline = "middle";
  ctx.fillText("FL", bL*0.10, 0);  // offset toward cab so tines don't overlap

  ctx.restore();
}

// ── Sidebar updates ──────────────────────────────────────────────────

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

// ── Main loop ────────────────────────────────────────────────────────

let lastFrame = -1, staleCount = 0;

function resizeCanvas() {
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width  = Math.floor(rect.width)  || 600;
  canvas.height = Math.floor(rect.height) - 36 || 400;
  computeTransform();
}

async function refresh() {
  try {
    const res  = await fetch("/api/state", { cache:"no-store" });
    const data = await res.json();
    resizeCanvas();
    if (data.x != null) { trail.push([data.x, data.y]); if (trail.length > TRAIL_MAX) trail.shift(); }
    drawScene();
    drawDynamic(data);
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
