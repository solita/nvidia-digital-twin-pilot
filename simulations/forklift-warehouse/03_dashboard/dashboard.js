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
    { label: "West Rack", xMin: -30.27, xMax: -27.02, yMin: -12.06, yMax: 36.38 },
];

// Structural columns — 0.60 m × 0.60 m pillars (full height Z).
// 4 column rows at known X positions (from controller obstacle data).
// Columns span Y from -27.80 (south end) to +45.12 (north end).
// Individual pillar Y positions estimated from even spacing within that range.
const COLUMN_ROWS_X = [-27.16, -4.37, 8.41, 26.52];
const COLUMN_Y_MIN = -27.80;
const COLUMN_Y_MAX = 45.12;
const COLUMN_SPACING = 14.5;   // approximate Y spacing between pillars
const COLUMN_SIZE = 0.60;  // metres each side

// Build individual column positions
const COLUMNS = [];
COLUMN_ROWS_X.forEach(cx => {
    for (let y = COLUMN_Y_MIN; y <= COLUMN_Y_MAX + 0.1; y += COLUMN_SPACING) {
        COLUMNS.push({ x: cx, y: y, w: COLUMN_SIZE, h: COLUMN_SIZE });
    }
});

// /World/Cube .. /World/Cube_06 — 1 m³ obstacle mesh cubes (from USD)
const CUBES = [
    { label: "Cube", x: -15.01, y: -23.87, size: 1.0 },
    { label: "Cube_01", x: -15.01, y: -26.38, size: 1.0 },
    { label: "Cube_02", x: -6.80, y: -26.54, size: 1.0 },
    { label: "Cube_03", x: -6.80, y: -27.56, size: 1.0 },
    { label: "Cube_04", x: -14.14, y: -29.98, size: 1.0 },
    { label: "Cube_06", x: -1.24, y: -33.05, size: 1.0 },
];

// /World/Obstacles — spawned by spawn_path_obstacles.py
// Traffic cones (≈0.75 m tall, 0.25 m radius), cardboard boxes (≈0.4–0.6 m), and stacks (4× scaled composite: pallet + palette + KLT bins)
const PATH_OBSTACLES = [
    { kind: "cone", x: 0.0, y: -24.8 },
    { kind: "box", x: 6.0, y: -27.2 },
    { kind: "cone", x: 12.0, y: -24.6 },
    { kind: "stack", x: 18.5, y: -5.0 },
    { kind: "stack", x: 18.2, y: 30.0 },
    { kind: "cone", x: 15.5, y: 15.0 },
    { kind: "box", x: 18.2, y: 30.0 },
    { kind: "cone", x: 10.0, y: 46.5 },
    { kind: "box", x: 0.0, y: 49.5 },
    { kind: "cone", x: -12.0, y: 46.4 },
    { kind: "stack", x: -22.5, y: 30.0 },
    { kind: "box", x: -22.5, y: 30.0 },
    { kind: "cone", x: -25.5, y: 10.0 },
    { kind: "box", x: -22.8, y: -10.0 },
    { kind: "cone", x: -17.0, y: -22.0 },
    { kind: "box", x: -12.0, y: -20.0 },
];

// Forklift dimensions (metres) — from USD ForkliftB bbox
const FL_WIDTH = 3.03;   // X extent
const FL_LENGTH = 4.00;   // Y extent (front-to-back)

// ═══════════════════════════════════════════════════════════════════════
// RENDERING
// ═══════════════════════════════════════════════════════════════════════

const canvas = document.getElementById("map");
const ctx = canvas.getContext("2d");
const trail = [];
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
    const [x1, y1] = toCanvas(xMin, yMax);  // top-left in canvas
    const [x2, y2] = toCanvas(xMax, yMin);  // bottom-right in canvas
    if (fill) { ctx.fillStyle = fill; ctx.fillRect(x1, y1, x2 - x1, y2 - y1); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lw || 1; ctx.strokeRect(x1, y1, x2 - x1, y2 - y1); }
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
    for (let x = Math.ceil(VIEW.xMin / 10) * 10; x <= VIEW.xMax; x += 10) {
        const [cx] = toCanvas(x, 0);
        ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, H); ctx.stroke();
    }
    for (let y = Math.ceil(VIEW.yMin / 10) * 10; y <= VIEW.yMax; y += 10) {
        const [, cy] = toCanvas(0, y);
        ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(W, cy); ctx.stroke();
    }

    // Axis labels
    ctx.fillStyle = "#3a4260"; ctx.font = "10px system-ui"; ctx.textBaseline = "alphabetic";
    ctx.textAlign = "center";
    for (let x = Math.ceil(VIEW.xMin / 10) * 10; x <= VIEW.xMax; x += 10) {
        const [cx, cy] = toCanvas(x, VIEW.yMin);
        ctx.fillText(x + "m", cx, cy - 4);
    }
    ctx.textAlign = "right";
    for (let y = Math.ceil(VIEW.yMin / 10) * 10; y <= VIEW.yMax; y += 10) {
        const [cx, cy] = toCanvas(VIEW.xMin, y);
        ctx.fillText(y + "m", cx + 22, cy + 4);
    }

    // Warehouse walls (thick outline)
    const wt = WAREHOUSE.wallThickness;
    ctx.fillStyle = "#3a4260";
    // bottom wall
    drawRect(WAREHOUSE.xMin, WAREHOUSE.yMin, WAREHOUSE.xMax, WAREHOUSE.yMin + wt, "#3a4260", null);
    // top wall
    drawRect(WAREHOUSE.xMin, WAREHOUSE.yMax - wt, WAREHOUSE.xMax, WAREHOUSE.yMax, "#3a4260", null);
    // left wall
    drawRect(WAREHOUSE.xMin, WAREHOUSE.yMin, WAREHOUSE.xMin + wt, WAREHOUSE.yMax, "#3a4260", null);
    // right wall
    drawRect(WAREHOUSE.xMax - wt, WAREHOUSE.yMin, WAREHOUSE.xMax, WAREHOUSE.yMax, "#3a4260", null);

    // Rack shelving
    RACKS.forEach(r => {
        drawRect(r.xMin, r.yMin, r.xMax, r.yMax, "#3d2810", "#5c3d1a", 1.5);
        // Hatching for shelves (horizontal lines every 4m)
        ctx.strokeStyle = "#5c3d1a55"; ctx.lineWidth = 1;
        for (let y = r.yMin + 4; y < r.yMax; y += 4) {
            const [lx, ly] = toCanvas(r.xMin, y);
            const [rx] = toCanvas(r.xMax, y);
            ctx.beginPath(); ctx.moveTo(lx, ly); ctx.lineTo(rx, ly); ctx.stroke();
        }
        // Label
        const [lx, ly] = toCanvas((r.xMin + r.xMax) / 2, (r.yMin + r.yMax) / 2);
        ctx.save();
        ctx.translate(lx, ly); ctx.rotate(-Math.PI / 2);
        ctx.fillStyle = "#8b6030"; ctx.font = "bold 11px system-ui";
        ctx.textAlign = "center"; ctx.textBaseline = "middle";
        ctx.fillText(r.label, 0, 0);
        ctx.restore();
    });

    // Columns
    COLUMNS.forEach(c => {
        const pw = Math.max(worldToPixel(c.w), 4);
        const ph = Math.max(worldToPixel(c.h), 4);
        const [cx, cy] = toCanvas(c.x, c.y);
        ctx.fillStyle = "#6b7a99";
        ctx.fillRect(cx - pw / 2, cy - ph / 2, pw, ph);
        ctx.strokeStyle = "#8b9ab9"; ctx.lineWidth = 1;
        ctx.strokeRect(cx - pw / 2, cy - ph / 2, pw, ph);
    });

    // Obstacle cubes
    CUBES.forEach(c => {
        const ps = Math.max(worldToPixel(c.size), 4);
        const [cx, cy] = toCanvas(c.x, c.y);
        ctx.fillStyle = "#c0505088";
        ctx.fillRect(cx - ps / 2, cy - ps / 2, ps, ps);
        ctx.strokeStyle = "#e06060"; ctx.lineWidth = 1.5;
        ctx.strokeRect(cx - ps / 2, cy - ps / 2, ps, ps);
        // Label
        ctx.fillStyle = "#e8a0a0"; ctx.font = "9px system-ui";
        ctx.textAlign = "center"; ctx.textBaseline = "bottom";
        ctx.fillText(c.label, cx, cy - ps / 2 - 2);
    });

    // Path obstacles (cones, boxes & stacks from spawn_path_obstacles.py)
    PATH_OBSTACLES.forEach(o => {
        const [cx, cy] = toCanvas(o.x, o.y);
        if (o.kind === "cone") {
            // Orange triangle (top-down view of a traffic cone)
            const r = Math.max(worldToPixel(0.35), 5);
            ctx.beginPath();
            ctx.moveTo(cx, cy - r);
            ctx.lineTo(cx - r * 0.866, cy + r * 0.5);
            ctx.lineTo(cx + r * 0.866, cy + r * 0.5);
            ctx.closePath();
            ctx.fillStyle = "rgba(255,140,0,0.7)";
            ctx.fill();
            ctx.strokeStyle = "#ff8c00";
            ctx.lineWidth = 1.5;
            ctx.stroke();
        } else if (o.kind === "box") {
            // Brown rectangle (top-down view of a cardboard box)
            const ps = Math.max(worldToPixel(0.5), 5);
            ctx.fillStyle = "rgba(140,90,40,0.7)";
            ctx.fillRect(cx - ps / 2, cy - ps / 2, ps, ps);
            ctx.strokeStyle = "#a06828";
            ctx.lineWidth = 1.5;
            ctx.strokeRect(cx - ps / 2, cy - ps / 2, ps, ps);
        } else if (o.kind === "stack") {
            // Green-gray square (top-down view of a 4× scaled composite stack: pallet + palette + KLT bins)
            const ps = Math.max(worldToPixel(1.0), 8);
            ctx.fillStyle = "rgba(90,140,105,0.6)";
            ctx.fillRect(cx - ps / 2, cy - ps / 2, ps, ps);
            ctx.strokeStyle = "#6b9e7f";
            ctx.lineWidth = 2;
            ctx.strokeRect(cx - ps / 2, cy - ps / 2, ps, ps);
        }
    });

    // Scale bar (bottom-right of warehouse floor area)
    const barM = 10;  // 10 metre scale bar
    const barPx = worldToPixel(barM);
    const bx = W - 30 - barPx;
    const by = H - 20;
    ctx.strokeStyle = "#7a8299"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(bx + barPx, by); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(bx, by - 4); ctx.lineTo(bx, by + 4); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(bx + barPx, by - 4); ctx.lineTo(bx + barPx, by + 4); ctx.stroke();
    ctx.fillStyle = "#7a8299"; ctx.font = "11px system-ui";
    ctx.textAlign = "center"; ctx.textBaseline = "bottom";
    ctx.fillText(barM + " m", bx + barPx / 2, by - 6);
}

// ── Dynamic overlay ──────────────────────────────────────────────────

function drawDynamic(data) {
    const sc = worldToPixel(1);
    const wps = data.waypoints || [];

    // Patrol path dashed loop
    if (wps.length > 1) {
        ctx.strokeStyle = "#2e4a60"; ctx.lineWidth = 2; ctx.setLineDash([6, 4]);
        ctx.beginPath();
        wps.forEach(([wx, wy], i) => {
            const [px, py] = toCanvas(wx, wy);
            i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
        });
        ctx.closePath(); ctx.stroke(); ctx.setLineDash([]);
    }

    // Trail
    if (trail.length > 1) {
        ctx.strokeStyle = "#4caf7d55"; ctx.lineWidth = 2;
        ctx.beginPath();
        trail.forEach(([tx, ty], i) => {
            const [cx, cy] = toCanvas(tx, ty);
            i === 0 ? ctx.moveTo(cx, cy) : ctx.lineTo(cx, cy);
        });
        ctx.stroke();
    }

    // Waypoint markers
    wps.forEach(([wx, wy], idx) => {
        const [cx, cy] = toCanvas(wx, wy);
        const active = idx === data.wp;
        ctx.beginPath();
        ctx.arc(cx, cy, active ? 8 : 5, 0, Math.PI * 2);
        ctx.fillStyle = active ? "#4a9eff" : "#2e3a50";
        ctx.strokeStyle = active ? "#4a9eff" : "#3e4f6a";
        ctx.lineWidth = 2; ctx.fill(); ctx.stroke();
        ctx.fillStyle = active ? "#fff" : "#7a8299";
        ctx.font = `bold ${active ? 11 : 9}px system-ui`;
        ctx.textAlign = "center"; ctx.textBaseline = "middle";
        ctx.fillText(idx, cx, cy);
    });

    // Forklift — to-scale rectangle + heading arrow
    if (data.x == null || data.y == null) return;
    const [fx, fy] = toCanvas(data.x, data.y);
    const hdgRad = (data.heading * Math.PI) / 180;
    const bW = FL_WIDTH * sc;
    const bL = FL_LENGTH * sc;

    ctx.save();
    ctx.translate(fx, fy);
    // body +X = cab/counterweight end; body -X = forks/tines end (driving direction).
    // ctx.rotate(-hdgRad) → local +X = body +X (cab), local -X = body -X (forks).
    // Canvas Y is flipped (south = canvas +Y), so negating hdgRad is the only correction needed.
    ctx.rotate(-hdgRad);

    // LIDAR pie chart — 9 sectors showing obstacle detection around forklift
    {
        const sl = data.lidar_slices || [false, false, false, false, false, false, false, false, false];
        const lrF = worldToPixel(8);  // 8 m front cone range, to scale
        const lrSB = worldToPixel(4);  // 4 m side/back range (half), to scale
        const D = Math.PI / 180;
        const clr = "rgba(76,175,125,0.15)";
        const hitC = "rgba(224,85,85,0.22)";
        const clrS = "rgba(76,175,125,0.30)";
        const hitS = "rgba(224,85,85,0.45)";
        const pie = [
            // Front 3 slices (full 8 m range)
            { s: Math.PI - 20 * D, e: Math.PI - 6.67 * D, h: sl[0], r: lrF },  // front-left
            { s: Math.PI - 6.67 * D, e: Math.PI + 6.67 * D, h: sl[1], r: lrF },  // front-center
            { s: Math.PI + 6.67 * D, e: Math.PI + 20 * D, h: sl[2], r: lrF },  // front-right
            // Right 2 slices (half range 4 m)
            { s: Math.PI + 20 * D, e: Math.PI + 73.33 * D, h: sl[3], r: lrSB },  // right-front
            { s: Math.PI + 73.33 * D, e: Math.PI + 126.67 * D, h: sl[4], r: lrSB },  // right-back
            // Back 2 slices (half range 4 m)
            { s: Math.PI + 126.67 * D, e: Math.PI + 180 * D, h: sl[5], r: lrSB },  // back-right
            { s: Math.PI - 180 * D, e: Math.PI - 126.67 * D, h: sl[6], r: lrSB },  // back-left
            // Left 2 slices (half range 4 m)
            { s: Math.PI - 126.67 * D, e: Math.PI - 73.33 * D, h: sl[7], r: lrSB },  // left-back
            { s: Math.PI - 73.33 * D, e: Math.PI - 20 * D, h: sl[8], r: lrSB },  // left-front
        ];
        pie.forEach(p => {
            ctx.beginPath(); ctx.moveTo(0, 0);
            ctx.arc(0, 0, p.r, p.s, p.e); ctx.closePath();
            ctx.fillStyle = p.h ? hitC : clr; ctx.fill();
            ctx.strokeStyle = p.h ? hitS : clrS; ctx.lineWidth = 1; ctx.stroke();
        });
        // Slice divider lines (front-cone boundaries at full range, others at half)
        [[Math.PI - 20 * D, lrF], [Math.PI - 6.67 * D, lrF], [Math.PI + 6.67 * D, lrF], [Math.PI + 20 * D, lrF],
        [Math.PI + 73.33 * D, lrSB], [Math.PI + 126.67 * D, lrSB], [Math.PI + 180 * D, lrSB],
        [Math.PI - 126.67 * D, lrSB], [Math.PI - 73.33 * D, lrSB]].forEach(([a, r]) => {
            ctx.beginPath(); ctx.moveTo(0, 0);
            ctx.lineTo(Math.cos(a) * r, Math.sin(a) * r);
            ctx.strokeStyle = "rgba(200,200,200,0.18)"; ctx.lineWidth = 1; ctx.stroke();
        });
    }

    // Shadow
    ctx.fillStyle = "rgba(0,0,0,0.35)";
    ctx.fillRect(-bL / 2 + 2, -bW / 2 + 2, bL, bW);

    // Body
    ctx.fillStyle = "#c8a832";
    ctx.strokeStyle = "#f5c842"; ctx.lineWidth = 2;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(-bL / 2, -bW / 2, bL, bW, 3);
    else ctx.rect(-bL / 2, -bW / 2, bL, bW);
    ctx.fill(); ctx.stroke();

    // Cab / counterweight area (rear 30%) — body +X end = local +X = right side
    ctx.fillStyle = "#a08520";
    ctx.fillRect(bL / 2 - bL * 0.30, -bW / 2, bL * 0.30, bW);

    // Fork tines — two thin rects extending from forks end (body -X = local -X = left side)
    const tineL = bL * 0.55, tineW = bW * 0.11, tineGap = bW * 0.22;
    ctx.fillStyle = "#8b9ab0"; ctx.strokeStyle = "#aabbd0"; ctx.lineWidth = 1;
    [-tineGap, tineGap].forEach(offset => {
        ctx.beginPath();
        ctx.rect(-bL / 2 - tineL, offset - tineW / 2, tineL, tineW);
        ctx.fill(); ctx.stroke();
    });

    // Direction arrow — points forks-forward (local -X = body -X = driving direction)
    ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(-bL / 2 - tineL - 5, 0); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(-bL / 2 - tineL - 5, 0); ctx.lineTo(-bL / 2 - tineL + 1, -4);
    ctx.moveTo(-bL / 2 - tineL - 5, 0); ctx.lineTo(-bL / 2 - tineL + 1, +4);
    ctx.stroke();

    // Label
    ctx.fillStyle = "#fff"; ctx.font = "bold 9px system-ui";
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText("FL", bL * 0.10, 0);  // offset toward cab so tines don't overlap

    ctx.restore();
}

// ── Tab switching ────────────────────────────────────────────────────

document.querySelectorAll('.sidebar-tab').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.sidebar-tab').forEach(b => b.classList.remove('active'));
        document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));
        btn.classList.add('active');
        document.getElementById(btn.dataset.tab).classList.add('active');
    });
});

// ── Fleet card builder (dynamic, supports N forklifts) ──────────────

const _fleetCards = {};  // keyed by forklift index

function buildFleetCard(idx) {
    const card = document.createElement('div');
    card.className = 'fleet-card';
    card.innerHTML = `
        <div class="fleet-card-header">
            <span class="fleet-card-name">Forklift ${idx + 1}</span>
            <span class="badge CLEAR" data-fld="lidarBadge">CLEAR</span>
        </div>
        <div class="fleet-section">
            <div class="fleet-section-title">Position &amp; Heading</div>
            <div class="fleet-metric-grid">
                <div><div class="lbl">X</div><div class="val" data-fld="x">--</div></div>
                <div><div class="lbl">Y</div><div class="val" data-fld="y">--</div></div>
                <div><div class="lbl">Heading</div><div class="val" data-fld="hdg">--</div></div>
                <div><div class="lbl">Hdg Error</div><div class="val" data-fld="err">--</div></div>
            </div>
        </div>
        <div class="fleet-section">
            <div class="fleet-section-title">Route</div>
            <div class="fleet-metric-grid">
                <div><div class="lbl">Waypoint</div><div class="val" data-fld="wp">--</div></div>
                <div><div class="lbl">Lap</div><div class="val" data-fld="lap">--</div></div>
                <div><div class="lbl">Dist to WP</div><div class="val" data-fld="dist">--</div></div>
                <div><div class="lbl">Frame</div><div class="val" data-fld="frame">--</div></div>
            </div>
        </div>
        <div class="fleet-section">
            <div class="fleet-section-title">LIDAR</div>
            <div class="fleet-metric-grid">
                <div><div class="lbl">Fwd Min</div><div class="val" data-fld="fwd">--</div></div>
                <div><div class="lbl">Repulsion</div><div class="val" data-fld="rep">--</div></div>
            </div>
        </div>
        <div class="fleet-section">
            <div class="fleet-section-title">Speed</div>
            <div class="val" data-fld="speed">--%</div>
            <div class="progress-bar"><div class="fill" data-fld="speedBar" style="width:0%"></div></div>
        </div>
        <div class="fleet-section">
            <div class="fleet-controls">
                <button class="btn btn-stop" data-fld="btnStop">⏹ Stop</button>
                <button class="btn btn-resume" data-fld="btnResume">▶ Resume</button>
            </div>
            <div data-fld="cmdStatus" style="font-size:0.72rem;color:var(--muted);margin-top:6px"></div>
        </div>`;
    // Bind control buttons
    const btnStop = card.querySelector('[data-fld="btnStop"]');
    const btnResume = card.querySelector('[data-fld="btnResume"]');
    btnStop.addEventListener('click', () => sendCmd('pause', idx));
    btnResume.addEventListener('click', () => sendCmd('resume', idx));
    return card;
}

function fld(card, name) {
    return card.querySelector('[data-fld="' + name + '"]');
}

function updateFleetCard(card, data) {
    fld(card, 'x').textContent = data.x != null ? data.x.toFixed(1) + ' m' : '--';
    fld(card, 'y').textContent = data.y != null ? data.y.toFixed(1) + ' m' : '--';
    fld(card, 'hdg').textContent = data.heading != null ? data.heading.toFixed(1) + '°' : '--';
    fld(card, 'err').textContent = data.heading_err != null ? (data.heading_err >= 0 ? '+' : '') + data.heading_err.toFixed(1) + '°' : '--';
    fld(card, 'wp').textContent = data.wp != null ? 'WP ' + data.wp : '--';
    fld(card, 'lap').textContent = data.lap != null ? data.lap : '--';
    fld(card, 'dist').textContent = data.dist_to_wp != null ? data.dist_to_wp.toFixed(1) + ' m' : '--';
    fld(card, 'frame').textContent = data.frame != null ? data.frame : '--';

    const ls = data.lidar_state || 'CLEAR';
    const badge = fld(card, 'lidarBadge');
    badge.textContent = ls;
    badge.className = 'badge ' + ls;

    fld(card, 'fwd').textContent = data.forward_min != null
        ? (data.forward_min >= 9.8 ? 'Clear' : data.forward_min.toFixed(1) + ' m') : '--';
    fld(card, 'rep').textContent = data.repulsion != null
        ? (data.repulsion >= 0 ? '+' : '') + data.repulsion.toFixed(1) + '°' : '--';

    const sp = Math.round((data.speed_frac || 0) * 100);
    fld(card, 'speed').textContent = sp + '%';
    fld(card, 'speedBar').style.width = sp + '%';

    // Button states — use optimistic state during command grace period
    const pending = Date.now() < _cmdPendingUntil;
    const paused = pending ? _simPaused : (data.paused || false);
    if (!pending && data.paused === _simPaused) _cmdPendingUntil = 0;
    const btnStop = fld(card, 'btnStop');
    const btnResume = fld(card, 'btnResume');
    btnStop.disabled = !_controllerAlive || paused;
    btnResume.disabled = !_controllerAlive || !paused;
}

function updateFleetTab(data) {
    // Wrap single forklift into array (future-proof: when data becomes array, iterate directly)
    const fleet = Array.isArray(data) ? data : [data];
    const container = document.getElementById('fleetCards');

    fleet.forEach((fl, idx) => {
        if (!_fleetCards[idx]) {
            _fleetCards[idx] = buildFleetCard(idx);
            container.appendChild(_fleetCards[idx]);
        }
        updateFleetCard(_fleetCards[idx], fl);
    });

    // Update fleet count in General tab
    const numEl = document.getElementById('fleetNum');
    if (numEl) numEl.textContent = fleet.length;
}

// ── Sidebar updates ──────────────────────────────────────────────────

function updateSidebar(data) {
    updateFleetTab(data);
}

// ── Control buttons ─────────────────────────────────────────────────

let _simPaused = false;
let _controllerAlive = false;
let _cmdPendingUntil = 0;

function sendCmd(cmd, fleetIdx) {
    if (!_controllerAlive) return;
    _simPaused = cmd === 'pause';
    _cmdPendingUntil = Date.now() + 3000;  // 3 s grace — don't let polls revert buttons
    // Optimistic update immediately (before fetch resolves)
    if (_fleetCards[fleetIdx]) {
        const card = _fleetCards[fleetIdx];
        fld(card, 'btnStop').disabled = _simPaused;
        fld(card, 'btnResume').disabled = !_simPaused;
    }
    fetch('/api/cmd/' + cmd, { method: 'POST' })
        .then(r => r.text())
        .catch(() => {
            _cmdPendingUntil = 0;  // cancel grace on failure
            if (_fleetCards[fleetIdx]) {
                fld(_fleetCards[fleetIdx], 'cmdStatus').textContent = 'Command failed — is the sim running?';
            }
        });
}

// Poll controller liveness every 3 s and enable/disable buttons
function pollControllerAlive() {
    fetch('/api/controller-alive', { cache: 'no-store' })
        .then(r => r.json())
        .then(d => {
            _controllerAlive = d.alive;
        })
        .catch(() => {
            _controllerAlive = false;
        });
}
pollControllerAlive();
setInterval(pollControllerAlive, 3000);

// ── Main loop ────────────────────────────────────────────────────────

let lastFrame = -1, staleCount = 0;

function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    canvas.width = Math.floor(rect.width) || 600;
    canvas.height = Math.floor(rect.height) || 400;
    computeTransform();
}

function handleData(data) {
    resizeCanvas();
    if (data.x != null) { trail.push([data.x, data.y]); if (trail.length > TRAIL_MAX) trail.shift(); }
    drawScene();
    drawDynamic(data);
    updateSidebar(data);
    if (data.frame !== lastFrame) { staleCount = 0; lastFrame = data.frame; } else staleCount++;
    const tag = document.getElementById("liveTag");
    if (staleCount > 4) { tag.textContent = "STALE"; tag.className = "pill stale"; }
    else { tag.textContent = "LIVE"; tag.className = "pill"; }
    document.getElementById("updated").textContent = "Frame " + (data.frame || 0);
}

// ── WebSocket with auto-reconnect ────────────────────────────────────
let _lastMsgTime = Date.now();

function connectWS() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${proto}://${location.host}/ws`);
    ws.onmessage = (e) => { _lastMsgTime = Date.now(); handleData(JSON.parse(e.data)); };
    ws.onclose = () => { setTimeout(connectWS, 1000); };
    ws.onerror = () => { ws.close(); };
}

// REST poll fallback — runs every 1 s regardless of WS state.
// Keeps the dashboard live even when WS is blocked by the tunnel/proxy.
setInterval(() => {
    fetch("/api/state", { cache: "no-store" })
        .then(r => r.json())
        .then(data => { _lastMsgTime = Date.now(); handleData(data); })
        .catch(() => { });
}, 1000);

// Stale guard: if nothing (WS or REST) arrived for 4 s, mark STALE
setInterval(() => {
    if (Date.now() - _lastMsgTime > 4000) {
        const tag = document.getElementById("liveTag");
        if (tag) { tag.textContent = "STALE"; tag.className = "pill stale"; }
    }
}, 1000);

window.addEventListener("resize", resizeCanvas);
resizeCanvas();

// Draw immediately on load using REST so the map isn't blank before the first WS push
fetch("/api/state", { cache: "no-store" })
    .then(r => r.json())
    .then(handleData)
    .catch(() => { drawScene(); });  // at minimum paint the warehouse if sim not running

connectWS();
