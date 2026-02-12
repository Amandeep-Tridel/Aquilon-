// =====================================================
// AQUILON — USV Obstacle Avoidance Simulation Engine
// Full pMarineViewer-like features + interactive obstacles
// =====================================================

const canvas = document.getElementById('simCanvas');
const ctx = canvas.getContext('2d');

// ============ CONFIGURATION ============
const WAYPOINTS = [
    [20, -20], [420, -20], [420, -60], [20, -60], [20, -100], [420, -100],
    [420, -140], [20, -140], [20, -180], [420, -180], [420, -220], [20, -220],
    [20, -260], [420, -260], [420, -300], [20, -300],
];
const DETECTION_RANGE = 25, SLOW_RANGE = 16, CREEP_RANGE = 10, FLEE_RANGE = 6, MIN_CLEARANCE = 3;
const SPEED_CRUISE = 2.5, SPEED_FAST = 3.5, SPEED_FLEE = 5, SPEED_SLOW = 1.8, SPEED_CREEP = 0.8;
const KP_HEADING = 5, KI_HEADING = 0.05, KD_HEADING = 2, MAX_INTEGRAL = 20, MAX_TURN_RATE = 120;
const KP_SPEED = 3, KD_SPEED = 0.8, MAX_ACCEL = 3, MAX_ACCEL_FLEE = 8, MAX_DECEL = 5;
const SIM_HZ = 60, WPT_CAPTURE_RADIUS = 10;

// ============ UTILITIES ============
const angleDiff = (a, b) => { let d = ((a - b) % 360 + 360) % 360; return d > 180 ? d - 360 : d; };
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const dist = (x1, y1, x2, y2) => Math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2);
const toRad = d => d * Math.PI / 180;
const toDeg = r => r * 180 / Math.PI;
const normAngle = a => ((a % 360) + 360) % 360;

class PIDController {
    constructor(kp, ki, kd, maxI = 100) { this.kp = kp; this.ki = ki; this.kd = kd; this.maxI = maxI; this.i = 0; this.pe = 0; }
    update(e, dt) { this.i = clamp(this.i + e * dt, -this.maxI, this.maxI); const d = dt > 0 ? (e - this.pe) / dt : 0; this.pe = e; return this.kp * e + this.ki * this.i + this.kd * d; }
    reset() { this.i = 0; this.pe = 0; }
}

// ============ DEFAULT OBSTACLES ============
const DEFAULT_OBS = [
    { cx: 120, cy: -40, r: 4, color: '#ef4444' }, { cx: 300, cy: -80, r: 5, color: '#f97316' },
    { cx: 80, cy: -160, r: 3, color: '#ef4444' }, { cx: 350, cy: -250, r: 6, color: '#f97316' },
    { cx: 200, cy: -60, r: 4, color: '#eab308' }, { cx: 380, cy: -140, r: 3, color: '#e2e8f0' },
    { cx: 250, cy: -220, r: 5, color: '#ef4444' }, { cx: 150, cy: -280, r: 4, color: '#f97316' },
];

// ============ STATE ============
let sim = null, running = false, simSpeed = 1, lastTime = 0;
let showDetection = true, showTrail = true, showGrid = true, showPath = true, showScope = true, showHash = true;
let trail = [], events = [], measurements = [];
let mode = 'select'; // select, add, delete, measure
let draggingObs = null, dragOffset = { x: 0, y: 0 };
let measureStart = null, measureEnd = null, measuringActive = false;
let hoveredObs = null, selectedObs = null;
let nextObsId = 8;
let cam = { x: 220, y: -160, zoom: 1.2, tZoom: 1.2 };
let panning = false, panStart = null;
let mouseScreen = { x: 0, y: 0 }, mouseWorld = { x: 0, y: 0 };

// ============ COORDINATE TRANSFORMS ============
function w2s(wx, wy) { const cx = canvas.clientWidth / 2, cy = canvas.clientHeight / 2; return [cx + (wx - cam.x) * cam.zoom, cy - (wy - cam.y) * cam.zoom]; }
function s2w(sx, sy) { const cx = canvas.clientWidth / 2, cy = canvas.clientHeight / 2; return [(sx - cx) / cam.zoom + cam.x, -((sy - cy) / cam.zoom) + cam.y]; }

// ============ SIMULATION INIT ============
function initSim() {
    trail = []; events = []; measurements = []; measureStart = null; measureEnd = null; measuringActive = false;
    selectedObs = null; hoveredObs = null; draggingObs = null;
    const obstacles = DEFAULT_OBS.map((d, i) => ({ id: i, x: d.cx, y: d.cy, r: d.r, color: d.color, vx: 0, vy: 0, name: `OBS ${i}` }));
    nextObsId = obstacles.length;
    sim = {
        x: WAYPOINTS[0][0], y: WAYPOINTS[0][1], heading: 0, speed: 0, tgtSpeed: 0,
        wptIdx: 0, deployed: false, state: 'IDLE', effMin: 999, obstacles,
        hPID: new PIDController(KP_HEADING, KI_HEADING, KD_HEADING, MAX_INTEGRAL),
        sPID: new PIDController(KP_SPEED, 0, KD_SPEED),
        clearTime: 0, tick: 0, elapsed: 0, deployT: 0, done: false,
        totalDist: 0, maxSpeed: 0, stateChanges: 0, closestEver: 999
    };
    addEvent('System initialized', 'info');
    updateUI();
}

function addEvent(msg, type = '') {
    const t = sim ? sim.elapsed.toFixed(1) : '0.0';
    events.unshift({ time: t, msg, type });
    if (events.length > 80) events.length = 80;
}

// ============ OBSTACLE MANAGEMENT ============
function addObstacle(x, y, r = 4, color = '#ef4444') {
    const obs = { id: nextObsId++, x, y, r, color, vx: 0, vy: 0, name: `OBS ${nextObsId - 1}` };
    sim.obstacles.push(obs);
    addEvent(`Added obstacle ${obs.name} at (${x.toFixed(0)},${y.toFixed(0)})`, 'info');
    updateUI();
    return obs;
}
function removeObstacle(id) {
    const idx = sim.obstacles.findIndex(o => o.id === id);
    if (idx >= 0) {
        const name = sim.obstacles[idx].name;
        sim.obstacles.splice(idx, 1);
        if (selectedObs && selectedObs.id === id) selectedObs = null;
        addEvent(`Removed ${name}`, 'warn');
        updateUI();
    }
}
function findObsAtScreen(sx, sy) {
    const [wx, wy] = s2w(sx, sy);
    for (let i = sim.obstacles.length - 1; i >= 0; i--) {
        const o = sim.obstacles[i];
        if (dist(wx, wy, o.x, o.y) <= o.r + 3 / cam.zoom) return o;
    }
    return null;
}

// ============ THREAT ANALYSIS ============
function getThreats() {
    const uVx = sim.speed * Math.sin(toRad(sim.heading)), uVy = sim.speed * Math.cos(toRad(sim.heading));
    return sim.obstacles.map(obs => {
        const dx = obs.x - sim.x, dy = obs.y - sim.y;
        let d = Math.sqrt(dx * dx + dy * dy) - obs.r;
        const rvx = obs.vx - uVx, rvy = obs.vy - uVy, rs = rvx * rvx + rvy * rvy;
        let tCpa = 0, cpaDist = d;
        if (rs > 0.01) { tCpa = clamp(-(dx * rvx + dy * rvy) / rs, 0, 8); const px = dx + rvx * tCpa, py = dy + rvy * tCpa; cpaDist = Math.sqrt(px * px + py * py) - obs.r; }
        const closing = d < DETECTION_RANGE ? (dx * obs.vx + dy * obs.vy) < 0 : false;
        return { id: obs.id, obs, dist: Math.max(0, d), cpaDist: Math.max(0, cpaDist), tCpa, closing, dx, dy };
    });
}

function computeAvoidHdg(threats, tx, ty) {
    const dx = tx - sim.x, dy = ty - sim.y, wd = Math.sqrt(dx * dx + dy * dy);
    let rx = 0, ry = 0, emg = false;
    for (const t of threats) {
        const ed = Math.min(t.dist, t.cpaDist), ad = t.dist;
        if (ed < DETECTION_RANGE) {
            const n = Math.sqrt(t.dx ** 2 + t.dy ** 2); if (n < 0.1) continue;
            let s;
            if (ad < MIN_CLEARANCE) { emg = true; s = 80; }
            else if (ed < FLEE_RANGE) s = 40 * FLEE_RANGE / Math.max(ed, 0.5);
            else if (ed < CREEP_RANGE) { const f = 1 - (ed - FLEE_RANGE) / (CREEP_RANGE - FLEE_RANGE); s = 8 + 20 * f * f; }
            else if (ed < SLOW_RANGE) { const f = 1 - (ed - CREEP_RANGE) / (SLOW_RANGE - CREEP_RANGE); s = 2 + 6 * f; }
            else { const f = 1 - (ed - SLOW_RANGE) / (DETECTION_RANGE - SLOW_RANGE); s = 0.5 + 1.5 * f; }
            if (t.closing) s *= 2.5;
            rx -= (t.dx / n) * s; ry -= (t.dy / n) * s;
        }
    }
    if (emg && Math.abs(rx) + Math.abs(ry) > 0.01) return normAngle(toDeg(Math.atan2(rx, ry)));
    let wnx = 0, wny = 0;
    if (wd > 0.1) { wnx = dx / wd * 10; wny = dy / wd * 10; } else { wnx = dx; wny = dy; }
    return normAngle(toDeg(Math.atan2(wnx + rx, wny + ry)));
}

function computeTgtSpeed(threats) {
    let md = 999, cc = false, ac = false;
    for (const t of threats) { const e = Math.min(t.dist, t.cpaDist); if (e < md) { md = e; cc = t.closing; } if (t.closing && t.dist < SLOW_RANGE) ac = true; }
    sim.effMin = md; if (md < sim.closestEver) sim.closestEver = md;
    const h = 1.5, fe = sim.state === 'FLEE' ? FLEE_RANGE + h : FLEE_RANGE, ce = sim.state === 'CREEP' ? CREEP_RANGE + h : CREEP_RANGE, se = sim.state === 'SLOW' ? SLOW_RANGE + h : SLOW_RANGE;
    if (md < fe) return [SPEED_FLEE, 'FLEE'];
    if (md < ce) { if (cc) { const u = clamp(1 - (md - FLEE_RANGE) / (CREEP_RANGE - FLEE_RANGE), 0, 1); return [SPEED_CREEP + (SPEED_FAST - SPEED_CREEP) * u, 'EVADE']; } return [SPEED_CREEP, 'CREEP']; }
    if (md < se) { let s = SPEED_CREEP + (SPEED_SLOW - SPEED_CREEP) * ((md - CREEP_RANGE) / (SLOW_RANGE - CREEP_RANGE)); if (ac) s *= 0.7; return [s, 'SLOW']; }
    if (md < DETECTION_RANGE) return [SPEED_SLOW + (SPEED_CRUISE - SPEED_SLOW) * ((md - SLOW_RANGE) / (DETECTION_RANGE - SLOW_RANGE)), 'CRUISE'];
    return [SPEED_CRUISE, 'CRUISE'];
}

// ============ SIMULATION TICK ============
function simTick(dt) {
    if (!sim || sim.done) return;
    sim.elapsed += dt; sim.tick++;
    if (!sim.deployed) { sim.deployT += dt; if (sim.deployT >= 1) { sim.deployed = true; sim.wptIdx = 1; sim.state = 'CRUISE'; addEvent('DEPLOYED — avoidance active', 'success'); } return; }
    if (sim.wptIdx >= WAYPOINTS.length) { if (!sim.done) { sim.done = true; sim.speed = 0; addEvent('Mission complete!', 'success'); } return; }

    const [tx, ty] = WAYPOINTS[sim.wptIdx], ddx = tx - sim.x, ddy = ty - sim.y, dw = Math.sqrt(ddx * ddx + ddy * ddy);
    const threats = getThreats(); let [ts, ns] = computeTgtSpeed(threats);

    if (dw < WPT_CAPTURE_RADIUS) {
        sim.x = tx; sim.y = ty; sim.wptIdx++; sim.hPID.reset();
        if (sim.wptIdx >= WAYPOINTS.length) { addEvent('Mission complete!', 'success'); sim.done = true; }
        else { const [nx, ny] = WAYPOINTS[sim.wptIdx]; addEvent(`WPT ${sim.wptIdx}/${WAYPOINTS.length}: → (${nx},${ny})`, 'info'); } return;
    }

    if (sim.effMin > DETECTION_RANGE * 1.1) sim.clearTime += dt; else sim.clearTime = 0;
    if (sim.clearTime > 5 && ns === 'CRUISE') { ts = SPEED_FAST; ns = 'FAST'; }
    if (ns !== sim.state) {
        const o = sim.state; sim.state = ns; sim.stateChanges++;
        const tp = ns === 'FLEE' ? 'danger' : ns === 'EVADE' ? 'warn' : ns === 'FAST' ? 'success' : 'info';
        addEvent(`${o} → ${ns}  spd=${ts.toFixed(1)} d=${sim.effMin.toFixed(0)}m`, tp);
    }
    sim.tgtSpeed = ts;

    let dh;
    if (dw < 20) dh = normAngle(toDeg(Math.atan2(ddx, ddy)));
    else dh = computeAvoidHdg(threats, tx, ty);
    const he = angleDiff(dh, sim.heading), tr = sim.state === 'FLEE' ? MAX_TURN_RATE * 2 : MAX_TURN_RATE;
    const po = sim.hPID.update(he, dt), turn = clamp(po * dt, -tr * dt, tr * dt);
    sim.heading = normAngle(sim.heading + turn);

    const se = ts - sim.speed, sc = sim.sPID.update(se, dt);
    const fl = sim.state === 'FLEE' || sim.state === 'EVADE';
    const ma = fl ? MAX_ACCEL_FLEE * dt : MAX_ACCEL * dt, ms = fl ? SPEED_FLEE : SPEED_FAST;
    sim.speed = clamp(sim.speed + (sc > 0 ? Math.min(sc, ma) : Math.max(sc, -MAX_DECEL * dt)), 0, ms);
    if (sim.speed > sim.maxSpeed) sim.maxSpeed = sim.speed;

    if (sim.speed > 0.005) {
        const step = sim.speed * dt, rad = toRad(sim.heading);
        let nx = sim.x + step * Math.sin(rad), ny = sim.y + step * Math.cos(rad), blocked = false;
        for (const o of sim.obstacles) if (dist(nx, ny, o.x, o.y) - o.r < MIN_CLEARANCE) { blocked = true; break; }
        if (blocked) for (const off of [90, -90, 45, -45, 135, -135]) {
            const ar = toRad(sim.heading + off), ax = sim.x + step * 0.5 * Math.sin(ar), ay = sim.y + step * 0.5 * Math.cos(ar);
            let ok = true; for (const o of sim.obstacles) if (dist(ax, ay, o.x, o.y) - o.r < MIN_CLEARANCE) { ok = false; break; }
            if (ok) { nx = ax; ny = ay; blocked = false; break; }
        }
        if (!blocked) { sim.totalDist += dist(sim.x, sim.y, nx, ny); sim.x = nx; sim.y = ny; }
    }
    if (sim.tick % 3 === 0) { trail.push({ x: sim.x, y: sim.y, s: sim.speed, st: sim.state }); if (trail.length > 8000) trail.shift(); }
}

// ============ RENDERING ============
function resizeCanvas() {
    const c = canvas.parentElement; canvas.width = c.clientWidth * (devicePixelRatio || 1); canvas.height = c.clientHeight * (devicePixelRatio || 1);
    canvas.style.width = c.clientWidth + 'px'; canvas.style.height = c.clientHeight + 'px';
    ctx.setTransform(devicePixelRatio || 1, 0, 0, devicePixelRatio || 1, 0, 0);
}

function drawGrid() {
    if (!showGrid) return;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    let step = 50; while (step * cam.zoom < 30) step *= 2; while (step * cam.zoom > 120) step /= 2;
    const [tlx, tly] = s2w(0, 0), [brx, bry] = s2w(w, h);
    const sx = Math.floor(Math.min(tlx, brx) / step) * step, sy = Math.floor(Math.min(tly, bry) / step) * step;
    const ex = Math.max(tlx, brx) + step, ey = Math.max(tly, bry) + step;
    ctx.strokeStyle = 'rgba(42,58,78,0.3)'; ctx.lineWidth = 0.5;
    for (let x = sx; x <= ex; x += step) { const [px] = w2s(x, 0); ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, h); ctx.stroke(); }
    for (let y = sy; y <= ey; y += step) { const [, py] = w2s(0, y); ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(w, py); ctx.stroke(); }
    if (showHash) {
        ctx.font = '9px "JetBrains Mono"'; ctx.fillStyle = 'rgba(85,102,119,0.5)';
        for (let x = sx; x <= ex; x += step) { const [px, py] = w2s(x, 0); ctx.fillText(x.toFixed(0), px + 2, Math.min(py - 3, h - 4)); }
        for (let y = sy; y <= ey; y += step) { const [px, py] = w2s(0, y); ctx.fillText(y.toFixed(0), Math.max(px + 2, 4), py - 3); }
    }
}

function drawSurvey() {
    const c = [[20, -20], [420, -20], [420, -300], [20, -300]];
    ctx.beginPath(); c.forEach((p, i) => { const [x, y] = w2s(p[0], p[1]); i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y); }); ctx.closePath();
    ctx.fillStyle = 'rgba(6,182,212,0.03)'; ctx.fill();
    ctx.strokeStyle = 'rgba(6,182,212,0.25)'; ctx.lineWidth = 1.5; ctx.setLineDash([8, 4]); ctx.stroke(); ctx.setLineDash([]);
}

function drawPath() {
    if (!showPath) return;
    ctx.beginPath(); WAYPOINTS.forEach((w, i) => { const [x, y] = w2s(w[0], w[1]); i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y); });
    ctx.strokeStyle = 'rgba(255,255,255,0.08)'; ctx.lineWidth = 1; ctx.setLineDash([4, 6]); ctx.stroke(); ctx.setLineDash([]);
    WAYPOINTS.forEach((w, i) => {
        const [x, y] = w2s(w[0], w[1]), done = sim && i < sim.wptIdx, cur = sim && i === sim.wptIdx;
        ctx.beginPath(); ctx.arc(x, y, cur ? 5 : 3, 0, Math.PI * 2);
        ctx.fillStyle = done ? 'rgba(16,185,129,0.6)' : cur ? '#3b82f6' : 'rgba(255,255,255,0.12)'; ctx.fill();
        if (cur) { ctx.strokeStyle = '#3b82f6'; ctx.lineWidth = 1.5; ctx.stroke(); }
    });
}

function drawTrail() {
    if (!showTrail || trail.length < 2) return;
    const stColors = { FLEE: '239,68,68', EVADE: '249,115,22', CREEP: '245,158,11', SLOW: '234,179,8', FAST: '16,185,129', CRUISE: '251,191,36' };
    for (let i = 1; i < trail.length; i++) {
        const t = trail[i], p = trail[i - 1], [sx, sy] = w2s(t.x, t.y), [px, py] = w2s(p.x, p.y);
        const a = 0.15 + 0.85 * (i / trail.length), c = stColors[t.st] || '251,191,36';
        ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(sx, sy); ctx.strokeStyle = `rgba(${c},${a})`; ctx.lineWidth = 2; ctx.stroke();
    }
}

function drawObstacles() {
    if (!sim) return;
    for (const obs of sim.obstacles) {
        const [sx, sy] = w2s(obs.x, obs.y), sr = obs.r * cam.zoom;
        const d = dist(sim.x, sim.y, obs.x, obs.y) - obs.r;
        if (showDetection) {
            [[DETECTION_RANGE, '59,130,246', 0.04, 0.15], [SLOW_RANGE, '245,158,11', 0.05, 0.2],
            [CREEP_RANGE, '249,115,22', 0.06, 0.25], [FLEE_RANGE, '239,68,68', 0.08, 0.35]].forEach(([range, col, fa, ba]) => {
                const zr = (obs.r + range) * cam.zoom; ctx.beginPath(); ctx.arc(sx, sy, zr, 0, Math.PI * 2);
                ctx.fillStyle = `rgba(${col},${fa})`; ctx.fill();
                ctx.strokeStyle = `rgba(${col},${ba})`; ctx.lineWidth = 0.7; ctx.setLineDash([3, 3]); ctx.stroke(); ctx.setLineDash([]);
            });
        }
        const g = ctx.createRadialGradient(sx, sy, sr * 0.2, sx, sy, sr * 3);
        g.addColorStop(0, obs.color + '40'); g.addColorStop(1, 'transparent');
        ctx.beginPath(); ctx.arc(sx, sy, sr * 3, 0, Math.PI * 2); ctx.fillStyle = g; ctx.fill();
        ctx.beginPath(); ctx.arc(sx, sy, sr, 0, Math.PI * 2);
        ctx.fillStyle = obs.color + '80'; ctx.fill();
        ctx.strokeStyle = (selectedObs && selectedObs.id === obs.id) ? '#fff' : (hoveredObs && hoveredObs.id === obs.id) ? '#60a5fa' : obs.color;
        ctx.lineWidth = (selectedObs && selectedObs.id === obs.id) ? 2.5 : 2; ctx.stroke();
        ctx.font = `bold ${Math.max(9, sr * 0.7)}px "JetBrains Mono"`; ctx.fillStyle = '#fff'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText(obs.id, sx, sy);
        if (d < DETECTION_RANGE) {
            ctx.font = '9px "JetBrains Mono"';
            ctx.fillStyle = d < FLEE_RANGE ? '#ef4444' : d < CREEP_RANGE ? '#f97316' : d < SLOW_RANGE ? '#f59e0b' : '#3b82f6';
            ctx.fillText(`${d.toFixed(1)}m`, sx, sy - sr - 10);
        }
    }
    ctx.textAlign = 'start'; ctx.textBaseline = 'alphabetic';
}

function drawUSV() {
    if (!sim) return;
    const [sx, sy] = w2s(sim.x, sim.y), rad = toRad(sim.heading), sz = Math.max(7, 5 * cam.zoom);
    const hdgRad = toRad(sim.heading), hl = sz * 3; ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(sx + Math.sin(hdgRad) * hl, sy - Math.cos(hdgRad) * hl);
    ctx.strokeStyle = 'rgba(59,130,246,0.4)'; ctx.lineWidth = 1; ctx.stroke();
    let gc; switch (sim.state) { case 'FLEE': gc = 'rgba(239,68,68,0.5)'; break; case 'EVADE': gc = 'rgba(249,115,22,0.4)'; break; case 'CREEP': gc = 'rgba(245,158,11,0.3)'; break; case 'FAST': gc = 'rgba(16,185,129,0.4)'; break; default: gc = 'rgba(59,130,246,0.3)'; }
    const g = ctx.createRadialGradient(sx, sy, 0, sx, sy, sz * 3); g.addColorStop(0, gc); g.addColorStop(1, 'transparent');
    ctx.beginPath(); ctx.arc(sx, sy, sz * 3, 0, Math.PI * 2); ctx.fillStyle = g; ctx.fill();
    ctx.save(); ctx.translate(sx, sy); ctx.rotate(rad); ctx.beginPath();
    ctx.moveTo(0, -sz * 1.5); ctx.lineTo(-sz * 0.8, sz * 0.8); ctx.lineTo(0, sz * 0.4); ctx.lineTo(sz * 0.8, sz * 0.8); ctx.closePath();
    let bc; switch (sim.state) { case 'FLEE': bc = '#ef4444'; break; case 'EVADE': bc = '#f97316'; break; case 'CREEP': bc = '#f59e0b'; break; case 'FAST': bc = '#10b981'; break; default: bc = '#3b82f6'; }
    ctx.fillStyle = bc; ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke(); ctx.restore();
}

function drawMeasurements() {
    // Active measurement
    if (measuringActive && measureStart) {
        const end = measureEnd || mouseWorld;
        const [sx, sy] = w2s(measureStart.x, measureStart.y), [ex, ey] = w2s(end.x, end.y);
        ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(ex, ey); ctx.strokeStyle = '#06b6d4'; ctx.lineWidth = 1.5; ctx.setLineDash([4, 4]); ctx.stroke(); ctx.setLineDash([]);
        const d = dist(measureStart.x, measureStart.y, end.x, end.y);
        const mx = (sx + ex) / 2, my = (sy + ey) / 2;
        ctx.font = '11px "JetBrains Mono"'; ctx.fillStyle = '#06b6d4'; ctx.textAlign = 'center'; ctx.fillText(`${d.toFixed(1)}m`, mx, my - 8); ctx.textAlign = 'start';
    }
    // Saved measurements
    for (const m of measurements) {
        const [sx, sy] = w2s(m.x1, m.y1), [ex, ey] = w2s(m.x2, m.y2);
        ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(ex, ey); ctx.strokeStyle = 'rgba(6,182,212,0.5)'; ctx.lineWidth = 1; ctx.setLineDash([3, 3]); ctx.stroke(); ctx.setLineDash([]);
        const mx = (sx + ex) / 2, my = (sy + ey) / 2;
        ctx.font = '9px "JetBrains Mono"'; ctx.fillStyle = 'rgba(6,182,212,0.7)'; ctx.textAlign = 'center'; ctx.fillText(`${m.d.toFixed(1)}m`, mx, my - 6); ctx.textAlign = 'start';
    }
}

function render() {
    const w = canvas.clientWidth, h = canvas.clientHeight; ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#0a0e17'; ctx.fillRect(0, 0, w, h);
    if (sim) { cam.x += (sim.x - cam.x) * 0.03; cam.y += (sim.y - cam.y) * 0.03; cam.zoom += (cam.tZoom - cam.zoom) * 0.1; }
    drawGrid(); drawSurvey(); drawPath(); drawTrail(); drawObstacles(); drawUSV(); drawMeasurements();
}

// ============ UI UPDATES ============
const stColors = {
    IDLE: { bg: 'rgba(107,114,128,0.2)', bc: '#6b7280', c: '#9ca3af' }, CRUISE: { bg: 'rgba(59,130,246,0.15)', bc: '#3b82f6', c: '#60a5fa' },
    FAST: { bg: 'rgba(16,185,129,0.15)', bc: '#10b981', c: '#34d399' }, SLOW: { bg: 'rgba(234,179,8,0.15)', bc: '#eab308', c: '#fbbf24' },
    CREEP: { bg: 'rgba(249,115,22,0.15)', bc: '#f97316', c: '#fb923c' }, EVADE: { bg: 'rgba(249,115,22,0.2)', bc: '#f97316', c: '#fb923c' },
    FLEE: { bg: 'rgba(239,68,68,0.2)', bc: '#ef4444', c: '#f87171' }
};

function updateUI() {
    if (!sim) return;
    const s = stColors[sim.state] || stColors.IDLE, b = document.getElementById('stateBadge');
    b.style.background = s.bg; b.style.borderColor = s.bc; b.style.color = s.c;
    b.querySelector('.dot').style.background = s.c;
    document.getElementById('stateText').textContent = sim.state;
    document.getElementById('statSpeed').innerHTML = `${sim.speed.toFixed(1)}<span class="stat-unit">m/s</span>`;
    document.getElementById('statHeading').textContent = `${sim.heading.toFixed(0)}°`;
    document.getElementById('statX').innerHTML = `${sim.x.toFixed(0)}<span class="stat-unit">m</span>`;
    document.getElementById('statY').innerHTML = `${sim.y.toFixed(0)}<span class="stat-unit">m</span>`;
    const nd = sim.effMin < 999 ? `${sim.effMin.toFixed(1)}` : '—';
    const nc = sim.effMin < FLEE_RANGE ? 'var(--accent-red)' : sim.effMin < CREEP_RANGE ? 'var(--accent-orange)' : sim.effMin < SLOW_RANGE ? 'var(--accent-yellow)' : 'var(--text-primary)';
    document.getElementById('statNearest').innerHTML = `<span style="color:${nc}">${nd}</span><span class="stat-unit">m</span>`;
    document.getElementById('wptText').textContent = `WPT ${sim.wptIdx} / ${WAYPOINTS.length}`;
    const pct = (sim.wptIdx / WAYPOINTS.length) * 100;
    document.getElementById('wptPct').textContent = `${pct.toFixed(0)}%`;
    document.getElementById('progressBar').style.width = `${pct}%`;

    // Scope
    const scope = document.getElementById('scopeBody');
    if (scope && showScope) {
        scope.innerHTML = `
      <tr><td>NAV_X</td><td>${sim.x.toFixed(2)}</td></tr>
      <tr><td>NAV_Y</td><td>${sim.y.toFixed(2)}</td></tr>
      <tr><td>NAV_HEADING</td><td>${sim.heading.toFixed(2)}</td></tr>
      <tr><td>NAV_SPEED</td><td>${sim.speed.toFixed(3)}</td></tr>
      <tr><td>TARGET_SPEED</td><td>${sim.tgtSpeed.toFixed(3)}</td></tr>
      <tr><td>OBS_NEAREST</td><td>${sim.effMin.toFixed(2)}</td></tr>
      <tr><td>OBS_COUNT</td><td>${sim.obstacles.length}</td></tr>
      <tr><td>TOTAL_DIST</td><td>${sim.totalDist.toFixed(1)}</td></tr>
      <tr><td>MAX_SPEED</td><td>${sim.maxSpeed.toFixed(2)}</td></tr>
      <tr><td>STATE_CHGS</td><td>${sim.stateChanges}</td></tr>
      <tr><td>CLOSEST_EVER</td><td>${sim.closestEver.toFixed(2)}</td></tr>
      <tr><td>ELAPSED</td><td>${sim.elapsed.toFixed(1)}</td></tr>`;
    }

    // Obstacles list
    const ol = document.getElementById('obstacleList'); if (!ol) return;
    let html = '';
    for (const obs of sim.obstacles) {
        const d = dist(sim.x, sim.y, obs.x, obs.y) - obs.r;
        const cls = d < FLEE_RANGE ? 'danger' : d < DETECTION_RANGE ? 'active' : '';
        const dc = d < FLEE_RANGE ? 'var(--accent-red)' : d < CREEP_RANGE ? 'var(--accent-orange)' : d < SLOW_RANGE ? 'var(--accent-yellow)' : d < DETECTION_RANGE ? 'var(--accent-blue)' : 'var(--text-dim)';
        const sel = selectedObs && selectedObs.id === obs.id ? ' style="border-color:var(--accent-cyan)"' : '';
        html += `<div class="obstacle-item ${cls}"${sel} data-id="${obs.id}">
      <div class="obs-color" style="background:${obs.color}"></div>
      <div class="obs-info"><div class="obs-name">${obs.name}</div><div class="obs-detail">(${obs.x.toFixed(0)},${obs.y.toFixed(0)}) r=${obs.r}m</div></div>
      <div class="obs-dist" style="color:${dc}">${d.toFixed(0)}m</div>
      <button class="obs-del" onclick="removeObstacle(${obs.id})" title="Delete">✕</button></div>`;
    }
    ol.innerHTML = html;
    document.getElementById('obsCount').textContent = sim.obstacles.length;

    // Events
    const el = document.getElementById('eventLog');
    el.innerHTML = events.slice(0, 25).map(e => `<div class="event-item"><span class="event-time">${e.time}s</span><span class="event-msg ${e.type}">${e.msg}</span></div>`).join('');

    // Time
    const te = document.getElementById('timeOverlay');
    if (te) te.textContent = `T+${sim.elapsed.toFixed(1)}s | ${simSpeed}×`;
}

// ============ MAIN LOOP ============
function mainLoop(ts) {
    requestAnimationFrame(mainLoop);
    if (!lastTime) { lastTime = ts; return; }
    const rawDt = Math.min((ts - lastTime) / 1000, 0.05); lastTime = ts;
    if (running && sim) { const sd = rawDt * simSpeed, td = 1 / SIM_HZ; let b = sd; while (b > 0) { simTick(Math.min(b, td)); b -= td; } }
    render();
    if (sim && sim.tick % 6 === 0) updateUI();
}

// ============ CANVAS EVENTS ============
window.addEventListener('resize', resizeCanvas);
canvas.addEventListener('wheel', e => { e.preventDefault(); cam.tZoom *= e.deltaY > 0 ? 0.9 : 1.1; cam.tZoom = clamp(cam.tZoom, 0.2, 10); });
canvas.addEventListener('mousemove', e => {
    const r = canvas.getBoundingClientRect(); mouseScreen = { x: e.clientX - r.left, y: e.clientY - r.top };
    const [wx, wy] = s2w(mouseScreen.x, mouseScreen.y); mouseWorld = { x: wx, y: wy };
    document.getElementById('coordsOverlay').textContent = `(${wx.toFixed(0)}, ${wy.toFixed(0)})`;
    if (panning && panStart) { cam.x = panStart.cx - (e.clientX - panStart.sx) / cam.zoom; cam.y = panStart.cy + (e.clientY - panStart.sy) / cam.zoom; return; }
    if (draggingObs) { draggingObs.x = wx + dragOffset.x; draggingObs.y = wy + dragOffset.y; return; }
    if (mode === 'select' || mode === 'delete') hoveredObs = findObsAtScreen(mouseScreen.x, mouseScreen.y);
    if (measuringActive) measureEnd = mouseWorld;
});

canvas.addEventListener('mousedown', e => {
    if (e.button === 1 || (e.button === 0 && e.shiftKey)) { panning = true; panStart = { sx: e.clientX, sy: e.clientY, cx: cam.x, cy: cam.y }; canvas.classList.add('panning'); e.preventDefault(); return; }
    if (e.button !== 0) return;
    const [wx, wy] = s2w(mouseScreen.x, mouseScreen.y);
    if (mode === 'add') { addObstacle(wx, wy); return; }
    if (mode === 'delete') { const o = findObsAtScreen(mouseScreen.x, mouseScreen.y); if (o) removeObstacle(o.id); return; }
    if (mode === 'measure') {
        if (!measuringActive) { measuringActive = true; measureStart = { x: wx, y: wy }; measureEnd = null; }
        else {
            measurements.push({ x1: measureStart.x, y1: measureStart.y, x2: wx, y2: wy, d: dist(measureStart.x, measureStart.y, wx, wy) });
            measuringActive = false; measureStart = null; measureEnd = null; addEvent(`Measurement: ${measurements[measurements.length - 1].d.toFixed(1)}m`, 'info');
        } return;
    }
    // Select mode
    const o = findObsAtScreen(mouseScreen.x, mouseScreen.y);
    if (o) { selectedObs = o; draggingObs = o; dragOffset = { x: o.x - wx, y: o.y - wy }; canvas.classList.add('panning'); }
    else { selectedObs = null; panning = true; panStart = { sx: e.clientX, sy: e.clientY, cx: cam.x, cy: cam.y }; canvas.classList.add('panning'); }
});

canvas.addEventListener('mouseup', () => { panning = false; draggingObs = null; canvas.classList.remove('panning'); });
canvas.addEventListener('mouseleave', () => { panning = false; draggingObs = null; hoveredObs = null; canvas.classList.remove('panning'); });

// Double-click to center
canvas.addEventListener('dblclick', e => { const [wx, wy] = s2w(mouseScreen.x, mouseScreen.y); cam.x = wx; cam.y = wy; });

// ============ BUTTON HANDLERS ============
function setMode(m) {
    mode = m;
    document.querySelectorAll('.tool-btn').forEach(b => b.classList.toggle('btn-active', b.dataset.mode === m));
    canvas.className = '';
    if (m === 'add') canvas.classList.add('mode-add');
    else if (m === 'delete') canvas.classList.add('mode-delete');
    else if (m === 'measure') canvas.classList.add('mode-measure');
}

document.querySelectorAll('.speed-btn').forEach(b => b.addEventListener('click', () => {
    document.querySelectorAll('.speed-btn').forEach(x => x.classList.remove('active'));
    b.classList.add('active'); simSpeed = parseFloat(b.dataset.speed);
}));

document.getElementById('btnStartStop').addEventListener('click', function () {
    if (!sim) initSim(); running = !running;
    this.innerHTML = running ? '⏸ Pause' : '▶ Resume';
    this.classList.toggle('btn-primary', !running);
    addEvent(running ? (sim.tick === 0 ? 'Started' : 'Resumed') : 'Paused', 'info');
});
document.getElementById('btnReset').addEventListener('click', () => {
    running = false; lastTime = 0;
    document.getElementById('btnStartStop').innerHTML = '▶ Start';
    document.getElementById('btnStartStop').classList.add('btn-primary');
    initSim();
});
document.getElementById('btnGrid').addEventListener('click', function () { showGrid = !showGrid; this.classList.toggle('btn-active', showGrid); });
document.getElementById('btnDetect').addEventListener('click', function () { showDetection = !showDetection; this.classList.toggle('btn-active', showDetection); });
document.getElementById('btnTrail').addEventListener('click', function () { showTrail = !showTrail; this.classList.toggle('btn-active', showTrail); });
document.getElementById('btnMarkPath').addEventListener('click', function () { showPath = !showPath; this.classList.toggle('btn-active', showPath); });
document.getElementById('btnHash').addEventListener('click', function () { showHash = !showHash; this.classList.toggle('btn-active', showHash); });
document.getElementById('btnClearMeasure').addEventListener('click', () => { measurements = []; measuringActive = false; measureStart = null; measureEnd = null; });
document.getElementById('btnClearTrail').addEventListener('click', () => { trail = []; });
document.getElementById('btnCenterVehicle').addEventListener('click', () => { if (sim) { cam.x = sim.x; cam.y = sim.y; } });
document.getElementById('btnZoomFit').addEventListener('click', () => { cam.x = 220; cam.y = -160; cam.tZoom = 1.0; });

// Obstacle modal
document.getElementById('btnAddObs').addEventListener('click', () => { document.getElementById('modalOverlay').classList.add('show'); });
document.getElementById('btnModalCancel').addEventListener('click', () => { document.getElementById('modalOverlay').classList.remove('show'); });
document.getElementById('btnModalAdd').addEventListener('click', () => {
    const x = parseFloat(document.getElementById('obsX').value) || 100;
    const y = parseFloat(document.getElementById('obsY').value) || -100;
    const r = parseFloat(document.getElementById('obsR').value) || 4;
    const c = document.getElementById('obsColor').value;
    addObstacle(x, y, r, c);
    document.getElementById('modalOverlay').classList.remove('show');
});

// Keyboard shortcuts
document.addEventListener('keydown', e => {
    if (e.target.tagName === 'INPUT') return;
    if (e.key === '1') setMode('select'); if (e.key === '2') setMode('add'); if (e.key === '3') setMode('delete'); if (e.key === '4') setMode('measure');
    if (e.key === ' ') { e.preventDefault(); document.getElementById('btnStartStop').click(); }
    if (e.key === 'r') document.getElementById('btnReset').click();
    if (e.key === 'c') document.getElementById('btnCenterVehicle').click();
    if (e.key === 'Delete' && selectedObs) { removeObstacle(selectedObs.id); }
    if (e.key === 'Escape') { setMode('select'); measuringActive = false; measureStart = null; }
});

// ============ INIT ============
resizeCanvas(); initSim(); requestAnimationFrame(mainLoop);
