// 실험: 주행 로봇. 차동 구동 로봇이 LiDAR와 주행 기록으로 목표까지 간다. (단위: m, s, kg, rad)
(() => {
'use strict';
const W = 8, H = 6, CELL = 0.2, GW = W / CELL, GH = H / CELL, N = GW * GH;
const R = 0.18, V_MAX = 1.0, W_MAX = 3.0;
const RAYS = 120, RANGE = 3.0, MARK_R = 0.8;
const KP = 60, C_ROLL = 2, F_MOTOR = 20;   // 속도 제어 이득 N·s/m, 구름 저항 N·s/m, 모터 최대 힘 N
const BOXES = [   // 장애물: 중심 (x, y), 크기 (w, h)
  { x: 3.0, y: 1.1, w: 0.4, h: 2.2 }, { x: 5.2, y: 4.7, w: 0.4, h: 2.6 },
  { x: 1.4, y: 3.8, w: 1.2, h: 0.4 }, { x: 6.8, y: 1.6, w: 1.0, h: 1.0 }, { x: 4.2, y: 4.2, w: 0.6, h: 0.6 },
];
const MARKS = [{ x: 2.4, y: 2.7 }, { x: 5.8, y: 2.6 }, { x: 6.6, y: 4.6 }];   // 위치를 아는 바닥 표식: 원 안에 들어오면 추정 위치가 보정된다
let p, S, V;   // 설정값, 시뮬레이션 상태, 화면 객체

const distBox = (x, y, b) => Math.hypot(Math.max(Math.abs(x - b.x) - b.w / 2, 0), Math.max(Math.abs(y - b.y) - b.h / 2, 0));
// 로봇(원)이 벽·장애물에 파고든 깊이. 0 이하이면 닿지 않음
const pen = (x, y) => Math.max(R - x, R - y, x - (W - R), y - (H - R), ...BOXES.map(b => R - distBox(x, y, b)));

// 경로 계획용 격자: 로봇 반지름 + 여유만큼 부풀린 장애물
const grid = new Uint8Array(N);
for (let i = 0; i < N; i++) grid[i] = pen((i % GW + .5) * CELL, ((i / GW | 0) + .5) * CELL) > -0.12 ? 1 : 0;
const cellOf = (x, y) => clamp(y / CELL | 0, 0, GH - 1) * GW + clamp(x / CELL | 0, 0, GW - 1);
function freeCell(c) {   // 막힌 칸이면 가장 가까운 빈 칸
  if (!grid[c]) return c;
  let best = c, bd = Infinity;
  for (let i = 0; i < N; i++) if (!grid[i]) { const d = Math.hypot(i % GW - c % GW, (i / GW | 0) - (c / GW | 0)); if (d < bd) { bd = d; best = i; } }
  return best;
}
const aCost = new Float32Array(N), aFrom = new Int32Array(N), aClosed = new Uint8Array(N);   // 매번 새로 만들지 않고 재사용
function plan(sx, sy, gx, gy) {   // A* (8방향, 모서리 가로지르기 금지) → [[x, y], …] 또는 null
  const s = freeCell(cellOf(sx, sy)), g = freeCell(cellOf(gx, gy));
  const cost = aCost.fill(Infinity), from = aFrom.fill(-1), closed = aClosed.fill(0), open = [s], gx0 = g % GW, gy0 = g / GW | 0;
  cost[s] = 0;
  while (open.length) {
    let bi = 0, bf = Infinity;   // 열린 칸 중 (지금까지 비용 + 목표까지 직선거리)가 가장 작은 칸
    for (let i = 0; i < open.length; i++) { const o = open[i], f = cost[o] + Math.hypot(o % GW - gx0, (o / GW | 0) - gy0); if (f < bf) { bf = f; bi = i; } }
    const c = open[bi]; open[bi] = open[open.length - 1]; open.pop();
    if (c === g) break;
    closed[c] = 1;
    const cx = c % GW, cy = c / GW | 0;
    for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
      const nx = cx + dx, ny = cy + dy, n = ny * GW + nx;
      if ((!dx && !dy) || nx < 0 || ny < 0 || nx >= GW || ny >= GH || grid[n] || closed[n]) continue;
      if (dx && dy && (grid[cy * GW + nx] || grid[ny * GW + cx])) continue;
      const d = cost[c] + (dx && dy ? Math.SQRT2 : 1);
      if (d < cost[n]) { if (cost[n] === Infinity) open.push(n); cost[n] = d; from[n] = c; }
    }
  }
  if (g !== s && from[g] < 0) return null;
  const path = [];
  for (let c = g; c >= 0; c = from[c]) path.push([(c % GW + .5) * CELL, ((c / GW | 0) + .5) * CELL]);
  path.reverse();
  if (g === cellOf(gx, gy)) path[path.length - 1] = [gx, gy];
  return path;
}
function carrot(path, x, y, look) {   // 경로 위에서 로봇보다 look 만큼 앞선 점
  let bi = 0, bd = Infinity;
  path.forEach((q, i) => { const d = Math.hypot(q[0] - x, q[1] - y); if (d < bd) { bd = d; bi = i; } });
  while (bi < path.length - 1 && Math.hypot(path[bi][0] - x, path[bi][1] - y) < look) bi++;
  return path[bi];
}
function rayBox(ox, oy, dx, dy, b) {   // 광선이 상자에 닿는 거리 (안 닿으면 Infinity)
  let t0 = 0, t1 = Infinity;
  for (const [o, d, c, hw] of [[ox, dx, b.x, b.w / 2], [oy, dy, b.y, b.h / 2]]) {
    if (Math.abs(d) < 1e-9) { if (Math.abs(o - c) > hw) return Infinity; continue; }
    let a = (c - hw - o) / d, z = (c + hw - o) / d;
    if (a > z) [a, z] = [z, a];
    if (a > t0) t0 = a;
    if (z < t1) t1 = z;
    if (t0 > t1) return Infinity;
  }
  return t0;
}

function reset() {
  S = { x: 1, y: 1, th: 0, v: 0, vp: 0, u: 0, w: 0, ex: 1, ey: 1, eth: 0,   // 실제 자세 / 속도 / 추정 자세
    goal: { x: 7, y: 5 }, path: null, planT: 1, scanT: 1, vc: 0, wc: 0, F: 0, fmax: 0,
    ranges: new Float32Array(RAYS).fill(RANGE), pts: new Float32Array(RAYS * 2).fill(NaN), sx: 1, sy: 1, front: RANGE,
    hits: 0, inHit: false, arrived: false, manual: false, tr: {} };
}
function setGoal(x, y) { if (x > 0 && x < W && y > 0 && y < H) { S.goal = { x, y }; S.planT = 1; S.arrived = false; } }
function fixPose() { S.ex = S.x; S.ey = S.y; S.eth = S.th; }

function scan() {   // LiDAR: 360°를 RAYS 개로 나눠 가장 가까운 면까지의 거리를 잰다
  let front = RANGE;
  S.sx = S.x; S.sy = S.y;
  for (let k = 0; k < RAYS; k++) {
    const rel = k * 2 * Math.PI / RAYS, a = S.th + rel, dx = Math.cos(a), dy = Math.sin(a);
    let d = Math.min(dx > 0 ? (W - S.x) / dx : dx < 0 ? -S.x / dx : Infinity, dy > 0 ? (H - S.y) / dy : dy < 0 ? -S.y / dy : Infinity);
    for (const b of BOXES) d = Math.min(d, rayBox(S.x, S.y, dx, dy, b));
    if (d < RANGE && p.lidarSd) d = Math.max(0, d + p.lidarSd * gauss());
    const hit = d < RANGE;
    S.ranges[k] = hit ? d : RANGE;
    S.pts[2 * k] = hit ? S.x + dx * d : NaN; S.pts[2 * k + 1] = S.y + dy * d;
    if (hit && (rel < 0.35 || rel > 2 * Math.PI - 0.35) && d < front) front = d;   // 전방 ±20°
  }
  S.front = Math.max(0, front - R);
}

function step(dt) {
  const tr = S.tr = { scan: false, plan: false, fix: false };

  // ① 감지: 센서가 세계를 읽는다. 로봇은 실제 위치를 모르고, 추정 위치(④에서 갱신)만 안다
  if ((S.scanT += dt) >= 0.1) { S.scanT = 0; scan(); tr.scan = true; }   // LiDAR 10 Hz
  if (Math.hypot(S.ex - S.x, S.ey - S.y) > 0.01 && MARKS.some(m => Math.hypot(m.x - S.x, m.y - S.y) < MARK_R)) { fixPose(); tr.fix = true; }
  mark(0);

  // ② 판단: 추정 위치와 지도로 경로를 정하고, 속도 명령(vc, wc)을 만든다
  let vc = 0, wc = 0;
  S.manual = !!(keys.v || keys.w);
  if (S.manual) { vc = keys.v * V_MAX; wc = keys.w * 2; }
  else {
    const gd = Math.hypot(S.goal.x - S.ex, S.goal.y - S.ey);
    S.arrived = gd < 0.12;
    if (!S.arrived) {
      if ((S.planT += dt) >= 0.5) { S.planT = 0; S.path = plan(S.ex, S.ey, S.goal.x, S.goal.y); tr.plan = true; }   // 재계획 2 Hz
      if (S.path) {
        const c = carrot(S.path, S.ex, S.ey, 0.4), err = wrap(Math.atan2(c[1] - S.ey, c[0] - S.ex) - S.eth);
        wc = clamp(3 * err, -W_MAX, W_MAX);
        vc = V_MAX * Math.max(0, Math.cos(err)) ** 2 * Math.min(1, gd / 0.6 + 0.1);
      }
    }
    if (vc > 0 && S.front < 0.6) vc *= Math.max(0, (S.front - 0.08) / 0.52);   // LiDAR 전방 거리로 감속·정지
  }
  S.vc = vc; S.wc = wc;
  mark(1);

  // ③ 행동: 명령이 실제 속도가 되는 과정. 운동학은 즉시, 동역학은 힘 → 가속
  const v0 = S.v;
  if (!p.dyn) { S.v = vc; S.w = wc; S.u = 0; S.F = 0; S.fmax = 0; S.vp = S.v; }
  else {
    const m = p.mass, fmax = S.fmax = Math.min(F_MOTOR, p.mu * m * G);   // 바퀴가 바닥에 전할 수 있는 힘은 μ·m·g 까지
    if (p.implicit) {
      const v1 = integ(v0, KP * vc, KP + C_ROLL, m, dt, true);
      if (Math.abs(KP * (vc - v1)) <= fmax) { S.F = KP * (vc - v1); S.v = v1; }
      else { S.F = Math.sign(vc - v0) * fmax; S.v = integ(v0, S.F, C_ROLL, m, dt, true); }
    } else { S.F = clamp(KP * (vc - v0), -fmax, fmax); S.v = integ(v0, S.F, C_ROLL, m, dt, false); }
    // shortcut: 회전 속도는 명령을 그대로 따른다 (회전 관성 없음). 회전 동역학이 필요해지면 관성 모멘트를 추가
    S.w = wc;
    const need = S.v * S.w, grip = p.mu * G;   // 회전에 필요한 옆 가속도 vs 마찰이 줄 수 있는 가속도
    if (Math.abs(need) > grip) S.u -= (need - Math.sign(need) * grip) * dt;
    else S.u -= Math.sign(S.u) * Math.min(Math.abs(S.u), (grip - Math.abs(need)) * dt);
    S.vp = p.implicit ? S.v : v0;   // 명시적 오일러는 이전 속도로 위치를 옮긴다
  }
  mark(2);

  // ④ 적분: 속도를 dt 만큼 더해 새 자세를 얻고, 충돌이면 막는다
  const c = Math.cos(S.th), s = Math.sin(S.th);
  const nx = S.x + (S.vp * c - S.u * s) * dt, ny = S.y + (S.vp * s + S.u * c) * dt, p1 = pen(nx, ny);
  const hit = p1 > 0 && p1 >= pen(S.x, S.y);   // 닿았고 더 파고드는 방향일 때만 막는다 (빠져나오는 움직임은 허용)
  if (hit) { if (!S.inHit) S.hits++; S.v = S.vp = S.u = 0; } else { S.x = nx; S.y = ny; }
  S.inHit = hit;
  S.th = wrap(S.th + S.w * dt);
  // 주행 기록(바퀴 엔코더): 이번 틱에 바퀴가 돈 만큼 추정 자세를 옮긴다. 옆미끄럼은 바퀴가 모르므로 오차가 된다
  S.ex += S.vp * Math.cos(S.eth) * dt * (1 + p.odoK); S.ey += S.vp * Math.sin(S.eth) * dt * (1 + p.odoK);
  S.eth = wrap(S.eth + S.w * dt + p.odoK * S.vp * dt);   // 좌우 바퀴 크기 차이: 달릴수록 방향이 틀어진다
  mark(3);
}

function init(k) {
  p = this.p;
  const floor = k.mesh(new THREE.PlaneGeometry(W, H), k.col('--floor'), W / 2, 0, H / 2); floor.rotation.x = -Math.PI / 2; floor.castShadow = false;
  const g = [];   // 1 m 눈금
  for (let x = 0; x <= W; x++) g.push(x, 0.003, 0, x, 0.003, H);
  for (let y = 0; y <= H; y++) g.push(0, 0.003, y, W, 0.003, y);
  k.lineSeg(g, k.col('--grid'));
  for (const [w, d, x, z] of [[W + 0.2, 0.1, W / 2, -0.05], [W + 0.2, 0.1, W / 2, H + 0.05], [0.1, H, -0.05, H / 2], [0.1, H, W + 0.05, H / 2]]) k.mesh(new THREE.BoxGeometry(w, 0.3, d), k.col('--wall'), x, 0.15, z);
  for (const b of BOXES) k.mesh(new THREE.BoxGeometry(b.w, 0.5, b.h), k.col('--box'), b.x, 0.25, b.y);
  for (const m of MARKS) { k.flat(new THREE.CircleGeometry(0.12, 24), 0xf2b705).position.set(m.x, 0.005, m.y); k.flat(new THREE.RingGeometry(MARK_R - 0.03, MARK_R, 48), 0xf2b705, 0.45).position.set(m.x, 0.005, m.y); }

  V = {};
  V.robot = new THREE.Group(); k.g.add(V.robot);   // 앞 = +x
  k.mesh(new THREE.CylinderGeometry(R, R, 0.14, 24), 0xe5352b, 0, 0.11, 0, V.robot);
  k.mesh(new THREE.BoxGeometry(0.12, 0.05, 0.1), 0x16202a, R - 0.06, 0.205, 0, V.robot);
  for (const z of [-R, R]) k.mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.04, 12), 0x16202a, 0, 0.06, z, V.robot).rotation.x = Math.PI / 2;
  V.ghost = new THREE.Group(); k.g.add(V.ghost);   // 로봇이 믿는 자신의 위치
  const wire = new THREE.MeshBasicMaterial({ color: k.css('--danger'), wireframe: true });
  V.ghost.add(new THREE.Mesh(new THREE.CylinderGeometry(R, R, 0.14, 12, 1, true), wire)); V.ghost.children[0].position.y = 0.11;
  V.ghost.add(new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.05, 0.1), wire)); V.ghost.children[1].position.set(R - 0.06, 0.205, 0);
  V.goal = k.flat(new THREE.RingGeometry(0.1, 0.17, 32), k.css('--accent'));
  V.path = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: k.css('--accent') })); V.path.frustumCulled = false; k.g.add(V.path);
  const pg = new THREE.BufferGeometry(); pg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(RAYS * 3), 3));
  V.pts = new THREE.Points(pg, new THREE.PointsMaterial({ color: 0xe5484d, size: 4 * k.Q.pr, sizeAttenuation: false })); V.pts.frustumCulled = false; k.g.add(V.pts);
  if (k.Q.rays) V.rays = k.lineSeg(new Array(RAYS * 6).fill(0), 0xe5484d, 0.14);
}

function draw() {
  V.robot.position.set(S.x, 0, S.y); V.robot.rotation.y = -S.th;
  V.ghost.visible = Math.hypot(S.ex - S.x, S.ey - S.y) > 0.02;
  V.ghost.position.set(S.ex, 0, S.ey); V.ghost.rotation.y = -S.eth;
  V.goal.position.set(S.goal.x, 0.008, S.goal.y);
  if (S.path !== V.shown) { V.shown = S.path; V.path.geometry.setFromPoints((S.path || []).map(q => new THREE.Vector3(q[0], 0.012, q[1]))); }
  const a = V.pts.geometry.attributes.position.array, b = V.rays && V.rays.geometry.attributes.position.array;
  for (let k = 0; k < RAYS; k++) {
    const x = S.pts[2 * k], on = x === x;   // NaN = 닿은 면 없음
    a[3 * k] = on ? x : S.sx; a[3 * k + 1] = on ? 0.17 : 1e4; a[3 * k + 2] = on ? S.pts[2 * k + 1] : S.sy;   // 1e4: 카메라 시야 밖으로 보내 숨김
    if (b) { b[6 * k] = S.sx; b[6 * k + 1] = a[3 * k + 1]; b[6 * k + 2] = S.sy; b[6 * k + 3] = a[3 * k]; b[6 * k + 4] = a[3 * k + 1]; b[6 * k + 5] = a[3 * k + 2]; }
  }
  V.pts.geometry.attributes.position.needsUpdate = true;
  if (b) V.rays.geometry.attributes.position.needsUpdate = true;
}

const f = (v, d = 2) => v.toFixed(d);
EXPS.push({
  id: 'rover', name: '주행 로봇',
  hint: '바닥 클릭: 목표 지정 · W A S D: 수동 조종 · 드래그: 회전 · 휠: 확대',
  cam: { az: 0, el: 0.95, d: 10.5, t: [W / 2, 0, H / 2] },
  graph: { title: '속도 (회색: 명령 · 색: 실제)', min: -0.4, max: 1.4, marks: [[0, '0'], [1, '1 m/s']] },
  params: [
    { key: 'dyn', label: '물리 모드', options: [[0, '운동학 (명령 = 속도)'], [1, '동역학 (힘 → 가속)']], value: 0 },
    { key: 'implicit', label: '적분 방식', options: [[0, '명시적 오일러'], [1, '암시적 오일러']], value: 0, when: q => q.dyn },
    { key: 'mass', label: '질량', min: 1, max: 30, step: 1, value: 5, fmt: v => `${v} kg`, when: q => q.dyn },
    { key: 'mu', label: '마찰 계수 μ', min: 0.05, max: 1, step: 0.05, value: 0.8, fmt: v => f(v), when: q => q.dyn },
    { key: 'lidarSd', label: 'LiDAR 거리 노이즈', min: 0, max: 0.2, step: 0.01, value: 0, fmt: v => `±${f(v * 100, 0)} cm` },
    { key: 'odoK', label: '주행 기록 오차', min: 0, max: 0.15, step: 0.01, value: 0, fmt: v => `${f(v)} rad/m` },
  ],
  buttons: [{ label: '위치 보정', run: fixPose }],
  labs: [
    '일시정지 후 <b>한 틱 실행</b>을 눌러 네 단계의 값이 바뀌는 순서를 본다.',
    '동역학 + 명시적 오일러 + 질량 1 kg + DT 1/10초: 속도 그래프가 진동한다. 암시적 오일러로 바꾸면 멈춘다.',
    '동역학 + μ 0.05: 빙판처럼 가속·제동이 늦고 회전에서 옆으로 미끄러진다.',
    '주행 기록 오차를 올리면 빨간 테두리(로봇이 믿는 위치)가 실제와 벌어진다. 노란 원 안에 들어오면 보정된다.',
  ],
  init, reset, step, draw, click: setGoal,
  sample: () => [S.vc, S.v],
  stages() {
    const tr = S.tr, err = Math.hypot(S.ex - S.x, S.ey - S.y);
    return [
      `LiDAR ${tr.scan ? '갱신' : '유지'} · 전방 ${f(S.front)} m · 추정 오차 ${f(err * 100, 0)} cm${tr.fix ? ' → 표식으로 보정' : ''}`,
      `${S.manual ? '수동 조종' : S.arrived ? '도착' : S.path ? `경로 ${S.path.length}점${tr.plan ? ' (재계획)' : ''}` : '경로 없음'} → 명령 v ${f(S.vc)} m/s · ω ${f(S.wc)} rad/s`,
      p.dyn ? `구동력 ${f(S.F, 1)} N (한계 ${f(S.fmax, 1)} N) → 가속 ${f((S.F - C_ROLL * S.v) / p.mass)} m/s²` : '운동학: 명령 속도를 그대로 적용',
      `v ${f(S.v)} m/s · 옆미끄럼 ${f(S.u)} m/s · 위치 (${f(S.x)}, ${f(S.y)})${S.inHit ? ' · 충돌' : ''}`,
    ];
  },
  stat: () => `충돌 ${S.hits}회 · 추정 오차 ${f(Math.hypot(S.ex - S.x, S.ey - S.y) * 100, 0)} cm`,
  test() {   // 해석해와 비교: 일정한 힘 F, 저항 c 이면 종단 속도는 F / c. 그리고 시작점 → 목표 경로가 있어야 한다
    const ok = [false, true].map(imp => { let v = 0; for (let i = 0; i < 5000; i++) v = integ(v, 3, C_ROLL, 5, 1 / 60, imp); return Math.abs(v - 3 / C_ROLL) < 0.01 * 3 / C_ROLL; });
    return ok[0] && ok[1] && !!plan(1, 1, 7, 5);
  },
  get S() { return S; },
});
})();
