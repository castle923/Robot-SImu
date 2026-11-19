// 실험: 매니퓰레이터. 3관절 팔(회전·어깨·팔꿈치)이 블록을 집어 옮긴다. (단위: m, s, kg, rad, N·m)
(() => {
'use strict';
const H0 = 0.12, L1 = 0.3, L2 = 0.3, M1 = 1.0, M2 = 0.8;   // 어깨 높이, 링크 길이, 링크 질량
const B = 0.05, BLK = 0.06, UP = 0.2, TABLE = 0.7, QD_MAX = 50;   // 관절 마찰, 블록 한 변, 이동 높이, 작업대 반지름
const LIM = [[-Infinity, Infinity], [-0.2, 1.9], [-2.8, 0]];   // 관절 가동 범위
const HOME = [0, 1.1, -1.9], COLORS = [0x2563eb, 0xf59e0b, 0x16a34a];
let p, A, V;   // 설정값, 시뮬레이션 상태, 화면 객체

// 정기구학: 관절 각도 → 끝점 위치 [x, 높이, z]
const fk = q => { const a = q[1] + q[2], r = L1 * Math.cos(q[1]) + L2 * Math.cos(a); return [r * Math.cos(q[0]), H0 + L1 * Math.sin(q[1]) + L2 * Math.sin(a), r * Math.sin(q[0])]; };
// 역기구학: 끝점 위치 → 관절 각도 (팔꿈치가 위로 가는 해). 닿지 않는 곳이면 그 방향으로 최대한 뻗은 자세
function ik(x, y, z) {
  const r = Math.hypot(x, z), h = y - H0, d2 = r * r + h * h;
  const c = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2), q2 = -Math.acos(clamp(c, -1, 1));
  return { q: [Math.atan2(z, x), Math.atan2(h, r) - Math.atan2(L2 * Math.sin(q2), L1 + L2 * Math.cos(q2)), q2], ok: c <= 1 };
}
// shortcut: 관절마다 따로 계산한다 (관절 사이 결합력·코리올리 힘 무시). 빠른 동작의 정확도가 필요해지면 전체 동역학 행렬로 교체
const inertia = (q, mp) => {   // 각 관절이 돌려야 하는 관성 모멘트 kg·m²
  const c2 = Math.cos(q[2]), r1 = L1 * Math.cos(q[1]), rt = r1 + L2 * Math.cos(q[1] + q[2]);
  return [0.02 + M1 * r1 * r1 / 3 + M2 * ((r1 + rt) / 2) ** 2 + mp * rt * rt,
    0.01 + M1 * L1 * L1 / 3 + M2 * (L1 * L1 + L2 * L2 / 3 + L1 * L2 * c2) + mp * (L1 * L1 + L2 * L2 + 2 * L1 * L2 * c2),
    0.005 + M2 * L2 * L2 / 3 + mp * L2 * L2];
};
const gravity = (q, mp) => {   // 중력이 각 관절에 거는 토크 (음수 = 아래로 끌어내림)
  const g2 = -G * Math.cos(q[1] + q[2]) * (M2 * L2 / 2 + mp * L2);
  return [0, -G * Math.cos(q[1]) * (M1 * L1 / 2 + (M2 + mp) * L1) + g2, g2];
};

function reset() {
  A = { q: [...HOME], qd: [0, 0, 0], qm: [...HOME], qdm: [0, 0, 0], qt: [...HOME], tgt: fk(HOME), tip: fk(HOME), reach: true,
    cmd: [0, 0, 0], tau: [0, 0, 0], tg: [0, 0, 0], acc: [0, 0, 0], sat: false, seq: [], wpT: 0, settle: 0, held: null, note: '블록을 클릭하면 집는다',
    blocks: [[0.36, 0.14], [0.22, -0.3], [-0.3, 0.26]].map(([x, z], i) => ({ x, y: BLK / 2, z, vy: 0, i })) };
}
function click(x, z, ray) {   // 블록 클릭: 집기. 들고 있을 때 클릭: 내려놓기. 그 외: 그 위로 이동
  const r = Math.hypot(x, z);
  if (r < 0.1 || r > TABLE) return;
  // 화면에서 블록 몸통을 누르면 바닥 좌표는 블록 뒤쪽이 된다. 그래서 광선이 블록에 닿았는지를 먼저 본다
  const hit = ray && ray.intersectObjects(V.blocks)[0], free = b => b !== A.held && b.y < BLK;
  const b = A.blocks.find(b => free(b) && (hit ? V.blocks[b.i] === hit.object : Math.hypot(b.x - x, b.z - z) < BLK));
  if (!A.held && b) A.seq = [{ p: [b.x, UP, b.z] }, { p: [b.x, BLK + 0.005, b.z], grip: 'on', blk: b }, { p: [b.x, UP, b.z] }];
  else if (A.held) A.seq = [{ p: [x, UP, z] }, { p: [x, BLK + 0.005, z], grip: 'off' }, { p: [x, UP, z] }];
  else A.seq = [{ p: [x, UP, z] }];
  A.wpT = A.settle = 0;
}
function task(dt) {   // 이동 단계를 차례로 수행: 목표 자세에 가까워져 잠잠해지면(또는 2.5초가 지나면) 다음 단계
  const w = A.seq[0];
  if (!w) return;
  if (!w.set) { w.set = true; const s = ik(...w.p); A.qt = s.q; A.tgt = w.p; A.reach = s.ok; }
  A.wpT += dt;
  const err = Math.max(Math.abs(wrap(A.qt[0] - A.qm[0])), Math.abs(A.qt[1] - A.qm[1]), Math.abs(A.qt[2] - A.qm[2]));
  A.settle = err < 0.02 ? A.settle + dt : 0;
  if (A.settle < 0.25 && A.wpT < 2.5) return;
  if (w.grip === 'on') {   // 흡착은 실제 끝점이 블록 윗면에 충분히 가까울 때만 된다
    const b = w.blk, miss = Math.hypot(A.tip[0] - b.x, A.tip[1] - (b.y + BLK / 2), A.tip[2] - b.z);
    if (miss < 0.035) { A.held = b; A.note = '집기 성공'; } else A.note = `집기 실패: 끝점이 ${(miss * 100).toFixed(1)} cm 벗어남`;
  } else if (w.grip === 'off' && A.held) { A.held = null; A.note = '내려놓음'; }
  A.seq.shift(); A.wpT = A.settle = 0;
}

function step(dt) {
  const mp = A.held ? p.load : 0;

  // ① 감지: 관절 엔코더가 각도를 읽는다. 속도는 직접 재지 못해 각도 차이를 dt로 나눠 추정한다
  for (let i = 0; i < 3; i++) { const m = A.q[i] + (p.encSd ? p.encSd * gauss() : 0); A.qdm[i] = (m - A.qm[i]) / dt; A.qm[i] = m; }
  mark(0);

  // ② 판단: 목표 끝점 → 역기구학으로 목표 각도 → PD 제어로 명령 토크
  task(dt);
  const In = inertia(A.qm, 0), comp = p.comp ? gravity(A.qm, p.comp === 2 ? mp : 0) : [0, 0, 0], wn = p.wn;
  for (let i = 0; i < 3; i++) {
    const e = i ? A.qt[i] - A.qm[i] : wrap(A.qt[0] - A.qm[0]);
    A.cmd[i] = In[i] * (wn * wn * e - 2 * p.zeta * wn * A.qdm[i]) - comp[i];   // 오차에 비례 + 속도에 반비례 − 중력 보상
  }
  mark(1);

  // ③ 행동: 모터가 낼 수 있는 토크로 자르고, 중력·마찰과 합쳐 각가속도를 구한다
  const I = inertia(A.q, mp);
  A.tg = gravity(A.q, mp); A.sat = false;
  for (let i = 0; i < 3; i++) {
    A.tau[i] = clamp(A.cmd[i], -p.tmax, p.tmax);
    if (A.tau[i] !== A.cmd[i]) A.sat = true;
    A.acc[i] = (A.tau[i] + A.tg[i] - B * A.qd[i]) / I[i];
  }
  mark(2);

  // ④ 적분: 명시적 오일러는 이전 속도로 각도를 옮기고, 반암시적 오일러는 새 속도로 옮긴다
  for (let i = 0; i < 3; i++) {
    if (p.semi) { A.qd[i] += A.acc[i] * dt; A.q[i] += A.qd[i] * dt; } else { A.q[i] += A.qd[i] * dt; A.qd[i] += A.acc[i] * dt; }
    A.qd[i] = clamp(A.qd[i], -QD_MAX, QD_MAX);   // 폭주해도 숫자가 무한대로 가지 않게
    if (A.q[i] < LIM[i][0]) { A.q[i] = LIM[i][0]; A.qd[i] = Math.max(0, A.qd[i]); }
    if (A.q[i] > LIM[i][1]) { A.q[i] = LIM[i][1]; A.qd[i] = Math.min(0, A.qd[i]); }
  }
  A.tip = fk(A.q);
  for (const b of A.blocks) {
    if (b === A.held) { b.x = A.tip[0]; b.y = Math.max(BLK / 2, A.tip[1] - BLK / 2); b.z = A.tip[2]; b.vy = 0; }
    else if (b.y > BLK / 2) { b.vy -= G * dt; b.y = Math.max(BLK / 2, b.y + b.vy * dt); }   // 놓인 블록은 떨어진다
  }
  mark(3);
}

function init(k) {
  p = this.p;
  k.mesh(new THREE.CylinderGeometry(TABLE, TABLE, 0.02, 64), k.col('--floor'), 0, -0.01, 0).castShadow = false;
  k.flat(new THREE.RingGeometry(L1 + L2 - 0.004, L1 + L2, 96), k.col('--grid')).position.y = 0.002;   // 최대 도달 범위
  V = { blocks: COLORS.map(c => k.mesh(new THREE.BoxGeometry(BLK, BLK, BLK), c, 0, 0, 0)) };
  const dark = 0x16202a, red = 0xe5352b;
  k.mesh(new THREE.CylinderGeometry(0.07, 0.09, H0 - 0.02, 24), dark, 0, (H0 - 0.02) / 2, 0);
  V.yaw = new THREE.Group(); k.g.add(V.yaw);
  V.sh = new THREE.Group(); V.sh.position.y = H0; V.yaw.add(V.sh);
  V.el = new THREE.Group(); V.el.position.x = L1; V.sh.add(V.el);
  const joint = parent => k.mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.07, 16), dark, 0, 0, 0, parent).rotation.x = Math.PI / 2;
  joint(V.sh); k.mesh(new THREE.BoxGeometry(L1, 0.04, 0.05), red, L1 / 2, 0, 0, V.sh);
  joint(V.el); k.mesh(new THREE.BoxGeometry(L2, 0.035, 0.04), red, L2 / 2, 0, 0, V.el);
  V.tip = k.mesh(new THREE.SphereGeometry(0.018, 16, 12), dark, L2, 0, 0, V.el);
  V.tgt = k.mesh(new THREE.SphereGeometry(0.012, 12, 8), k.col('--accent'), 0, 0, 0); V.tgt.castShadow = false;
  V.ring = k.flat(new THREE.RingGeometry(0.03, 0.04, 24), k.css('--accent'));
}
function draw() {
  V.yaw.rotation.y = -A.q[0]; V.sh.rotation.z = A.q[1]; V.el.rotation.z = A.q[2];
  A.blocks.forEach(b => V.blocks[b.i].position.set(b.x, b.y, b.z));
  V.tgt.position.set(...A.tgt); V.ring.position.set(A.tgt[0], 0.003, A.tgt[2]);
  V.tip.material.color.set(A.held ? 0x16a34a : 0x16202a);
}

const f = (v, d = 2) => v.toFixed(d), deg = v => (v * 180 / Math.PI).toFixed(0);
EXPS.push({
  id: 'arm', name: '매니퓰레이터',
  hint: '블록 클릭: 집기 · 빈 곳 클릭: 이동 / 내려놓기 · 드래그: 회전 · 휠: 확대',
  cam: { az: 0.5, el: 0.5, d: 1.9, t: [0, 0.15, 0] },
  graph: { title: '어깨 관절 각도 (회색: 목표 · 색: 실제)', min: -0.4, max: 2.1, marks: [[0, '0°'], [Math.PI / 2, '90°']] },
  params: [
    { key: 'wn', label: '제어 강성 ωn', min: 2, max: 40, step: 1, value: 12, fmt: v => `${v} rad/s` },
    { key: 'zeta', label: '감쇠비 ζ', min: 0, max: 2, step: 0.05, value: 1, fmt: v => f(v) },
    { key: 'comp', label: '중력 보상', options: [[2, '팔 + 짐 무게'], [1, '팔 무게만'], [0, '없음']], value: 2 },
    { key: 'tmax', label: '모터 토크 한계', min: 2, max: 30, step: 1, value: 12, fmt: v => `±${v} N·m` },
    { key: 'load', label: '짐(블록) 무게', min: 0, max: 3, step: 0.1, value: 0.3, fmt: v => `${f(v, 1)} kg` },
    { key: 'encSd', label: '엔코더 노이즈', min: 0, max: 0.02, step: 0.001, value: 0, fmt: v => `±${f(v * 180 / Math.PI)}°` },
    { key: 'semi', label: '적분 방식', options: [[1, '반암시적 오일러'], [0, '명시적 오일러']], value: 1 },
  ],
  buttons: [{ label: '기본 자세로', run() { A.seq = []; A.qt = [...HOME]; A.tgt = fk(HOME); A.reach = true; } }],
  labs: [
    '블록을 클릭해 집고, 빈 곳을 클릭해 내려놓는다. <b>한 틱 실행</b>으로 각도 → 토크 → 가속 순서를 본다.',
    '중력 보상 "없음": 팔이 처져 집기에 실패한다. 제어 강성을 올리면 처짐이 줄어든다.',
    '감쇠비 0.1: 목표를 지나쳐 출렁인다. 그래프에서 오버슈트를 본다.',
    '블록을 든 채 짐 무게 3 kg + 토크 한계 5 N·m: 토크가 포화되어 들지 못한다.',
    '명시적 오일러 + 감쇠비 0: 에너지가 불어나 진동이 커진다. DT 1/10초에서는 기본 강성에서도 폭주한다.',
    '엔코더 노이즈를 올리면 속도 추정(각도 차이 ÷ dt)이 튀어 팔이 떤다. DT가 작을수록 심하다.',
  ],
  init, reset, step, draw, click,
  sample: () => [A.qt[1], A.q[1]],
  stages() {
    const miss = Math.hypot(A.tip[0] - A.tgt[0], A.tip[1] - A.tgt[1], A.tip[2] - A.tgt[2]);
    return [
      `엔코더: 회전 ${deg(A.qm[0])}° · 어깨 ${deg(A.qm[1])}° · 팔꿈치 ${deg(A.qm[2])}° · 어깨 속도 추정 ${f(A.qdm[1])} rad/s`,
      `${A.seq.length ? `이동 단계 ${A.seq.length}개 남음` : '대기'}${A.reach ? '' : ' · 도달 범위 밖'} → 목표 어깨 ${deg(A.qt[1])}° · 명령 토크 ${f(A.cmd[1], 1)} N·m`,
      `어깨 토크 ${f(A.tau[1], 1)} N·m (한계 ±${p.tmax})${A.sat ? ' · 포화' : ''} + 중력 ${f(A.tg[1], 1)} N·m → 각가속도 ${f(A.acc[1], 1)} rad/s²`,
      `끝점 (${f(A.tip[0])}, ${f(A.tip[1])}, ${f(A.tip[2])}) m · 목표와 ${f(miss * 100, 1)} cm${A.held ? ` · 짐 ${f(p.load, 1)} kg` : ''}`,
    ];
  },
  stat: () => A.note,
  test() { const pt = [0.3, 0.15, 0.2], t = fk(ik(...pt).q); return Math.hypot(t[0] - pt[0], t[1] - pt[1], t[2] - pt[2]) < 1e-9 && !ik(2, 0, 0).ok; },   // 역기구학 → 정기구학이 원래 점으로 돌아와야 한다
  get S() { return A; },
});
})();
