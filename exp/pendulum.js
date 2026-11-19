// 실험: 진자. 제어 없는 가장 단순한 물리계로 적분 방식과 DT가 결과를 어떻게 바꾸는지 본다. (단위: m, s, rad)
(() => {
'use strict';
const PIVOT = 2.3;
let p, S, V;
// 질량 1 kg 기준 역학적 에너지 = 운동 에너지 + 위치 에너지. 마찰이 없으면 변하지 않아야 한다
const energy = () => 0.5 * (p.len * S.om) ** 2 + G * p.len * (1 - Math.cos(S.th));

function reset() { S = { th: p.a0 * Math.PI / 180, om: 0, al: 0, e0: 1 }; S.e0 = energy() || 1; }
function step(dt) {
  // ① 감지: 각도와 각속도 (이 실험에는 센서 오차가 없다)
  mark(0);
  // ② 판단: 제어기가 없다. 물리 법칙만으로 움직인다
  mark(1);
  // ③ 행동: 중력과 마찰이 만드는 각가속도
  S.al = -G / p.len * Math.sin(S.th) - p.damp * S.om;
  mark(2);
  // ④ 적분: 명시적 오일러는 이전 각속도로 각도를 옮기고, 반암시적 오일러는 새 각속도로 옮긴다
  if (p.semi) { S.om += S.al * dt; S.th += S.om * dt; } else { S.th += S.om * dt; S.om += S.al * dt; }
  S.om = clamp(S.om, -200, 200);   // 폭주해도 숫자가 무한대로 가지 않게
  mark(3);
}

function init(k) {
  p = this.p;
  k.mesh(new THREE.CylinderGeometry(1.2, 1.2, 0.02, 48), k.col('--floor'), 0, -0.01, 0).castShadow = false;
  k.mesh(new THREE.BoxGeometry(0.06, PIVOT, 0.06), k.col('--wall'), 0, PIVOT / 2, -0.25);
  k.mesh(new THREE.BoxGeometry(0.06, 0.06, 0.3), k.col('--wall'), 0, PIVOT, -0.12);
  V = { arm: new THREE.Group() }; V.arm.position.y = PIVOT; k.g.add(V.arm);
  V.rod = k.mesh(new THREE.BoxGeometry(0.02, 1, 0.02), 0x16202a, 0, 0, 0, V.arm);
  V.bob = k.mesh(new THREE.SphereGeometry(0.09, 24, 16), 0xe5352b, 0, 0, 0, V.arm);
}
function draw() { V.arm.rotation.z = S.th; V.rod.scale.y = p.len; V.rod.position.y = -p.len / 2; V.bob.position.y = -p.len; }

const f = (v, d = 2) => v.toFixed(d);
EXPS.push({
  id: 'pendulum', name: '진자',
  hint: '드래그: 회전 · 휠: 확대 · 설정을 바꾼 뒤 "다시 놓기"',
  cam: { az: 0, el: 0.12, d: 5.2, t: [0, 1.2, 0] },
  graph: { title: '에너지 (회색: 처음 · 색: 현재)', min: 0, max: 3, marks: [[1, '처음 에너지'], [2, '2배']] },
  params: [
    { key: 'semi', label: '적분 방식', options: [[0, '명시적 오일러'], [1, '반암시적 오일러']], value: 0 },
    { key: 'a0', label: '시작 각도', min: 10, max: 175, step: 5, value: 60, fmt: v => `${v}°` },
    { key: 'len', label: '줄 길이', min: 0.3, max: 2, step: 0.1, value: 1, fmt: v => `${f(v, 1)} m` },
    { key: 'damp', label: '마찰(감쇠)', min: 0, max: 1, step: 0.05, value: 0, fmt: v => `${f(v)} /s` },
  ],
  buttons: [{ label: '다시 놓기', run: reset }],
  labs: [
    '명시적 오일러: 마찰이 없는데 에너지가 계속 늘어난다. 실제 물리가 아니라 적분 오차다.',
    '반암시적 오일러로 바꾸고 "다시 놓기": 같은 DT인데 에너지가 유지된다.',
    'DT를 1/240초로 줄이면 명시적 오일러의 오차도 줄어든다. 느려질 뿐 사라지지는 않는다.',
    '마찰을 올리면 에너지가 줄어들며 멈춘다. 이것은 오차가 아니라 물리다.',
  ],
  init, reset, step, draw,
  sample: () => [1, energy() / S.e0],
  stages: () => [
    `각도 ${f(S.th * 180 / Math.PI, 1)}° · 각속도 ${f(S.om)} rad/s`,
    '제어 없음 (자유 운동)',
    `각가속도 ${f(S.al)} rad/s² = −(g/L)·sin θ − 마찰`,
    `에너지 ${f(energy() / S.e0 * 100, 1)}% (처음 대비)`,
  ],
  stat: () => '',
  test() {   // 반암시적 오일러는 10초 뒤에도 에너지가 2% 안에서 유지되어야 한다
    const keep = [S, p.semi]; p.semi = 1; S = { th: Math.PI / 4, om: 0 }; S.e0 = energy();
    for (let i = 0; i < 2400; i++) { S.om += -G / p.len * Math.sin(S.th) / 240; S.th += S.om / 240; }
    const ok = Math.abs(energy() / S.e0 - 1) < 0.02;
    [S, p.semi] = keep;
    return ok;
  },
  get S() { return S; },
});
})();
