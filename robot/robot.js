// Hoya Botto: scroll-driven exploded view of the XLeRobot and the farm capabilities.
// World frame is MuJoCo's: z up, the robot faces -x, the robot's left arm is on -y.
// Everything on screen is a pure function of the scroll position s (chapter index + progress),
// except idle motion (belt, LiDAR, waving), so scrolling back replays scenes in reverse.
import * as THREE from 'three';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { toCreasedNormals, mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
window.__step?.('WebGL');

// If anything fails (old GPU, lost context, out of memory), drop the 3D and keep the story readable.
let failed = false;
function fail(msg) {
  if (failed) return; failed = true;
  console.warn('3D unavailable, switching to video:', msg);
  window.__startVideoMode?.();
}
addEventListener('error', e => fail(e.message || 'error'));
addEventListener('unhandledrejection', e => fail((e.reason && e.reason.message) || e.reason || 'error'));
setTimeout(() => { if (!document.getElementById('loading')?.classList.contains('done')) fail('読み込みが時間切れになりました'); }, 25000);
// Phones get a lighter renderer: standard materials, no environment pre-pass, no shadows.
const HQ = new URLSearchParams(location.search).has('hq');  // video capture: full quality at any size
const LITE = !HQ && (matchMedia('(max-width: 760px)').matches || /Android|iPhone|iPad|Mobile/i.test(navigator.userAgent));

// ---------------------------------------------------------------- helpers
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const seg = (t, a, b) => clamp((t - a) / (b - a), 0, 1);
const ease = t => t * t * (3 - 2 * t);
const E = (t, a, b) => ease(seg(t, a, b));
const lerp = (a, b, t) => a + (b - a) * t;
const lerp3 = (a, b, t) => [lerp(a[0], b[0], t), lerp(a[1], b[1], t), lerp(a[2], b[2], t)];
const V3 = (x = 0, y = 0, z = 0) => new THREE.Vector3(x, y, z);
const REDUCED = matchMedia('(prefers-reduced-motion: reduce)').matches;
const rand = (() => { let x = 1234567; return () => ((x = (x * 16807) % 2147483647) / 2147483647); })();

const BG = 0xf3eee7;
const TABLE_H = 0.70;
const ST = [V3(0, 0, 0), V3(0, 5, 0), V3(0, 10, 0), V3(0, 15, 0), V3(0, 20, 0)];
// Station frame (robot parked facing -x): forward f, left l, up u  ->  world.
const SP = (k, f, l, u) => V3(ST[k].x - f, ST[k].y - l, u);
const RP = (f, l, u) => V3(-f, -l, u); // robot frame -> robot root local

// ---------------------------------------------------------------- renderer, scene
const canvas = document.getElementById('stage');
let renderer;
try { renderer = new THREE.WebGLRenderer({ canvas, antialias: !LITE, powerPreference: LITE ? 'default' : 'high-performance' }); }
catch (e) { fail('WebGL: ' + e.message); throw e; }
canvas.addEventListener('webglcontextlost', ev => { ev.preventDefault(); fail('GPUの接続が切れました'); });
const SMALL = matchMedia('(max-width: 760px)').matches;
renderer.setPixelRatio(Math.min(devicePixelRatio, LITE ? 1.25 : 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.02;
renderer.shadowMap.enabled = !LITE;
renderer.shadowMap.type = THREE.PCFShadowMap;
renderer.localClippingEnabled = true;

const scene = new THREE.Scene();
scene.background = new THREE.Color(BG);
scene.fog = new THREE.Fog(BG, 9, 30);
if (!LITE) {
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  scene.environmentIntensity = 0.55;
}
scene.environmentRotation.x = Math.PI / 2;

const camera = new THREE.PerspectiveCamera(32, 1, 0.05, 220);
camera.up.set(0, 0, 1);

scene.add(new THREE.HemisphereLight(0xffffff, 0xd9cfc2, LITE ? 2.0 : 1.25));
const key = new THREE.DirectionalLight(0xffffff, 2.3);
key.castShadow = true;
key.shadow.mapSize.set(SMALL ? 1024 : 2048, SMALL ? 1024 : 2048);
Object.assign(key.shadow.camera, { left: -2.6, right: 2.6, top: 2.6, bottom: -2.6, near: 0.5, far: 20 });
key.shadow.bias = -0.0004; key.shadow.normalBias = 0.02;
scene.add(key, key.target);

const ground = new THREE.Mesh(new THREE.PlaneGeometry(400, 400), new THREE.MeshStandardMaterial({ color: 0xece4d8, roughness: 1 }));
ground.receiveShadow = true;
scene.add(ground);

// ---------------------------------------------------------------- materials
const M = (color, o = {}) => {
  if (!LITE) return new THREE.MeshPhysicalMaterial({ color, roughness: 0.5, metalness: 0, ...o });
  const { clearcoat, clearcoatRoughness, transmission, envMapIntensity, ...rest } = o;
  return new THREE.MeshStandardMaterial({ color, roughness: 0.5, metalness: 0, ...rest, metalness: Math.min(rest.metalness || 0, 0.2) });
};
const MAT = {
  orange: M(0xf88335, { roughness: 0.42, clearcoat: 0.35, clearcoatRoughness: 0.4 }),
  blue: M(0x2c86c7, { roughness: 0.42, clearcoat: 0.35, clearcoatRoughness: 0.4 }),
  motor: M(0x25282d, { roughness: 0.55, metalness: 0.15 }),
  cart: M(0xe7eaed, { roughness: 0.35, metalness: 0.45 }),
  wheel: M(0x3a3f45, { roughness: 0.8 }),
  head: M(0xf4f3f0, { roughness: 0.45, clearcoat: 0.2 }),
  oak: M(0x30353c, { roughness: 0.35, metalness: 0.3 }),
  pla: M(0xece6db, { roughness: 0.62 }),
  tbl: M(0xc6d4e2, { roughness: 0.55 }),
  plaGreen: M(0x8fc7a0, { roughness: 0.6 }),
  plaCream: M(0xf3ead7, { roughness: 0.6 }),
  plaBlue: M(0x9cc5e6, { roughness: 0.6 }),
  wood: M(0xf6f3ee, { roughness: 0.55 }),
  steel: M(0xa9b0b7, { roughness: 0.35, metalness: 0.7 }),
  dark: M(0x2b2f35, { roughness: 0.6 }),
  card: M(0xc8a06a, { roughness: 0.9 }),
  cardIn: M(0xb48b57, { roughness: 0.95, side: THREE.DoubleSide }),
  crate: M(0x2f7fc1, { roughness: 0.45 }),
  soil: M(0x6e5038, { roughness: 1 }),
  leaf: M(0x74c24b, { roughness: 0.65 }),
  pcbPurple: M(0x5b3f8f, { roughness: 0.5 }),
  pcbBlack: M(0x1d2126, { roughness: 0.45 }),
  shield: M(0xc7ccd1, { roughness: 0.3, metalness: 0.8 }),
  tape: M(0xd8b878, { roughness: 0.3, transparent: true, opacity: 0.88 }),
  belt: M(0x2a2d31, { roughness: 0.85 }),
  cupFilm: M(0xffffff, { roughness: 0.15, transmission: 0.6, transparent: true, opacity: 0.55 }),
  mekabu: M(0x3f4f22, { roughness: 0.4 }),
};
MAT.pick = (group, mat, part) => {
  if (part === 'tophead6') return MAT.oak;
  if (mat === 'motor') return MAT.motor;
  if (mat === 'grey') return MAT.wheel;
  if (mat === 'blue') return MAT.cart;
  if (group === 'armL') return MAT.orange;
  if (group === 'armR') return MAT.blue;
  return MAT.head;
};

// ---------------------------------------------------------------- asset loading
window.__step?.('モデル');
const meta = await (await fetch('robot/assets/robot.json')).json();
const bin = await (await fetch('robot/assets/robot.bin')).arrayBuffer();
const geoCache = new Map();
function geo(k, crease = 0.55) {
  if (geoCache.has(k)) return geoCache.get(k);
  const m = meta.meshes[k];
  const q = new Uint16Array(bin, m.v[0], m.v[1] * 3);
  const pos = new Float32Array(q.length);
  for (let i = 0; i < q.length; i++) pos[i] = m.lo[i % 3] + (q[i] / 65535) * m.span[i % 3];
  let g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setIndex(new THREE.BufferAttribute(new Uint16Array(bin, m.i[0], m.i[1]).slice(), 1));
  g = toCreasedNormals(g, crease);
  g.computeBoundingBox();
  geoCache.set(k, g);
  return g;
}
const propGeo = name => geo(meta.props[name].mesh, 0.7);
const propSize = name => meta.props[name].size;

// ---------------------------------------------------------------- robot rig
class Robot {
  constructor() {
    this.root = new THREE.Group();
    this.joints = {}; this.bodies = []; this.parts = [];
    meta.bodies.forEach(b => {
      const g = new THREE.Group(); g.position.fromArray(b.pos); g.quaternion.fromArray(b.quat);
      const pivot = new THREE.Group(); g.add(pivot);
      (b.parent < 0 ? this.root : this.bodies[b.parent].pivot).add(g);
      if (b.joint) this.joints[b.joint.name] = { pivot, axis: V3().fromArray(b.joint.axis), lo: b.joint.range[0], hi: b.joint.range[1], v: 0 };
      for (const gm of b.geoms) gm.pieces.forEach((k, piece) => {
        const mesh = new THREE.Mesh(geo(k), MAT.pick(b.group, gm.mat, gm.part));
        mesh.position.fromArray(gm.pos); mesh.quaternion.fromArray(gm.quat);
        mesh.castShadow = mesh.receiveShadow = true;
        pivot.add(mesh);
        this.parts.push({ mesh, pivot, body: b.name, group: b.group, part: gm.part, piece, base: mesh.position.clone(), off: V3(), delay: 0 });
      });
      this.bodies.push({ name: b.name, g, pivot, group: b.group });
    });
    this.by = Object.fromEntries(this.bodies.map(b => [b.name, b]));
    this.tip = {}; this.wrist = {};
    for (const [s, jaw, wr] of [['L', 'Fixed_Jaw', 'Wrist_Pitch_Roll'], ['R', 'Fixed_Jaw_2', 'Wrist_Pitch_Roll_2']]) {
      const t = new THREE.Object3D(); t.position.fromArray(meta.tip); this.by[jaw].pivot.add(t);
      this.tip[s] = t; this.wrist[s] = this.by[wr].g;
    }
  }
  set(name, v) { const j = this.joints[name]; j.v = clamp(v, j.lo, j.hi); j.pivot.quaternion.setFromAxisAngle(j.axis, j.v); }
  q() { return Object.fromEntries(Object.entries(this.joints).map(([k, j]) => [k, j.v])); }
  setQ(q) { for (const k in q) this.set(k, q[k]); }
}
const _p = V3(), _a = V3(), _e = V3(), _u = V3(), _w = V3(), _c = V3(), _q = new THREE.Quaternion();
const CHAIN = ['Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch'];
// Cyclic coordinate descent on the four positioning joints, matching two points: the claw tip and
// the wrist (tip minus approach direction), which also sets the approach angle.
function solveArm(r, side, tipT, wristT, iters) {
  r.root.updateMatrixWorld(true);
  const effs = [[r.tip[side], tipT, 1], [r.wrist[side], wristT, 0.6]];
  for (let it = 0; it < iters; it++) {
    for (let k = CHAIN.length - 1; k >= 0; k--) {
      const name = `${CHAIN[k]}_${side}`, j = r.joints[name];
      j.pivot.getWorldPosition(_p); j.pivot.getWorldQuaternion(_q);
      _a.copy(j.axis).applyQuaternion(_q).normalize();
      let num = 0, den = 0;
      for (const [obj, tgt, w] of effs) {
        obj.getWorldPosition(_e);
        _u.subVectors(_e, _p); _u.addScaledVector(_a, -_u.dot(_a));
        _w.subVectors(tgt, _p); _w.addScaledVector(_a, -_w.dot(_a));
        num += w * _a.dot(_c.crossVectors(_u, _w)); den += w * _u.dot(_w);
      }
      r.set(name, j.v + clamp(Math.atan2(num, den), -0.3, 0.3));
      j.pivot.updateMatrixWorld(true);
    }
  }
  r.tip[side].getWorldPosition(_e);
  return _e.distanceTo(tipT);
}

window.__step?.('ロボット');
const robot = new Robot();
scene.add(robot.root);
let WRIST_LEN = 0.16;
{ // wrist-to-tip distance, for turning an approach direction into a wrist target
  robot.root.updateMatrixWorld(true);
  WRIST_LEN = robot.tip.L.getWorldPosition(V3()).distanceTo(robot.wrist.L.getWorldPosition(V3()));
}

// Arm pose spec in the robot frame: tip [f,l,u], approach dir [f,l,u], wrist roll, jaw.
const JAW_OPEN = 0.55, JAW_SHUT = -0.3, JAW_REST = -0.3;
const HOME = {
  L: { tip: [0.34, 0.19, 0.83], dir: [0.75, 0.0, -0.66], roll: 0, jaw: JAW_REST },
  R: { tip: [0.34, -0.19, 0.83], dir: [0.75, 0.0, -0.66], roll: 0, jaw: JAW_REST },
};
const ROLL = { L: 'Wrist_Roll_L', R: 'Wrist_Roll_R' }, JAW = { L: 'Jaw_L', R: 'Jaw_R' };
const _tt = V3(), _wt = V3(), _dd = V3();
function poseArm(side, spec, iters = 10) {
  robot.root.updateMatrixWorld(true);
  _tt.copy(RP(...spec.tip)); robot.root.localToWorld(_tt);
  _dd.copy(RP(...spec.dir)).normalize().applyQuaternion(robot.root.quaternion);
  _wt.copy(_tt).addScaledVector(_dd, -WRIST_LEN);
  robot.set(ROLL[side], spec.roll ?? 0);
  robot.set(JAW[side], spec.jaw ?? JAW_REST);
  let err = solveArm(robot, side, _tt, _wt, iters);
  if (err > 0.01) err = solveArm(robot, side, _tt, _wt, 40);
  return err;
}
robot.setQ({ Pitch_L: 1.2, Elbow_L: 1.4, Pitch_R: 1.2, Elbow_R: 1.4 });
for (let i = 0; i < 6; i++) { poseArm('L', HOME.L, 30); poseArm('R', HOME.R, 30); }
const HOME_Q = robot.q();
const headLink = robot.by.head_pan_link.g;

function look(worldPoint) {
  robot.root.updateMatrixWorld(true);
  const h = headLink.getWorldPosition(V3());
  const v = robot.root.worldToLocal(worldPoint.clone()).sub(robot.root.worldToLocal(h.clone()));
  robot.set('head_pan_joint', Math.atan2(-v.y, -v.x));
  robot.set('head_tilt_joint', Math.atan2(v.z, Math.hypot(v.x, v.y)));
}
function head(pan, tilt) { robot.set('head_pan_joint', pan); robot.set('head_tilt_joint', tilt); }

// ---------------------------------------------------------------- exploded view
robot.root.updateMatrixWorld(true);
{
  const centroid = p => { const c = V3(); p.mesh.geometry.boundingBox.getCenter(c); return p.mesh.localToWorld(c); };
  const DEPTH = { Base: 1, Rotation_Pitch: 2, Upper_Arm: 3, Lower_Arm: 4, Wrist_Pitch_Roll: 5, Fixed_Jaw: 6, Moving_Jaw: 7, Right_Arm_Camera: 7, Left_Arm_Camera: 7 };
  const first = {};
  for (const p of robot.parts) { p.c = centroid(p); if (p.piece === 0) first[p.body + '|' + p.part] = p.c; }
  const trays = robot.parts.filter(p => p.part === 'raskogbody' && p.piece > 0).sort((a, b) => b.c.z - a.c.z);
  // base plate, tower, head mount: [forward(-x), up, delay]
  const TOWER = [[0, 0.08, 0.06], [-0.0, 0.2, 0.12], [0, 0.31, 0.18]];
  const HEADZ = { topbase2: [0.05, 0.37, 0.2], tophead1: [0, 0.45, 0.24], tophead4: [0.06, 0.5, 0.27], tophead5: [0, 0.57, 0.3], tophead6: [-0.13, 0.63, 0.33] };
  for (const p of robot.parts) {
    const w = V3();
    if (p.part === 'raskogbody') { if (p.piece > 0) { const r = trays.indexOf(p); w.set(-0.42 - 0.1 * r, 0, 0.0); p.delay = 0.04 + 0.05 * r; } }
    else if (p.part === 'raskogwheel1' || p.part === 'raskogwheel2') { w.set(p.c.x, p.c.y, 0).normalize().multiplyScalar(p.part === 'raskogwheel1' ? 0.17 : 0.27); p.delay = 0.1; }
    else if (p.part === 'topbase1') { const t = TOWER[p.piece] || TOWER[1]; w.set(-t[0], 0, t[1]); p.delay = t[2]; }
    else if (p.group === 'head') { const h = HEADZ[p.part] || [0, 0.4, 0.22]; w.set(-h[0], 0, h[1]); p.delay = h[2]; }
    else {
      const name = p.body.replace(/_2$/, ''), d = DEPTH[name] || 1, sy = Math.sign(p.c.y);
      w.set(-0.02 * d, sy * (0.06 + 0.075 * d), 0.16 + 0.012 * d);
      if (p.mesh.material === MAT.motor) w.add(V3(-0.075, 0, 0.035));
      else if (p.piece > 0) w.add(V3().subVectors(p.c, first[p.body + '|' + p.part]).normalize().multiplyScalar(0.045));
      p.delay = 0.1 + 0.045 * d;
    }
    const q = p.pivot.getWorldQuaternion(new THREE.Quaternion()).invert();
    p.off.copy(w).applyQuaternion(q);
  }
}
function explode(e) {
  for (const p of robot.parts) p.mesh.position.copy(p.base).addScaledVector(p.off, E(e, p.delay, p.delay + 0.5));
}

// ---------------------------------------------------------------- labels
const labelLayer = document.getElementById('labels');
const labels = [];
function label(title, sub, anchor, cls = '') {
  const el = document.createElement('div');
  el.className = 'label ' + cls;
  el.innerHTML = `<i></i><span>${title}${sub ? `<small>${sub}</small>` : ''}</span>`;
  labelLayer.appendChild(el);
  const L = { el, anchor, on: false }; labels.push(L); return L;
}
const partBy = (part, group) => robot.parts.find(p => p.part === part && (!group || p.group === group));
const meshAnchor = (p, dx = 0, dy = 0, dz = 0) => () => { const c = V3(); p.mesh.geometry.boundingBox.getCenter(c); return p.mesh.localToWorld(c).add(V3(dx, dy, dz)); };
const EXPLODE_LABELS = [
  label('IKEA RÅSKOG カート', '3段のトレーも外れる', meshAnchor(partBy('raskogbody'), 0.05, -0.24, -0.15)),
  label('車輪とキャスター', '駆動輪はSTS3215 ×2', meshAnchor(robot.parts.find(p => p.part === 'raskogwheel2'), 0, 0, 0.03)),
  label('3Dプリント：ベース板', 'アームと首を載せる', meshAnchor(robot.parts.find(p => p.part === 'topbase1' && p.piece === 0), 0, -0.2, 0)),
  label('3Dプリント：タワー', '頭を高く持ち上げる柱', meshAnchor(robot.parts.find(p => p.part === 'topbase1' && p.piece === 1), 0, 0.03, 0)),
  label('SO-101 アーム ×2', '部品はすべて3Dプリント', meshAnchor(partBy('Upper_Arm', 'armL'), 0, 0, 0.04)),
  label('STS3215 サーボ', '全部で16個', meshAnchor(partBy('Lower_Arm_Motor', 'armR'), 0, 0, 0.03)),
  label('手首カメラ ×2', 'USBカメラ', meshAnchor(partBy('XLeRobot_camera1', 'armL'), 0, 0, 0.03)),
  label('OAK-D Lite', '深度カメラ', meshAnchor(partBy('tophead6'), -0.02, 0, 0.03)),
  label('首（パン・チルト）', 'サーボ2個', meshAnchor(partBy('tophead1'), 0, 0.03, 0)),
];

// ---------------------------------------------------------------- scenery helpers
const shadowed = o => { o.traverse(c => { if (c.isMesh) { c.castShadow = true; c.receiveShadow = true; } }); return o; };
const box = (sx, sy, sz, mat, r = 0.004) => new THREE.Mesh(r ? new RoundedBoxGeometry(sx, sy, sz, 2, Math.min(r, sx / 2, sy / 2, sz / 2)) : new THREE.BoxGeometry(sx, sy, sz), mat);
const at = (o, v, rz = 0) => { o.position.copy(v); o.rotation.z = rz; return o; };
const prop = (name, mat) => { const m = new THREE.Mesh(propGeo(name), mat); m.castShadow = m.receiveShadow = true; return m; };

function makeTable(lw, fd, h, topMat, legMat) {
  const g = new THREE.Group();
  const top = box(fd, lw, 0.025, topMat, 0.006); top.position.z = h - 0.0125; g.add(top);
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) { const leg = box(0.03, 0.03, h - 0.025, legMat, 0.004); leg.position.set(sx * (fd / 2 - 0.03), sy * (lw / 2 - 0.03), (h - 0.025) / 2); g.add(leg); }
  return shadowed(g);
}
function cressGeometry(w, d, n, seed = 1, lo = false) {
  let x = seed * 9973; const r = () => ((x = (x * 16807) % 2147483647) / 2147483647);
  const parts = [];
  for (let i = 0; i < n; i++) {
    const h = 0.02 + r() * 0.03, px = (r() - 0.5) * w, py = (r() - 0.5) * d;
    const stem = new THREE.CylinderGeometry(0.0011, 0.0013, h, lo ? 3 : 4, 1); stem.rotateX(Math.PI / 2); stem.translate(px, py, h / 2);
    parts.push(stem);
    for (const s of [-1, 1]) {
      const leaf = new THREE.SphereGeometry(0.006, lo ? 4 : 6, lo ? 3 : 4); leaf.scale(1.3, 0.75, 0.28);
      const a = r() * Math.PI; leaf.rotateZ(a); leaf.translate(px + s * 0.005 * Math.cos(a), py + s * 0.005 * Math.sin(a), h + 0.001);
      parts.push(leaf);
    }
  }
  const g = mergeGeometries(parts.map(p => { p.deleteAttribute('uv'); return p.index ? p.toNonIndexed() : p; }));
  g.computeVertexNormals();
  return g;
}
const CRESS = cressGeometry(0.12, 0.072, 46);
const CRESS_LO = cressGeometry(0.13, 0.064, 18, 2, true);
function tagMesh() { // raised cells (printed in black) above 1.0 mm; base tile white
  const g = propGeo('tag').clone(); const p = g.attributes.position, col = new Float32Array(p.count * 3);
  for (let i = 0; i < p.count; i++) { const k = p.getZ(i) > 0.00105 ? 0.06 : 0.95; col.set([k, k, k], i * 3); }
  g.setAttribute('color', new THREE.BufferAttribute(col, 3));
  const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.6 })); m.castShadow = m.receiveShadow = true; return m;
}
function frustum(apex, dir, up, hfov, vfov, len, color) {
  const g = new THREE.Group();
  const d = dir.clone().normalize(), r = V3().crossVectors(d, up).normalize(), u = V3().crossVectors(r, d).normalize();
  const w = Math.tan(hfov / 2) * len, h = Math.tan(vfov / 2) * len, c = apex.clone().addScaledVector(d, len);
  const corners = [[1, 1], [-1, 1], [-1, -1], [1, -1]].map(([a, b]) => c.clone().addScaledVector(r, a * w).addScaledVector(u, b * h));
  const pos = []; for (let i = 0; i < 4; i++) { const p = corners[i], q = corners[(i + 1) % 4]; pos.push(...apex.toArray(), ...p.toArray(), ...q.toArray()); }
  const fg = new THREE.BufferGeometry(); fg.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  const faces = new THREE.Mesh(fg, new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.13, side: THREE.DoubleSide, depthWrite: false }));
  const lp = []; for (const p of corners) lp.push(...apex.toArray(), ...p.toArray()); for (let i = 0; i < 4; i++) lp.push(...corners[i].toArray(), ...corners[(i + 1) % 4].toArray());
  const lg = new THREE.BufferGeometry(); lg.setAttribute('position', new THREE.Float32BufferAttribute(lp, 3));
  const lines = new THREE.LineSegments(lg, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.7 }));
  g.add(faces, lines); g.userData.mats = [faces.material, lines.material]; g.userData.base = [0.13, 0.7];
  return g;
}
const fade = (g, k) => { g.visible = k > 0.01; g.userData.mats.forEach((m, i) => { m.opacity = g.userData.base[i] * k; }); };
const show = (o, k) => { o.visible = k > 0.001; o.scale.setScalar(Math.max(k, 0.001)); };

// ---------------------------------------------------------------- G4 shared-funnel planter
// Deck: r3-autonomous-planter/original-gravity-v4. Assembly frame in metres: table surface z = 0,
// slots at x = -45, -15, 15, 45 mm, robot on the -y side; robot frame: f = F0 + y, l = -x.
const G4C = { trough: M(0x306b85, { roughness: 0.55 }), holder: M(0xdb9b43, { roughness: 0.55 }), carrier: M(0x515d78, { roughness: 0.55 }),
  guide: M(0xb96932, { roughness: 0.55 }), tool: M(0x526787, { roughness: 0.5 }), wheel: M(0x427a84, { roughness: 0.4 }),
  paper: M(0xeee1b9, { roughness: 0.9 }), sheet: M(0xfff9e9, { roughness: 0.95 }), water: M(0x45a4d5, { roughness: 0.1, transparent: true, opacity: 0.6 }) };
const G4Z = 0.0235, G4XS = [-0.045, -0.015, 0.015, 0.045];
const g4mesh = (name, mat) => { const m = new THREE.Mesh(propGeo(name), mat); m.castShadow = m.receiveShadow = true; return m; };
const SHEET_CRESS = cressGeometry(0.13, 0.064, 70, 3);
function g4Assembled() {
  const g = new THREE.Group(), add = (o, x = 0, y = 0, z = 0) => { o.position.set(x, y, z); g.add(o); return o; };
  add(g4mesh('g4_trough', G4C.trough), 0, 0, G4Z); add(g4mesh('g4_holder', G4C.holder), 0, 0, G4Z);
  for (const x of G4XS) { add(g4mesh('g4_carrier', G4C.carrier), x, 0, G4Z); add(g4mesh('g4_fold' + Math.round(x * 1000), G4C.paper), 0, 0, G4Z); }
  add(box(0.146, 0.076, 0.0006, G4C.sheet, 0), 0, 0, G4Z + 0.001);
  const cr = new THREE.Mesh(SHEET_CRESS, MAT.leaf); cr.castShadow = true; add(cr, 0, 0, G4Z + 0.0013);
  return shadowed(g);
}

window.__step?.('シーン');
// ---------------------------------------------------------------- station 1: sensors & measuring
const S1 = new THREE.Group(); scene.add(S1);
const T1 = makeTable(0.5, 0.48, TABLE_H, MAT.wood, MAT.steel); at(T1, SP(1, 0.22 + 0.24, 0, 0)); S1.add(T1);
const planter1 = g4Assembled(); at(planter1, SP(1, 0.47, 0.1, TABLE_H), Math.PI / 2); S1.add(planter1);
const PADDLE_REST = SP(1, 0.355, -0.15, TABLE_H);
const paddle = new THREE.Group();
{ const m = prop('paddle', M(0xf2c14e, { roughness: 0.55 })); paddle.add(m); const chip = box(0.019, 0.013, 0.003, MAT.pcbPurple, 0.001); chip.position.set(-0.055, 0, 0.0085); paddle.add(chip); shadowed(paddle); }
at(paddle, PADDLE_REST); S1.add(paddle);
const esp = new THREE.Group();
{ const b = box(0.052, 0.028, 0.004, MAT.pcbBlack, 0.001); b.position.z = 0.002; const s = box(0.018, 0.016, 0.003, MAT.shield, 0.0005); s.position.set(-0.012, 0, 0.0055); esp.add(b, s); shadowed(esp); }
at(esp, SP(1, 0.6, -0.12, TABLE_H)); S1.add(esp);
{ const curve = new THREE.CatmullRomCurve3([SP(1, 0.6, -0.15, TABLE_H + 0.004), SP(1, 0.66, -0.2, TABLE_H + 0.004), SP(1, 0.69, -0.24, TABLE_H - 0.05), SP(1, 0.66, -0.25, 0.25), SP(1, 0.5, -0.3, 0.01)]);
  const cable = new THREE.Mesh(new THREE.TubeGeometry(curve, 40, 0.0025, 6), MAT.dark); cable.castShadow = true; S1.add(cable); }
const tags1 = [SP(1, 0.27, 0.2, TABLE_H), SP(1, 0.64, -0.21, TABLE_H), SP(1, 0.62, 0.21, TABLE_H)].map(p => { const t = tagMesh(); at(t, p); S1.add(t); return t; });
const tagBoxes = tags1.map(t => { const s = 0.026; const g = new THREE.BufferGeometry().setFromPoints([V3(-s, -s, 0), V3(s, -s, 0), V3(s, s, 0), V3(-s, s, 0)]);
  const l = new THREE.LineLoop(g, new THREE.LineBasicMaterial({ color: 0x19b36b, transparent: true, opacity: 0 })); l.position.copy(t.position).add(V3(0, 0, 0.004)); S1.add(l); return l; });
// phone overview camera on a tripod, on the robot's right
const phone = new THREE.Group();
{ const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.85, 10).rotateX(Math.PI / 2), MAT.dark); pole.position.z = 0.67; phone.add(pole);
  for (let i = 0; i < 3; i++) { const a = i * 2.094 + 0.4, top = V3(0, 0, 0.3), foot = V3(Math.cos(a) * 0.2, Math.sin(a) * 0.2, 0.005);
    const curve = new THREE.LineCurve3(foot, top); phone.add(new THREE.Mesh(new THREE.TubeGeometry(curve, 1, 0.006, 8), MAT.dark)); }
  const clamp1 = box(0.02, 0.09, 0.02, MAT.dark, 0.004); clamp1.position.set(0.006, 0, 1.1); phone.add(clamp1);
  const body = box(0.009, 0.075, 0.155, MAT.dark, 0.006); body.position.set(0, 0, 1.17); phone.add(body);
  const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.004, 16).rotateZ(Math.PI / 2), MAT.shield); lens.position.set(-0.006, 0.02, 1.22); phone.add(lens);
  shadowed(phone); }
phone.position.copy(SP(1, 0.25, -0.8, 0)); phone.rotation.z = -0.95; S1.add(phone);
const phoneFrustum = frustum(V3(-0.01, 0.02, 1.2), V3(-1, 0, -0.55), V3(0, 0, 1), 0.75, 1.1, 0.55, 0x7a5cc7); phone.add(phoneFrustum);
// Joy-Con pair, waiting at the table edge
const joycons = new THREE.Group();
[[0xff4554, 0.035], [0x00a7e1, -0.035]].forEach(([c, dy]) => { const j = box(0.035, 0.102, 0.03, M(c, { roughness: 0.35 }), 0.012); j.position.set(0, dy, 0.015); joycons.add(j);
  const stick = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.008, 0.008, 16).rotateX(Math.PI / 2), MAT.dark); stick.position.set(0, dy + (dy > 0 ? -0.02 : 0.02), 0.033); joycons.add(stick); });
shadowed(joycons); at(joycons, SP(1, 0.29, -0.03, TABLE_H), 0.25); S1.add(joycons);
// in the cart only: SHT31 air sensor and the screw-terminal moisture probe (drawn translucent)
const cartOnly = new THREE.Group();
{ const ghost = c => M(c, { roughness: 0.5, transparent: true, opacity: 0.45 });
  const sht = box(0.02, 0.016, 0.0016, ghost(0x3355aa), 0.0006); sht.position.set(0, 0, 0.001); cartOnly.add(sht);
  const probe = box(0.062, 0.023, 0.0016, ghost(0x2f7d3a), 0.0006); probe.position.set(0.0, 0.05, 0.001); cartOnly.add(probe); }
at(cartOnly, SP(1, 0.56, 0.0, TABLE_H)); S1.add(cartOnly);
// depth point cloud from the head camera, swept in from the front
const cloud = (() => {
  const pts = [], cols = [], c = new THREE.Color();
  for (let f = 0.23; f <= 0.69; f += 0.009) for (let l = -0.24; l <= 0.24; l += 0.009) {
    let z = TABLE_H + 0.001 + (rand() - 0.5) * 0.002;
    if (Math.abs(f - 0.47) < 0.04 && Math.abs(l - 0.11) < 0.088) z = TABLE_H + 0.026 + rand() * 0.045;
    else if (Math.abs(f - 0.47) < 0.044 && Math.abs(l - 0.11) < 0.092) z = TABLE_H + 0.04;
    if (Math.abs(f - 0.6) < 0.026 && Math.abs(l + 0.12) < 0.014) z = TABLE_H + 0.006;
    const p = SP(1, f, l, z + 0.002); pts.push(p.x, p.y, p.z);
    c.setHSL(lerp(0.58, 0.04, clamp((z - TABLE_H) / 0.07, 0, 1)), 0.75, 0.52); cols.push(c.r, c.g, c.b);
  }
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3)); g.setAttribute('color', new THREE.Float32BufferAttribute(cols, 3));
  const m = new THREE.Points(g, new THREE.PointsMaterial({ size: 0.0065, vertexColors: true, transparent: true, opacity: 0.95 }));
  m.userData.n = pts.length / 3; S1.add(m); return m;
})();

// LiDAR (designed rim mount + STL-19P puck), on the cart's back rim
const lidar = new THREE.Group();
{ const mount = prop('lidar_mount', MAT.pla); lidar.add(mount);
  const puck = box(0.0386, 0.0386, 0.022, MAT.dark, 0.003); puck.position.z = 0.035 + 0.011; lidar.add(puck);
  const cap = new THREE.Mesh(new THREE.CylinderGeometry(0.017, 0.017, 0.013, 24).rotateX(Math.PI / 2), MAT.motor); cap.position.z = 0.035 + 0.0285; lidar.add(cap);
  shadowed(lidar); }
lidar.position.set(0.185, 0, 0.775); robot.root.add(lidar);
const scan = (() => {
  const n = 540, pos = new Float32Array(n * 3), col = new Float32Array(n * 3);
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3)); g.setAttribute('color', new THREE.BufferAttribute(col, 3));
  const m = new THREE.Points(g, new THREE.PointsMaterial({ size: 0.02, vertexColors: true, transparent: true, opacity: 0.9, depthWrite: false }));
  m.userData.r = Array.from({ length: n }, (_, i) => 1.4 + 0.5 * Math.sin(i * 0.05) + 0.4 * Math.sin(i * 0.013 + 1) + rand() * 0.05);
  m.position.set(0.185, 0, 0.775 + 0.06); robot.root.add(m); return m;
})();
function updateScan(time, k) {
  scan.visible = k > 0.01; if (!scan.visible) return;
  const n = scan.userData.r.length, pos = scan.geometry.attributes.position.array, col = scan.geometry.attributes.color.array;
  const sweep = (time * 2.2) % (Math.PI * 2);
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2, r = scan.userData.r[i];
    pos[i * 3] = Math.cos(a) * r; pos[i * 3 + 1] = Math.sin(a) * r; pos[i * 3 + 2] = 0;
    const age = ((sweep - a) % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2), b = Math.exp(-age * 0.8);
    col[i * 3] = 0.95; col[i * 3 + 1] = 0.35 + 0.45 * b; col[i * 3 + 2] = 0.15 + 0.2 * b;
  }
  scan.geometry.attributes.position.needsUpdate = scan.geometry.attributes.color.needsUpdate = true;
  scan.material.opacity = 0.9 * k;
}

// frustums
robot.root.updateMatrixWorld(true);
const oakFrustum = (() => {
  const pivot = robot.by.head_tilt_link.pivot, oakPart = partBy('tophead6');
  const c = V3(); oakPart.mesh.geometry.boundingBox.getCenter(c); oakPart.mesh.updateMatrix(); c.applyMatrix4(oakPart.mesh.matrix);
  const f = frustum(c.clone().add(V3(-0.012, 0, 0)), V3(-1, 0, 0), V3(0, 0, 1), THREE.MathUtils.degToRad(69), THREE.MathUtils.degToRad(54), 0.95, 0xf88335);
  pivot.add(f); return f;
})();
const wristFrustums = ['L', 'R'].map(s => {
  const camBody = robot.by[s === 'L' ? 'Right_Arm_Camera' : 'Left_Arm_Camera'], jaw = robot.by[s === 'L' ? 'Fixed_Jaw' : 'Fixed_Jaw_2'].pivot;
  const camPart = robot.parts.find(p => p.body === camBody.name && p.part === 'XLeRobot_camera2');
  const c = V3(); camPart.mesh.geometry.boundingBox.getCenter(c); camPart.mesh.localToWorld(c); jaw.worldToLocal(c);
  const tipLocal = V3().fromArray(meta.tip);
  const f = frustum(c, V3().subVectors(tipLocal, c), V3(0, 0, 1), 1.0, 0.8, 0.2, 0x2c86c7); jaw.add(f); return f;
});

// ---------------------------------------------------------------- station 2: belt printers, modular table, G4 planter
const S2 = new THREE.Group(); scene.add(S2);
const W2 = (f, l, u) => SP(2, f, l, u);
const TILE = propSize('table_top')[0], TILE_T = propSize('table_top')[2];
// Modular Table (100% printed): node pitch about 142.6 mm with 80 mm rods; 3 bays along l, 2 along f.
const MT = { P: 0.14257, nx: 3, ny: 2, f0: 0.215 };
const nodeL = i => (i - MT.nx / 2) * MT.P, nodeF = j => MT.f0 + j * MT.P;
const TOP_Z = TABLE_H - TILE_T;
const ROD_LEN = propSize('tbl_rod_80mm')[2];
// legs: uprights at every perimeter node, rails at the bottom and halfway (shown as a fast-forward)
const legs = (() => {
  const items = [];  // {kind, pos, rot, scale, z}
  const LV = [0.02, 0.1626, 0.3052, 0.4478, 0.5904], TOPN = TOP_Z - 0.019;
  const perim = [];
  for (let i = 0; i <= MT.nx; i++) for (let j = 0; j <= MT.ny; j++) if (i === 0 || j === 0 || i === MT.nx || j === MT.ny) perim.push([i, j]);
  for (const [i, j] of perim) {
    const f = nodeF(j), l = nodeL(i), corner = (i === 0 || i === MT.nx) && (j === 0 || j === MT.ny);
    const zs = [...LV, TOPN];
    zs.forEach((z, k) => { if (k === 0 || k === 2) items.push({ kind: k === 0 && corner ? 'tbl_bottom_corner' : 'tbl_leg_t', f, l, z, rz: (i === 0 || i === MT.nx) ? Math.PI / 2 : 0 }); });
    for (let k = 0; k < zs.length - 1; k++) { const len = zs[k + 1] - zs[k]; items.push({ kind: 'tbl_rod_80mm', f, l, z: (zs[k] + zs[k + 1]) / 2, sz: Math.max(len - 0.03, 0.02) / ROD_LEN }); }
  }
  for (const z of [0.02, 0.3052]) {
    for (let j of [0, MT.ny]) for (let i = 0; i < MT.nx; i++) items.push({ kind: 'tbl_rod_80mm', f: nodeF(j), l: (nodeL(i) + nodeL(i + 1)) / 2, z, rx: Math.PI / 2 });
    for (let i of [0, MT.nx]) for (let j = 0; j < MT.ny; j++) items.push({ kind: 'tbl_rod_80mm', f: (nodeF(j) + nodeF(j + 1)) / 2, l: nodeL(i), z, ry: Math.PI / 2 });
  }
  const byKind = {};
  items.forEach(it => (byKind[it.kind] ||= []).push(it));
  const ims = Object.entries(byKind).map(([kind, list]) => {
    const im = new THREE.InstancedMesh(propGeo(kind), MAT.tbl, list.length); im.castShadow = im.receiveShadow = true; im.frustumCulled = false; S2.add(im);
    return { im, list };
  });
  const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), e = new THREE.Euler(), sc = V3();
  function update(k) {
    for (const { im, list } of ims) list.forEach((it, n) => {
      const a = E(k, it.z / 0.75 * 0.8, it.z / 0.75 * 0.8 + 0.2);
      e.set(it.rx || 0, it.ry || 0, it.rz || 0); q.setFromEuler(e); sc.set(a, a, a * (it.sz || 1));
      im.setMatrixAt(n, m4.compose(W2(it.f, it.l, it.z), q, sc.x < 1e-3 ? sc.setScalar(1e-4) : sc));
    });
    ims.forEach(({ im }) => { im.instanceMatrix.needsUpdate = true; im.visible = k > 0.001; });
  }
  return { update };
})();
// top lattice and tiles: placed by the robot, one print at a time from the belt printers
const topParts = [];
function addTop(kind, f, l, z, rot, top) {
  const m = new THREE.Mesh(propGeo(kind), MAT.tbl); m.castShadow = m.receiveShadow = true; S2.add(m);
  topParts.push({ m, kind, final: [f, l, z], rot, top });
}
for (let j = 0; j <= MT.ny; j++) for (let i = 0; i <= MT.nx; i++) {
  const corner = (i === 0 || i === MT.nx) && (j === 0 || j === MT.ny);
  if (corner) { const cf = nodeF(MT.ny / 2), a = Math.atan2(-(0 - nodeL(i)), -(cf - nodeF(j))) - Math.PI / 4; addTop('tbl_top_corner', nodeF(j), nodeL(i), TOP_Z - 0.0192, [0, 0, a], 0.0192); }
  else addTop('tbl_top_cross', nodeF(j), nodeL(i), TOP_Z - 0.0071, [Math.PI / 2, 0, 0], 0.0071);
}
for (let j = 0; j <= MT.ny; j++) for (let i = 0; i < MT.nx; i++) addTop('tbl_rod_80mm', nodeF(j), (nodeL(i) + nodeL(i + 1)) / 2, TOP_Z - 0.006, [Math.PI / 2, 0, 0], 0.006);
for (let i = 0; i <= MT.nx; i++) for (let j = 0; j < MT.ny; j++) addTop('tbl_rod_80mm', (nodeF(j) + nodeF(j + 1)) / 2, nodeL(i), TOP_Z - 0.006, [0, Math.PI / 2, 0], 0.006);
for (let j = MT.ny - 1; j >= 0; j--) for (let i = 0; i < MT.nx; i++) addTop('table_top', nodeF(j) + MT.P / 2, (nodeL(i) + nodeL(i + 1)) / 2, TOP_Z, [0, 0, 0], TILE_T);
// assign each print to the arm on its side (the centre line alternates), far parts first within each type
const JOBS = { L: [], R: [] };
{ let alt = 0;
  const order = ['tbl_top_corner', 'tbl_top_cross', 'tbl_rod_80mm', 'table_top'];
  topParts.sort((a, b) => order.indexOf(a.kind) - order.indexOf(b.kind) || (a.kind === 'table_top' ? 0 : b.final[0] - a.final[0]));
  for (const p of topParts) { const side = p.final[1] > 0.001 ? 'L' : p.final[1] < -0.001 ? 'R' : (alt++ % 2 ? 'R' : 'L'); p.side = side; JOBS[side].push(p); }
  for (const side of ['L', 'R']) { const n = JOBS[side].length, t0 = 0.3 + (side === 'R' ? 0.012 : 0), span = 0.665 / n;
    JOBS[side].forEach((p, k) => { p.a = t0 + k * span; p.b = p.a + span; }); }
}
// belt printers (Creality CR-30 "PrintMill" style: 45-degree gantry over a moving belt), one per arm
const BELT_V = 4.5, BELT_PICK_F = 0.3, BELT_START_F = 0.56, PRINTER_L = 0.46;
function makeBeltPrinter(sgn) {
  const g = new THREE.Group(), cl = sgn * PRINTER_L, top = TABLE_H - 0.06;
  const stand = makeTable(0.25, 0.52, top, MAT.wood, MAT.steel); at(stand, W2(0.44, cl, 0)); g.add(stand);
  const base = box(0.47, 0.21, 0.05, MAT.dark, 0.008); at(base, W2(0.44, cl, top + 0.025)); g.add(base);
  const tex = (() => { const c = document.createElement('canvas'); c.width = 64; c.height = 16; const x = c.getContext('2d'); x.fillStyle = '#202326'; x.fillRect(0, 0, 64, 16); x.fillStyle = '#34383d'; x.fillRect(0, 0, 6, 16);
    const t = new THREE.CanvasTexture(c); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(14, 1); t.colorSpace = THREE.SRGBColorSpace; return t; })();
  const belt = new THREE.Mesh(new THREE.BoxGeometry(0.45, 0.17, 0.008), new THREE.MeshStandardMaterial({ map: tex, roughness: 0.7 })); belt.receiveShadow = true;
  at(belt, W2(0.44, cl, TABLE_H - 0.004)); g.add(belt);
  for (const f of [0.215, 0.665]) { const r = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.18, 20), MAT.steel); r.position.copy(W2(f, cl, TABLE_H - 0.012)); g.add(r); }
  for (const s2 of [-1, 1]) { const side = box(0.13, 0.014, 0.3, MAT.dark, 0.004); at(side, W2(0.6, cl + s2 * 0.105, TABLE_H + 0.15)); g.add(side); }
  const cross = box(0.04, 0.224, 0.04, MAT.dark, 0.006); at(cross, W2(0.63, cl, TABLE_H + 0.29)); g.add(cross);
  const beam = box(0.03, 0.21, 0.03, MAT.steel, 0.004); at(beam, W2(0.6, cl, TABLE_H + 0.1)); beam.rotation.y = Math.PI / 4; g.add(beam);
  const head = new THREE.Group(); head.position.copy(W2(0.575, cl, TABLE_H + 0.06)); head.rotation.y = Math.PI / 4; g.add(head);
  const hb = box(0.045, 0.05, 0.06, M(sgn > 0 ? 0xf88335 : 0x2c86c7, { roughness: 0.4 }), 0.008); head.add(hb);
  const nz = new THREE.Mesh(new THREE.ConeGeometry(0.006, 0.016, 10).rotateX(-Math.PI / 2), MAT.shield); nz.position.z = -0.037; head.add(nz);
  const spool = new THREE.Mesh(new THREE.CylinderGeometry(0.085, 0.085, 0.055, 32), M(0xf4f1ea, { roughness: 0.5 })); spool.position.copy(W2(0.63, cl, TABLE_H + 0.37)); g.add(spool);
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.06, 20), MAT.dark); hub.position.copy(spool.position); g.add(hub);
  shadowed(g); S2.add(g);
  return { g, tex, head, cl, base: head.position.clone() };
}
const printers = { L: makeBeltPrinter(1), R: makeBeltPrinter(-1) };

// G4 parts staged on the finished table; positions in the assembly frame (metres)
const G4F0 = 0.33;
const A2R = (x, y, z) => [G4F0 + y, -x, TABLE_H + z];
const g4 = {};
const g4add = (key, name, mat, rest) => { const m = g4mesh(name, mat); S2.add(m); g4[key] = { m, rest }; return m; };
g4add('trough', 'g4_trough', G4C.trough, [0, 0.14, G4Z]);
g4add('holder', 'g4_holder', G4C.holder, [0.195, 0.055, 0.0015]);
g4add('guide', 'g4_guide', G4C.guide, [-0.2, 0.06, 0]);
[[-0.215, -0.06], [-0.245, -0.06], [0.245, -0.06], [0.215, -0.06]].forEach((p, i) => g4add('car' + i, 'g4_carrier', G4C.carrier, [p[0], p[1], 0.0206]));
const PAPER_REST = [[-0.15, -0.065], [-0.18, -0.065], [0.18, -0.065], [0.15, -0.065]];
PAPER_REST.forEach((p, i) => { const m = box(0.0006, 0.046, 0.042, G4C.paper, 0); m.castShadow = true; S2.add(m); g4['pap' + i] = { m, rest: [p[0], p[1], 0.021] }; });
const folds = G4XS.map(x => { const m = g4mesh('g4_fold' + Math.round(x * 1000), G4C.paper); S2.add(m); return m; });
const roller = new THREE.Group(); { roller.add(g4mesh('g4_handle', G4C.tool), g4mesh('g4_roller', G4C.wheel)); const ax = new THREE.Mesh(new THREE.CylinderGeometry(0.0025, 0.0025, 0.07, 12), MAT.steel); ax.position.z = 0.008; roller.add(ax); shadowed(roller); }
S2.add(roller); g4.roller = { m: roller, rest: [-0.23, 0.15, 0] };
const sheet = box(0.146, 0.076, 0.0006, G4C.sheet, 0); sheet.receiveShadow = true; S2.add(sheet); g4.sheet = { m: sheet, rest: [0.19, 0.14, 0.0003] };
const sheetCress = new THREE.Mesh(SHEET_CRESS, MAT.leaf); sheetCress.castShadow = true; S2.add(sheetCress);
const water = box(0.15, 0.07, 0.012, G4C.water, 0.002); S2.add(water);
const placeG4 = (m, x, y, z) => { const r = A2R(x, y, z); m.position.copy(W2(...r)); m.rotation.set(0, 0, Math.PI / 2); };
const G4_STEPS = [0, 0.05, 0.13, 0.2, 0.27, 0.4, 0.42, 0.5, 0.57, 0.64, 0.69, 0.84, 0.92];

// ---------------------------------------------------------------- the endless garden (instanced)
const garden = (() => {
  const cells = [];
  for (let k = 0; k < 7; k++) for (const side of [-1, 1]) for (let j = -12; j <= 19; j++) {
    const x = side * (0.46 + 1.1 * k), y = ST[2].y + j * 0.72;
    if (k === 0 && side < 0 && Math.abs(y - ST[2].y) < 1.1) continue;
    if (k === 0 && Math.abs(y - ST[3].y) < 1.1) continue;
    if (Math.abs(y - ST[4].y - 0.8) < 0.8 && Math.abs(x) < 3) continue;
    cells.push({ x, y, side, d: Math.hypot(x, y - ST[2].y) });
  }
  cells.sort((a, b) => a.d - b.d);
  const tableGeo = mergeGeometries([new THREE.BoxGeometry(0.285, 0.428, 0.0126).translate(0, 0, TABLE_H - 0.006),
    ...[-1, 1].flatMap(a => [-1, 1].map(b => new THREE.BoxGeometry(0.014, 0.014, TABLE_H).translate(a * 0.135, b * 0.207, TABLE_H / 2))),
    ...[-1, 1].map(a => new THREE.BoxGeometry(0.012, 0.414, 0.012).translate(a * 0.135, 0, 0.02)), ...[-1, 1].map(a => new THREE.BoxGeometry(0.012, 0.414, 0.012).translate(a * 0.135, 0, 0.305))]);
  const n = cells.length;
  const tablesIM = new THREE.InstancedMesh(tableGeo, MAT.pla, n);
  const troughIM = new THREE.InstancedMesh(propGeo('g4_trough'), G4C.trough, n * 2);
  const holderIM = new THREE.InstancedMesh(propGeo('g4_holder'), G4C.holder, n * 2);
  const cressIM = new THREE.InstancedMesh(CRESS_LO, MAT.leaf, n * 2);
  for (const im of [tablesIM, troughIM, holderIM, cressIM]) { im.castShadow = im === tablesIM; im.receiveShadow = true; im.frustumCulled = false; scene.add(im); }
  const clones = [V3(1.01, ST[2].y + 2.2, 0), V3(-1.01, ST[2].y + 4.6, 0), V3(2.11, ST[2].y - 1.6, 0), V3(-2.11, ST[2].y + 7.2, 0)].map((p, i) => {
    if (LITE) return new THREE.Group();
    const c = robot.root.clone(); c.position.copy(p); c.rotation.z = i % 2 ? Math.PI / 2 : -Math.PI / 2; scene.add(c); c.visible = false; return c;
  });
  const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), s = V3(), p = V3(), zr = new THREE.Quaternion().setFromAxisAngle(V3(0, 0, 1), Math.PI / 2);
  function update(k, grow) {
    const vis = k > 0.001; tablesIM.visible = troughIM.visible = holderIM.visible = cressIM.visible = vis;
    clones.forEach((c, i) => { c.visible = vis && k > 0.35 + i * 0.12; });
    if (!vis) return;
    cells.forEach((c, i) => {
      const a = E(k, (i / n) * 0.75, (i / n) * 0.75 + 0.12);
      s.setScalar(Math.max(a, 1e-4)); q.identity(); p.set(c.x, c.y, 0);
      tablesIM.setMatrixAt(i, m4.compose(p, q, s));
      for (let j = 0; j < 2; j++) {
        p.set(c.x + c.side * 0.02, c.y + (j - 0.5) * 0.24, TABLE_H + G4Z); q.copy(zr);
        troughIM.setMatrixAt(i * 2 + j, m4.compose(p, q, s)); holderIM.setMatrixAt(i * 2 + j, m4.compose(p, q, s));
        p.z = TABLE_H + G4Z + 0.0013; s.setScalar(Math.max(a * grow * 1.4, 1e-4));
        cressIM.setMatrixAt(i * 2 + j, m4.compose(p, q, s)); s.setScalar(Math.max(a, 1e-4));
      }
    });
    tablesIM.instanceMatrix.needsUpdate = troughIM.instanceMatrix.needsUpdate = holderIM.instanceMatrix.needsUpdate = cressIM.instanceMatrix.needsUpdate = true;
  }
  return { update };
})();

// ---------------------------------------------------------------- station 3: carton
const S3 = new THREE.Group(); scene.add(S3);
const T3 = makeTable(0.5, 0.48, TABLE_H, MAT.wood, MAT.steel); at(T3, SP(3, 0.21 + 0.24, 0, 0)); S3.add(T3);
const CT = { L: 0.379, W: 0.283, H: 0.108, f: 0.34 };
const carton = new THREE.Group(); at(carton, SP(3, CT.f, 0, TABLE_H)); S3.add(carton);
const flaps = {};
{ const th = 0.004, { L, W, H } = CT;
  const bottom = box(W, L, th, MAT.card, 0); bottom.position.z = th / 2; carton.add(bottom);
  for (const s of [-1, 1]) { const w = box(th, L, H, MAT.card, 0); w.position.set(s * W / 2, 0, H / 2); carton.add(w); const e = box(W, th, H, MAT.card, 0); e.position.set(0, s * L / 2, H / 2); carton.add(e); }
  const inner = box(W - 0.01, L - 0.01, 0.002, MAT.cardIn, 0); inner.position.z = th + 0.001; carton.add(inner);
  // mekabu cups, 2 x 3
  for (let i = 0; i < 2; i++) for (let j = 0; j < 3; j++) {
    const cup = box(0.12, 0.11, 0.07, MAT.mekabu, 0.012); cup.position.set((i - 0.5) * 0.13, (j - 1) * 0.12, th + 0.035); carton.add(cup);
    const film = box(0.122, 0.112, 0.004, MAT.cupFilm, 0.002); film.position.set((i - 0.5) * 0.13, (j - 1) * 0.12, th + 0.072); carton.add(film);
  }
  // flaps hinge on the top edges; local +x of each flap group points inward when closed (angle 0)
  const mk = (name, hx, hy, rotZ, span, depth, z) => {
    const g = new THREE.Group(); g.position.set(hx, hy, H + z); g.rotation.z = rotZ; carton.add(g);
    const h = new THREE.Group(); g.add(h);
    const plate = box(depth, span, th, MAT.card, 0); plate.position.set(depth / 2, 0, th / 2); h.add(plate);
    flaps[name] = { g, h, depth, span };
  };
  // carton axes = world axes; the robot (at x = 0) is on the carton's +x side, its left on -y
  mk('sL', 0, -L / 2, Math.PI / 2, W - 0.004, 0.125, 0);       // short flap, robot-left end
  mk('sR', 0, L / 2, -Math.PI / 2, W - 0.004, 0.125, 0);       // short flap, robot-right end
  mk('front', W / 2, 0, Math.PI, L - 0.004, W / 2 - 0.002, 0.0045); // long flap nearest the robot
  mk('back', -W / 2, 0, 0, L - 0.004, W / 2 - 0.002, 0.0045);
  shadowed(carton); }
const tape = box(0.048, 1, 0.0015, MAT.tape, 0); tape.castShadow = false; carton.add(tape);
function setFlap(name, ang) { flaps[name].h.rotation.y = -ang; } // positive ang lifts the flap up and out

// ---------------------------------------------------------------- station 4: crate + conveyor
const S4 = new THREE.Group(); scene.add(S4);
const CR = { w: 0.30, d: 0.26, h: 0.2, stand: 0.52, f: 0.38 };
const crateStand = makeTable(0.36, 0.3, CR.stand, MAT.wood, MAT.steel); at(crateStand, SP(4, CR.f, 0, 0)); S4.add(crateStand);
const crate = new THREE.Group(); S4.add(crate);
{ const t = 0.008, { w, d, h } = CR;
  const b = box(d, w, t, MAT.crate, 0.002); b.position.z = t / 2; crate.add(b);
  for (const s of [-1, 1]) { const a = box(t, w, h, MAT.crate, 0.002); a.position.set(s * d / 2, 0, h / 2); crate.add(a); const c = box(d, t, h, MAT.crate, 0.002); c.position.set(0, s * w / 2, h / 2); crate.add(c); }
  for (const s of [-1, 1]) { const hole = box(0.012, 0.06, 0.025, MAT.dark, 0.004); hole.position.set(0, s * (w / 2 + 0.002), h - 0.04); crate.add(hole); }
  shadowed(crate); }
const BELT = { y: ST[4].y + 0.8, top: 0.56, x0: 1.3, x1: -3.2, w: 0.42 };
const beltTex = (() => { const c = document.createElement('canvas'); c.width = 64; c.height = 64; const g = c.getContext('2d'); g.fillStyle = '#2a2d31'; g.fillRect(0, 0, 64, 64); g.fillStyle = '#3a3e44'; g.fillRect(0, 0, 64, 10);
  const t = new THREE.CanvasTexture(c); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(1, 30); t.colorSpace = THREE.SRGBColorSpace; return t; })();
{ const len = BELT.x0 - BELT.x1, cx = (BELT.x0 + BELT.x1) / 2;
  const belt = new THREE.Mesh(new THREE.BoxGeometry(len, BELT.w, 0.03), new THREE.MeshStandardMaterial({ map: beltTex, roughness: 0.9 }));
  beltTex.rotation = Math.PI / 2; beltTex.center.set(0.5, 0.5);
  belt.position.set(cx, BELT.y, BELT.top - 0.015); S4.add(belt);
  for (const s of [-1, 1]) { const rail = box(len, 0.02, 0.06, MAT.steel, 0.004); rail.position.set(cx, BELT.y + s * (BELT.w / 2 + 0.012), BELT.top - 0.005); S4.add(rail); }
  for (let x = BELT.x1 + 0.2; x < BELT.x0; x += 0.9) for (const s of [-1, 1]) { const leg = box(0.03, 0.03, BELT.top - 0.03, MAT.steel, 0.004); leg.position.set(x, BELT.y + s * (BELT.w / 2), (BELT.top - 0.03) / 2); S4.add(leg); }
  for (const x of [BELT.x0, BELT.x1]) { const roller = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, BELT.w + 0.02, 18), MAT.steel); roller.position.set(x, BELT.y, BELT.top - 0.03); S4.add(roller); }
  shadowed(S4); }
// seaweed pieces: inside the crate, then poured along a ballistic path onto the moving belt
const WEED_N = 170;
const weed = new THREE.InstancedMesh((() => { const g = new THREE.IcosahedronGeometry(1, 1); g.scale(0.024, 0.016, 0.006); return g; })(), M(0xffffff, { roughness: 0.35, clearcoat: 0.8, envMapIntensity: 0.4 }), WEED_N);
weed.castShadow = true; weed.frustumCulled = false; S4.add(weed);
const weedData = Array.from({ length: WEED_N }, (_, i) => {
  const c = new THREE.Color().setHSL(lerp(0.17, 0.26, rand()), lerp(0.45, 0.7, rand()), lerp(0.07, 0.15, rand()));
  weed.setColorAt(i, c);
  return { local: V3((rand() - 0.5) * (CR.d - 0.04), (rand() - 0.5) * (CR.w - 0.04), 0.015 + rand() * 0.14),
    rot: new THREE.Euler(rand() * 6, rand() * 6, rand() * 6), rel: lerp(0.35, 1, Math.pow(rand(), 0.8)), v: lerp(0.4, 0.9, rand()), jitter: (rand() - 0.5) * 0.25 };
});

// ---------------------------------------------------------------- base pose and camera tracks
function drive(A, B, t, a, b) {
  const p = seg(t, a, b), dy = B.y - A.y, turn = dy > 0 ? -Math.PI / 2 : Math.PI / 2;
  const yaw = turn * (E(p, 0, 0.2) - E(p, 0.8, 1));
  const y = lerp(A.y, B.y, E(p, 0.18, 0.82));
  return { x: A.x, y, yaw };
}
const LIFT = { a: 0.18 };
function baseAt(s) {
  const c = Math.floor(s), t = s - c;
  if (c < 2) return { x: 0, y: ST[0].y, yaw: 0 };
  if (c === 2) return drive(ST[0], ST[1], t, 0, 0.17);
  if (c === 3) return { x: 0, y: ST[1].y, yaw: 0 };
  if (c === 4) return drive(ST[1], ST[2], t, 0, 0.15);
  if (c === 5) return { x: 0, y: ST[2].y, yaw: 0 };
  if (c === 6) return { x: 0, y: lerp(ST[2].y, ST[3].y, E(t, 0.3, 0.97)), yaw: -Math.PI / 2 * E(t, 0.22, 0.3) };
  if (c === 7) return { x: 0, y: ST[3].y, yaw: -Math.PI / 2 * (1 - E(t, 0, 0.07)) };
  if (c === 8) {
    if (t < LIFT.a) return drive(ST[3], ST[4], t, 0, 0.16);
    const u = seg(t, LIFT.a, 1);
    return { x: 0, y: ST[4].y, yaw: -Math.PI / 2 * (E(u, 0.32, 0.45) - E(u, 0.84, 0.92)) };
  }
  return { x: 0, y: ST[4].y, yaw: 0 };
}
const K = (s, p, t) => ({ s, p: V3(...p), t: V3(...t) });
const off = (k, p) => [ST[k].x + p[0], ST[k].y + p[1], p[2]];
const CAM = [
  K(0.00, off(0, [-2.9, -2.1, 1.6]), off(0, [0, 0, 0.62])),
  K(0.85, off(0, [-2.3, -1.7, 1.4]), off(0, [0, 0, 0.68])),
  K(1.12, off(0, [-2.9, -2.6, 1.9]), off(0, [0, 0, 0.8])),
  K(1.48, off(0, [-1.2, -3.4, 1.9]), off(0, [0, 0, 0.82])),
  K(1.82, off(0, [-2.8, -1.6, 1.7]), off(0, [0, 0, 0.78])),
  K(2.02, off(0, [-2.7, -1.0, 1.7]), off(0, [0, 0.6, 0.7])),
  K(2.17, off(1, [-2.35, -1.6, 1.95]), off(1, [-0.25, 0.15, 0.8])),
  K(2.95, off(1, [-2.15, -1.25, 1.85]), off(1, [-0.25, 0.15, 0.8])),
  K(3.10, off(1, [-1.75, -1.2, 1.6]), off(1, [-0.35, 0, 0.86])),
  K(3.92, off(1, [-1.6, -0.75, 1.55]), off(1, [-0.35, 0, 0.88])),
  K(4.06, off(1, [-2.2, -0.6, 1.7]), off(1, [-0.2, 2.0, 0.8])),
  K(4.17, off(2, [-2.7, -1.5, 2.15]), off(2, [-0.38, 0, 0.66])),
  K(4.3, off(2, [-2.3, -2.0, 1.6]), off(2, [-0.4, 0, 0.45])),
  K(4.58, off(2, [-2.15, 1.45, 2.0]), off(2, [-0.42, 0.12, 0.7])),
  K(4.95, off(2, [-1.95, -1.1, 1.95]), off(2, [-0.38, 0, 0.72])),
  K(5.08, off(2, [-1.6, -0.5, 1.9]), off(2, [-0.34, 0, 0.72])),
  K(5.6, off(2, [-1.45, -0.22, 1.8]), off(2, [-0.33, 0, 0.72])),
  K(5.92, off(2, [-1.5, 0.4, 1.8]), off(2, [-0.33, 0, 0.72])),
  K(6.16, off(2, [-3.8, -3.2, 3.1]), off(2, [-0.8, 1.4, 0.4])),
  K(6.55, off(2, [-9.5, -5.0, 8.8]), off(2, [-2.8, 3.2, 0])),
  K(6.93, off(3, [-6.0, -6.5, 5.8]), off(3, [-0.8, 0, 0.3])),
  K(7.12, off(3, [-1.05, -0.95, 2.3]), off(3, [-0.32, 0, 0.8])),
  K(7.55, off(3, [-1.0, 0.25, 2.35]), off(3, [-0.32, 0, 0.8])),
  K(7.95, off(3, [-1.05, 0.9, 2.15]), off(3, [-0.32, 0, 0.8])),
  K(8.06, off(3, [-2.1, 0.4, 1.85]), off(3, [0, 2.2, 0.7])),
  K(8.18, off(4, [-2.5, -1.6, 1.75]), off(4, [-0.2, 0.3, 0.7])),
  K(8.42, off(4, [-2.2, -0.5, 1.55]), off(4, [-0.2, 0.35, 0.75])),
  K(8.58, off(4, [-1.9, 2.4, 1.7]), off(4, [0, 0.65, 0.65])),
  K(8.80, off(4, [-1.3, 2.5, 1.55]), off(4, [-0.4, 0.75, 0.6])),
  K(9.06, off(4, [-2.3, -0.9, 1.4]), off(4, [0, 0, 0.85])),
  K(10.0, off(4, [-2.0, -0.65, 1.32]), off(4, [0, 0, 0.88])),
];
function camAt(s) {
  let i = 0; while (i < CAM.length - 2 && s > CAM[i + 1].s) i++;
  const a = CAM[i], b = CAM[i + 1], u = ease(seg(s, a.s, b.s));
  return { p: a.p.clone().lerp(b.p, u), t: a.t.clone().lerp(b.t, u) };
}

// ---------------------------------------------------------------- motion helpers
const DOWN = [0.18, 0, -0.98];
// Pick-and-place in the robot frame. Returns arm spec and whether the object is held.
function pnp(u, side, pick, place, home = HOME[side]) {
  const up = p => [p[0], p[1], p[2] + 0.09];
  const k = [
    [0.00, home.tip, home.dir, JAW_REST], [0.14, up(pick), DOWN, JAW_OPEN], [0.24, pick, DOWN, JAW_OPEN], [0.32, pick, DOWN, JAW_SHUT],
    [0.42, up(pick), DOWN, JAW_SHUT], [0.64, up(place), DOWN, JAW_SHUT], [0.74, place, DOWN, JAW_SHUT], [0.82, place, DOWN, JAW_OPEN],
    [0.9, up(place), DOWN, JAW_OPEN], [1.0, home.tip, home.dir, JAW_REST]];
  let i = 0; while (i < k.length - 2 && u > k[i + 1][0]) i++;
  const a = k[i], b = k[i + 1], w = ease(seg(u, a[0], b[0]));
  return { spec: { tip: lerp3(a[1], b[1], w), dir: lerp3(a[2], b[2], w), roll: 0, jaw: lerp(a[3], b[3], w) }, held: u > 0.29 && u < 0.78, placed: u >= 0.78 };
}
// Object-driven carry in the robot frame: the object follows a fixed path and the claw tips follow
// the object at fixed grip offsets, so a held part never drifts from the gripper.
function carry(u, from, to, grips, lift = 0.07) {
  let obj;
  if (u < 0.26) obj = from;
  else if (u < 0.4) obj = [from[0], from[1], from[2] + lift * E(u, 0.26, 0.4)];
  else if (u < 0.62) obj = lerp3([from[0], from[1], from[2] + lift], [to[0], to[1], to[2] + lift], E(u, 0.4, 0.62));
  else if (u < 0.76) obj = [to[0], to[1], to[2] + lift * (1 - E(u, 0.62, 0.76))];
  else obj = to;
  const arms = {};
  for (const side in grips) {
    const g = grips[side], tipAt = (o, dz = 0) => [o[0] + g[0], o[1] + g[1], o[2] + g[2] + dz], home = HOME[side];
    let tip, jaw = JAW_OPEN, dir = DOWN;
    if (u < 0.18) { const w = E(u, 0, 0.18); tip = lerp3(home.tip, tipAt(from, 0.06), w); dir = lerp3(home.dir, DOWN, w); jaw = lerp(JAW_REST, JAW_OPEN, w); }
    else if (u < 0.26) { tip = lerp3(tipAt(from, 0.06), tipAt(from), E(u, 0.18, 0.23)); jaw = lerp(JAW_OPEN, JAW_SHUT, E(u, 0.23, 0.26)); }
    else if (u < 0.76) { tip = tipAt(obj); jaw = JAW_SHUT; }
    else if (u < 0.84) { tip = lerp3(tipAt(to), tipAt(to, 0.06), E(u, 0.8, 0.84)); jaw = lerp(JAW_SHUT, JAW_OPEN, E(u, 0.76, 0.8)); }
    else { const w = E(u, 0.84, 1); tip = lerp3(tipAt(to, 0.06), home.tip, w); dir = lerp3(DOWN, home.dir, w); jaw = lerp(JAW_OPEN, JAW_REST, w); }
    arms[side] = { tip, dir, roll: 0, jaw };
  }
  return { obj, arms, held: u >= 0.26 && u < 0.76 };
}
function track(t, side, wins) {
  for (const w of wins) if (t >= w.a && t <= w.b) return { ...pnp(seg(t, w.a, w.b), side, w.pick, w.place), win: w };
  return { spec: HOME[side], held: false };
}
const tipWorld = side => robot.tip[side].getWorldPosition(V3());
// place an object relative to a claw tip: offset in the robot frame, object yaw follows the robot
function follow(obj, side, off, yaw = 0) {
  const p = tipWorld(side).add(RP(...off).applyQuaternion(robot.root.quaternion));
  obj.position.copy(p); obj.rotation.set(0, 0, robot.root.rotation.z + yaw);
}
const toWorld = (k, v) => SP(k, v[0], v[1], v[2]);

// ---------------------------------------------------------------- chapters
const sensorItems = [...document.querySelectorAll('#sensor-list li')];
const g4Steps = [...document.querySelectorAll('#g4-steps li')];
const g4StepNum = document.getElementById('g4-step-num'), g4StepName = document.getElementById('g4-step-name');
const SENSOR_LABELS = [
  label('OAK-D Lite', '深度＋カラー · 視野69°', meshAnchor(partBy('tophead6'), -0.03, 0, 0.05)),
  label('手首カメラ', '左右に1台ずつ', () => tipWorld('R').add(V3(0, 0, 0.06))),
  label('スマホの俯瞰カメラ', '720×1280 · ロボットの右', () => phone.localToWorld(V3(0, 0, 1.3))),
  label('AprilTag 36h11', '検出 4/4 フレーム', () => tags1[0].position.clone().add(V3(0, 0, 0.03)), 'tag'),
  label('サーボのフィードバック', '位置・負荷・電圧・電流', meshAnchor(partBy('Upper_Arm_Motor', 'armL'), 0, 0, 0.05)),
  label('車輪のエンコーダー', '走行距離を測る', meshAnchor(robot.parts.find(p => p.part === 'raskogwheel2'), 0, 0, 0.06)),
  label('Joy-Con（左右）', 'ボタン・スティック・IMU', () => joycons.localToWorld(V3(0, 0, 0.06))),
  label('BH1750 ＋ ESP32', '未配線', () => paddle.localToWorld(V3(-0.055, 0, 0.03))),
  label('D500 LiDAR', '取付具を設計（未取付）', () => lidar.localToWorld(V3(0, 0, 0.09))),
  label('SHT31・土壌水分', 'カートに入れたのみ', () => cartOnly.localToWorld(V3(0, 0.02, 0.03))),
];
const MEASURE_LABEL = label('照度 — lx', 'センサー未配線のため表示のみ', () => paddle.localToWorld(V3(-0.055, 0, 0.05)));
const DEPTH_LABEL = label('深度 → 3Dの点', 'OAK-D Lite', () => SP(1, 0.62, 0.2, TABLE_H + 0.06));

function sceneAt(s, time) {
  const c = Math.floor(s), t = s - c;
  const base = baseAt(s);
  robot.root.position.set(base.x, base.y, 0); robot.root.rotation.z = base.yaw;
  robot.root.updateMatrixWorld(true);

  // station visibility windows
  S1.visible = s > 1.9 && s < 4.4; S2.visible = s > 3.45 && s < 7.3; S3.visible = s > 6.55 && s < 8.35; S4.visible = s > 7.55;

  // exploded view
  const e = c === 1 ? E(t, 0.1, 0.5) * (1 - E(t, 0.78, 0.96)) : 0;
  explode(e);
  EXPLODE_LABELS.forEach(L => L.on = e > 0.82);

  // arms: default home
  let armL = null, armR = null;
  robot.setQ(HOME_Q);
  head(REDUCED ? 0 : Math.sin(time * 0.5) * 0.18 * (1 - seg(s, 0.7, 1)), -0.2);

  // sensors: ten inputs, one at a time
  const NS = SENSOR_LABELS.length, S0 = 0.06, SW = 0.9 / NS;
  const sIdx = c === 2 && t >= S0 ? Math.min(NS - 1, Math.floor((t - S0) / SW)) : -1;
  sensorItems.forEach((li, i) => li.classList.toggle('on', i === sIdx));
  SENSOR_LABELS.forEach((L, i) => L.on = sIdx === i);
  const sw = i => c === 2 ? E(t, S0 + i * SW, S0 + i * SW + 0.02) * (1 - E(t, S0 + (i + 1) * SW - 0.015, S0 + (i + 1) * SW)) : 0;
  fade(oakFrustum, Math.max(sw(0), c === 3 ? E(t, 0.0, 0.05) * (1 - E(t, 0.3, 0.36)) * 0.6 : 0));
  wristFrustums.forEach(f => fade(f, sw(1)));
  fade(phoneFrustum, sw(2));
  tagBoxes.forEach(b => b.material.opacity = sw(3) * (0.6 + 0.4 * Math.sin(time * 6)));
  const pulse = 0.5 + 0.5 * Math.sin(time * 5);
  MAT.motor.emissive.setRGB(0.97, 0.45, 0.1).multiplyScalar(sw(4) * 0.55 * pulse);
  MAT.wheel.emissive.setRGB(0.97, 0.45, 0.1).multiplyScalar(sw(5) * 0.8 * pulse);
  joycons.position.z = TABLE_H + sw(6) * 0.03 * (0.5 + 0.5 * Math.sin(time * 3));
  const lidarK = Math.max(sw(8), c === 6 ? E(t, 0.3, 0.4) * (1 - E(t, 0.94, 1)) : 0);
  show(lidar, lidarK > 0 ? 1 : 0); updateScan(REDUCED ? 0 : time, lidarK);
  if (c === 2 && t > 0.2) look(sIdx === 2 ? phone.localToWorld(V3(0, 0, 1.2)) : SP(1, 0.45, 0, TABLE_H));
  if (c === 2 && sIdx === 1) { armL = { ...HOME.L, tip: [0.27, 0.22, 0.97], dir: [1, 0.1, -0.35] }; armR = { ...HOME.R, tip: [0.27, -0.22, 0.97], dir: [1, -0.1, -0.35] }; }

  // measuring: depth sweep + paddle pick, hold over planter, put back
  cloud.visible = c === 3;
  if (c === 3) {
    const n = cloud.userData.n; cloud.geometry.setDrawRange(0, Math.floor(n * E(t, 0.03, 0.3)));
    cloud.material.opacity = 0.95 * (1 - E(t, 0.36, 0.48));
    look(SP(1, 0.45, 0, TABLE_H));
    const grip = [0.3, -0.15, TABLE_H + 0.012];
    const hold = [0.36, 0.06, TABLE_H + 0.17];
    const k = [[0.3, HOME.R.tip, HOME.R.dir, JAW_REST], [0.38, [grip[0], grip[1], grip[2] + 0.08], DOWN, JAW_OPEN], [0.44, grip, DOWN, JAW_OPEN], [0.48, grip, DOWN, JAW_SHUT],
      [0.54, [grip[0], grip[1], grip[2] + 0.12], DOWN, JAW_SHUT], [0.62, hold, [0.5, 0, -0.86], JAW_SHUT], [0.78, hold, [0.5, 0, -0.86], JAW_SHUT],
      [0.86, [grip[0], grip[1], grip[2] + 0.08], DOWN, JAW_SHUT], [0.9, grip, DOWN, JAW_SHUT], [0.93, grip, DOWN, JAW_OPEN], [1.0, HOME.R.tip, HOME.R.dir, JAW_REST]];
    if (t > 0.3) {
      let i = 0; while (i < k.length - 2 && t > k[i + 1][0]) i++;
      const a = k[i], b = k[i + 1], w = ease(seg(t, a[0], b[0]));
      armR = { tip: lerp3(a[1], b[1], w), dir: lerp3(a[2], b[2], w), roll: 0, jaw: lerp(a[3], b[3], w) };
    }
    if (t > 0.6 && t < 0.86) look(paddle.localToWorld(V3(-0.05, 0, 0)));
  }
  DEPTH_LABEL.on = c === 3 && t > 0.12 && t < 0.38;
  MEASURE_LABEL.on = c === 3 && t > 0.64 && t < 0.8;

  // modular table: legs fast-forward, then the arms take each print off its belt and fit it
  const pt = c === 4 ? t : c > 4 ? 1 : 0;
  legs.update(c === 4 ? E(t, 0.13, 0.3) : c > 4 ? 1 : 0);
  const printing = c === 4 && t > 0.14;
  for (const side of ['L', 'R']) {
    const pr = printers[side];
    pr.head.position.copy(pr.base); if (printing && !REDUCED) pr.head.position.y += Math.sin(time * 7 + (side === 'L' ? 0 : 1.7)) * 0.05;
    pr.tex.offset.x = -(pt * BELT_V / 0.45) * 1.0;
  }
  for (const side of ['L', 'R']) {
    let active = null;
    for (const p of JOBS[side]) {
      const pickT = p.a + 0.22 * (p.b - p.a);
      p.m.rotation.set(...p.rot);
      p.m.visible = S2.visible;
      if (pt >= p.b || c > 4) { p.m.position.copy(W2(...p.final)); continue; }
      if (pt >= p.a) { active = p; continue; }
      // waiting on the belt: emerges from under the gantry and rides to the pick point
      const f = BELT_PICK_F + BELT_V * (pickT - pt);
      if (f > BELT_START_F || c < 4) { p.m.visible = false; continue; }
      p.m.position.copy(W2(f, printers[side].cl, TABLE_H + (p.kind === 'table_top' ? 0 : p.top)));
      p.m.scale.setScalar(Math.max(E(BELT_START_F - f, 0, 0.06), 0.001));
    }
    for (const p of JOBS[side]) if (p !== active && (pt >= p.b || pt < p.a)) { if (pt >= p.b || c > 4) p.m.scale.setScalar(1); }
    if (active && c === 4) {
      const p = active, from = [BELT_PICK_F, printers[side].cl, TABLE_H + (p.kind === 'table_top' ? 0 : p.top)];
      const cr = carry(seg(pt, p.a, p.b), from, p.final, { [side]: [0, 0, p.top - 0.004 + (p.kind === 'table_top' ? 0 : 0)] }, 0.07);
      p.m.position.copy(W2(...cr.obj)); p.m.scale.setScalar(1);
      if (side === 'L') armL = cr.arms.L; else armR = cr.arms.R;
    }
  }
  if (c === 4 && t > 0.2) look(W2(0.35, 0, TABLE_H));

  // G4 planter: the deck's 13 steps on the finished table
  const gt = c === 5 ? t : c > 5 ? 1 : 0;
  const g4on = S2.visible && (c > 5 || (c === 5 && t > 0.0));
  const stepI = c === 5 ? G4_STEPS.filter(x => t >= x).length - 1 : -1;
  g4Steps.forEach((li, i) => li.classList.toggle('on', i <= stepI));
  if (g4StepNum) { g4StepNum.textContent = stepI >= 0 ? `手順 ${stepI} / 12` : ''; g4StepName.textContent = stepI >= 0 ? g4Steps[stepI].dataset.name : ''; }
  const fadeIn = c === 5 ? E(t, 0, 0.04) : 1;
  const OPS = [
    { a: 0.05, b: 0.13, key: 'trough', to: [0, 0, G4Z], grips: { L: [-0.058, -0.04, 0.0055], R: [0.058, -0.04, 0.0055] } },
    { a: 0.13, b: 0.2, key: 'holder', to: [0, 0, G4Z], grips: { R: [0.03, 0, 0.018] } },
    { a: 0.2, b: 0.27, key: 'guide', to: [0, 0, G4Z], grips: { L: [-0.069, 0, 0.045], R: [0.069, 0, 0.045] } },
    { a: 0.27, b: 0.335, key: 'car0', to: [-0.045, 0, G4Z], grips: { L: [0, 0, 0] } },
    { a: 0.335, b: 0.4, key: 'car1', to: [-0.015, 0, G4Z], grips: { L: [0, 0, 0] } },
    { a: 0.28, b: 0.345, key: 'car3', to: [0.045, 0, G4Z], grips: { R: [0, 0, 0] } },
    { a: 0.345, b: 0.41, key: 'car2', to: [0.015, 0, G4Z], grips: { R: [0, 0, 0] } },
    { a: 0.42, b: 0.49, key: 'pap0', to: [-0.045, 0, G4Z + 0.0019], grips: { L: [0, 0, 0.016] } },
    { a: 0.49, b: 0.56, key: 'pap1', to: [-0.015, 0, G4Z + 0.0019], grips: { L: [0, 0, 0.016] } },
    { a: 0.43, b: 0.5, key: 'pap3', to: [0.045, 0, G4Z + 0.0019], grips: { R: [0, 0, 0.016] } },
    { a: 0.5, b: 0.57, key: 'pap2', to: [0.015, 0, G4Z + 0.0019], grips: { R: [0, 0, 0.016] } },
    { a: 0.57, b: 0.64, key: 'guide', from: [0, 0, G4Z], to: [-0.2, 0.06, 0], grips: { L: [-0.069, 0, 0.045], R: [0.069, 0, 0.045] }, lift: 0.06 },
    { a: 0.84, b: 0.92, key: 'sheet', to: [0, 0, G4Z + 0.001], grips: { L: [-0.055, -0.035, 0.004], R: [0.055, -0.035, 0.004] } },
  ];
  const posOf = {};
  for (const key in g4) posOf[key] = g4[key].rest;
  for (const op of OPS) { op.from = op.from || posOf[op.key]; if (gt >= op.b) posOf[op.key] = op.to; }
  const conv = g => [g[1], -g[0], g[2]];  // assembly offset -> robot frame offset
  for (const op of OPS) {
    if (!(gt >= op.a && gt < op.b) || c !== 5) continue;
    const grips = {}; for (const sd in op.grips) grips[sd] = conv(op.grips[sd]);
    const cr = carry(seg(gt, op.a, op.b), A2R(...op.from), A2R(...op.to), grips, op.lift || 0.07);
    posOf[op.key] = [-(cr.obj[1]), cr.obj[0] - G4F0, cr.obj[2] - TABLE_H];
    if (cr.arms.L) armL = cr.arms.L; if (cr.arms.R) armR = cr.arms.R;
  }
  // roller: pick, roll across all four slots, return
  let rollX = -1;
  { const a = 0.64, b = 0.84, u = seg(gt, a, b), rest = g4.roller.rest, start = [-0.075, 0, G4Z + 0.0006], end = [0.075, 0, G4Z + 0.0006];
    let pos = rest;
    if (gt >= a && gt < b && c === 5) {
      const grip = [0, 0, 0.044];
      if (u < 0.32) { const cr = carry(seg(u, 0, 0.32), A2R(...rest), A2R(...start), { L: conv(grip) }, 0.07); pos = [-cr.obj[1], cr.obj[0] - G4F0, cr.obj[2] - TABLE_H];
        armL = u > 0.32 * 0.76 ? { ...cr.arms.L, jaw: JAW_SHUT, tip: A2R(start[0], start[1], start[2] + 0.044), dir: DOWN } : cr.arms.L; }
      else if (u < 0.78) { const w = E(u, 0.34, 0.76); pos = lerp3(start, end, w); rollX = pos[0];
        armL = { tip: A2R(pos[0], pos[1], pos[2] + 0.044), dir: DOWN, roll: 0, jaw: JAW_SHUT }; }
      else { const cr = carry(seg(u, 0.78, 1) * 0.76 + 0.24, A2R(...end), A2R(...rest), { L: conv(grip) }, 0.07); pos = [-cr.obj[1], cr.obj[0] - G4F0, cr.obj[2] - TABLE_H]; armL = cr.arms.L; rollX = 1; }
    } else if (gt >= b) rollX = 1;
    posOf.roller = pos;
  }
  if (c === 5) look(W2(G4F0, 0, TABLE_H));
  for (const key in g4) {
    const o = g4[key], ps = posOf[key] || o.rest;
    o.m.visible = g4on && fadeIn > 0.02; placeG4(o.m, ...ps);
  }
  // papers fold once the roller has passed their slot
  const papSlot = { pap0: 0, pap1: 1, pap2: 2, pap3: 3 };
  for (const key in papSlot) { const i = papSlot[key], folded = rollX >= G4XS[i] + 0.008; g4[key].m.visible = g4on && !folded; folds[i].visible = g4on && folded; placeG4(folds[i], 0, 0, G4Z); }
  const wet = c === 5 ? E(t, 0.92, 0.97) : c > 5 ? 1 : 0;
  water.visible = g4on && wet > 0.01; placeG4(water, -0.0125, 0, 0.003 + 0.006 * wet); water.scale.set(1, 1, Math.max(wet, 0.01));
  const grow = c === 5 ? E(t, 0.94, 1) : c > 5 ? 1 : 0;
  sheetCress.visible = g4on && grow > 0.01; placeG4(sheetCress, 0, 0, G4Z + 0.0013); sheetCress.scale.setScalar(Math.max(grow, 0.01));

  // garden
  const gk = c === 6 ? E(t, 0.08, 0.62) : 0;
  const gOut = c === 7 ? 1 - E(t, 0, 0.08) : c === 6 ? 1 : 0;
  garden.update(c === 6 ? gk : c === 7 ? gOut : 0, c === 6 ? E(t, 0.3, 0.8) : 1);

  // carton
  const ct = c === 7 ? t : c > 7 ? 1 : 0;
  const OPEN = 1.85;
  const aS = OPEN * (1 - E(ct, 0.1, 0.3)), aF = OPEN * (1 - E(ct, 0.34, 0.52)), aB = 1.65 * (1 - E(ct, 0.55, 0.74));
  setFlap('sL', aS); setFlap('sR', aS); setFlap('front', aF); setFlap('back', aB);
  const tapeL = (CT.L + 0.1) * E(ct, 0.78, 0.96);
  tape.visible = tapeL > 0.005; tape.scale.set(1, Math.max(tapeL, 0.001), 1); tape.position.set(0, -(CT.L / 2 + 0.05) + tapeL / 2, CT.H + 0.0105);
  if (c === 7 && t > 0.07) {
    look(SP(3, CT.f, 0, TABLE_H + 0.1));
    // claw tips push on the flap faces; flap point at radius r, angle a (robot frame)
    const top = TABLE_H + CT.H;
    const shortPt = (sgn, a, r) => { const n = [0, Math.cos(a) * 0 + sgn * Math.sin(a), Math.cos(a)];
      return { tip: [CT.f, sgn * CT.L / 2 - sgn * r * Math.cos(a) + n[1] * 0.018, top + r * Math.sin(a) + n[2] * 0.018], dir: [0.15, -n[1], -n[2]] }; };
    const longPt = (sgn, a, r, l) => { const hinge = CT.f + sgn * CT.W / 2; const nf = sgn * Math.sin(a), nu = Math.cos(a);
      return { tip: [hinge - sgn * r * Math.cos(a) + nf * 0.018, l, top + 0.005 + r * Math.sin(a) + nu * 0.018], dir: [-nf, 0, -nu] }; };
    const homeBlend = (spec, side, w) => ({ tip: lerp3(HOME[side].tip, spec.tip, w), dir: lerp3(HOME[side].dir, spec.dir, w), roll: 0, jaw: JAW_SHUT });
    if (t < 0.33) {
      const w = E(t, 0.07, 0.12) * (1 - E(t, 0.3, 0.33));
      armL = homeBlend(shortPt(1, aS, 0.09), 'L', w); armR = homeBlend(shortPt(-1, aS, 0.09), 'R', w);
    } else if (t < 0.54) {
      const w = E(t, 0.33, 0.37) * (1 - E(t, 0.51, 0.54));
      armL = homeBlend(longPt(-1, aF, 0.11, 0.1), 'L', w); armR = homeBlend(longPt(-1, aF, 0.11, -0.1), 'R', w);
    } else if (t < 0.77) {
      const w = E(t, 0.54, 0.58) * (1 - E(t, 0.74, 0.77));
      armL = homeBlend(longPt(1, aB, 0.13, 0.1), 'L', w); armR = homeBlend(longPt(1, aB, 0.13, -0.1), 'R', w);
    } else {
      const lt = CT.L / 2 + 0.05 - tapeL;
      const onL = lt > 0.02, w = E(t, 0.77, 0.8) * (1 - E(t, 0.96, 1));
      const spec = { tip: [CT.f, lt, top + 0.03], dir: [0.3, 0, -0.95] };
      if (onL) armL = homeBlend(spec, 'L', w); else armR = homeBlend(spec, 'R', w);
    }
  }

  // crate lift and pour
  const lt = c === 8 ? seg(t, LIFT.a, 1) : c > 8 ? 1 : 0;
  const lift = 0.18 * (E(lt, 0.22, 0.32) - E(lt, 0.92, 0.97));
  const PSI_MAX = 2.15, psi = PSI_MAX * (E(lt, 0.45, 0.66) - E(lt, 0.76, 0.84));
  const gripU = CR.stand + CR.h - 0.012;
  // crate pose in the robot frame: rotate about the grip line (through both claw tips, along l)
  function crateMatrix(u, base) {
    const lf = 0.18 * (E(u, 0.22, 0.32) - E(u, 0.92, 0.97)), ps = PSI_MAX * (E(u, 0.45, 0.66) - E(u, 0.76, 0.84));
    const fwd = 0.04 * E(u, 0.32, 0.45) * (1 - E(u, 0.84, 0.92));
    const m = new THREE.Matrix4().makeTranslation(base.x, base.y, 0).multiply(new THREE.Matrix4().makeRotationZ(base.yaw));
    m.multiply(new THREE.Matrix4().makeTranslation(-(CR.f + fwd), 0, gripU + lf));
    m.multiply(new THREE.Matrix4().makeRotationY(-ps)); // tip the open top away from the robot (towards -x local)
    m.multiply(new THREE.Matrix4().makeTranslation(0, 0, -(CR.h - 0.012)));
    return m;
  }
  const crateM = crateMatrix(lt, c >= 8 && t >= LIFT.a ? base : { x: 0, y: ST[4].y, yaw: 0 });
  crate.matrixAutoUpdate = false; crate.matrix.copy(crateM); crate.matrixWorldNeedsUpdate = true;
  if (c === 8 && t >= LIFT.a) {
    const fwd = 0.04 * E(lt, 0.32, 0.45) * (1 - E(lt, 0.84, 0.92));
    const g = [CR.f + fwd, CR.w / 2 + 0.004, gripU + lift];
    const reach = E(lt, 0, 0.12) * (1 - E(lt, 0.97, 1));
    const desc = E(lt, 0.12, 0.18) * (1 - E(lt, 0.97, 1));
    const jaw = lerp(JAW_OPEN, JAW_SHUT, E(lt, 0.18, 0.22) * (1 - E(lt, 0.97, 1)));
    const spec = (sgn, side) => { const tip = [g[0], sgn * g[1], g[2] + 0.07 * (1 - desc)]; return { tip: lerp3(HOME[side].tip, tip, reach), dir: lerp3(HOME[side].dir, [0.1, -sgn * 0.25, -0.96], reach), roll: 0, jaw }; };
    armL = spec(1, 'L'); armR = spec(-1, 'R');
    look(robot.root.localToWorld(RP(0.6, 0, 0.55)));
  }
  // seaweed
  const m4 = new THREE.Matrix4(), qq = new THREE.Quaternion(), sc = V3(0.001, 0.001, 0.001), pp = V3();
  const pourU = u => PSI_MAX * (E(u, 0.45, 0.66));
  weedData.forEach((d, i) => {
    if (!S4.visible) return;
    // release when the crate has tipped past this piece's angle
    if (d.tRel === undefined) { let lo = 0.45, hi = 0.66; for (let k = 0; k < 30; k++) { const mid = (lo + hi) / 2; if (pourU(mid) < d.rel * PSI_MAX * 0.92 + 0.08 * PSI_MAX) lo = mid; else hi = mid; } d.tRel = (lo + hi) / 2; d.mRel = crateMatrix(d.tRel, baseAt(8 + LIFT.a + d.tRel * (1 - LIFT.a))); }
    const sNow = c + t, sRel = 8 + LIFT.a + d.tRel * (1 - LIFT.a);
    qq.setFromEuler(d.rot); sc.setScalar(1);
    if (sNow < sRel) { pp.copy(d.local).applyMatrix4(crateM); }
    else {
      const tau = (sNow - sRel) * 9, p0 = d.local.clone().applyMatrix4(d.mRel);
      const v = V3(0, d.v * 0.9, 0.2), g = 4.9;
      const zLand = BELT.top + 0.006 + rand() * 0;
      const A = g, B = -v.z, C = zLand - p0.z; const tl = (-B + Math.sqrt(Math.max(B * B - 4 * A * C, 0))) / (2 * A);
      if (tau < tl) pp.set(p0.x + d.jitter * tau * 0.4, p0.y + v.y * tau, p0.z + v.z * tau - g * tau * tau);
      else { const land = V3(p0.x + d.jitter * tl * 0.4, clamp(p0.y + v.y * tl, BELT.y - BELT.w / 2 + 0.03, BELT.y + BELT.w / 2 - 0.03), zLand);
        pp.set(land.x - (tau - tl) * 0.45, land.y, land.z); if (pp.x < BELT.x1 + 0.05) sc.setScalar(0.001); }
    }
    weed.setMatrixAt(i, m4.compose(pp, qq, sc));
  });
  weed.instanceMatrix.needsUpdate = true;

  // outro: wave
  if (c >= 9) {
    look(camera.position);
    const w = E(t, 0.05, 0.2), wv = REDUCED ? 0 : Math.sin(time * 5.5);
    armR = { tip: lerp3(HOME.R.tip, [0.16, -0.3, 1.12 + 0.02 * wv], w), dir: lerp3(HOME.R.dir, [0.25, -0.35 + 0.35 * wv, 0.9], w), roll: 0.6 * wv * w, jaw: lerp(JAW_REST, JAW_OPEN, w) };
  }

  // solve arms after all specs are known
  if (armL) poseArm('L', armL); if (armR) poseArm('R', armR);
  robot.root.updateMatrixWorld(true);

  // things held by the claws follow the tips
  if (c === 3 && t > 0.48 && t < 0.92) follow(paddle, 'R', [0.045, 0, -0.012]);
  else { paddle.position.copy(PADDLE_REST); paddle.rotation.set(0, 0, 0); }
}

// ---------------------------------------------------------------- scroll, resize, loop
const chapters = [...document.querySelectorAll('.chapter')];
const bar = document.getElementById('progress-bar');
// The training chapter is a 2D overlay: the 3D scene holds at the end of packing while it plays.
const LEARN = chapters.findIndex(ch => ch.dataset.chapter === 'learn');
let domS = 0;
function scrollS() {
  const y = scrollY, vh = innerHeight, max = document.documentElement.scrollHeight - vh;
  bar.style.width = (100 * y / Math.max(1, max)).toFixed(2) + '%';
  domS = 0;
  for (let i = chapters.length - 1; i >= 0; i--) {
    const el = chapters[i], top = el.offsetTop, h = el.offsetHeight;
    if (y >= top || i === 0) { const span = Math.max(1, Math.min(h, max - top)); domS = i + clamp((y - top) / span, 0, 0.9999); break; }
  }
  const i = Math.floor(domS), t = domS - i;
  if (LEARN < 0 || i < LEARN) return domS;
  return i === LEARN ? LEARN - 0.0001 : i - 1 + t;
}
// training: one pre-rendered video (1 -> 9 -> 36 -> 144 -> every simulation clip), played when the chapter is on screen
const wall = document.getElementById('wall'), trainVid = document.getElementById('trainvid');
let wasOn = false;
function updateWall() {
  const i = Math.floor(domS), t = domS - i, on = i === LEARN;
  wall.style.opacity = on ? E(t, 0, 0.05) * (1 - E(t, 0.94, 0.995)) : 0;
  if (LEARN >= 0 && domS > LEARN - 1.5 && trainVid.preload !== 'auto') { trainVid.preload = 'auto'; trainVid.load(); }
  if (on && !wasOn) { try { trainVid.currentTime = 0; } catch (_) {} trainVid.play().catch(() => {}); }
  if (!on && wasOn) trainVid.pause();
  wasOn = on;
}
let W = 0, H = 0, mobile = false, centered = false;
function resize() {
  W = innerWidth; H = innerHeight; mobile = W < 760;
  renderer.setSize(W, H, false);
  camera.aspect = W / H;
  camera.fov = mobile ? 46 : 32;
  if (centered) camera.clearViewOffset(); else if (mobile) camera.setViewOffset(W, H, 0, H * 0.2, W, H); else camera.setViewOffset(W, H, -W * 0.17, 0, W, H);
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize); resize();

let sSmooth = scrollS();
const t0 = performance.now();
const _v = V3();
const cards = chapters.map(ch => ch.querySelector('.card'));
function fadeCards(s) {
  const c = Math.floor(s), t = s - c;
  cards.forEach((el, i) => { const k = i === c ? (i === 0 ? 1 : E(t, 0, 0.03)) * (i === cards.length - 1 ? 1 : 1 - E(t, 0.9, 0.985)) : 0; el.style.opacity = k.toFixed(3); el.style.visibility = k < 0.01 ? 'hidden' : 'visible'; });
}
function frame() {
  if (failed) return;
  try { frameBody(); } catch (e) { console.error(e); fail(e.message); return; }
  requestAnimationFrame(frame);
}
function frameBody() {
  const time = (performance.now() - t0) / 1000;
  const target = window.__hoya?.forceS ?? scrollS();
  if (window.__hoya?.forceS != null) sSmooth = target;
  sSmooth += (target - sSmooth) * (Math.abs(target - sSmooth) > 1.5 ? 1 : 0.12);
  const s = sSmooth;
  sceneAt(s, time);
  fadeCards(domS);
  updateWall();
  const cam = camAt(s);
  if (s < 1) { const a = (REDUCED ? 0 : Math.sin(time * 0.22) * 0.18) * (1 - seg(s, 0.6, 1)); cam.p.sub(cam.t).applyAxisAngle(V3(0, 0, 1), a).add(cam.t); }
  const dbg = window.__hoya.debugCam; if (dbg) { cam.p.set(...dbg.p); cam.t.set(...dbg.t); }
  const wantCenter = !!((dbg && dbg.center) || window.__hoya.capture);
  if (wantCenter !== centered) { centered = wantCenter; resize(); }
  camera.position.copy(cam.p); camera.lookAt(cam.t);
  const dist = cam.p.distanceTo(cam.t);
  scene.fog.near = dist * 1.6; scene.fog.far = dist * 6 + 6;
  key.position.copy(cam.t).add(V3(-2.5, -3.2, 6.5)); key.target.position.copy(cam.t);
  const sh = clamp(dist * 0.9, 2.2, 14); Object.assign(key.shadow.camera, { left: -sh, right: sh, top: sh, bottom: -sh }); key.shadow.camera.updateProjectionMatrix();
  beltTex.offset.y = REDUCED ? 0 : -time * 0.25;
  renderer.render(scene, camera);
  for (const L of labels) {
    let vis = L.on;
    if (vis) { _v.copy(L.anchor()).project(camera); vis = _v.z < 1 && Math.abs(_v.x) < 1.1 && Math.abs(_v.y) < 1.1;
      if (vis) L.el.style.transform = `translate(${((_v.x + 1) / 2 * W).toFixed(1)}px, ${((1 - _v.y) / 2 * H).toFixed(1)}px) translate(-9px, -50%)`; }
    L.el.classList.toggle('on', vis);
  }
}
window.__step?.('描画');
requestAnimationFrame(() => { frame(); document.getElementById('loading').classList.add('done'); });
window.__hoya = { robot, sceneAt, camAt, scrollS, HOME_Q, debugCam: null, poseArm, JOBS, W2 };
