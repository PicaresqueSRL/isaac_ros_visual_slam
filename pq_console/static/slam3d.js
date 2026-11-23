// SLAM view in 3D (three.js): trajectories, landmarks, loop closures, current positions.
// Frames: odometry/map of pq_cuvslam, z up. app.js calls window.slam3d.update(slam) with the
// accumulated state; without WebGL window.slam3d.available is false and app.js draws the 2D view.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const COLORS = ['#e5484d', '#3b82f6', '#22a559', '#f59e0b', '#a855f7', '#14b8a6', '#64748b'];
const LANDMARK_COLORS = ['#7c8796', '#5b8def', '#4cb37a'];

let scene, camera, renderer, orbit, container;
let tracks = {};          // topic -> {line, positions, count, marker}
let clouds = {};          // topic -> {points, number}
let loops = null;         // loop closure markers
let loopCount = 0;
let generation = null;
let view = '3d';
let fitted = false;

function init() {
  THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
  container = $('slam3d');
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  } catch (e) {
    window.slam3d.available = false;
    return false;
  }
  renderer.setPixelRatio(window.devicePixelRatio);
  container.appendChild(renderer.domElement);
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(50, 1, 0.05, 2000);
  scene.add(new THREE.AmbientLight(0xffffff, 1.0));
  const grid = new THREE.GridHelper(200, 200, 0x8a8f98, 0x8a8f98);
  grid.rotation.x = Math.PI / 2;
  grid.material.transparent = true;
  grid.material.opacity = 0.18;
  scene.add(grid);
  scene.add(new THREE.AxesHelper(1));
  orbit = new OrbitControls(camera, renderer.domElement);
  orbit.enableDamping = true;
  setView('3d');
  new ResizeObserver(resize).observe(container);
  resize();
  animate();
  return true;
}

function resize() {
  const w = container.clientWidth, h = container.clientHeight;
  if (!w || !h) return;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
}

function animate() {
  requestAnimationFrame(animate);
  if (container.offsetParent === null) return;   // tab hidden
  orbit.update();
  renderer.render(scene, camera);
}

function clear() {
  for (const t of Object.values(tracks)) { scene.remove(t.line); scene.remove(t.marker); }
  for (const c of Object.values(clouds)) scene.remove(c.points);
  if (loops) scene.remove(loops);
  tracks = {};
  clouds = {};
  loops = null;
  loopCount = 0;
  fitted = false;
}

function track(topic, index) {
  if (tracks[topic]) return tracks[topic];
  const color = new THREE.Color(COLORS[index % COLORS.length]);
  const positions = new Float32Array(3 * 4096);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geo.setDrawRange(0, 0);
  const line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color }));
  line.frustumCulled = false;
  const marker = new THREE.Mesh(new THREE.SphereGeometry(0.08, 16, 12), new THREE.MeshBasicMaterial({ color }));
  marker.visible = false;
  scene.add(line, marker);
  return (tracks[topic] = { line, positions, count: 0, marker });
}

function append(t, pts) {
  if (!pts.length) return;
  let pos = t.positions;
  if (3 * (t.count + pts.length) > pos.length) {   // grow the buffer
    const bigger = new Float32Array(Math.max(pos.length * 2, 3 * (t.count + pts.length) + 3 * 4096));
    bigger.set(pos.subarray(0, 3 * t.count));
    pos = t.positions = bigger;
    t.line.geometry.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  }
  for (const p of pts) {
    pos.set([p[1], p[2], p[3]], 3 * t.count);
    t.count++;
  }
  const geo = t.line.geometry;
  geo.attributes.position.needsUpdate = true;
  geo.setDrawRange(0, t.count);
  geo.computeBoundingSphere();
  const last = pts[pts.length - 1];
  t.marker.position.set(last[1], last[2], last[3]);
  t.marker.visible = true;
}

function setCloud(topic, index, l) {
  const old = clouds[topic];
  if (old && old.number === l.number) return;
  if (old) scene.remove(old.points);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(l.xy, 3));
  const mat = new THREE.PointsMaterial({ color: LANDMARK_COLORS[index % LANDMARK_COLORS.length], size: 0.05,
    sizeAttenuation: true, transparent: true, opacity: 0.8 });
  const points = new THREE.Points(geo, mat);
  points.visible = $('view-landmarks').checked;
  scene.add(points);
  clouds[topic] = { points, number: l.number };
}

function setLoops(list) {
  if (list.length === loopCount) return;
  if (loops) scene.remove(loops);
  loopCount = list.length;
  const geo = new THREE.SphereGeometry(0.15, 16, 12);
  loops = new THREE.InstancedMesh(geo, new THREE.MeshBasicMaterial({ color: 0xffc800 }), Math.max(1, list.length));
  const m = new THREE.Matrix4();
  list.forEach((l, i) => { m.makeTranslation(l.position[0], l.position[1], l.position[2]); loops.setMatrixAt(i, m); });
  loops.count = list.length;
  scene.add(loops);
}

// The trajectory followed by the view: fused, else guarded, else the first one.
function main() {
  const names = Object.keys(tracks).filter((t) => tracks[t].count);
  return names.find((t) => t.endsWith('odometry_fused')) || names.find((t) => t.endsWith('odometry_guarded')) || names[0];
}

function lastPosition() {
  const t = tracks[main()];
  return t ? t.marker.position.clone() : null;
}

function bounds() {
  const box = new THREE.Box3();
  for (const t of Object.values(tracks)) if (t.count) box.expandByObject(t.line);
  if (box.isEmpty()) for (const c of Object.values(clouds)) box.expandByObject(c.points);
  return box;
}

function setView(v) {
  view = v;
  const target = orbit.target.clone();
  const box = bounds();
  const size = box.isEmpty() ? 6 : Math.max(4, box.getSize(new THREE.Vector3()).length());
  if (v === 'top') camera.position.set(target.x, target.y - 0.001, target.z + size * 1.1);
  else camera.position.set(target.x - size * 0.6, target.y - size * 0.6, target.z + size * 0.5);
  orbit.update();
  $('view-3d').classList.toggle('secondary', v !== '3d');
  $('view-top').classList.toggle('secondary', v !== 'top');
}

function fit() {
  const box = bounds();
  if (box.isEmpty()) return;
  const c = box.getCenter(new THREE.Vector3());
  orbit.target.copy(c);
  setView(view);
}

window.slam3d = {
  available: true,
  update(slam) {
    if (!renderer && !init()) return;
    if (slam.generation !== generation) { clear(); generation = slam.generation; }
    const before = lastPosition();
    Object.entries(slam.paths).forEach(([topic, pts], i) => {
      const t = track(topic, i);
      append(t, pts.slice(t.count));
    });
    Object.entries(slam.landmarks || {}).forEach(([topic, l], i) => setCloud(topic, i, l));
    setLoops(slam.loop_closures || []);
    const now = lastPosition();
    if (!fitted && now) { fitted = true; orbit.target.copy(now); setView(view); }
    else if ($('view-follow').checked && before && now) {
      // Keep the camera at the same offset from the moving position.
      const d = now.clone().sub(before);
      orbit.target.add(d);
      camera.position.add(d);
    }
  },
};

$('view-3d').onclick = () => window.slam3d.available && renderer && setView('3d');
$('view-top').onclick = () => window.slam3d.available && renderer && setView('top');
$('view-fit').onclick = () => renderer && fit();
$('view-landmarks').onchange = () => { for (const c of Object.values(clouds)) c.points.visible = $('view-landmarks').checked; };
