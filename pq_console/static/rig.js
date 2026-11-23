// Rig 3D: cameras in space (three.js). Frames follow ROS: z up; the pose of a camera is the pose
// of its link (x forward, y left, z up) in the rig frame, rotation roll pitch yaw about the fixed
// x y z axes (R = Rz Ry Rx, three.js Euler order 'ZYX').
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { CSS2DObject, CSS2DRenderer } from 'three/addons/renderers/CSS2DRenderer.js';

const $ = (id) => document.getElementById(id);
const DEG = Math.PI / 180;
const COLORS = { ir: 0x3b82f6, rgb: 0xf59e0b, imu: 0xe5484d };

let models = {};
let rig = null;           // {name, description, cameras: [{name, model, position, rotation}]}
let activeName = '';
let selected = null;
let objects = {};         // camera name -> THREE.Group
let scene, camera, renderer, labels, orbit, gizmo, container;
let started = false;
let visible = false;

async function api(path, body) {
  const r = await fetch(path, body ? { method: 'POST', body: JSON.stringify(body) } : { cache: 'no-store' });
  const j = await r.json();
  if (!r.ok) throw new Error(j.error || r.statusText);
  return j;
}
function error(msg) {
  const e = $('error');
  e.textContent = msg;
  e.classList.toggle('hidden', !msg);
}
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

// ---- scene ----
function init() {
  THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
  container = $('rig3d');
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(45, 1, 0.005, 200);
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  } catch (e) {
    // No WebGL: the panel (values, import, overlap) still works without the drawing.
    container.innerHTML = '<p class="muted" style="padding:12px">3D view not available: WebGL is off in this browser.</p>';
    return;
  }
  renderer.setPixelRatio(window.devicePixelRatio);
  container.appendChild(renderer.domElement);
  labels = new CSS2DRenderer();
  labels.domElement.style.position = 'absolute';
  labels.domElement.style.top = '0';
  labels.domElement.style.pointerEvents = 'none';
  container.appendChild(labels.domElement);

  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const sun = new THREE.DirectionalLight(0xffffff, 0.8);
  sun.position.set(1, -2, 3);
  scene.add(sun);
  const grid = new THREE.GridHelper(4, 40, 0x888888, 0x888888);
  grid.rotation.x = Math.PI / 2;     // XY plane (z up)
  grid.material.opacity = 0.25;
  grid.material.transparent = true;
  scene.add(grid);
  scene.add(new THREE.AxesHelper(0.25));

  orbit = new OrbitControls(camera, renderer.domElement);
  gizmo = new TransformControls(camera, renderer.domElement);
  gizmo.setSize(0.7);
  gizmo.addEventListener('dragging-changed', (e) => { orbit.enabled = !e.value; });
  gizmo.addEventListener('objectChange', () => { fromObject(); });
  scene.add(gizmo);
  home();

  // Click (not drag) on a camera body selects it.
  let down = null;
  renderer.domElement.addEventListener('pointerdown', (e) => { down = [e.clientX, e.clientY]; });
  renderer.domElement.addEventListener('pointerup', (e) => {
    if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4 || gizmo.dragging) return;
    const r = renderer.domElement.getBoundingClientRect();
    const ray = new THREE.Raycaster();
    ray.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), camera);
    const hit = ray.intersectObjects(Object.values(objects).map((g) => g.userData.body), false)[0];
    if (hit) select(hit.object.userData.camera);
  });
  new ResizeObserver(resize).observe(container);
  resize();
  animate();
}

function resize() {
  if (!renderer) return;
  const w = container.clientWidth, h = container.clientHeight;
  if (!w || !h) return;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
  labels.setSize(w, h);
}

function animate() {
  requestAnimationFrame(animate);
  if (!visible || !renderer) return;
  orbit.update();
  renderer.render(scene, camera);
  labels.render(scene, camera);
}

function home() {
  camera.position.set(-0.9, 0.9, 0.6);
  orbit.target.set(0, 0, 0);
  orbit.update();
}

// ---- cameras ----
function frustum(sensor, length, color) {
  const [h, v] = sensor.fov.map((a) => Math.tan(a * DEG / 2));
  const o = new THREE.Vector3(...sensor.position);
  const c = [[1, h, v], [1, -h, v], [1, -h, -v], [1, h, -v]].map(([x, y, z]) => o.clone().add(new THREE.Vector3(x, y, z).multiplyScalar(length)));
  const pts = [];
  for (let i = 0; i < 4; i++) pts.push(o, c[i], c[i], c[(i + 1) % 4]);
  const geo = new THREE.BufferGeometry().setFromPoints(pts);
  return new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.8 }));
}

function build(cam) {
  const model = models[cam.model];
  const g = new THREE.Group();
  const b = model.body;
  const body = new THREE.Mesh(new THREE.BoxGeometry(...b.size),
    new THREE.MeshStandardMaterial({ color: 0x5b6472, roughness: 0.6 }));
  body.position.set(...b.center);
  body.userData.camera = cam.name;
  g.add(body);
  g.userData.body = body;
  g.add(new THREE.AxesHelper(0.06));
  const len = +$('rig-frustum').value;
  for (const s of model.sensors) {
    const dot = new THREE.Mesh(s.kind === 'imu' ? new THREE.BoxGeometry(0.006, 0.006, 0.006) : new THREE.SphereGeometry(0.004, 12, 8),
      new THREE.MeshBasicMaterial({ color: COLORS[s.kind] }));
    dot.position.set(...s.position);
    g.add(dot);
    if (s.fov && (s.name !== 'ir2' || $('rig-ir2').checked)) g.add(frustum(s, len, COLORS[s.kind]));
  }
  const div = document.createElement('div');
  div.className = 'rig-label';
  div.textContent = cam.name;
  const label = new CSS2DObject(div);
  label.position.set(b.center[0], b.center[1], b.center[2] + 0.04);
  g.add(label);
  place(g, cam);
  return g;
}

function place(g, cam) {
  g.position.set(...cam.position);
  g.rotation.set(cam.rotation[0] * DEG, cam.rotation[1] * DEG, cam.rotation[2] * DEG, 'ZYX');
  g.updateMatrixWorld(true);
}

function rebuild() {
  if (gizmo) gizmo.detach();
  for (const g of Object.values(objects)) {
    g.traverse((o) => { if (o.element) o.element.remove(); });
    if (scene) scene.remove(g);
  }
  objects = {};
  for (const cam of rig.cameras) {
    objects[cam.name] = build(cam);
    if (scene) scene.add(objects[cam.name]);
  }
  if (selected && !objects[selected]) selected = null;
  select(selected);
  renderList();
  overlap();
}

function select(name) {
  selected = name;
  for (const [n, g] of Object.entries(objects)) g.userData.body.material.color.set(n === name ? 0x2563eb : 0x5b6472);
  if (gizmo) { if (name && objects[name]) gizmo.attach(objects[name]); else gizmo.detach(); }
  renderList();
  renderEdit();
}

// Gizmo moved the camera: back to the rig values.
function fromObject() {
  const g = objects[selected];
  const cam = rig.cameras.find((c) => c.name === selected);
  if (!g || !cam) return;
  const e = new THREE.Euler().setFromQuaternion(g.quaternion, 'ZYX');
  cam.position = [g.position.x, g.position.y, g.position.z].map((v) => +v.toFixed(4));
  cam.rotation = [e.x, e.y, e.z].map((v) => +(v / DEG).toFixed(2));
  renderEdit();
  overlap();
}

function renderList() {
  $('rig-cams').innerHTML = rig.cameras.map((c) => `<li class="${c.name === selected ? 'selected' : ''}" data-cam="${esc(c.name)}">
    <span class="icon">■</span>${esc(c.name)}<span class="meta">${esc(c.model)}</span></li>`).join('');
  $('rig-cams').querySelectorAll('li').forEach((li) => li.onclick = () => select(li.dataset.cam));
}

function renderEdit() {
  const cam = rig && rig.cameras.find((c) => c.name === selected);
  if (!cam) { $('rig-edit').textContent = 'click a camera'; return; }
  const f = (label, key, i, step) => `<span>${label}</span><input type="number" step="${step}" data-k="${key}" data-i="${i}" value="${cam[key][i]}">`;
  $('rig-edit').innerHTML = `<span>name</span><input type="text" id="rig-cam-name" value="${esc(cam.name)}"><span>model</span><span>${esc(cam.model)}</span>
    ${f('x (m)', 'position', 0, 0.001)}${f('roll (°)', 'rotation', 0, 0.1)}
    ${f('y (m)', 'position', 1, 0.001)}${f('pitch (°)', 'rotation', 1, 0.1)}
    ${f('z (m)', 'position', 2, 0.001)}${f('yaw (°)', 'rotation', 2, 0.1)}`;
  $('rig-edit').querySelectorAll('input[data-k]').forEach((i) => i.onchange = () => {
    cam[i.dataset.k][+i.dataset.i] = +i.value;
    place(objects[cam.name], cam);
    overlap();
  });
  $('rig-cam-name').onchange = (e) => {
    const name = e.target.value.trim();
    if (!/^[A-Za-z0-9][A-Za-z0-9_.-]*$/.test(name) || rig.cameras.some((c) => c.name === name)) { error('invalid or repeated camera name'); return; }
    cam.name = name;
    selected = name;
    rebuild();
  };
}

// Share of the view of camera a (sensor kind) seen by camera b at 2-4 m, as the frustum overlap
// check of cuVSLAM (points sampled in the view of a, at 2, 3 and 4 m).
function share(a, b, kind) {
  const sa = models[a.model].sensors.find((s) => s.kind === kind && s.fov);
  const sb = models[b.model].sensors.find((s) => s.kind === kind && s.fov);
  if (!sa || !sb) return null;
  const ga = objects[a.name], gb = objects[b.name];
  const toB = new THREE.Matrix4().copy(gb.matrixWorld).invert().multiply(ga.matrixWorld);
  const [ha, va] = sa.fov.map((x) => Math.tan(x * DEG / 2));
  const [hb, vb] = sb.fov.map((x) => Math.tan(x * DEG / 2));
  const ob = new THREE.Vector3(...sb.position);
  let inside = 0, n = 0;
  for (const d of [2, 3, 4]) {
    for (let i = 0; i <= 14; i++) {
      for (let j = 0; j <= 9; j++) {
        const y = (i / 14 * 2 - 1) * ha, z = (j / 9 * 2 - 1) * va;
        const p = new THREE.Vector3(d, d * y, d * z).add(new THREE.Vector3(...sa.position)).applyMatrix4(toB).sub(ob);
        n++;
        if (p.x > 0.05 && Math.abs(p.y / p.x) <= hb && Math.abs(p.z / p.x) <= vb) inside++;
      }
    }
  }
  return Math.round(100 * inside / n);
}

function overlap() {
  const cams = rig.cameras;
  const rows = [];
  for (let i = 0; i < cams.length; i++) {
    for (let k = i + 1; k < cams.length; k++) {
      const ir = share(cams[i], cams[k], 'ir'), irr = share(cams[k], cams[i], 'ir');
      const rgb = share(cams[i], cams[k], 'rgb'), rgbr = share(cams[k], cams[i], 'rgb');
      const fmt = (x) => x == null ? '—' : `<span style="color:var(--${x > 50 ? 'ok' : 'warn'})">${x}%</span>`;
      rows.push(`<tr><td>${esc(cams[i].name)} / ${esc(cams[k].name)}</td><td>${fmt(ir)} / ${fmt(irr)}</td><td>${fmt(rgb)} / ${fmt(rgbr)}</td></tr>`);
    }
  }
  $('rig-overlap').innerHTML = rows.length ? `<table><thead><tr><th>Pair</th><th>IR</th><th>RGB</th></tr></thead><tbody>${rows.join('')}</tbody></table>
    <p class="small-text">Share of the view of one camera seen by the other. cuVSLAM Multisensor without depth needs &gt; 50 %
    for at least one pair; IR stereo pairs and depth do not need it.</p>` : 'one camera only';
}

// ---- rigs ----
async function loadList(select) {
  const r = await api('/api/rigs');
  models = r.models;
  activeName = r.active;
  $('rig-list').innerHTML = r.names.map((n) => `<option>${esc(n)}</option>`).join('');
  $('rig-list').value = select || r.active;
  $('rig-model').innerHTML = Object.entries(models).map(([k, m]) => `<option value="${esc(k)}">${esc(m.label)}</option>`).join('');
  $('rig-active').innerHTML = `rig used by the SLAM profiles built from sensors: <b>${esc(r.active)}</b>`;
}

async function loadRig(name) {
  try {
    rig = await api('/api/rig?name=' + encodeURIComponent(name));
    $('rig-name').value = rig.name;
    selected = null;
    rebuild();
  } catch (e) { error(e.message); }
}

$('rig-load').onclick = () => loadRig($('rig-list').value);
$('rig-save').onclick = async () => {
  try {
    rig.name = $('rig-name').value.trim();
    rig = await api('/api/rig/save', { rig, activate: false });
    await loadList(rig.name);
    error('');
  } catch (e) { error('save: ' + e.message); }
};
$('rig-activate').onclick = async () => {
  try {
    rig.name = $('rig-name').value.trim();
    rig = await api('/api/rig/save', { rig, activate: true });
    await loadList(rig.name);
    error('');
  } catch (e) { error('use: ' + e.message); }
};
$('rig-import').onchange = async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  try {
    rig = await api('/api/rig/import', { filename: file.name, text: await file.text() });
    $('rig-name').value = rig.name;
    selected = null;
    rebuild();
    error('');
  } catch (err) { error('import: ' + err.message); }
  e.target.value = '';
};
$('rig-export').onclick = () => {
  const n = (v) => +(+v).toFixed(6);
  const text = `# Camera rig of pq_console: pose of each <camera>_link in the rig frame\n` +
    `# (position m, rotation roll pitch yaw deg about the fixed x y z axes).\n` +
    `name: ${rig.name}\ndescription: ${JSON.stringify(rig.description || '')}\ncameras:\n` +
    rig.cameras.map((c) => `- {name: ${c.name}, model: ${c.model}, position: [${c.position.map(n).join(', ')}], rotation: [${c.rotation.map(n).join(', ')}]}`).join('\n') + '\n';
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type: 'text/yaml' }));
  a.download = `${rig.name}.yaml`;
  a.click();
};
$('rig-add').onclick = () => {
  let i = rig.cameras.length + 1;
  while (rig.cameras.some((c) => c.name === `cam${i}`)) i++;
  rig.cameras.push({ name: `cam${i}`, model: $('rig-model').value, position: [0, 0.2 * rig.cameras.length, 0], rotation: [0, 0, 0] });
  selected = `cam${i}`;
  rebuild();
};
$('rig-remove').onclick = () => {
  if (!selected || rig.cameras.length < 2) return;
  rig.cameras = rig.cameras.filter((c) => c.name !== selected);
  selected = null;
  rebuild();
};
$('rig-move').onclick = () => { if (!gizmo) return; gizmo.setMode('translate'); gizmo.setSpace('world'); $('rig-move').classList.remove('secondary'); $('rig-rotate').classList.add('secondary'); };
$('rig-rotate').onclick = () => { if (!gizmo) return; gizmo.setMode('rotate'); gizmo.setSpace('local'); $('rig-rotate').classList.remove('secondary'); $('rig-move').classList.add('secondary'); };
$('rig-home').onclick = () => { if (orbit) home(); };
$('rig-frustum').oninput = () => rig && rebuild();
$('rig-ir2').onchange = () => rig && rebuild();

// Called by app.js when the tab is shown or hidden.
window.rigTab = async (show) => {
  visible = show;
  if (!show) return;
  if (!started) {
    started = true;
    try {
      init();
      await loadList();
      await loadRig(activeName);
    } catch (e) { error('rig: ' + e.message); }
  }
  resize();
};
if (window.rigPending) window.rigTab(true);
