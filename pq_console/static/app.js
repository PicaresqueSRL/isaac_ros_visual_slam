// pq console. Tabs: bags (browse, view, SLAM), cameras (live, recording, settings), rig 3D, graph.
// State is polled from /api/state (1 s), /api/slam (1 s on the bags tab) and /api/topics (2 s).
'use strict';

const $ = (id) => document.getElementById(id);
const COLORS = ['#e5484d', '#3b82f6', '#22a559', '#f59e0b', '#a855f7', '#14b8a6', '#64748b'];
// Image streams of a D455 (RealSense driver), in display order.
const CAM_STREAMS = [['infra1/image_rect_raw', 'IR left'], ['infra2/image_rect_raw', 'IR right'],
  ['color/image_raw', 'RGB'], ['depth/image_rect_raw', 'depth (IR1)'],
  ['aligned_depth_to_color/image_raw', 'depth (RGB)']];
const TABS = ['bags', 'cameras', 'rig', 'graph'];

let tab = 'bags';
let state = null;
let topics = { nodes: [], topics: [] };
let slam = { generation: null, paths: {}, since: {}, landmarks: {} };
let bag = null;           // content of the loaded bag (/api/bag)
let lastJobState = null;

// ---- helpers ----
function load(key, fallback) {
  try { const v = localStorage.getItem('pq_console.' + key); return v === null ? fallback : JSON.parse(v); }
  catch (e) { return fallback; }
}
function save(key, value) {
  try { localStorage.setItem('pq_console.' + key, JSON.stringify(value)); } catch (e) { /* private mode */ }
}
function showError(msg) {
  $('error').textContent = msg;
  $('error').classList.toggle('hidden', !msg);
}
$('error').onclick = () => showError('');
// Errors of the page itself in the banner (instead of only the browser console).
window.addEventListener('error', (e) => showError('page error: ' + e.message));
window.addEventListener('unhandledrejection', (e) => showError('page error: ' + (e.reason && e.reason.message || e.reason)));

async function get(path) {
  const r = await fetch(path, { cache: 'no-store' });
  const j = await r.json();
  if (!r.ok) throw new Error(j.error || r.statusText);
  return j;
}
async function post(name, body = {}) {
  try {
    const r = await fetch('/api/' + name, { method: 'POST', body: JSON.stringify(body),
      headers: { 'Content-Type': 'application/json' } });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || r.statusText);
    showError('');
    return j;
  } catch (e) { showError(name + ': ' + e.message); throw e; }
}
const q = (path, params) => path + '?' + new URLSearchParams(params).toString();
const fmtBytes = (b) => b == null ? '' : b > 1e9 ? (b / 1e9).toFixed(1) + ' GB' : b > 1e6 ? (b / 1e6).toFixed(0) + ' MB' : (b / 1e3).toFixed(0) + ' kB';
const fmtDur = (s) => { s = Math.round(s || 0); return Math.floor(s / 60) + ':' + String(s % 60).padStart(2, '0'); };
const fmtDate = (t) => t ? new Date(t * 1000).toLocaleString([], { dateStyle: 'short', timeStyle: 'short' }) : '';
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const chip = (text, cls = '') => `<span class="chip ${cls}">${esc(text)}</span>`;

// Image previews: one request (/api/frames) for all the visible <img data-topic>, then the next
// one 120 ms after the answer; only new frames are sent. The server takes the latest image of
// each topic at that moment (it does not receive the streams continuously).
const frameSeq = {};
const frameInfo = {};
async function framesLoop() {
  const imgs = [...document.querySelectorAll('img[data-topic]')].filter((i) => i.dataset.topic && i.offsetParent !== null);
  const wanted = [...new Set(imgs.map((i) => i.dataset.topic))];
  if (wanted.length) {
    try {
      const r = await get(q('/api/frames', { topics: wanted.join(','), have: wanted.map((t) => frameSeq[t] || 0).join(',') }));
      for (const [t, f] of Object.entries(r)) {
        frameInfo[t] = f.info;
        if (f.jpeg) {
          frameSeq[t] = f.seq;
          const src = 'data:image/jpeg;base64,' + f.jpeg;
          imgs.filter((i) => i.dataset.topic === t).forEach((i) => { i.src = src; });
        }
      }
    } catch (e) { /* console restarting */ }
  }
  // About 8 fps for a few previews, 5 fps for eight: the cost is per decoded image.
  setTimeout(framesLoop, wanted.length ? Math.max(120, 25 * wanted.length) : 400);
}
function infoText(info) {
  if (!info) return 'waiting…';
  return [info.hz != null ? `${info.hz} fps` : '', info.width ? `${info.width}×${info.height}` : '',
    info.saturated_pct != null ? `saturated ${info.saturated_pct}%` : '',
    info.valid_depth_pct != null ? `valid depth ${info.valid_depth_pct}%` : '', info.error || '']
    .filter(Boolean).join(' · ');
}

// Order of the cameras in the pages (cam1 first, or swapped).
const camOrder = (names) => { const n = [...names].sort(); return load('swap_cams', false) ? n.reverse() : n; };

// ---- tabs ----
document.querySelectorAll('button.tab').forEach((b) => b.onclick = () => setTab(b.dataset.tab));
function setTab(t) {
  tab = t;
  save('tab', t);
  document.querySelectorAll('button.tab').forEach((b) => b.classList.toggle('active', b.dataset.tab === t));
  document.querySelectorAll('.tab-body').forEach((s) => s.classList.toggle('hidden', s.id !== 'tab-' + t));
  renderLivePreviews();
  renderBagPreviews();
  if (t === 'graph') refreshTopics();
  // rig.js is a module (three.js), loaded after this script: it picks up rigPending.
  if (window.rigTab) window.rigTab(t === 'rig'); else window.rigPending = t === 'rig';
  if (t === 'cameras') { refreshCameras(); if ($('settings-card').open) refreshSettings(); }
}

// ---- state ----
async function refreshState() {
  try { state = await get('/api/state'); } catch (e) { $('status').innerHTML = chip('console unreachable', 'bad'); return; }
  renderStatus();
  renderCameras();
  renderRecording();
  renderJob();
}

function renderStatus() {
  const s = state;
  const parts = [chip(s.cameras_running ? 'cameras on' : 'cameras off', s.cameras_running ? 'ok' : '')];
  for (const [cam, v] of Object.entries(s.emitters || {})) {
    parts.push(chip(`${cam} emitter ${v === null ? '?' : v ? 'ON' : 'off'}`, v ? 'bad' : v === null ? 'warn' : 'ok'));
  }
  if (s.recording) parts.push(chip('recording ' + s.recording.name, 'bad'));
  if (s.job && s.job.state === 'running') {
    parts.push(chip((s.job.kind === 'view' ? 'playing ' : 'SLAM ') + s.job.name, 'warn'));
  }
  if (s.disk) parts.push(chip(fmtBytes(s.disk.free) + ' free', s.disk.free < 20e9 ? 'bad' : ''));
  $('status').innerHTML = parts.join('');
}

// =====================================================================================
// Bags
// =====================================================================================
async function browse(path) {
  let r;
  try { r = await get(q('/api/browse', { path })); } catch (e) {
    if (path) return browse('');
    showError(e.message);
    return;
  }
  save('browse', r.path);
  const crumbs = ['<a data-path="">roots</a>'];
  if (r.path) {
    const root = r.roots.find((x) => r.path === x || r.path.startsWith(x.replace(/\/$/, '') + '/')) || '';
    crumbs.push(`<a data-path="${esc(root)}">${esc(root)}</a>`);
    let acc = root.replace(/\/$/, '');
    for (const part of r.path.slice(root.length).split('/').filter(Boolean)) {
      acc += '/' + part;
      crumbs.push(`<a data-path="${esc(acc)}">${esc(part)}</a>`);
    }
  }
  $('crumbs').innerHTML = crumbs.join(' / ');
  $('crumbs').querySelectorAll('a').forEach((a) => a.onclick = () => browse(a.dataset.path));
  const items = [];
  if (r.path) items.push(`<li data-dir="${esc(r.parent || '')}"><span class="icon">↑</span>..</li>`);
  if (r.is_bag) items.push(`<li class="bag" data-bag="${esc(r.path)}"><span class="icon">●</span>this bag</li>`);
  for (const e of r.entries) {
    const sel = bag && bag.path === e.path ? ' selected' : '';
    if (e.kind === 'dir') items.push(`<li data-dir="${esc(e.path)}"><span class="icon">▸</span>${esc(e.name)}</li>`);
    else items.push(`<li class="bag${sel}" data-bag="${esc(e.path)}"><span class="icon">●</span>${esc(e.name)}` +
      `<span class="meta">${e.kind === 'bagfile' ? fmtBytes(e.size) : fmtDate(e.mtime)}</span></li>`);
  }
  $('entries').innerHTML = items.join('') || '<li class="muted">empty</li>';
  $('entries').querySelectorAll('[data-dir]').forEach((li) => li.onclick = () => browse(li.dataset.dir));
  $('entries').querySelectorAll('[data-bag]').forEach((li) => li.onclick = () => loadBag(li.dataset.bag));
}

async function loadBag(path) {
  $('bag-summary').innerHTML = '<span class="muted">reading ' + esc(path) + '…</span>';
  try { bag = await get(q('/api/bag', { path, version: $('version').value || '' })); }
  catch (e) { $('bag-summary').textContent = e.message; bag = null; renderBagPreviews(); return; }
  save('bag', path);
  document.querySelectorAll('#entries li.bag').forEach((li) => li.classList.toggle('selected', li.dataset.bag === path));
  renderBag();
  renderBagPreviews();
  renderSlamSetup();
  renderRuns();
  if (state) renderJob();
}

function renderBag() {
  const b = bag;
  const camRows = camOrder(Object.keys(b.cameras)).map((cam) => [cam, b.cameras[cam]]).map(([cam, c]) => {
    const streams = c.streams.map((s) =>
      `<tr><td>${esc(cam)}</td><td>${esc(s.label)}${s.sidecar ? ' <span class="badge">_aligned bag</span>' : ''}</td>
      <td><code>${esc(s.topic)}</code></td><td class="num">${s.width ? s.width + '×' + s.height : ''}</td>
      <td class="num">${s.hz ?? ''} Hz</td><td class="num">${s.count}</td>
      <td class="muted">${s.distortion === 'none' ? '' : esc(s.distortion)}</td></tr>`);
    if (c.imu) streams.push(`<tr><td>${esc(cam)}</td><td>IMU</td><td><code>${esc(c.imu.topic)}</code></td><td></td>
      <td class="num">${c.imu.hz} Hz</td><td class="num">${c.imu.count}</td><td></td></tr>`);
    return streams.join('');
  }).join('');
  const others = b.others.map((t) => `<tr><td><code>${esc(t.topic)}</code></td><td class="muted">${esc(t.type)}</td>
    <td class="num">${t.count}</td></tr>`).join('');
  let settings = '';
  if (b.camera_settings) {
    const keys = ['depth_module.emitter_enabled', 'depth_module.enable_auto_exposure', 'depth_module.exposure',
      'depth_module.gain', 'rgb_camera.enable_auto_exposure', 'rgb_camera.exposure', 'depth_module.infra_profile',
      'rgb_camera.color_profile'];
    const cams = Object.keys(b.camera_settings);
    settings = `<details><summary>camera settings of the recording</summary><table><thead><tr><th>Parameter</th>
      ${cams.map((c) => `<th>${esc(c)}</th>`).join('')}</tr></thead><tbody>${keys.map((k) =>
      `<tr><td>${esc(k)}</td>${cams.map((c) => `<td>${esc(JSON.stringify((b.camera_settings[c] || {})[k] ?? ''))}</td>`).join('')}</tr>`).join('')}
      </tbody></table></details>`;
  }
  $('bag-summary').innerHTML = `<h2>${esc(b.name)} <span class="badge">${esc(b.storage)}</span></h2>
    <div class="muted"><code>${esc(b.path)}</code></div>
    <div class="form" style="margin:6px 0">${chip(fmtDate(b.start))}${chip('duration ' + fmtDur(b.duration_s))}
      ${chip(fmtBytes(b.size))}${chip(b.messages + ' messages')}
      ${b.aligned_bag ? chip('+ aligned depth bag') : ''}</div>
    <details id="bag-streams" ${load('bag_streams_open', true) ? 'open' : ''}>
      <summary>streams (${Object.values(b.cameras).reduce((n, c) => n + c.streams.length + (c.imu ? 1 : 0), 0)})</summary>
    <table><thead><tr><th>Camera</th><th>Stream</th><th>Topic</th><th class="num">Resolution</th>
      <th class="num">Rate</th><th class="num">Messages</th><th>Distortion</th></tr></thead><tbody>${camRows ||
      '<tr><td colspan="7" class="muted">no camera streams</td></tr>'}</tbody></table></details>
    ${others ? `<details><summary>other topics (${b.others.length})</summary><table>${others}</table></details>` : ''}
    ${settings}`;
  $('bag-streams').ontoggle = () => save('bag_streams_open', $('bag-streams').open);
}

// Previews of all the image streams of the bag, one row per camera.
function renderBagPreviews() {
  const el = $('bag-previews');
  if (!bag) { el.innerHTML = ''; el.dataset.key = ''; return; }
  const j = state && state.job;
  const playing = j && j.state === 'running' && j.meta && j.meta.path === bag.path;
  const show = tab === 'bags' && playing;
  const order = camOrder(Object.keys(bag.cameras));
  const key = JSON.stringify([bag.path, show, order]);
  if (el.dataset.key !== key) {
    el.dataset.key = key;
    el.innerHTML = order.map((cam) => [cam, bag.cameras[cam]]).filter(([, c]) => c.streams.length).map(([cam, c]) =>
      `<div class="cam-row"><h3>${esc(cam)}</h3><div class="tiles">${c.streams.map((s) =>
        `<div class="tile" data-topic="${esc(s.topic)}"><div class="label"><span>${esc(s.label)}</span>
         <span class="muted">${s.width ? s.width + '×' + s.height : ''}</span></div>
         <div class="frame" ${s.width ? `style="aspect-ratio:${s.width}/${s.height}"` : ''}>${show ? `<img data-topic="${esc(s.topic)}" alt="">` : 'press Play'}</div>
         <div class="info"></div></div>`).join('')}</div></div>`).join('');
  }
  if (!show) return;
  el.querySelectorAll('.tile').forEach((t) => {
    t.querySelector('.info').textContent = infoText(frameInfo[t.dataset.topic]);
  });
}

// ---- player ----
let seeking = false;
function renderJob() {
  const j = state.job;
  const running = j && j.state === 'running';
  const m = (j && j.meta) || {};
  const mine = running && bag && m.path === bag.path;
  const view = mine && j.kind === 'view';
  const busy = running || state.cameras_running || !!state.recording;
  // One play/pause button: starts the view of the bag, then pauses and resumes it.
  const playing = view && !m.paused;
  $('pb-play').disabled = view ? false : busy || !bag;
  $('pb-play').classList.toggle('playing', playing || (mine && j.kind === 'replay'));
  $('pb-play').title = playing ? 'Pause' : view ? 'Resume' : 'Play';
  $('pb-stop').disabled = !running;
  $('pb-seek').disabled = !view;
  if (mine && m.rate) $('pb-rate').value = String(m.rate);
  const dur = mine ? m.duration_s || 0 : bag ? bag.duration_s : 0;
  const pos = mine && state.clock && m.start ? Math.min(Math.max(state.clock - m.start, 0), dur) : 0;
  $('pb-seek').max = Math.max(dur, 0.1);
  if (!seeking) $('pb-seek').value = pos;
  $('pb-pos').textContent = fmtDur(pos);
  $('pb-dur').textContent = fmtDur(dur);
  const last = j && j.log.length ? j.log[j.log.length - 1].slice(9) : '';
  $('job-line').textContent = !j ? (state.cameras_running ? 'cameras running: stop them to play a bag' : '')
    : `${j.kind === 'view' ? 'view' : 'SLAM'} ${j.name}: ${j.state}` +
      (running ? ' · ' + fmtDur(state.time - j.started) : '') + (last && last !== j.state ? ' — ' + last : '');
  $('job-log').textContent = j && j.kind === 'replay' ? j.log.join('\n') : '';
  $('slam-start').disabled = busy || !bag;
  $('slam-stop').disabled = !(running && j.kind === 'replay');
  renderOutput(j);
  if (j && lastJobState === 'running' && j.state !== 'running' && bag) loadBag(bag.path);
  lastJobState = j ? j.state : null;
  renderBagPreviews();
}
$('pb-play').onclick = () => {
  const j = state && state.job;
  if (j && j.state === 'running' && j.kind === 'view') {
    post('player', { command: j.meta.paused ? 'resume' : 'pause' }).then(refreshState);
  } else {
    post('view/start', { path: bag.path, rate: +$('pb-rate').value })
      .then(() => { slam = { generation: null, paths: {}, since: {}, landmarks: {} }; refreshState(); });
  }
};
$('pb-stop').onclick = () => post('job/stop').then(refreshState);
$('swap-cams').onclick = () => {
  save('swap_cams', !load('swap_cams', false));
  if (bag) renderBag();
  renderBagPreviews();
  renderLivePreviews();
};
$('pb-rate').onchange = () => {
  if (state && state.job && state.job.state === 'running') post('player', { command: 'rate', value: +$('pb-rate').value });
};
$('pb-seek').oninput = () => { seeking = true; };
$('pb-seek').onchange = async () => {
  try { await post('player', { command: 'seek', value: state.job.meta.start + +$('pb-seek').value }); }
  finally { setTimeout(() => { seeking = false; }, 1500); }
};

// ---- SLAM setup ----
async function loadVersions() {
  try {
    const v = await get('/api/versions');
    $('version').innerHTML = v.map((x) => `<option>${esc(x)}</option>`).join('');
    const saved = load('version', '');
    if (v.includes(saved)) $('version').value = saved;
  } catch (e) { showError(e.message); }
}
$('version').onchange = () => { save('version', $('version').value); if (bag) loadBag(bag.path); };
const slamSource = () => document.querySelector('input[name=slam-src]:checked').value;
document.querySelectorAll('input[name=slam-src]').forEach((r) => r.onchange = () => {
  save('slam_src', slamSource());
  renderSlamSetup();
});

function renderSlamSetup() {
  const src = slamSource();
  $('slam-profile').classList.toggle('hidden', src !== 'profile');
  $('slam-sensors').classList.toggle('hidden', src !== 'sensors');
  if (!bag) return;
  const ok = bag.profiles.filter((p) => p.ok);
  const saved = load('profile', '');
  $('profile').innerHTML = bag.profiles.map((p) => `<option value="${esc(p.name)}" ${p.ok ? '' : 'disabled'}
    title="${esc(p.ok ? p.description : 'missing: ' + p.missing.join(', '))}">${esc(p.name)}${p.ok ? '' : ' (missing topics)'}</option>`).join('');
  if (ok.some((p) => p.name === saved)) $('profile').value = saved;
  else if (ok.length) $('profile').value = ok[0].name;
  showProfile();
  // Sensors: one row per camera, only what the bag has.
  const sel = load('sensors', {});
  const rows = Object.entries(bag.cameras).map(([cam, c]) => {
    const has = (k) => c.streams.some((s) => s.key === k);
    const box = (k, avail) => avail ? `<input type="checkbox" data-cam="${esc(cam)}" data-k="${k}" ${((sel.cameras || {})[cam] || {})[k] ? 'checked' : ''}>`
      : '<span class="muted">—</span>';
    return `<tr><td>${esc(cam)}</td><td>${box('ir', has('infra1') && has('infra2'))}</td><td>${box('rgb', has('color'))}</td>
      <td>${box('depth', has('depth') || has('aligned'))}</td>
      <td>${c.imu ? `<input type="radio" name="imu" value="${esc(cam)}" ${sel.imu === cam ? 'checked' : ''}>` : '<span class="muted">—</span>'}</td></tr>`;
  }).join('');
  $('sensors').innerHTML = `<thead><tr><th>Camera</th><th>IR stereo</th><th>RGB</th><th>Depth</th><th>IMU</th></tr></thead>
    <tbody>${rows}<tr><td class="muted">no IMU</td><td></td><td></td><td></td>
    <td><input type="radio" name="imu" value="" ${sel.imu ? '' : 'checked'}></td></tr></tbody>`;
  $('sensors').querySelectorAll('input').forEach((i) => i.onchange = () => { save('sensors', selection()); planSensors(); });
  if (src === 'sensors') planSensors();
}
function selection() {
  const cams = {};
  $('sensors').querySelectorAll('input[type=checkbox]').forEach((i) => {
    (cams[i.dataset.cam] = cams[i.dataset.cam] || {})[i.dataset.k] = i.checked;
  });
  const imu = $('sensors').querySelector('input[name=imu]:checked');
  return { cameras: cams, imu: imu ? imu.value : '' };
}
let planOk = false;
async function planSensors() {
  planOk = false;
  try {
    const r = await fetch('/api/slam/plan', { method: 'POST', body: JSON.stringify({ path: bag.path, selection: selection(), version: $('version').value }) });
    const j = await r.json();
    if (!r.ok) { $('plan').innerHTML = chip(j.error, 'warn'); $('plan-yaml').textContent = ''; return; }
    planOk = true;
    $('plan').innerHTML = `cuVSLAM mode <b>${esc(j.mode)}</b> · ${esc(j.name)}` + j.notes.map((n) => '<br>' + esc(n)).join('');
    $('plan-yaml').textContent = j.yaml;
  } catch (e) { $('plan').textContent = e.message; }
}
function showProfile() {
  const p = bag && bag.profiles.find((x) => x.name === $('profile').value);
  $('profile-desc').textContent = p && p.ok ? `${p.mode}: ${p.description}` : 'no profile compatible with this bag: choose the sensors';
}
$('profile').onchange = () => { save('profile', $('profile').value); showProfile(); };
$('slam-start').onclick = async () => {
  const body = { path: bag.path, version: $('version').value, rerun: $('rerun').checked, save: $('save-results').checked };
  if (slamSource() === 'profile') body.profile = $('profile').value;
  else if (!planOk) { showError('the chosen sensors do not make a valid configuration'); return; }
  else body.selection = selection();
  await post('replay/start', body);
  slam = { generation: null, paths: {}, since: {}, landmarks: {} };
  refreshState();
};
$('slam-stop').onclick = () => confirm('Stop the SLAM?') && post('job/stop').then(refreshState);
$('save-results').checked = load('save_results', true);
$('rerun').disabled = !$('save-results').checked;
$('save-results').onchange = () => { save('save_results', $('save-results').checked); $('rerun').disabled = !$('save-results').checked; };

// What the SLAM run saves and where: shown while it runs and when it ends.
function renderOutput(j) {
  const el = $('slam-output');
  if (!j || j.kind !== 'replay' || !bag || !j.meta || j.meta.path !== bag.path || !j.meta.outputs) {
    el.classList.add('hidden');
    return;
  }
  const m = j.meta, o = m.outputs, sv = m.saved || {};
  if (m.save === false) {   // test run
    const r = m.result ? ` · ${m.result.frames} frames, path ${m.result.path_m} m, start-end ${m.result.start_end_m} m` : '';
    el.className = 'output run';
    el.innerHTML = `<b>${j.state === 'running' ? '● test run: shown live, nothing will be saved' : '■ test run finished: nothing saved'}</b>${esc(r)}`;
    return;
  }
  const item = (label, path, key) => {
    if (!path) return '';
    const mark = j.state === 'running' && !(key in sv) ? '…' : sv[key] ? '✓' : '✗';
    return `<li>${mark} ${label}: <code>${esc(path)}</code></li>`;
  };
  const phases = { starting: 'starting the SLAM', running: 'SLAM running, recording its output',
    saving: 'bag finished: writing the run output', rerun: 'run saved, creating the Rerun file', done: 'done' };
  let head, cls;
  if (j.state === 'running') { head = `● ${phases[m.phase] || m.phase} — the output is being saved`; cls = 'run'; }
  else if (j.state === 'done') { head = '✓ SLAM saved'; cls = 'ok'; }
  else if (j.state === 'stopped') { head = '■ stopped: the run output up to the stop is saved'; cls = 'ok'; }
  else { head = '✗ SLAM failed: see the log'; cls = 'bad'; }
  const r = m.result ? ` · ${m.result.frames} frames, path ${m.result.path_m} m, start-end ${m.result.start_end_m} m` : '';
  el.className = 'output ' + cls;
  el.innerHTML = `<b>${esc(head)}</b>${esc(r)}<ul>${item('run (CSV, run_info.json)', o.run, 'run')}
    ${item('SLAM output bag', o.slam_bag, 'slam_bag')}${item('Rerun file', o.rerun, 'rerun')}
    <li class="muted">log: <code>${esc(o.log)}</code></li></ul>`;
}

function renderRuns() {
  const runs = (bag && bag.runs) || [];
  const ev = (r, k) => Object.entries(r.events || {}).filter(([n]) => n.endsWith(k)).reduce((a, [, v]) => a + v, 0);
  const rej = (r) => Object.entries(r.health || {}).filter(([n]) => /rejected|lost/.test(n)).reduce((a, [, v]) => a + v, 0);
  $('runs').querySelector('tbody').innerHTML = runs.map((r) => `<tr>
    <td>${esc(r.name)}</td><td>${esc(r.mode || r.profile)}</td><td class="num">${r.frames ?? ''}</td>
    <td class="num">${r.path_m != null ? r.path_m + ' m' : ''}</td>
    <td class="num">${r.start_end_m != null ? r.start_end_m + ' m' : ''}</td>
    <td class="num">${rej(r)}</td><td class="num">${ev(r, 'tracker_reset')}</td>
    <td class="num">${ev(r, 'loop_closure')}</td><td class="num">${r.track_ms ?? ''}</td>
    <td>${r.rrd ? `<code>${esc(r.rrd)}</code>` : ''}</td><td class="num">${fmtBytes(r.size)}</td>
    <td><button class="small danger" data-delrun="${esc(r.name)}" title="Delete the run, its SLAM bag and Rerun file">🗑</button></td></tr>`).join('') ||
    '<tr><td colspan="12" class="muted">no runs</td></tr>';
  const total = runs.reduce((a, r) => a + (r.size || 0), 0);
  $('runs-size').textContent = runs.length ? `${runs.length} runs · ${fmtBytes(total)}` : '';
  $('runs').querySelectorAll('[data-delrun]').forEach((b) => b.onclick = async () => {
    if (!confirm(`Delete ${b.dataset.delrun}: run output, SLAM bag and Rerun file?`)) return;
    await post('run/delete', { name: b.dataset.delrun }).catch(() => {});
    loadBag(bag.path);
  });
}

// ---- SLAM live view ----
async function refreshSlam() {
  let s;
  const have = Object.fromEntries(Object.entries(slam.landmarks).map(([t, l]) => [t, l.number]));
  try { s = await get(q('/api/slam', { since: JSON.stringify(slam.since), landmarks: JSON.stringify(have) })); } catch (e) { return; }
  if (s.generation !== slam.generation) slam = { generation: s.generation, paths: {}, since: {}, landmarks: {} };
  Object.assign(slam.landmarks, s.landmarks || {});
  slam.loop_closures = s.loop_closures || [];
  for (const [topic, pts] of Object.entries(s.paths)) {
    const path = slam.paths[topic] || (slam.paths[topic] = []);
    path.push(...pts);
    if (path.length) slam.since[topic] = path[path.length - 1][0];
  }
  const in3d = window.slam3d && window.slam3d.available;
  if (in3d) {
    const j = state && state.job;
    const points = Object.values(slam.paths).reduce((n, p) => n + p.length, 0);
    const running = j && j.state === 'running' && j.kind === 'replay';
    window.slam3d.update(slam, !points ? (running ? 'waiting for the SLAM…' : 'no SLAM data') : '');
  }
  if (window.slam3d && !window.slam3d.available) {   // no WebGL: top view on a canvas
    $('slam3d').classList.add('hidden');
    $('traj').classList.remove('hidden');
    drawTrajectory();
  }
  renderLegend();
  renderDiag(s);
}

function renderLegend() {
  const items = Object.entries(slam.paths).filter(([, p]) => p.length).map(([topic, p], i) => {
    let len = 0;
    for (let k = 1; k < p.length; k++) len += Math.hypot(p[k][1] - p[k - 1][1], p[k][2] - p[k - 1][2], p[k][3] - p[k - 1][3]);
    return `<span style="--c:${COLORS[i % COLORS.length]}">${esc(topic)} · ${len.toFixed(1)} m</span>`;
  });
  for (const [topic, l] of Object.entries(slam.landmarks)) {
    items.push(`<span style="--c:#7c8796">${esc(topic.replace('/vis/landmarks_cloud', ''))} landmarks · ${l.xy.length / 3}</span>`);
  }
  if ((slam.loop_closures || []).length) items.push(`<span style="--c:#ffc800">loop closures · ${slam.loop_closures.length}</span>`);
  $('legend').innerHTML = items.join('');
}
// Reload: draw the SLAM again from the data the console holds (the data is kept).
$('slam-reset').onclick = () => {
  slam = { generation: null, paths: {}, since: {}, landmarks: {} };
  if (window.slam3d && window.slam3d.reset) window.slam3d.reset();
  refreshSlam();
};
$('slam-full').onclick = () => {
  if (document.fullscreenElement) document.exitFullscreen();
  else $('slam-row').requestFullscreen().catch((e) => showError('full screen: ' + e.message));
};
document.addEventListener('fullscreenchange', () => {
  $('slam-full').textContent = document.fullscreenElement ? '✕ exit full screen' : '⛶ full screen';
});

function drawTrajectory() {
  const cv = $('traj');
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  ctx.clearRect(0, 0, W, H);
  const entries = Object.entries(slam.paths).filter(([, p]) => p.length);
  const clouds = Object.entries(slam.landmarks).filter(([, l]) => l.xy.length);
  const css = getComputedStyle(document.body);
  if (!entries.length && !clouds.length) {
    ctx.fillStyle = css.getPropertyValue('--muted');
    ctx.font = '14px system-ui';
    ctx.fillText('no trajectory yet', 20, 30);
    $('legend').innerHTML = '';
    return;
  }
  // Top view (x right, y up), same scale on both axes.
  // Frame on the trajectories (landmarks can be far away); on the landmarks without trajectory.
  let minx = Infinity, maxx = -Infinity, miny = Infinity, maxy = -Infinity;
  const grow = (x, y) => { minx = Math.min(minx, x); maxx = Math.max(maxx, x); miny = Math.min(miny, y); maxy = Math.max(maxy, y); };
  for (const [, p] of entries) for (const v of p) grow(v[1], v[2]);
  if (!entries.length) for (const [, l] of clouds) for (let k = 0; k < l.xy.length; k += 3) grow(l.xy[k], l.xy[k + 1]);
  const span = Math.max(maxx - minx, maxy - miny, 2) * 1.1;
  const scale = Math.min(W, H) / span;
  const cx = (minx + maxx) / 2, cy = (miny + maxy) / 2;
  const X = (x) => W / 2 + (x - cx) * scale, Y = (y) => H / 2 - (y - cy) * scale;
  const grid = css.getPropertyValue('--border');
  ctx.strokeStyle = grid; ctx.lineWidth = 1;
  const step = span > 60 ? 10 : span > 15 ? 5 : 1;
  for (let g = Math.floor((cx - W / 2 / scale) / step) * step; g < cx + W / 2 / scale; g += step) {
    ctx.beginPath(); ctx.moveTo(X(g), 0); ctx.lineTo(X(g), H); ctx.stroke();
  }
  for (let g = Math.floor((cy - H / 2 / scale) / step) * step; g < cy + H / 2 / scale; g += step) {
    ctx.beginPath(); ctx.moveTo(0, Y(g)); ctx.lineTo(W, Y(g)); ctx.stroke();
  }
  // Landmarks (top view cells of 5 cm) under the trajectories.
  const dot = Math.max(1.5, 0.05 * scale);
  clouds.forEach(([topic, l], i) => {
    ctx.fillStyle = i ? 'rgba(59,130,246,0.35)' : 'rgba(100,116,139,0.45)';
    for (let k = 0; k < l.xy.length; k += 3) ctx.fillRect(X(l.xy[k]) - dot / 2, Y(l.xy[k + 1]) - dot / 2, dot, dot);
  });
  entries.forEach(([topic, p], i) => {
    const color = COLORS[i % COLORS.length];
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
    p.forEach((v, k) => k ? ctx.lineTo(X(v[1]), Y(v[2])) : ctx.moveTo(X(v[1]), Y(v[2])));
    ctx.stroke();
    const last = p[p.length - 1];
    ctx.fillStyle = color; ctx.beginPath(); ctx.arc(X(last[1]), Y(last[2]), 5, 0, 2 * Math.PI); ctx.fill();
  });
}

function renderDiag(s) {
  const diags = Object.entries(s.diagnostics || {});
  if (!diags.length) $('slam-diag').textContent = 'no SLAM running';
  else {
    const now = Date.now() / 1000;
    $('slam-diag').innerHTML = diags.map(([topic, d]) => {
      const stale = now - d.time > 3;
      const obs = Object.entries(d.observations || {}).map(([k, v]) => `${k} ${v}`).join(', ');
      const sat = (d.streams || []).filter((x) => x.saturated_pct > 5).map((x) => `${x.name} ${x.saturated_pct}%`).join(', ');
      return `<h3>${esc(topic.replace('/diagnostics/frame', ''))}${stale ? ' (stopped)' : ''}</h3><dl class="kv">
        <dt>mode</dt><dd>${esc(d.mode)} ${esc(d.profile)}</dd>
        <dt>tracking</dt><dd>${d.tracked ? chip('tracked', 'ok') : chip('lost', 'bad')} ${chip(d.health + (d.reasons ? ' ' + d.reasons : ''), d.health === 'ok' ? 'ok' : 'warn')}</dd>
        <dt>landmarks</dt><dd>${d.landmarks}</dd>
        <dt>observations</dt><dd>${esc(obs)}</dd>
        <dt>track()</dt><dd>${d.track_ms} ms</dd>
        ${d.speed_mps != null ? `<dt>speed</dt><dd>${d.speed_mps} m/s</dd>` : ''}
        ${d.sync_offset_ms != null ? `<dt>sync offset</dt><dd>${d.sync_offset_ms} ms</dd>` : ''}
        ${sat ? `<dt>saturated</dt><dd>${esc(sat)}</dd>` : ''}</dl>`;
    }).join('');
  }
  $('slam-events').innerHTML = (s.events || []).slice().reverse().map((e) =>
    `<li>${new Date(e.time * 1000).toLocaleTimeString()} <b>${esc(e.type)}</b> ${esc(e.description)}</li>`).join('');
}

// =====================================================================================
// Cameras: live, recording, settings
// =====================================================================================
let camInfo = null;       // /api/cameras: configured and connected cameras, presets

async function refreshCameras() {
  try { camInfo = await get('/api/cameras'); } catch (e) { return; }
  renderCameras();
  renderPresets();
}

function renderCameras() {
  const s = state;
  if (!s) return;
  const rows = ((camInfo && camInfo.configured) || s.cameras.map((name) => ({ name }))).map((c) => {
    const v = (s.emitters || {})[c.name];
    const usb = c.usb ? `${c.usb.speed_mbps >= 5000 ? chip('USB 3', 'ok') : chip('USB 2: low bandwidth', 'bad')}` :
      c.connected === false ? chip('not connected', 'bad') : c.connected === null ? chip('usb_serial not set', 'warn') : '';
    const driver = c.driver ? chip('driver running', 'ok') : chip('driver off');
    const em = !c.driver ? '' : v === null || v === undefined ? chip('emitter ?', 'warn')
      : v ? chip('emitter ON', 'bad') + ` <button class="small danger" data-emitter="${esc(c.name)}">turn off</button>` : chip('emitter off', 'ok');
    return `<tr><td><b>${esc(c.name)}</b></td><td>${esc(c.usb ? c.usb.model : '')}</td><td><code>${esc(c.serial || '')}</code></td>
      <td>${usb}</td><td>${driver}</td><td>${em}</td></tr>`;
  });
  $('cam-table').innerHTML = `<thead><tr><th>Name</th><th>Model</th><th>Serial</th><th>USB</th><th>Driver</th><th>IR emitter</th></tr></thead>
    <tbody>${rows.join('') || '<tr><td colspan="6" class="muted">no cameras in the configuration</td></tr>'}</tbody>`;
  const other = (camInfo && camInfo.other) || [];
  $('cam-other').innerHTML = other.length ? 'RealSense on USB not linked to a camera of the configuration (set its ' +
    '<code>usb_serial</code>, the USB serial is not the one of the label): ' +
    other.map((u) => `${esc(u.model)} USB serial <code>${esc(u.serial)}</code> ${u.speed_mbps >= 5000 ? 'USB 3' : 'USB 2'}`).join(', ') : '';
  $('cam-table').querySelectorAll('[data-emitter]').forEach((b) => b.onclick = () =>
    post('emitter', { camera: b.dataset.emitter, value: 0 }).then(refreshState));
  $('cam-start').disabled = s.cameras_running || !s.cameras.length || !!(s.job && s.job.state === 'running');
  $('cam-stop').disabled = !s.cameras_running || !!s.recording;
}

function renderPresets() {
  if (!camInfo) return;
  const cur = camInfo.preset || {};
  const html = Object.entries(camInfo.presets).map(([k, v]) => `<option value="${esc(k)}">${esc(v.label)}</option>`).join('');
  if ($('preset').dataset.html !== html) {
    $('preset').innerHTML = html;
    $('preset').dataset.html = html;
    $('preset').value = cur.preset || 'all';
  }
  const camsHtml = camInfo.configured.map((c) => `<label><input type="checkbox" data-reccam="${esc(c.name)}"
    ${(cur.cameras || []).includes(c.name) ? 'checked' : ''}> ${esc(c.name)}</label>`).join('');
  if ($('rec-cams').dataset.html !== camsHtml) { $('rec-cams').innerHTML = camsHtml; $('rec-cams').dataset.html = camsHtml; }
  $('streams-state').textContent = cur.preset ? `applied: ${camInfo.presets[cur.preset].label} on ${cur.cameras.join(', ')}` : '';
}
const recCameras = () => [...document.querySelectorAll('[data-reccam]')].filter((i) => i.checked).map((i) => i.dataset.reccam);
$('streams-apply').onclick = async () => {
  $('streams-apply').disabled = true;
  $('streams-state').textContent = 'switching the streams (the cameras restart them)…';
  try {
    const r = await post('streams/apply', { preset: $('preset').value, cameras: recCameras() });
    const ch = Object.entries(r.changed).map(([c, v]) => `${c}: ${v.join(', ')}`).join('; ');
    $('streams-state').textContent = ch ? 'changed ' + ch : 'streams already as the preset';
  } catch (e) { $('streams-state').textContent = ''; }
  $('streams-apply').disabled = false;
  refreshCameras();
};

function renderRecording() {
  const r = state.recording;
  $('rec-start').disabled = !!r || !state.cameras_running;
  $('rec-stop').disabled = !r;
  $('streams-apply').disabled = !!r || !state.cameras_running;
  if (!$('rec-name').value) {
    const d = new Date();
    const p = (n) => String(n).padStart(2, '0');
    $('rec-name').value = `rec_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}`;
  }
  if (!r) { $('rec-state').textContent = state.cameras_running ? 'not recording' : 'start the cameras to record'; return; }
  const el = r.started ? fmtDur(state.time - r.started) : '';
  $('rec-state').innerHTML = `${chip('REC', 'bad')} ${esc(r.name)} ${el} · ${fmtBytes(r.size)}` +
    (r.label ? ` · ${esc(r.label)} · ${esc((r.cameras || []).join(', '))}` : '') +
    (r.running ? '' : ' · <b>recorder stopped</b>');
}
$('rec-start').onclick = async () => {
  const body = { name: $('rec-name').value.trim(), preset: $('preset').value, cameras: recCameras() };
  try { await post('record/start', body); } catch (e) {
    if (/emitter|streams off/.test(e.message) && confirm(e.message + '\n\nRecord anyway?')) await post('record/start', { ...body, force: true });
  }
  refreshState();
};
$('rec-stop').onclick = async () => {
  $('rec-stop').disabled = true;
  const r = await post('record/stop').catch(() => null);
  $('rec-name').value = '';
  if (r) showError('');
  refreshState();
};
$('cam-start').onclick = () => post('cameras/start').then(() => { refreshState(); setTimeout(refreshCameras, 8000); });
$('cam-stop').onclick = () => confirm('Stop the cameras?') && post('cameras/stop').then(() => { refreshState(); refreshCameras(); });

// Live view: every image stream the cameras publish, one row per camera.
function renderLivePreviews() {
  const el = $('live-previews');
  const published = new Set(topics.topics.filter((t) => t.publishers).map((t) => t.name));
  const cams = camOrder((state && state.cameras) || []);
  const rows = cams.map((cam) => [cam, CAM_STREAMS.map(([suffix, label]) => [`/${cam}/${suffix}`, label])
    .filter(([t]) => published.has(t))]).filter(([, st]) => st.length);
  const show = tab === 'cameras';
  const key = JSON.stringify([rows, show]);
  if (el.dataset.key !== key) {
    el.dataset.key = key;
    el.innerHTML = !show ? '' : rows.length ? rows.map(([cam, st]) => `<div class="cam-row"><h3>${esc(cam)}</h3><div class="tiles">${
      st.map(([t, label]) => `<div class="tile" data-topic="${esc(t)}"><div class="label"><span>${esc(label)}</span>
      <span class="muted">${esc(t)}</span></div><div class="frame"><img data-topic="${esc(t)}" alt=""></div><div class="info"></div></div>`).join('')}</div></div>`).join('')
      : '<div class="card muted">No camera streams published: start the cameras.</div>';
  }
  el.querySelectorAll('.tile').forEach((t) => {
    const info = frameInfo[t.dataset.topic];
    t.querySelector('.info').textContent = infoText(info);
    if (info && info.width) t.querySelector('.frame').style.aspectRatio = `${info.width}/${info.height}`;
  });
}

let settings = null;
async function refreshSettings() {
  try { settings = await get('/api/camera_settings'); } catch (e) { showError(e.message); return; }
  renderSettings();
}
function paramEditor(cam, p) {
  if (!p) return '<span class="muted">—</span>';
  const attrs = `data-cam="${esc(cam)}" data-name="${esc(p.name)}" title="${esc(p.description)}"`;
  if (p.read_only || !['bool', 'integer', 'double', 'string'].includes(p.type)) {
    return `<span title="${esc(p.description)}">${esc(JSON.stringify(p.value))}</span>`;
  }
  if (p.type === 'bool') return `<input type="checkbox" ${attrs} ${p.value ? 'checked' : ''}>`;
  if (p.type === 'string') return `<input type="text" ${attrs} value="${esc(p.value)}">`;
  const range = p.min != null ? `<span class="range">${p.min}–${p.max}</span>` : '';
  const step = p.type === 'integer' ? (p.step || 1) : (p.step || 'any');
  return `<input type="number" ${attrs} value="${p.value}" step="${step}"` +
    (p.min != null ? ` min="${p.min}" max="${p.max}"` : '') + `>${range}`;
}
function settingsTable(names, cams, byCam) {
  const saved = settings.saved || {};
  const show = (v) => typeof v === 'string' ? v : JSON.stringify(v);
  const rows = names.filter((n) => cams.some((c) => byCam[c][n])).map((n) => {
    const cells = cams.map((c) => {
      const p = byCam[c][n];
      const differs = saved[c] && n in saved[c] && p && JSON.stringify(saved[c][n]) !== JSON.stringify(p.value);
      const warn = n === 'depth_module.emitter_enabled' && p && p.value ? ' warnval' : '';
      return `<td class="${differs ? 'changed' : ''}${warn}">${paramEditor(c, p)}</td>`;
    }).join('');
    const sv = cams.map((c) => saved[c] && n in saved[c] ? show(saved[c][n]) : '').filter(Boolean);
    const startup = new Set(sv).size <= 1 ? (sv[0] || '') : sv.join(' / ');
    return `<tr><td>${esc(n)}</td>${cells}<td class="muted">${esc(startup)}</td></tr>`;
  }).join('');
  return rows ? `<table><thead><tr><th>Parameter</th>${cams.map((c) => `<th>${esc(c)}</th>`).join('')}<th>startup</th></tr></thead><tbody>${rows}</tbody></table>` : '';
}
function renderSettings() {
  const s = settings;
  const cams = Object.keys(s.cameras);
  const byCam = {};
  for (const c of cams) byCam[c] = Object.fromEntries(s.cameras[c].params.map((p) => [p.name, p]));
  const down = cams.filter((c) => !s.cameras[c].available);
  let html = down.length ? `<p class="muted">Not running: ${esc(down.join(', '))}</p>` : '';
  for (const [title, names] of s.groups) {
    const t = settingsTable(names, cams, byCam);
    if (t) html += `<h3>${esc(title)}</h3>${t}`;
  }
  $('settings').innerHTML = html || '<p class="muted">Start the cameras to see their settings.</p>';
  const grouped = new Set(s.groups.flatMap(([, n]) => n));
  const f = $('settings-filter').value.trim().toLowerCase();
  const others = [...new Set(cams.flatMap((c) => Object.keys(byCam[c])))].sort()
    .filter((n) => !grouped.has(n) && (!f || n.toLowerCase().includes(f)));
  $('settings-other').innerHTML = settingsTable(others, cams, byCam);
  $('settings-saved').textContent = s.saved_time ? 'startup settings saved ' + fmtDate(s.saved_time) : 'no startup settings saved';
  if (s.startup_log && s.startup_log.length) $('settings-saved').textContent += ' · ' + s.startup_log[s.startup_log.length - 1];
  document.querySelectorAll('#tab-cameras [data-name]').forEach((el) => el.onchange = async () => {
    const value = el.type === 'checkbox' ? el.checked : el.type === 'number' ? +el.value : el.value;
    const cameras = $('both').checked ? cams.filter((c) => byCam[c][el.dataset.name]) : [el.dataset.cam];
    el.disabled = true;
    try { await post('param', { cameras, name: el.dataset.name, value }); } catch (e) { /* shown */ }
    refreshSettings();
  });
}
$('settings-reload').onclick = refreshSettings;
$('settings-card').ontoggle = () => { save('settings_open', $('settings-card').open); if ($('settings-card').open) refreshSettings(); };
$('settings-filter').oninput = () => settings && renderSettings();
$('settings-save').onclick = () => post('settings/save').then(refreshSettings);

// =====================================================================================
// Graph (nodes and topics)
// =====================================================================================
async function refreshTopics() {
  try { topics = await get('/api/topics'); } catch (e) { return; }
  renderLivePreviews();
  if (tab !== 'graph') return;
  refreshGraph();
  const f = $('topic-filter').value.trim().toLowerCase();
  $('topics').querySelector('tbody').innerHTML = topics.topics
    .filter((t) => !f || t.name.toLowerCase().includes(f) || t.type.toLowerCase().includes(f))
    .map((t) => `<tr><td>${esc(t.name)}</td><td class="muted">${esc(t.type)}</td>
      <td class="num">${t.publishers ?? ''}</td><td class="num">${t.subscribers ?? ''}</td>
      <td class="num">${t.measuring ? t.hz : ''}</td><td class="num">${t.measuring ? t.kbps : ''}</td>
      <td class="num">${t.measuring ? (t.age_s == null ? 'none' : t.age_s + ' s ago') : ''}</td>
      <td>${t.measuring ? '' : `<button class="small secondary" data-measure="${esc(t.name)}">measure</button>`}</td></tr>`).join('');
  $('topics').querySelectorAll('[data-measure]').forEach((b) => b.onclick = () =>
    post('measure', { topic: b.dataset.measure }).then(refreshTopics));
}
$('topic-filter').oninput = refreshTopics;

// ---- graph: publisher node -> topic -> subscriber node (cytoscape + dagre layout) ----
let cy = null;
let graphKey = '';
const HIDDEN_TOPICS = /^\/(rosout|parameter_events|clock)$|^\/events\/|\/(get_type_description|_action)\b|\/transition_event$/;
// rosbag2_recorder subscribes to everything it records: hidden with the infrastructure.
const HIDDEN_NODES = /^\/(pq_console|pq_console_latest|rosbag2_recorder|_ros2cli.*|transform_listener_impl_.*|launch_ros_.*)$/;
function hasDagre() {
  try { return !!cytoscape('layout', 'dagre'); } catch (e) { return false; }
}
async function refreshGraph() {
  if (typeof cytoscape === 'undefined') { $('graph').textContent = 'graph library not loaded (no internet in the browser?)'; return; }
  let g;
  try { g = await get('/api/graph'); } catch (e) { return; }
  const hide = $('graph-hide').checked;
  const elements = [];
  const topicSet = new Set();
  for (const [node, c] of Object.entries(g)) {
    if (hide && HIDDEN_NODES.test(node)) continue;
    const pubs = c.pub.filter(([t]) => !(hide && HIDDEN_TOPICS.test(t)));
    const subs = c.sub.filter(([t]) => !(hide && HIDDEN_TOPICS.test(t)));
    if (hide && !pubs.length && !subs.length) continue;
    elements.push({ data: { id: 'n:' + node, label: node, kind: 'node' } });
    for (const [t, type] of pubs) { topicSet.add(t + '\t' + type); elements.push({ data: { id: `p:${node}>${t}`, source: 'n:' + node, target: 't:' + t } }); }
    for (const [t, type] of subs) { topicSet.add(t + '\t' + type); elements.push({ data: { id: `s:${t}>${node}`, source: 't:' + t, target: 'n:' + node } }); }
  }
  const seen = new Set();
  for (const x of topicSet) {
    const [t, type] = x.split('\t');
    if (seen.has(t)) continue;
    seen.add(t);
    elements.unshift({ data: { id: 't:' + t, label: t, kind: 'topic', type } });
  }
  const nNodes = elements.filter((e) => e.data.kind === 'node').length;
  $('graph-info').textContent = `${nNodes} nodes, ${seen.size} topics` + (hasDagre() ? '' : ' (dagre layout not loaded)');
  const key = JSON.stringify(elements.map((e) => e.data.id).sort());
  if (key === graphKey && cy) { highlight(); return; }
  graphKey = key;
  const css = getComputedStyle(document.body);
  const text = css.getPropertyValue('--text').trim(), muted = css.getPropertyValue('--muted').trim();
  if (cy) cy.destroy();
  cy = cytoscape({
    container: $('graph'), elements, wheelSensitivity: 0.3,
    style: [
      { selector: 'node[kind="node"]', style: { shape: 'round-rectangle', 'background-color': '#2563eb', label: 'data(label)',
        color: '#fff', 'text-valign': 'center', 'font-size': 11, width: 'label', height: 22, padding: 8 } },
      { selector: 'node[kind="topic"]', style: { shape: 'rectangle', 'background-color': css.getPropertyValue('--chip').trim(),
        'border-width': 1, 'border-color': muted, label: 'data(label)', color: text, 'text-valign': 'center',
        'font-size': 10, width: 'label', height: 16, padding: 4 } },
      { selector: 'edge', style: { width: 1.2, 'line-color': muted, 'target-arrow-color': muted, 'target-arrow-shape': 'triangle',
        'curve-style': 'bezier', 'arrow-scale': 0.8 } },
      { selector: '.faded', style: { opacity: 0.15 } },
      { selector: '.hit', style: { 'border-width': 3, 'border-color': '#f59e0b' } },
    ],
    layout: hasDagre() ? { name: 'dagre', rankDir: 'LR', nodeSep: 8, rankSep: 60 } : { name: 'breadthfirst', directed: true },
  });
  cy.on('tap', 'node[kind="topic"]', (e) => {
    const t = e.target.data('label');
    $('topic-filter').value = t;
    post('measure', { topic: t }).then(refreshTopics).catch(() => {});
    $('topics').scrollIntoView({ behavior: 'smooth' });
  });
  highlight();
}
function highlight() {
  if (!cy) return;
  const f = $('graph-find').value.trim().toLowerCase();
  cy.elements().removeClass('faded hit');
  if (!f) return;
  const hits = cy.nodes().filter((n) => n.data('label').toLowerCase().includes(f));
  const keep = hits.union(hits.neighborhood());
  cy.elements().not(keep).addClass('faded');
  hits.addClass('hit');
}
$('graph-hide').onchange = () => { graphKey = ''; refreshGraph(); };
$('graph-find').oninput = highlight;
$('graph-fit').onclick = () => cy && cy.fit(undefined, 20);

// ---- start ----
(async () => {
  const src = load('slam_src', 'profile');
  const radio = document.querySelector(`input[name=slam-src][value="${src}"]`);
  if (radio) radio.checked = true;
  $('settings-card').open = load('settings_open', false);
  const hashTab = location.hash.slice(1);
  setTab(TABS.includes(hashTab) ? hashTab : load('tab', 'bags'));
  await loadVersions();
  await refreshState();
  // ?bag=<path> opens a bag (link from elsewhere); otherwise the last one.
  const linked = new URLSearchParams(location.search).get('bag');
  await browse(linked ? linked.replace(/\/[^/]*\/?$/, '') : load('browse', ''));
  const lastBag = linked || load('bag', '');
  if (lastBag) loadBag(lastBag);
  refreshTopics();
  setInterval(refreshState, 1000);
  setInterval(() => { if (tab === 'cameras') renderLivePreviews(); }, 1000);
  setInterval(() => { if (tab === 'cameras') refreshCameras(); }, 5000);
  setInterval(() => { if (tab === 'bags') refreshSlam(); }, 250);
  framesLoop();
  setInterval(refreshTopics, 2000);
})();
