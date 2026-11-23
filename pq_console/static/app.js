// pq console: polling of /api/state, /api/slam and /api/topics; actions with POST /api/<name>.
'use strict';

const $ = (id) => document.getElementById(id);
const COLORS = ['#e5484d', '#3b82f6', '#22a559', '#f59e0b', '#a855f7', '#14b8a6', '#64748b'];
// Default previews: cam2 above, cam1 below (as the Rerun layout).
const DEFAULT_PREVIEWS = ['/cam2/infra1/image_rect_raw', '/cam2/color/image_raw',
  '/cam1/infra1/image_rect_raw', '/cam1/color/image_raw'];

let tab = 'live';
let state = null;
let topics = { nodes: [], topics: [] };
let profiles = [];
let slam = { generation: null, paths: {}, since: {} };
let lastJobState = null;

function load(key, fallback) {
  try { const v = localStorage.getItem('pq_console.' + key); return v === null ? fallback : JSON.parse(v); }
  catch (e) { return fallback; }
}
function save(key, value) {
  try { localStorage.setItem('pq_console.' + key, JSON.stringify(value)); } catch (e) { /* private mode */ }
}
let previewTopics = load('previews', DEFAULT_PREVIEWS);

function showError(msg) {
  const e = $('error');
  e.textContent = msg;
  e.classList.toggle('hidden', !msg);
}
$('error').onclick = () => showError('');

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

const fmtBytes = (b) => b == null ? '' : b > 1e9 ? (b / 1e9).toFixed(1) + ' GB' : (b / 1e6).toFixed(0) + ' MB';
const fmtDur = (s) => { s = Math.round(s); return Math.floor(s / 60) + ':' + String(s % 60).padStart(2, '0'); };
const fmtDate = (t) => t ? new Date(t * 1000).toLocaleString([], { dateStyle: 'short', timeStyle: 'short' }) : '';
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const chip = (text, cls = '') => `<span class="chip ${cls}">${esc(text)}</span>`;

// ---- tabs ----
document.querySelectorAll('button.tab').forEach((b) => b.onclick = () => setTab(b.dataset.tab));
function setTab(t) {
  tab = t;
  save('tab', t);
  document.querySelectorAll('button.tab').forEach((b) => b.classList.toggle('active', b.dataset.tab === t));
  document.querySelectorAll('.tab-body').forEach((s) => s.classList.toggle('hidden', s.id !== 'tab-' + t));
  updatePreviews();
  if (t === 'sessions') refreshLists();
  if (t === 'graph') refreshTopics();
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
  if (s.job && s.job.state === 'running') parts.push(chip('replay ' + s.job.name, 'warn'));
  if (s.disk) parts.push(chip(fmtBytes(s.disk.free) + ' free', s.disk.free < 20e9 ? 'bad' : ''));
  $('status').innerHTML = parts.join('');
}

function renderCameras() {
  const s = state;
  const rows = s.cameras.map((cam) => {
    const v = (s.emitters || {})[cam];
    const em = !s.cameras_running ? chip('stopped') : v === null || v === undefined ? chip('emitter ?', 'warn')
      : v ? chip('emitter ON', 'bad') : chip('emitter off', 'ok');
    const btn = s.cameras_running && v ? `<button class="small danger" data-emitter="${esc(cam)}">turn off</button>` : '';
    return `<div class="cam"><b>${esc(cam)}</b>${em}${btn}</div>`;
  });
  const diag = Object.entries(s.diagnostics || {}).filter(([k]) => /cam[12]/.test(k)).slice(0, 8)
    .map(([k, d]) => `<div class="muted">${esc(k)}: ${esc(d.message)}</div>`);
  $('cameras').innerHTML = (rows.join('') || '<div class="muted">no cameras in the configuration</div>') + diag.join('');
  $('cameras').querySelectorAll('[data-emitter]').forEach((b) => b.onclick = () =>
    post('emitter', { camera: b.dataset.emitter, value: 0 }).then(refreshState));
  $('cam-start').disabled = s.cameras_running || !s.cameras.length;
  $('cam-stop').disabled = !s.cameras_running || !!s.recording;
}

function renderRecording() {
  const r = state.recording;
  $('rec-start').disabled = !!r;
  $('rec-stop').disabled = !r;
  if (!$('rec-name').value) {
    const d = new Date();
    const p = (n) => String(n).padStart(2, '0');
    $('rec-name').value = `rec_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}`;
  }
  if (!r) { $('rec-state').textContent = 'not recording'; return; }
  const el = r.started ? fmtDur(state.time - r.started) : '';
  $('rec-state').innerHTML = `${chip('REC', 'bad')} ${esc(r.name)} ${el} · ${fmtBytes(r.size)}` +
    (r.running ? '' : ' · <b>recorder stopped</b>');
}
$('rec-start').onclick = async () => {
  const body = { name: $('rec-name').value.trim(), depth: $('rec-depth').checked, aligned: $('rec-aligned').checked };
  try { await post('record/start', body); } catch (e) {
    if (/emitter/.test(e.message) && confirm(e.message + '\n\nRecord anyway?')) await post('record/start', { ...body, force: true });
  }
  refreshState();
};
$('rec-stop').onclick = async () => {
  $('rec-stop').disabled = true;
  await post('record/stop').catch(() => {});
  $('rec-name').value = '';
  refreshState();
};
$('cam-start').onclick = () => post('cameras/start').then(refreshState);
$('cam-stop').onclick = () => confirm('Stop the cameras?') && post('cameras/stop').then(refreshState);

function renderJob() {
  const j = state.job;
  const text = j ? `${j.name}: ${j.state}${j.state === 'running' ? ' · ' + fmtDur(state.time - j.started) : ''}` : 'none';
  $('job').textContent = text;
  const lastLog = j && j.log.length ? j.log[j.log.length - 1].slice(9) : '';
  $('job-live').textContent = j ? text + (lastLog && lastLog !== j.state ? ' — ' + lastLog : '') : 'none';
  $('job-log').textContent = j ? j.log.join('\n') : '';
  $('job-stop').disabled = !j || j.state !== 'running';
  if (j && lastJobState === 'running' && j.state !== 'running' && tab === 'sessions') refreshLists();
  lastJobState = j ? j.state : null;
}
$('job-stop').onclick = () => confirm('Stop the replay?') && post('job/stop').then(refreshState);

// ---- previews ----
function imageTopics() {
  return topics.topics.filter((t) => /sensor_msgs\/msg\/(Compressed)?Image$/.test(t.type)).map((t) => t.name);
}
function updatePreviews() {
  const names = imageTopics();
  document.querySelectorAll('.preview').forEach((el) => {
    const slot = +el.dataset.slot;
    const want = tab === 'live' ? previewTopics[slot] || '' : '';
    if (!el.firstChild) {
      el.innerHTML = '<select></select><div class="frame"><img alt=""></div><div class="info"></div>';
      el.querySelector('select').onchange = (ev) => {
        previewTopics[slot] = ev.target.value;
        save('previews', previewTopics);
        updatePreviews();
      };
    }
    const sel = el.querySelector('select');
    const options = ['', ...new Set([...names, previewTopics[slot] || ''])].filter((x, i) => i === 0 || x);
    const html = options.map((n) => `<option value="${esc(n)}">${n ? esc(n) : '(none)'}</option>`).join('');
    if (sel.dataset.html !== html) { sel.innerHTML = html; sel.dataset.html = html; }
    sel.value = previewTopics[slot] || '';
    const img = el.querySelector('img');
    const available = want && names.includes(want);
    const src = available ? '/stream?topic=' + encodeURIComponent(want) : '';
    if (img.dataset.src !== src) {
      img.dataset.src = src;
      if (src) img.src = src; else img.removeAttribute('src');
    }
    const info = (state && state.previews && state.previews[want]) || null;
    el.querySelector('.info').textContent = !want ? '' : !available ? 'topic not published' : !info ? 'waiting…'
      : [`${info.hz} fps`, info.width ? `${info.width}×${info.height}` : '',
        info.saturated_pct != null ? `saturated ${info.saturated_pct}%` : '',
        info.valid_depth_pct != null ? `valid depth ${info.valid_depth_pct}%` : '',
        info.error || ''].filter(Boolean).join(' · ');
  });
}

// ---- SLAM ----
async function refreshSlam() {
  let s;
  try { s = await get('/api/slam?since=' + encodeURIComponent(JSON.stringify(slam.since))); } catch (e) { return; }
  if (s.generation !== slam.generation) { slam = { generation: s.generation, paths: {}, since: {} }; }
  for (const [topic, pts] of Object.entries(s.paths)) {
    const path = slam.paths[topic] || (slam.paths[topic] = []);
    path.push(...pts);
    if (path.length) slam.since[topic] = path[path.length - 1][0];
  }
  drawTrajectory();
  renderDiag(s);
}
$('slam-reset').onclick = () => post('slam/reset').then(refreshSlam);

function drawTrajectory() {
  const cv = $('traj');
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  ctx.clearRect(0, 0, W, H);
  const entries = Object.entries(slam.paths).filter(([, p]) => p.length);
  const legend = [];
  if (!entries.length) {
    ctx.fillStyle = getComputedStyle(document.body).getPropertyValue('--muted');
    ctx.font = '14px system-ui';
    ctx.fillText('no trajectory yet', 20, 30);
    $('legend').innerHTML = '';
    return;
  }
  // Top view (x right, y up), same scale on both axes, 1 m grid.
  let minx = Infinity, maxx = -Infinity, miny = Infinity, maxy = -Infinity;
  for (const [, p] of entries) for (const q of p) {
    minx = Math.min(minx, q[1]); maxx = Math.max(maxx, q[1]); miny = Math.min(miny, q[2]); maxy = Math.max(maxy, q[2]);
  }
  const span = Math.max(maxx - minx, maxy - miny, 2) * 1.1;
  const scale = Math.min(W, H) / span;
  const cx = (minx + maxx) / 2, cy = (miny + maxy) / 2;
  const X = (x) => W / 2 + (x - cx) * scale, Y = (y) => H / 2 - (y - cy) * scale;
  const muted = getComputedStyle(document.body).getPropertyValue('--border');
  ctx.strokeStyle = muted; ctx.lineWidth = 1;
  const step = span > 60 ? 10 : span > 15 ? 5 : 1;
  for (let g = Math.floor((cx - W / 2 / scale) / step) * step; g < cx + W / 2 / scale; g += step) {
    ctx.beginPath(); ctx.moveTo(X(g), 0); ctx.lineTo(X(g), H); ctx.stroke();
  }
  for (let g = Math.floor((cy - H / 2 / scale) / step) * step; g < cy + H / 2 / scale; g += step) {
    ctx.beginPath(); ctx.moveTo(0, Y(g)); ctx.lineTo(W, Y(g)); ctx.stroke();
  }
  entries.forEach(([topic, p], i) => {
    const color = COLORS[i % COLORS.length];
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
    p.forEach((q, k) => k ? ctx.lineTo(X(q[1]), Y(q[2])) : ctx.moveTo(X(q[1]), Y(q[2])));
    ctx.stroke();
    const last = p[p.length - 1];
    ctx.fillStyle = color; ctx.beginPath(); ctx.arc(X(last[1]), Y(last[2]), 5, 0, 2 * Math.PI); ctx.fill();
    let len = 0;
    for (let k = 1; k < p.length; k++) len += Math.hypot(p[k][1] - p[k - 1][1], p[k][2] - p[k - 1][2], p[k][3] - p[k - 1][3]);
    legend.push(`<span style="--c:${color}">${esc(topic)} · ${len.toFixed(1)} m</span>`);
  });
  legend.push(`<span style="--c:${muted}">grid ${step} m</span>`);
  $('legend').innerHTML = legend.join('');
}

function renderDiag(s) {
  const diags = Object.entries(s.diagnostics || {});
  if (!diags.length) { $('slam-diag').textContent = 'no SLAM running'; }
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

// ---- sessions ----
async function refreshLists() {
  try {
    if (!profiles.length) {
      profiles = await get('/api/profiles');
      const saved = load('profile', 'cam1_cam2_ir');
      $('profile').innerHTML = profiles.map((p) => `<option value="${esc(p.name)}">${esc(p.name)}</option>`).join('');
      if (profiles.some((p) => p.name === saved)) $('profile').value = saved;
      $('profile').onchange = () => { save('profile', $('profile').value); showProfile(); };
      showProfile();
    }
    const [recs, runs] = await Promise.all([get('/api/recordings'), get('/api/runs')]);
    $('recordings').querySelector('tbody').innerHTML = recs.map((r) => `<tr>
      <td>${esc(r.name)}</td><td>${fmtDate(r.start)}</td><td class="num">${fmtDur(r.duration_s)}</td>
      <td class="num">${fmtBytes(r.size)}</td><td>${esc(r.cameras.join(', '))}</td>
      <td>${r.depth ? 'depth' : ''}${r.aligned ? (r.depth ? ', ' : '') + 'aligned' : ''}</td>
      <td><button class="small" data-play="${esc(r.name)}">Play + SLAM</button></td></tr>`).join('') ||
      '<tr><td colspan="7" class="muted">no recordings</td></tr>';
    $('recordings').querySelectorAll('[data-play]').forEach((b) => b.onclick = async () => {
      await post('replay/start', { bag: b.dataset.play, profile: $('profile').value, rerun: $('rerun').checked });
      slam = { generation: null, paths: {}, since: {} };
      setTab('live');
    });
    const ev = (r, k) => Object.entries(r.events || {}).filter(([n]) => n.endsWith(k)).reduce((a, [, v]) => a + v, 0);
    const rej = (r) => Object.entries(r.health || {}).filter(([n]) => /rejected|lost/.test(n)).reduce((a, [, v]) => a + v, 0);
    $('runs').querySelector('tbody').innerHTML = runs.map((r) => `<tr>
      <td>${esc(r.name)}</td><td>${esc(r.mode || r.profile)}</td><td class="num">${r.frames ?? ''}</td>
      <td class="num">${r.path_m != null ? r.path_m + ' m' : ''}</td>
      <td class="num">${r.start_end_m != null ? r.start_end_m + ' m' : ''}</td>
      <td class="num">${rej(r)}</td><td class="num">${ev(r, 'tracker_reset')}</td>
      <td class="num">${ev(r, 'loop_closure')}</td><td class="num">${r.track_ms ?? ''}</td>
      <td>${r.rrd ? `<code>${esc(r.rrd)}</code>` : ''}</td></tr>`).join('') ||
      '<tr><td colspan="10" class="muted">no runs</td></tr>';
  } catch (e) { showError(e.message); }
}
function showProfile() {
  const p = profiles.find((x) => x.name === $('profile').value);
  $('profile-desc').textContent = p ? `${p.mode}: ${p.description}` : '';
}

// ---- nodes and topics ----
async function refreshTopics() {
  try { topics = await get('/api/topics'); } catch (e) { return; }
  updatePreviews();
  if (tab !== 'graph') return;
  $('nodes').innerHTML = topics.nodes.map((n) => `<li>${esc(n)}</li>`).join('');
  const f = $('topic-filter').value.trim().toLowerCase();
  $('topics').querySelector('tbody').innerHTML = topics.topics
    .filter((t) => !f || t.name.toLowerCase().includes(f) || t.type.toLowerCase().includes(f))
    .map((t) => `<tr><td>${esc(t.name)}</td><td class="muted">${esc(t.type)}</td>
      <td class="num">${t.publishers}</td><td class="num">${t.subscribers}</td>
      <td class="num">${t.measuring ? t.hz : ''}</td><td class="num">${t.measuring ? t.kbps : ''}</td>
      <td class="num">${t.measuring ? (t.age_s == null ? 'none' : t.age_s + ' s ago') : ''}</td>
      <td>${t.measuring ? '' : `<button class="small secondary" data-measure="${esc(t.name)}">measure</button>`}</td></tr>`).join('');
  $('topics').querySelectorAll('[data-measure]').forEach((b) => b.onclick = () =>
    post('measure', { topic: b.dataset.measure }).then(refreshTopics));
}
$('topic-filter').oninput = refreshTopics;

// ---- loops ----
const hashTab = location.hash.slice(1);
setTab(['live', 'sessions', 'graph'].includes(hashTab) ? hashTab : load('tab', 'live'));
refreshState();
refreshTopics();
setInterval(refreshState, 1000);
setInterval(() => { if (tab === 'live') { refreshSlam(); updatePreviews(); } }, 1000);
setInterval(refreshTopics, 2000);
