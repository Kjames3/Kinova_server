const TOK_KEY = 'kv2tok';
let TOKEN = localStorage.getItem(TOK_KEY) || '';
let wsRobot = null, wsLogs = null, wsTerm = null;
let activeSl = null;
let poseEditing = false;
let depthOn = {rs: true, oak: true};
let robotJoints = [0,0,0,0,0,0,0];

// ── WebSocket helper: token travels in Sec-WebSocket-Protocol, not the URL ──
function wsConnect(path) {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  return new WebSocket(`${proto}://${location.host}${path}`, TOKEN ? [TOKEN] : []);
}

// ── Auth ──────────────────────────────────────────────────────────────────
async function doLogin() {
  const pass = document.getElementById('lpass').value;
  const r = await fetch('/api/auth/login', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({password: pass})
  });
  if (r.ok) {
    TOKEN = (await r.json()).token;
    localStorage.setItem(TOK_KEY, TOKEN);
    showApp();
  } else {
    document.getElementById('lerr').textContent = 'Invalid password';
  }
}
function doLogout() {
  fetch(`/api/auth/logout?token=${TOKEN}`, {method:'POST'});
  localStorage.removeItem(TOK_KEY);
  location.reload();
}
async function checkAuth() {
  if (!TOKEN) { return; }
  const r = await fetch(`/api/status?token=${TOKEN}`);
  if (r.status === 401) { TOKEN = ''; localStorage.removeItem(TOK_KEY); return; }
  showApp();
}
function showApp() {
  document.getElementById('lo').style.display = 'none';
  const app = document.getElementById('app');
  app.style.display = 'flex';
  initJoints();
  connectRobotWs();
  connectLogsWs();
  connectTermWs();
  pollStatus();
  pollSystem();
  setInterval(pollStatus, 4000);
  setInterval(pollSystem, 3000);
  initRos2Panel();
  initEndEffector();
  setTimeout(() => { initArmViewer(); initPcViewer(); }, 100);
}

// ── Joints ──────────────────────────────────────────────────────────────────
function initJoints() {
  const c = document.getElementById('jsliders');
  c.innerHTML = '';
  for (let i = 1; i <= 7; i++) {
    c.innerHTML += `<div class="jrow">
      <span class="jlbl">J${i}</span>
      <input type="range" id="j${i}" min="-180" max="180" value="0" step="0.5"
        onmousedown="activeSl='j${i}'" onmouseup="activeSl=null"
        ontouchstart="activeSl='j${i}'" ontouchend="activeSl=null"
        oninput="document.getElementById('jv${i}').textContent=parseFloat(this.value).toFixed(1)+'°'">
      <span class="jval" id="jv${i}">0.0°</span>
    </div>`;
  }
  ['pX','pY','pZ','pRX','pRY','pRZ'].forEach(id => {
    const el = document.getElementById(id);
    if(el) { el.onfocus = ()=>poseEditing=true; el.onblur = ()=>poseEditing=false; }
  });
}
function syncJointsFromRobot() {
  robotJoints.forEach((v, i) => {
    const el = document.getElementById(`j${i+1}`);
    if (el) { el.value = v; document.getElementById(`jv${i+1}`).textContent = v.toFixed(1)+'°'; }
  });
}

// ── Robot WS ──────────────────────────────────────────────────────────────
function connectRobotWs() {
  wsRobot = wsConnect('/ws/robot');
  wsRobot.onmessage = ({data}) => {
    const s = JSON.parse(data);
    const st = document.getElementById('hStatus');
    st.textContent = s.connected ? 'ONLINE' : 'OFFLINE';
    st.className = 'h-status ' + (s.connected ? 'online' : 'offline');
    if (s.joints) {
      robotJoints = s.joints;
      if (armReady) updateArm(s.joints);
      s.joints.forEach((v, i) => {
        if (activeSl !== `j${i+1}`) {
          const el = document.getElementById(`j${i+1}`);
          if (el) { el.value = v; document.getElementById(`jv${i+1}`).textContent = v.toFixed(1)+'°'; }
        }
      });
    }
    if (s.pose && s.pose.x !== undefined && !poseEditing) {
      document.getElementById('pX').value  = s.pose.x.toFixed(4);
      document.getElementById('pY').value  = s.pose.y.toFixed(4);
      document.getElementById('pZ').value  = s.pose.z.toFixed(4);
      document.getElementById('pRX').value = s.pose.theta_x.toFixed(2);
      document.getElementById('pRY').value = s.pose.theta_y.toFixed(2);
      document.getElementById('pRZ').value = s.pose.theta_z.toFixed(2);
    }
    if (s.gripper !== undefined && activeSl !== 'g') {
      const pct = Math.round(s.gripper * 100);
      document.getElementById('gSlider').value = pct;
      document.getElementById('gPct').textContent = pct + '%';
    }
  };
  wsRobot.onclose = () => setTimeout(connectRobotWs, 2000);
}
function connectRobot() { /* arm connects on server start */ }

// ── Logs WS ─────────────────────────────────────────────────────────────────
function connectLogsWs() {
  wsLogs = wsConnect('/ws/logs');
  wsLogs.onmessage = ({data}) => appendLog(data);
  wsLogs.onclose = () => setTimeout(connectLogsWs, 3000);
}
function appendLog(msg) {
  const box = document.getElementById('logBox');
  const d = document.createElement('div');
  const ts = new Date().toLocaleTimeString('en-GB',{hour12:false});
  let cls = 'll-d';
  if (msg.includes('[INFO]'))  cls = 'll-i';
  if (msg.includes('[WARN'))   cls = 'll-w';
  if (msg.includes('[ERROR]') || msg.includes('[CRITICAL]')) cls = 'll-e';
  d.className = cls;
  d.textContent = `[${ts}] ${msg}`;
  box.appendChild(d);
  box.scrollTop = box.scrollHeight;
  while (box.children.length > 300) box.removeChild(box.firstChild);
}

// ── Terminal WS ─────────────────────────────────────────────────────────────
function connectTermWs() {
  wsTerm = wsConnect('/ws/terminal');
  wsTerm.onmessage = ({data}) => {
    const out = document.getElementById('termOut');
    out.textContent += data;
    out.scrollTop = out.scrollHeight;
  };
  wsTerm.onclose = () => {
    document.getElementById('termOut').textContent += '\n[Disconnected — reconnecting...]\n';
    setTimeout(connectTermWs, 3000);
  };
}
function runScript() {
  const code = document.getElementById('termIn').value;
  if (!code.trim()) return;
  document.getElementById('termOut').textContent = '';
  if (wsTerm && wsTerm.readyState === WebSocket.OPEN)
    wsTerm.send(JSON.stringify({code}));
  else
    document.getElementById('termOut').textContent = '[Terminal not connected]\n';
}

// ── Camera helpers ──────────────────────────────────────────────────────────
function startCam(cam) {
  const t = TOKEN;
  if (cam === 'rs') {
    document.getElementById('rsBody').innerHTML =
      `<img class="cam-rgb" src="/api/cameras/realsense/rgb?token=${t}&_=${Date.now()}" alt="">` +
      (depthOn.rs ? `<img class="cam-depth" src="/api/cameras/realsense/depth?token=${t}&_=${Date.now()}" alt="">` : '');
  } else if (cam === 'oak') {
    document.getElementById('oakBody').innerHTML =
      `<img class="cam-rgb" src="/api/cameras/oakd/rgb?token=${t}&_=${Date.now()}" alt="">` +
      (depthOn.oak ? `<img class="cam-depth" src="/api/cameras/oakd/depth?token=${t}&_=${Date.now()}" alt="">` : '');
  } else if (cam === 'wrist') {
    // ROS2 image relay of the Kinova bracelet camera (needs the Wrist Vision node running)
    const topic = encodeURIComponent('/camera/color/image_raw');
    document.getElementById('wristBody').innerHTML =
      `<img class="cam-rgb" src="/api/ros2/image?name=${topic}&token=${t}&_=${Date.now()}" alt=""` +
      ` onerror="this.parentNode.innerHTML='<div class=cam-off>NO WRIST FEED — START VISION NODE</div>'">`;
  }
}
function toggleDepth(cam, btn) {
  depthOn[cam] = !depthOn[cam];
  btn.textContent = depthOn[cam] ? 'DEPTH ON' : 'DEPTH OFF';
  btn.className = depthOn[cam] ? 'btn bn' : 'btn bp';
}

// ── End effector (PLAN GUI item 7) ──────────────────────────────────────────
let eeOptions = {};
async function initEndEffector() {
  try {
    const d = await fetch(`/api/robot/end_effector?token=${TOKEN}`).then(r=>r.json());
    eeOptions = d.options || {};
    const sel = document.getElementById('eeSelect');
    sel.innerHTML = Object.entries(eeOptions)
      .map(([k,v]) => `<option value="${k}">${v.label}</option>`).join('');
    const saved = localStorage.getItem('kv2ee');
    const active = (saved && eeOptions[saved]) ? saved : d.selected;
    sel.value = active;
    if (active !== d.selected) await postEndEffector(active);
    applyEndEffector(active);
  } catch {}
}
async function setEndEffector(name) {
  localStorage.setItem('kv2ee', name);
  await postEndEffector(name);
  applyEndEffector(name);
}
async function postEndEffector(name) {
  await apiPost('/api/robot/end_effector', {name});
}
function applyEndEffector(name) {
  const cfg = eeOptions[name] || {};
  // gripper controls only make sense for a real gripper
  const hasGrip = cfg.has_gripper !== false;
  const gSlider = document.getElementById('gSlider');
  if (gSlider) gSlider.disabled = !hasGrip;
  document.getElementById('eeInfo').textContent =
    `TCP +${(cfg.tcp_offset||0).toFixed(3)} m (${cfg.tcp_axis||'z'})` + (hasGrip ? '' : ' · rigid tool');
  // hook for the URDF viewer (item 3) to swap the gripper model
  if (typeof setArmModel === 'function') setArmModel(cfg.model || name);
}

// ── Status & system ─────────────────────────────────────────────────────────
async function pollStatus() {
  try {
    const s = await fetch(`/api/status?token=${TOKEN}`).then(r=>r.json());
    Object.entries(s).forEach(([k,v]) => {
      const d = document.getElementById(`dot-${k}`);
      if (d) d.className = 'sdot' + (v ? ' on' : '');
      const f = document.getElementById(`feed-${k}`);
      if (f) { f.textContent = v ? 'Online' : 'Offline'; f.className = v ? 'on' : 'off'; }
    });
  } catch {}
}
async function pollSystem() {
  try {
    const s = await fetch(`/api/system?token=${TOKEN}`).then(r=>r.json());
    document.getElementById('hCpu').textContent = s.cpu.toFixed(0);
    document.getElementById('hMem').textContent = s.mem.toFixed(0);
  } catch {}
}

// ── Robot API calls ─────────────────────────────────────────────────────────
async function apiPost(path, body) {
  try {
    const r = await fetch(`${path}?token=${TOKEN}`, {
      method: 'POST',
      headers: body ? {'Content-Type':'application/json'} : {},
      body: body ? JSON.stringify(body) : undefined,
    });
    if (!r.ok) {
      const e = await r.json().catch(()=>({detail:'unknown error'}));
      appendLog(`[ERROR] ${path}: ${e.detail}`);
    }
    return r;
  } catch(e) { appendLog(`[ERROR] ${e.message}`); }
}
async function sendJoints() {
  const angles = Array.from({length:7},(_,i)=>parseFloat(document.getElementById(`j${i+1}`).value));
  await apiPost('/api/robot/joints', {angles});
}
async function moveToPose() {
  await apiPost('/api/robot/pose', {
    x:       +document.getElementById('pX').value,
    y:       +document.getElementById('pY').value,
    z:       +document.getElementById('pZ').value,
    theta_x: +document.getElementById('pRX').value,
    theta_y: +document.getElementById('pRY').value,
    theta_z: +document.getElementById('pRZ').value,
  });
}
async function setGripper(pct) {
  document.getElementById('gSlider').value = pct;
  document.getElementById('gPct').textContent = pct + '%';
  await apiPost('/api/robot/gripper', {position: pct/100});
}
async function sendGripper() {
  const pct = parseInt(document.getElementById('gSlider').value);
  await apiPost('/api/robot/gripper', {position: pct/100});
}
async function sendCmd() {
  const el = document.getElementById('cmdIn');
  const t = el.value.trim().toLowerCase();
  el.value = '';
  if (!t) return;
  appendLog(`[CMD] ${t}`);
  if      (t.includes('home'))    await apiPost('/api/robot/home');
  else if (t.includes('retract')) await apiPost('/api/robot/retract');
  else if (t.includes('vertical'))await apiPost('/api/robot/vertical');
  else if (t.includes('stop'))    await apiPost('/api/robot/stop');
  else if (t.includes('open'))    await setGripper(0);
  else if (t.includes('close'))   await setGripper(100);
  else appendLog('[INFO] Commands: home · retract · vertical · stop · open · close');
}

// ═══════════════════════════════════════════════════════════════════════════
// AUTOMATION & ROS 2 CONTROL (PLAN 3.5)
// ═══════════════════════════════════════════════════════════════════════════
let wsProcLogs = {}, wsFusion = null, wsInsert = null;

function initRos2Panel() {
  refreshRos2Status();
  setInterval(refreshRos2Status, 4000);
  connectFusionWs();
  connectInsertWs();
}

async function refreshRos2Status() {
  try {
    const s = await fetch(`/api/ros2/status?token=${TOKEN}`).then(r=>r.json());
    const avail = s.available;
    document.getElementById('ros2Avail').textContent = avail ? 'BRIDGE ONLINE' : 'BRIDGE OFFLINE';
    document.getElementById('ros2Avail').className = 'badge ' + (avail ? 'running' : 'error');
    ['system','fusion','pcfusion','wrist'].forEach(name => {
      const st = (s.processes[name] || {}).state || 'idle';
      const b = document.getElementById(`procBadge-${name}`);
      if (b) { b.textContent = st.toUpperCase(); b.className = 'badge ' + st; }
    });
    const wrun = (s.processes.wrist || {}).running;
    const wd = document.getElementById('dot-wrist');
    if (wd) wd.className = 'sdot' + (wrun ? ' on' : '');
  } catch {}
}

function procArgs(name) {
  if (name === 'system') {
    return {
      launch_oak_camera:   document.getElementById('sysOak').checked   ? 'true' : 'false',
      launch_wrist_camera: document.getElementById('sysWrist').checked ? 'true' : 'false',
    };
  }
  return null;
}
async function toggleProcess(name, btn) {
  const st = await fetch(`/api/ros2/process/status?token=${TOKEN}`).then(r=>r.json()).catch(()=>({}));
  const running = (st[name] || {}).running;
  const path = running ? '/api/ros2/process/stop' : '/api/ros2/process/start';
  const body = {process: name};
  if (!running) { const a = procArgs(name); if (a) body.args = a; }
  const r = await apiPost(path, body);
  if (r && r.ok && !running) openProcConsole(name);
  refreshRos2Status();
}

function openProcConsole(name) {
  const box = document.getElementById(`console-${name}`);
  box.classList.add('open');
  if (wsProcLogs[name]) wsProcLogs[name].close();
  const ws = wsConnect(`/ws/ros2/process/logs?name=${name}`);
  ws.onmessage = ({data}) => {
    box.textContent += data + '\n';
    box.scrollTop = box.scrollHeight;
    if (box.textContent.length > 20000) box.textContent = box.textContent.slice(-15000);
  };
  wsProcLogs[name] = ws;
}
function toggleConsole(name) {
  const box = document.getElementById(`console-${name}`);
  if (box.classList.contains('open')) box.classList.remove('open');
  else { box.classList.add('open'); if (!wsProcLogs[name] || wsProcLogs[name].readyState>1) openProcConsole(name); }
}

// ── Fusion diagnostics ──────────────────────────────────────────────────────
function connectFusionWs() {
  wsFusion = wsConnect('/ws/ros2/fusion');
  wsFusion.onmessage = ({data}) => {
    let f; try { f = JSON.parse(data); } catch { return; }
    if (f.error) return;
    const c = f.center;
    document.getElementById('fusX').textContent = c ? fmtM(c.x) : '--';
    document.getElementById('fusY').textContent = c ? fmtM(c.y) : '--';
    document.getElementById('fusZ').textContent = c ? fmtM(c.z) : '--';
    const n = f.corners || 0;
    const q = n>=4 ? '4/4 (Perfect)' : (n>0 ? `${n}/4 (Reconstructed)` : '0/4 (Awaiting Markers)');
    const cd = document.getElementById('fusCorners');
    cd.textContent = q;
    cd.style.color = n>=4 ? '#81c784' : (n>0 ? '#ffb74d' : '#777');
  };
  wsFusion.onclose = () => setTimeout(connectFusionWs, 4000);
}
function fmtM(v) { return (v>=0?'+':'') + v.toFixed(3) + ' m'; }

// ── Container insertion ─────────────────────────────────────────────────────
function connectInsertWs() {
  wsInsert = wsConnect('/ws/ros2/insert/feedback');
  wsInsert.onmessage = ({data}) => {
    let f; try { f = JSON.parse(data); } catch { return; }
    const phase = document.getElementById('insPhase');
    const bar = document.getElementById('insBar');
    const txt = document.getElementById('insBarTxt');
    if (f.event === 'feedback') {
      phase.textContent = f.current_phase || '—';
      const pct = Math.round((f.progress||0)*100);
      bar.style.width = pct + '%'; txt.textContent = pct + '%';
    } else if (f.event === 'result') {
      phase.textContent = (f.success ? '✓ ' : '✗ ') + (f.message || (f.success?'complete':'failed'));
      bar.style.width = '100%'; txt.textContent = f.success ? 'DONE' : 'FAILED';
      appendLog(`[INFO] Insertion ${f.success?'succeeded':'failed'}: ${f.message||''}`);
    } else if (f.event === 'accepted') {
      phase.textContent = 'Goal accepted';
    } else if (f.event === 'rejected' || f.event === 'error') {
      phase.textContent = 'Error: ' + (f.message||'');
      appendLog(`[ERROR] Insertion: ${f.message||f.event}`);
    } else if (f.event === 'cancel_requested') {
      phase.textContent = 'Cancelling…';
    }
  };
  wsInsert.onclose = () => setTimeout(connectInsertWs, 4000);
}
async function startInsertion() {
  document.getElementById('insBar').style.width = '0%';
  document.getElementById('insBarTxt').textContent = '0%';
  document.getElementById('insPhase').textContent = 'Sending goal…';
  const body = {
    target_x:        +document.getElementById('insX').value || 0,
    target_y:        +document.getElementById('insY').value || 0,
    hover_above_top: +document.getElementById('insHover').value || 0.03,
    dry_run:         document.getElementById('insDry').checked,
    use_dynamic_tf:  document.getElementById('insDyn').checked,
  };
  await apiPost('/api/ros2/insert/start', body);
}
async function cancelInsertion() {
  await apiPost('/api/ros2/insert/cancel');
}

// ── Boot ────────────────────────────────────────────────────────────────────
checkAuth();
