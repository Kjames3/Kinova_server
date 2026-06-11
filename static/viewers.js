// ═══════════════════════════════════════════════════════════════════════════
// 3-D VIEWERS — Kinova FK arm + live point cloud
// ═══════════════════════════════════════════════════════════════════════════

// ── Kinova Gen3 modified-DH forward kinematics ────────────────────────────
const DH = [
  [0,            0, 0.2848],
  [-Math.PI/2,   0, 0.0118],
  [ Math.PI/2,   0, 0.2506],
  [-Math.PI/2,   0, 0.0114],
  [ Math.PI/2,   0, 0.2085],
  [-Math.PI/2,   0, 0.0116],
  [ Math.PI/2,   0, 0.1059],
];

function mdhMat(alpha, a, d, theta) {
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  const ct = Math.cos(theta), st = Math.sin(theta);
  const m = new THREE.Matrix4();
  m.set(
    ct,    -st,     0,    a,
    st*ca,  ct*ca, -sa,  -sa*d,
    st*sa,  ct*sa,  ca,   ca*d,
    0,      0,      0,    1
  );
  return m;
}

function forwardKinematics(deg) {
  const T = new THREE.Matrix4();
  const pos = [new THREE.Vector3()];
  DH.forEach(([alpha, a, d], i) => {
    T.multiply(mdhMat(alpha, a, d, deg[i] * Math.PI / 180));
    pos.push(new THREE.Vector3().setFromMatrixPosition(T));
  });
  return pos;
}

let armReady = false;
let armRenderer, armCamera, armScene, armControls, armGroup;
let linkMeshes = [], jointMeshes = [], baseMesh = null;

// URDF model (PLAN GUI item 3). When loaded, the mesh model is shown and the
// simple cylinder/sphere arm is hidden; falls back to cylinders if loading fails.
let urdfRobot = null, useUrdf = true, gripperGroup = null, lastJointsDeg = [0,0,0,0,0,0,0];

function initArmViewer() {
  const container = document.getElementById('armCanvas');
  const w = container.clientWidth, h = container.clientHeight || 340;

  armRenderer = new THREE.WebGLRenderer({antialias: true});
  armRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  armRenderer.setSize(w, h);
  armRenderer.setClearColor(0x1e1e1e);
  container.appendChild(armRenderer.domElement);

  armCamera = new THREE.PerspectiveCamera(45, w / h, 0.001, 20);
  armCamera.position.set(0.95, 0.75, 0.95);

  armScene = new THREE.Scene();
  armScene.add(new THREE.AmbientLight(0xffffff, 0.45));
  const sun = new THREE.DirectionalLight(0xffffff, 0.9);
  sun.position.set(1, 2, 1.5);
  armScene.add(sun);

  armGroup = new THREE.Group();
  armGroup.rotation.x = -Math.PI / 2;
  armScene.add(armGroup);

  // Table surface — armGroup is Z-up (DH frame), so the slab must be thin on Z
  // to lie flat under the base. (Item 2: the old (0.9,0.015,0.9) box was thin on
  // Y → a vertical wall passing through the arm.) Top surface sits at z=0.
  const tg = new THREE.BoxGeometry(0.9, 0.9, 0.015);
  const tm = new THREE.MeshLambertMaterial({color: 0x1a2030});
  const table = new THREE.Mesh(tg, tm);
  table.position.set(0, 0, -0.0075);
  armGroup.add(table);

  const grid = new THREE.GridHelper(0.9, 18, 0x222233, 0x1a1a28);
  grid.rotation.x = Math.PI / 2;
  armGroup.add(grid);

  const bg = new THREE.CylinderGeometry(0.055, 0.065, 0.08, 16);
  const bm = new THREE.MeshLambertMaterial({color: 0x4caf50});
  baseMesh = new THREE.Mesh(bg, bm);
  baseMesh.position.set(0, 0, 0.04);
  baseMesh.rotation.x = Math.PI / 2;
  armGroup.add(baseMesh);

  const linkMat = new THREE.MeshLambertMaterial({color: 0x1565c0});
  for (let i = 0; i < 7; i++) {
    const g = new THREE.CylinderGeometry(0.022, 0.022, 1, 8);
    const m = new THREE.Mesh(g, linkMat.clone());
    armGroup.add(m);
    linkMeshes.push(m);
  }

  for (let i = 0; i <= 7; i++) {
    const r = i === 0 ? 0.04 : (i === 7 ? 0.022 : 0.030);
    const c = i === 0 ? 0x4caf50 : (i === 7 ? 0xffc107 : 0x4fc3f7);
    const g = new THREE.SphereGeometry(r, 12, 8);
    const m = new THREE.Mesh(g, new THREE.MeshLambertMaterial({color: c}));
    armGroup.add(m);
    jointMeshes.push(m);
  }

  armControls = new THREE.OrbitControls(armCamera, armRenderer.domElement);
  armControls.target.set(0, 0.42, 0);
  armControls.update();

  const ro = new ResizeObserver(() => {
    const w = container.clientWidth, h = container.clientHeight || 340;
    armCamera.aspect = w / h;
    armCamera.updateProjectionMatrix();
    armRenderer.setSize(w, h);
  });
  ro.observe(container);

  (function loop() {
    requestAnimationFrame(loop);
    armControls.update();
    armRenderer.render(armScene, armCamera);
  })();

  armReady = true;
  updateArm([0,0,0,0,0,0,0]);
  loadArmUrdf();   // item 3 — replace cylinders with the real meshes
}

// ── URDF mesh model (item 3) ──────────────────────────────────────────────
function _setSimpleArmVisible(v) {
  linkMeshes.forEach(m => m.visible = v);
  jointMeshes.forEach(m => m.visible = v);
  if (baseMesh) baseMesh.visible = v;
}

function loadArmUrdf() {
  if (typeof URDFLoader === 'undefined' || typeof THREE.ColladaLoader === 'undefined') {
    return;  // loaders unavailable → keep the cylinder fallback
  }
  const loader = new URDFLoader();
  loader.loadMeshCb = (path, manager, done) => {
    const ext = path.split('.').pop().toLowerCase();
    if (ext === 'dae') {
      new THREE.ColladaLoader(manager).load(path, res => done(res.scene), undefined, () => done(null));
    } else if (ext === 'stl') {
      new THREE.STLLoader(manager).load(path, geom => {
        done(new THREE.Mesh(geom, new THREE.MeshPhongMaterial({color: 0x9aa3ad})));
      }, undefined, () => done(null));
    } else { done(null); }
  };
  loader.load('/static/models/gen3/gen3.urdf', robot => {
    urdfRobot = robot;
    // attach the EE proxy at the wrist/tool link so it follows joint 7
    const eeLink = robot.links['end_effector_link'] || robot.links['bracelet_with_vision_link']
                 || robot.links['bracelet_link'] || robot.links['tool_frame'] || robot;
    gripperGroup = new THREE.Group();
    eeLink.add(gripperGroup);
    armGroup.add(robot);
    if (useUrdf) _setSimpleArmVisible(false);
    else robot.visible = false;
    updateArm(lastJointsDeg);
    if (setArmModel._pending) { const p = setArmModel._pending; setArmModel._pending = null; setArmModel(p); }
  }, undefined, err => {
    console.warn('URDF load failed, keeping simple arm:', err);
    urdfRobot = null;
  });
}

function toggleArmModel() {
  useUrdf = !useUrdf;
  if (urdfRobot) {
    urdfRobot.visible = useUrdf;
    _setSimpleArmVisible(!useUrdf);
  }
  updateArm(lastJointsDeg);
}

// EE proxy (item 7): a small tool at the wrist that swaps with the selection.
function setArmModel(model) {
  if (!gripperGroup) { setArmModel._pending = model; return; }
  while (gripperGroup.children.length) gripperGroup.remove(gripperGroup.children[0]);
  if (model === 'gen3_assembly_tip') {
    // lightweight proxy for the 0.255 m assembly tip (full CAD mesh is 32 MB)
    const g = new THREE.CylinderGeometry(0.008, 0.004, 0.255, 12);
    const m = new THREE.Mesh(g, new THREE.MeshPhongMaterial({color: 0xffc107}));
    m.position.set(0, 0, 0.1275); m.rotation.x = Math.PI / 2;
    gripperGroup.add(m);
  } else if (typeof THREE.STLLoader !== 'undefined') {
    new THREE.STLLoader().load('/static/models/grippers/robotiq_2f_140_base_link.stl', geom => {
      gripperGroup.add(new THREE.Mesh(geom, new THREE.MeshPhongMaterial({color: 0x2b2b2b})));
    });
  }
}

const _dhY = new THREE.Vector3(0, 1, 0);
const _qTmp = new THREE.Quaternion();

function updateArm(deg) {
  lastJointsDeg = deg;
  if (urdfRobot && useUrdf) {
    for (let i = 0; i < 7; i++) {
      const j = urdfRobot.joints['joint_' + (i + 1)];
      if (j) j.setJointValue(deg[i] * Math.PI / 180);
    }
    return;
  }
  const pos = forwardKinematics(deg);
  pos.forEach((p, i) => {
    if (jointMeshes[i]) jointMeshes[i].position.set(p.x, p.y, p.z);
  });
  for (let i = 0; i < 7; i++) {
    const a = pos[i], b = pos[i + 1];
    const dir = new THREE.Vector3().subVectors(b, a);
    const len = dir.length();
    linkMeshes[i].position.copy(new THREE.Vector3().addVectors(a, b).multiplyScalar(0.5));
    linkMeshes[i].scale.set(1, len, 1);
    if (len > 1e-4) {
      _qTmp.setFromUnitVectors(_dhY, dir.normalize());
      linkMeshes[i].quaternion.copy(_qTmp);
    }
  }
}

// ── Point cloud viewer ────────────────────────────────────────────────────
const MAX_PC = 12000;
let pcRenderer, pcCamera, pcScene, pcControls, pcGeom, pcPoints;
let pcReady = false, pcWs = null, pcStreaming = false;

function initPcViewer() {
  const container = document.getElementById('pcCanvas');
  const w = container.clientWidth, h = container.clientHeight || 340;

  pcRenderer = new THREE.WebGLRenderer({antialias: false});
  pcRenderer.setPixelRatio(1);
  pcRenderer.setSize(w, h);
  pcRenderer.setClearColor(0x1e1e1e);
  container.appendChild(pcRenderer.domElement);

  pcCamera = new THREE.PerspectiveCamera(60, w / h, 0.001, 50);
  pcCamera.position.set(0, 0, -0.4);
  pcCamera.lookAt(0, 0, 0.8);

  pcScene = new THREE.Scene();
  pcScene.add(new THREE.AxesHelper(0.25));

  pcGeom = new THREE.BufferGeometry();
  const pos = new Float32Array(MAX_PC * 3);
  const col = new Float32Array(MAX_PC * 3).fill(0.3);
  pcGeom.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  pcGeom.setAttribute('color',    new THREE.BufferAttribute(col, 3));
  pcGeom.setDrawRange(0, 0);

  pcPoints = new THREE.Points(pcGeom,
    new THREE.PointsMaterial({size: 0.004, vertexColors: true, sizeAttenuation: true}));
  pcScene.add(pcPoints);

  pcControls = new THREE.OrbitControls(pcCamera, pcRenderer.domElement);
  pcControls.target.set(0, 0, 0.8);
  pcControls.update();

  const ro = new ResizeObserver(() => {
    const w = container.clientWidth, h = container.clientHeight || 340;
    pcCamera.aspect = w / h;
    pcCamera.updateProjectionMatrix();
    pcRenderer.setSize(w, h);
  });
  ro.observe(container);

  (function loop() {
    requestAnimationFrame(loop);
    pcControls.update();
    pcRenderer.render(pcScene, pcCamera);
  })();

  pcReady = true;
}

function updatePointCloud(posArr, colArr, count) {
  const n = Math.min(count, MAX_PC);
  const p = pcGeom.attributes.position.array;
  const c = pcGeom.attributes.color.array;
  for (let i = 0; i < n; i++) {
    p[i*3]   =  posArr[i*3];
    p[i*3+1] = -posArr[i*3+1];
    p[i*3+2] =  posArr[i*3+2];
    c[i*3]   = colArr[i*3]   / 255;
    c[i*3+1] = colArr[i*3+1] / 255;
    c[i*3+2] = colArr[i*3+2] / 255;
  }
  pcGeom.attributes.position.needsUpdate = true;
  pcGeom.attributes.color.needsUpdate    = true;
  pcGeom.setDrawRange(0, n);

  const overlay = document.getElementById('pcOverlay');
  if (overlay) overlay.style.display = 'none';
  const dot = document.getElementById('dot-pc');
  if (dot) dot.className = 'sdot on';
}

function togglePcStream() {
  if (pcStreaming) {
    pcStreaming = false;
    if (pcWs) { pcWs.close(); pcWs = null; }
    document.getElementById('btnPcStream').textContent = '▶ STREAM';
    document.getElementById('btnPcStream').className = 'btn bg';
    const dot = document.getElementById('dot-pc');
    if (dot) dot.className = 'sdot';
  } else {
    pcStreaming = true;
    document.getElementById('btnPcStream').textContent = '■ STOP';
    document.getElementById('btnPcStream').className = 'btn br';
    startPcWs();
  }
}

function startPcWs() {
  const src = document.getElementById('pcSource').value;
  pcWs = wsConnect(`/ws/pointcloud/${src}`);
  pcWs.binaryType = 'arraybuffer';
  pcWs.onmessage = ({data}) => {
    const count = new DataView(data).getUint32(0, true);
    if (count === 0) return;
    const posF32 = new Float32Array(data, 4,          count * 3);
    const colU8  = new Uint8Array  (data, 4 + count*12, count * 3);
    updatePointCloud(posF32, colU8, count);
  };
  pcWs.onclose = () => { if (pcStreaming) setTimeout(startPcWs, 2000); };
}
