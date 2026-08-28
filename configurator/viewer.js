// The 3D robot preview, shared by the configurator's two pages.
//
// index.html poses it by hand from sliders; calibrate.html drives it from live
// servo telemetry, so the model tracks the physical arm while it's swept. Both
// need the same scene, the same URDF fetch and the same Z-up correction, and a
// second copy of that would drift — hence one module.
//
// Everything here is display-only: nothing in this file talks to the servo bus
// or writes to the repo. `load()` fetches /urdf (server.py runs xacro) and the
// meshes come back through /pkg/<pkg>/meshes/<file>.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';

// Display order for the arm's canonical joints; tool joints sort after, by
// name. Shared so the slider list and the calibration rail agree.
export const ARM_ORDER = ['joint_base', 'joint_shoulder', 'joint_elbow',
                          'joint_wrist_tilt', 'joint_wrist_roll'];

export function jointOrder(a, b) {
  const ia = ARM_ORDER.indexOf(a), ib = ARM_ORDER.indexOf(b);
  if (ia !== -1 && ib !== -1) return ia - ib;
  if (ia !== -1) return -1;
  if (ib !== -1) return 1;
  return a.localeCompare(b);
}

// A `continuous` joint, or one whose URDF limits are degenerate, has no usable
// band — fall back to a full turn so a slider still spans something sane.
export function jointRange(joint) {
  let lower = joint.limit ? Number(joint.limit.lower) : -Math.PI;
  let upper = joint.limit ? Number(joint.limit.upper) : Math.PI;
  const bounded = joint.jointType !== 'continuous' && lower !== upper;
  if (!bounded) { lower = -Math.PI; upper = Math.PI; }
  return { lower, upper, bounded, isLinear: joint.jointType === 'prismatic' };
}

export function createViewer(container, opts = {}) {
  const {
    background = 0x0a0d12,
    highlightColor = 0x14b8a6,      // --accent
    onStatus = () => {},            // (text, kind) — kind: '', 'ok', 'bad'
  } = opts;

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.shadowMap.enabled = true;
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(background);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 20);
  camera.position.set(0.6, 0.5, 0.6);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.25, 0);
  controls.update();

  scene.add(new THREE.AmbientLight(0xffffff, 0.4));
  const sun = new THREE.DirectionalLight(0xffffff, 0.9);
  sun.position.set(2, 4, 2);
  sun.castShadow = true;
  scene.add(sun);
  scene.add(new THREE.GridHelper(2, 20, 0x444444, 0x222222));

  // URDF convention is Z-up. Rotate the whole robot for three.js's Y-up camera.
  const robotRoot = new THREE.Group();
  robotRoot.rotation.x = -Math.PI / 2;
  scene.add(robotRoot);

  const loader = new URDFLoader();
  let robot = null;
  let framed = false;

  // ---- sizing -------------------------------------------------------
  // A ResizeObserver rather than a window listener: calibrate.html hides the
  // viewer's whole tab, so the container goes 0×0 and comes back at a size the
  // window never changed to. Zero-size frames are skipped — resizing the
  // renderer to 0 makes the next real frame come back blank.
  function resize() {
    const w = container.clientWidth, h = container.clientHeight;
    if (!w || !h) return;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }
  new ResizeObserver(resize).observe(container);
  resize();

  // ---- highlight ----------------------------------------------------
  // Which link a joint actually moves, shown in the scene: the joint's own
  // child link burns bright, everything downstream of it glows faintly. That
  // reads as "this is the piece that turns, and this is what it carries".
  const savedMaterials = new Map();   // mesh -> original material

  // A link's own visuals hang directly off it; anything under a child joint
  // belongs to the next link along, so the two sets are separated by walking
  // children rather than by depth.
  function ownMeshes(link) {
    const out = [];
    for (const child of link.children) {
      if (child.isURDFJoint) continue;
      child.traverse(o => { if (o.isMesh) out.push(o); });
    }
    return out;
  }
  function paint(mesh, intensity) {
    if (!savedMaterials.has(mesh)) savedMaterials.set(mesh, mesh.material);
    const base = savedMaterials.get(mesh);
    const mat = Array.isArray(base) ? base[0] : base;
    const clone = mat.clone();
    if (clone.emissive) {
      clone.emissive.setHex(highlightColor);
      clone.emissiveIntensity = intensity;
    }
    mesh.material = clone;
  }

  function clearHighlight() {
    for (const [mesh, mat] of savedMaterials) {
      if (mesh.material !== mat && mesh.material.dispose) mesh.material.dispose();
      mesh.material = mat;
    }
    savedMaterials.clear();
  }

  // name: joint name to highlight, or null to clear.
  //
  // Only the link the joint drives directly. Washing the whole downstream
  // subtree was tried and is useless on a serial arm: `joint_base` carries
  // everything, so the wash covered the entire robot and said nothing. One
  // bright link answers the actual question — which piece does this joint turn.
  function highlight(name) {
    clearHighlight();
    if (!robot || !name) return;
    const joint = robot.joints[name];
    if (!joint) return;
    for (const link of joint.children.filter(c => c.isURDFLink)) {
      for (const m of ownMeshes(link)) paint(m, 0.6);
    }
  }

  // ---- joints -------------------------------------------------------
  // Actuated joints only, in display order. `mimic` joints follow their source
  // and must not be posed directly.
  function actuatedJoints() {
    if (!robot) return [];
    return Object.entries(robot.joints)
      .filter(([, j]) => j.jointType !== 'fixed' && !j.mimicJoint)
      .sort(([a], [b]) => jointOrder(a, b))
      .map(([name, joint]) => ({ name, joint, ...jointRange(joint) }));
  }

  // URDFLoader clamps a joint to its <limit> unless told not to. calibrate.html
  // needs the opposite: a joint driven past what the URDF declares has to show
  // up in the scene, because that mismatch is the thing being looked for.
  function setIgnoreLimits(flag) {
    if (!robot) return;
    for (const j of Object.values(robot.joints)) j.ignoreLimits = !!flag;
  }

  function setJoint(name, value) {
    if (!robot || !robot.joints[name]) return false;
    robot.setJointValue(name, value);
    return true;
  }

  function getJoint(name) {
    const j = robot && robot.joints[name];
    return j ? Number(j.angle) : null;
  }

  function setPose(pose) {
    for (const [name, v] of Object.entries(pose)) setJoint(name, v);
  }

  // ---- camera -------------------------------------------------------
  function boundsOf() {
    robotRoot.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(robot);
    return (box.isEmpty() || !isFinite(box.min.x)) ? null : box;
  }

  // Meshes stream in one at a time, and the bounding box is "real" the moment
  // the FIRST one lands — framing then puts the camera around a fraction of the
  // arm and leaves the rest cropped off-screen. Wait for the box to stop growing
  // instead, with a ceiling so a mesh that never loads can't stall the framing.
  function fitWhenSettled() {
    let stable = 0, last = -1, tries = 0;
    const tick = () => {
      if (!robot) return;
      const box = boundsOf();
      const size = box ? box.getSize(new THREE.Vector3()).length() : -1;
      if (size > 0 && Math.abs(size - last) < 1e-4) stable++; else stable = 0;
      last = size;
      if (stable >= 8 || ++tries > 240) { if (size > 0) fit(); return; }
      requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
  }

  function fit(attempt = 0) {
    if (!robot) return;
    const box = boundsOf();
    if (!box) {
      if (attempt < 40) requestAnimationFrame(() => fit(attempt + 1));
      return;
    }
    const size   = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z) || 0.5;
    const fov    = camera.fov * Math.PI / 180;
    const dist   = (maxDim / 2) / Math.tan(fov / 2) * 1.55;   // 1.55 = a little padding
    const dir    = new THREE.Vector3(0.85, 0.5, 1).normalize();
    camera.position.copy(center).add(dir.multiplyScalar(dist));
    camera.near = Math.max(dist / 200, 0.001);
    camera.far  = dist * 200;
    camera.updateProjectionMatrix();
    controls.target.copy(center);
    controls.minDistance = dist * 0.25;
    controls.maxDistance = dist * 6;
    controls.update();
  }

  // ---- load ---------------------------------------------------------
  // `params` are xacro args (rail lengths, mounts); omitted, the server renders
  // what's on disk. Returns the robot, or throws with the server's message.
  async function load(params = null) {
    onStatus('Loading URDF…', '');
    const qs = params ? new URLSearchParams(params).toString() : '';
    const resp = await fetch('/urdf' + (qs ? '?' + qs : ''), { cache: 'no-store' });
    if (!resp.ok) {
      const detail = await resp.json().catch(() => ({}));
      const msg = detail.error || resp.statusText;
      onStatus(`URDF load failed: ${msg}. Source ROS before starting server.py.`, 'bad');
      throw new Error(msg);
    }
    const text = await resp.text();
    clearHighlight();
    if (robot) {
      robotRoot.remove(robot);
      robot.traverse(o => {
        if (o.geometry) o.geometry.dispose();
        if (o.material) (Array.isArray(o.material) ? o.material : [o.material]).forEach(m => m.dispose());
      });
    }
    robot = loader.parse(text);
    robotRoot.add(robot);
    if (!framed) { framed = true; fitWhenSettled(); }
    onStatus('URDF loaded.', 'ok');
    return robot;
  }

  (function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  })();

  return {
    get robot() { return robot; },
    load, fit, resize, setJoint, getJoint, setPose, actuatedJoints, setIgnoreLimits,
    highlight, clearHighlight,
    scene, camera, controls, renderer,
  };
}
