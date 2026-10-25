import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import URDFLoader from 'urdf-loader';

const V = (x, y, z) => new THREE.Vector3(x, y, z);
const ink = 0x152f3c;
const teal = 0x43d8cb;
const mint = 0x88efe2;
const metal = 0x8ca5ac;
const cream = 0xf6f1e9;

function material(color, roughness = .55, metalness = 0) {
  return new THREE.MeshStandardMaterial({ color, roughness, metalness });
}

function add(parent, geometry, mat, x = 0, y = 0, z = 0) {
  const object = new THREE.Mesh(geometry, mat);
  object.position.set(x, y, z);
  object.castShadow = true;
  object.receiveShadow = true;
  parent.add(object);
  return object;
}

function box(parent, w, h, d, mat, x, y, z, radius = 0) {
  const shape = radius ? new THREE.BoxGeometry(w, h, d, 1, 1, 1) : new THREE.BoxGeometry(w, h, d);
  return add(parent, shape, mat, x, y, z);
}

function ball(parent, radius, mat, x, y, z, scale) {
  const object = add(parent, new THREE.SphereGeometry(radius, 24, 16), mat, x, y, z);
  if (scale) object.scale.set(...scale);
  return object;
}

function rod(parent, from, to, radius, mat, radial = 12) {
  const diff = to.clone().sub(from);
  const object = add(parent, new THREE.CylinderGeometry(radius, radius, diff.length(), radial), mat);
  object.position.copy(from).addScaledVector(diff, .5);
  object.quaternion.setFromUnitVectors(V(0, 1, 0), diff.normalize());
  return object;
}

function beam(parent, radius, mat) {
  return add(parent, new THREE.CylinderGeometry(radius, radius, 1, 14), mat);
}

function moveBeam(object, from, to) {
  const diff = to.clone().sub(from);
  object.position.copy(from).addScaledVector(diff, .5);
  object.scale.y = Math.max(.001, diff.length());
  object.quaternion.setFromUnitVectors(V(0, 1, 0), diff.normalize());
}

export function createScene(mount) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xe8efed);
  scene.fog = new THREE.Fog(0xe8efed, 6, 13);

  const camera = new THREE.PerspectiveCamera(37, 1, .05, 50);
  camera.position.set(2.85, 2.36, 3.45);
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.16;
  mount.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(-.17, .88, 0);
  controls.enableDamping = true;
  controls.dampingFactor = .07;
  controls.minDistance = 1.35;
  controls.maxDistance = 6.4;
  controls.maxPolarAngle = Math.PI * .49;

  scene.add(new THREE.HemisphereLight(0xe2eff7, 0x918c84, 1.35));
  const sun = new THREE.DirectionalLight(0xfff3df, 2.2);
  sun.position.set(-2.5, 5, 4);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.left = -4;
  sun.shadow.camera.right = 4;
  sun.shadow.camera.top = 4;
  sun.shadow.camera.bottom = -4;
  sun.shadow.bias = -.0003;
  scene.add(sun);
  const fill = new THREE.PointLight(0xe2ebee, 9, 7);
  fill.position.set(1.5, 2.8, -2.5);
  scene.add(fill);

  const textureLoader = new THREE.TextureLoader();
  const floorMap = textureLoader.load('/textures/wood_floor_diff.jpg');
  const floorNormal = textureLoader.load('/textures/wood_floor_normal.jpg');
  const tableMap = textureLoader.load('/textures/wood_table_diff.jpg');
  const tableNormal = textureLoader.load('/textures/wood_table_normal.jpg');
  floorMap.colorSpace = tableMap.colorSpace = THREE.SRGBColorSpace;
  for (const texture of [floorMap, floorNormal]) {
    texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
    texture.repeat.set(110, 110);
    texture.anisotropy = Math.min(renderer.capabilities.getMaxAnisotropy(), 8);
  }
  tableMap.anisotropy = tableNormal.anisotropy = Math.min(renderer.capabilities.getMaxAnisotropy(), 8);
  const floorMat = new THREE.MeshStandardMaterial({ map: floorMap, normalMap: floorNormal, normalScale: new THREE.Vector2(.38, .38), roughness: .88 });
  const floor = add(scene, new THREE.PlaneGeometry(200, 200), floorMat);
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  floor.castShadow = false;

  const wall = box(scene, 8, 4, .08, material(0xf5f4ed), -.2, 1.6, -2.35);
  wall.castShadow = false;
  box(scene, 8, .08, .1, material(0xc4ccc6), -.2, .08, -2.30);
  const windowFrame = material(0xd9e4df, .4);
  box(scene, 1.55, 1.28, .045, windowFrame, -1.75, 1.7, -2.29);
  box(scene, 1.42, 1.14, .05, material(0xbdd9dd, .15), -1.75, 1.7, -2.26);
  box(scene, .045, 1.24, .07, windowFrame, -1.75, 1.7, -2.22);
  box(scene, 1.5, .045, .07, windowFrame, -1.75, 1.7, -2.22);

  // The detailed local model is the only person shown. Wait for it before revealing the scene.
  const chairX = -.62;
  const frameMat = material(0x59777c, .34, .42);
  const loader = new GLTFLoader();
  const personReady = new Promise((resolve, reject) => loader.load('/models/wheelchair-user.glb', gltf => {
    const real = gltf.scene;
    real.position.x = chairX;
    real.traverse(object => {
      if (!object.isMesh) return;
      object.castShadow = true;
      object.receiveShadow = true;
      for (const mat of [object.material].flat()) if (mat) mat.side = THREE.DoubleSide;
    });
    scene.add(real);
    resolve();
  }, undefined, reject));

  // Close side table: the cup is deliberately positioned inside the small arm's conceptual workspace.
  const table = new THREE.Group();
  scene.add(table);
  const wood = new THREE.MeshStandardMaterial({ map: tableMap, normalMap: tableNormal, normalScale: new THREE.Vector2(.3, .3), roughness: .72 });
  const edge = new THREE.MeshStandardMaterial({ color: 0xc1b1a2, map: tableMap, roughness: .78 });
  box(table, .72, .055, .60, wood, .52, 1.005, .06);
  box(table, .72, .035, .60, edge, .52, .97, .06);
  for (const x of [.22, .82]) for (const z of [-.17, .29]) {
    rod(table, V(x, .06, z), V(x, .97, z), .022, edge);
  }
  box(table, .50, .025, .42, wood, .52, .30, .06);

  // A lightweight lidded cup. The liquid surface stays horizontal in the animated scene.
  const sourceCupBase = V(.22, 1.035, -.02);
  const cup = new THREE.Group();
  cup.position.copy(sourceCupBase);
  scene.add(cup);
  const glass = new THREE.MeshPhysicalMaterial({ color: 0x74d7e0, roughness: .2, metalness: .08, transparent: true, opacity: .76, clearcoat: .8 });
  const lid = material(0xe9f5f2, .3);
  const water = new THREE.MeshPhysicalMaterial({ color: 0x5ca8de, roughness: .15, transparent: true, opacity: .56 });
  add(cup, new THREE.CylinderGeometry(.046, .037, .135, 32), glass, 0, .069, 0);
  add(cup, new THREE.CylinderGeometry(.044, .039, .072, 32), water, 0, .038, 0);
  add(cup, new THREE.CylinderGeometry(.052, .052, .018, 32), lid, 0, .147, 0);
  add(cup, new THREE.TorusGeometry(.049, .003, 8, 32), material(0x2998aa), 0, .145, 0).rotation.x = Math.PI / 2;
  rod(cup, V(0, .155, 0), V(0, .262, 0), .006, lid, 12);
  rod(cup, V(0, .262, 0), V(-.14, .290, -.16), .006, lid, 12);
  const handle = add(cup, new THREE.TorusGeometry(.047, .009, 10, 24, Math.PI * 1.2), lid, .048, .085, 0);
  handle.rotation.y = Math.PI / 2;

  // Fixed camera on a wheelchair-mounted mast, looking down at the table.
  const cameraMount = new THREE.Group();
  scene.add(cameraMount);
  const cameraPos = V(-.08, 1.88, -.44);
  rod(cameraMount, V(-.16, .78, -.40), V(-.16, 1.88, -.40), .016, frameMat);
  rod(cameraMount, V(-.16, 1.88, -.40), cameraPos, .014, frameMat);
  const camBody = box(cameraMount, .15, .075, .10, material(0x243d48, .45), ...cameraPos.toArray());
  camBody.rotation.x = -.3;
  ball(cameraMount, .023, material(0x7ce5dd, .16, .4), cameraPos.x, cameraPos.y - .013, cameraPos.z + .06);
  ball(cameraMount, .007, material(0x71edc6, .2), cameraPos.x + .052, cameraPos.y + .018, cameraPos.z + .055);
  const cameraRays = new THREE.Group();
  scene.add(cameraRays);
  const rayMat = new THREE.LineBasicMaterial({ color: 0x58dbc9, transparent: true, opacity: .21 });
  for (const end of [V(.06, 1.02, -.13), V(.46, 1.02, -.13), V(.06, 1.02, .31), V(.46, 1.02, .31)]) {
    const ray = new THREE.Line(new THREE.BufferGeometry().setFromPoints([cameraPos, end]), rayMat);
    cameraRays.add(ray);
  }
  cameraRays.visible = false;

  // Approximate SO-101 sized visual linkage. This does not generate raw servo targets.
  const shoulder = V(-.18, 1.16, .08);
  const arm = new THREE.Group();
  arm.visible = false;
  scene.add(arm);
  rod(scene, V(-.30, .78, .08), V(-.18, 1.10, .08), .024, frameMat);
  box(scene, .16, .018, .14, frameMat, -.18, 1.095, .08);
  let cadArm = null;
  const cadJointNames = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex'];
  const cadBounds = [[-1.9, 1.9], [-1.74, 1.74], [-1.68, 1.68], [-1.65, 1.65]];
  const cadValues = [0, 0, 0, 0];
  function cadToolPosition() {
    cadArm.updateMatrixWorld(true);
    return cadArm.links.gripper_frame_link.getWorldPosition(new THREE.Vector3());
  }
  function solveCad(target) {
    if (!cadArm) return;
    for (let iteration = 0; iteration < 11; iteration++) {
      for (let j = 0; j < cadJointNames.length; j++) {
        const at = cadToolPosition();
        const error = target.clone().sub(at);
        if (error.lengthSq() < .000018) break;
        const old = cadValues[j];
        cadArm.setJointValue(cadJointNames[j], Math.min(cadBounds[j][1], old + .006));
        const moved = cadToolPosition();
        const derivative = moved.sub(at).divideScalar(.006);
        const delta = THREE.MathUtils.clamp(error.dot(derivative) / (derivative.lengthSq() + .012), -.18, .18);
        cadValues[j] = THREE.MathUtils.clamp(old + delta * .72, ...cadBounds[j]);
        cadArm.setJointValue(cadJointNames[j], cadValues[j]);
      }
    }
  }
  const armLoader = new URDFLoader();
  const armReady = new Promise((resolve, reject) => armLoader.load('/models/so101/so101_new_calib.urdf', robot => {
    cadArm = robot;
    const cadMount = new THREE.Group();
    cadMount.position.set(-.18, 1.10, .08);
    cadMount.rotation.y = -Math.PI / 2;
    robot.rotation.x = -Math.PI / 2;
    robot.traverse(object => {
      if (object.isMesh) {
        object.castShadow = true;
        object.receiveShadow = true;
        if (object.material?.color) {
          const isPrinted = object.material.color.r > .5;
          object.material.color.set(isPrinted ? 0xe4b229 : 0x252b30);
          object.material.roughness = isPrinted ? .74 : .48;
        }
      }
    });
    cadMount.add(robot);
    scene.add(cadMount);
    poseArm(armTarget);
    resolve();
  }, undefined, reject));
  const printed = material(0x41b7ad, .48, .07);
  const printedLight = material(0x75ddd0, .41, .08);
  const motorMat = material(0x203943, .41, .17);
  const ringMat = material(0xa9f2e7, .33, .28);
  rod(arm, V(-.18, .83, .08), shoulder, .026, motorMat);
  box(arm, .17, .048, .16, frameMat, -.18, .84, .08);
  ball(arm, .054, motorMat, ...shoulder.toArray());
  ball(arm, .043, printedLight, ...shoulder.toArray(), [1, .38, 1]);
  const upper = beam(arm, .034, printed);
  const lower = beam(arm, .027, printedLight);
  const wristBeam = beam(arm, .019, motorMat);
  const elbowJoint = ball(arm, .045, motorMat, 0, 0, 0);
  const elbowRing = ball(arm, .027, ringMat, 0, 0, 0, [1, .45, 1]);
  const wristJoint = ball(arm, .037, motorMat, 0, 0, 0);
  const gripper = new THREE.Group();
  arm.add(gripper);
  box(gripper, .105, .032, .07, motorMat, 0, .047, 0);
  const fingerLeft = box(gripper, .015, .075, .022, printedLight, -.073, -.008, 0);
  const fingerRight = box(gripper, .015, .075, .022, printedLight, .073, -.008, 0);
  const padLeft = box(gripper, .023, .015, .025, ringMat, -.073, -.048, .005);
  const padRight = box(gripper, .023, .015, .025, ringMat, .073, -.048, .005);
  const armTarget = V(-.025, 1.20, .12);
  let fingerGap = .073;
  const upperLength = .225;
  const lowerLength = .220;
  const wristOffset = .060;
  let reachState = { reachable: true, distance: 0, max: upperLength + lowerLength };

  function poseArm(target) {
    armTarget.copy(target);
    const wrist = target.clone().add(V(0, wristOffset, 0));
    const toWrist = wrist.clone().sub(shoulder);
    const rawDistance = toWrist.length();
    const distance = THREE.MathUtils.clamp(rawDistance, Math.abs(upperLength - lowerLength) + .005, upperLength + lowerLength - .006);
    const direction = toWrist.normalize();
    const constrainedWrist = shoulder.clone().addScaledVector(direction, distance);
    const along = (upperLength ** 2 - lowerLength ** 2 + distance ** 2) / (2 * distance);
    const side = Math.sqrt(Math.max(0, upperLength ** 2 - along ** 2));
    const bend = V(0, 1, -.32).projectOnPlane(direction).normalize();
    const elbow = shoulder.clone().addScaledVector(direction, along).addScaledVector(bend, side);
    moveBeam(upper, shoulder, elbow);
    moveBeam(lower, elbow, constrainedWrist);
    moveBeam(wristBeam, constrainedWrist, constrainedWrist.clone().add(V(0, -.055, 0)));
    elbowJoint.position.copy(elbow);
    elbowRing.position.copy(elbow);
    wristJoint.position.copy(constrainedWrist);
    gripper.position.copy(constrainedWrist).add(V(0, -.055, 0));
    reachState = { reachable: rawDistance <= upperLength + lowerLength - .005, distance: rawDistance, max: upperLength + lowerLength };
    solveCad(target);
  }

  function setGrip(closed) {
    const fraction = typeof closed === 'number' ? THREE.MathUtils.clamp(closed, 0, 1) : (closed ? 1 : 0);
    fingerGap = THREE.MathUtils.lerp(.077, .051, fraction);
    fingerLeft.position.x = -fingerGap;
    fingerRight.position.x = fingerGap;
    padLeft.position.x = -fingerGap;
    padRight.position.x = fingerGap;
    if (cadArm) cadArm.setJointValue('gripper', THREE.MathUtils.lerp(1.3, .1, fraction));
  }
  setGrip(false);
  poseArm(armTarget);

  const marker = new THREE.Group();
  scene.add(marker);
  const markerMat = new THREE.MeshBasicMaterial({ color: teal, transparent: true, opacity: .95 });
  const markerRing = add(marker, new THREE.TorusGeometry(.082, .0035, 8, 48), markerMat, 0, .22, 0);
  markerRing.rotation.x = Math.PI / 2;
  for (let i = 0; i < 4; i++) {
    const angle = i * Math.PI / 2;
    const x = Math.cos(angle) * .08;
    const z = Math.sin(angle) * .08;
    rod(marker, V(x, .17, z), V(x, .24, z), .0025, markerMat, 5);
  }
  marker.visible = false;

  const pathPoints = [V(.22, 1.20, -.02), V(.22, 1.12, -.02), V(.22, 1.25, -.02), V(.01, 1.35, .17), V(-.45, 1.07, .28)];
  const pathGeo = new THREE.BufferGeometry().setFromPoints(pathPoints);
  const pathLine = new THREE.Line(pathGeo, new THREE.LineDashedMaterial({ color: 0x47cbbc, dashSize: .03, gapSize: .022, transparent: true, opacity: .64 }));
  pathLine.computeLineDistances();
  scene.add(pathLine);
  pathLine.visible = false;
  const pathNodes = pathPoints.map(p => ball(scene, .009, new THREE.MeshBasicMaterial({ color: 0x44dccc }), ...p.toArray()));
  pathNodes.forEach(node => { node.visible = false; node.castShadow = false; });

  const views = {
    overview: { position: V(2.85, 2.36, 3.45), target: V(-.17, .88, 0) },
    reach: { position: V(2.25, 1.62, 2.55), target: V(-.16, 1.03, .08) },
    side: { position: V(2.75, 1.75, .36), target: V(-.15, 1.03, .08) },
    user: { position: V(-.56, 1.55, 1.45), target: V(.20, 1.16, .06) },
    detail: { position: V(1.45, 1.60, 1.65), target: V(.02, 1.12, -.02) },
    sip: { position: V(1.45, 1.72, 1.82), target: V(-.48, 1.23, .12) },
  };
  let cameraTween = null;

  function setView(name) {
    const view = views[name] || views.overview;
    cameraTween = {
      fromPosition: camera.position.clone(), fromTarget: controls.target.clone(),
      toPosition: view.position.clone(), toTarget: view.target.clone(), progress: 0,
    };
  }

  function resize() {
    const width = Math.max(1, mount.clientWidth);
    const height = Math.max(1, mount.clientHeight);
    camera.aspect = width / height;
    camera.fov = width < 650 ? 50 : width < 900 ? 42 : 37;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
  }
  new ResizeObserver(resize).observe(mount);
  resize();

  let scanGlow = 0;
  function render(dt) {
    scanGlow += dt;
    if (cameraTween) {
      cameraTween.progress = Math.min(1, cameraTween.progress + dt / 1.25);
      const t = cameraTween.progress * cameraTween.progress * (3 - 2 * cameraTween.progress);
      camera.position.copy(cameraTween.fromPosition).lerp(cameraTween.toPosition, t);
      controls.target.copy(cameraTween.fromTarget).lerp(cameraTween.toTarget, t);
      if (cameraTween.progress >= 1) cameraTween = null;
    }
    marker.position.set(cup.position.x, cup.position.y, cup.position.z);
    markerRing.scale.setScalar(1 + .08 * Math.sin(scanGlow * 3));
    controls.update();
    renderer.render(scene, camera);
  }

  function cupScreenPosition() {
    const point = cup.position.clone().add(V(0, .20, 0)).project(camera);
    return { x: (point.x * .5 + .5) * mount.clientWidth, y: (-point.y * .5 + .5) * mount.clientHeight, visible: point.z < 1 };
  }

  return {
    ready: Promise.all([personReady, armReady]),
    render, resize, setView, poseArm, setGrip, cupScreenPosition,
    graphicsInfo: () => {
      const gl = renderer.getContext();
      const debug = gl.getExtension('WEBGL_debug_renderer_info');
      return { renderer: debug ? gl.getParameter(debug.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER), webgl2: renderer.capabilities.isWebGL2 };
    },
    getToolError: () => cadArm ? cadToolPosition().distanceTo(armTarget) : null,
    getReach: () => reachState,
    canReach: target => target.clone().add(V(0, wristOffset, 0)).distanceTo(shoulder) <= upperLength + lowerLength - .006,
    getArmTarget: () => armTarget.clone(),
    getCupPosition: () => cup.position.clone(),
    getSourceCupPosition: () => sourceCupBase.clone(),
    setCupPosition: point => cup.position.copy(point),
    setDetection: visible => { marker.visible = visible; cameraRays.visible = visible; },
    setPath: visible => { pathLine.visible = visible; pathNodes.forEach(node => { node.visible = visible; }); },
    setCupVisible: visible => { cup.visible = visible; },
    dispose: () => { controls.dispose(); renderer.dispose(); },
    points: {
      rest: V(-.025, 1.20, .12),
      hover: V(.22, 1.20, -.02),
      grip: V(.22, 1.12, -.02),
      lift: V(.22, 1.25, -.02),
      transit: V(.01, 1.35, .17),
      sip: V(-.45, 1.07, .28),
    },
  };
}
