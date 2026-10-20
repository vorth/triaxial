import * as THREE from "three";
import { TrackballControls } from "three/addons/controls/TrackballControls.js";
import { WeaveGraph } from "./weaveGraph.js";
import { DEFAULT_STRIP_PARAMS, buildStripModel, createRibbonLayout, writeRibbonMesh, crossingPosition } from "./strips.js";
import { seedPlacementPositions } from "./seeding.js";
import { reconcileStripState } from "./stripState.js";
import { DEFAULT_SOLVER_PARAMS, stepSolver, adjustTails } from "./solver.js";

// ---------------------------------------------------------------------------
// Scene setup
// ---------------------------------------------------------------------------

const container = document.getElementById("canvas-container");

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14161a);

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.01, 1000);
camera.position.set(0, 0, 8);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
container.appendChild(renderer.domElement);

scene.add(new THREE.AmbientLight(0xffffff, 1.0));

const controls = new TrackballControls(camera, renderer.domElement);
controls.rotateSpeed = 3.0;
controls.zoomSpeed = 1.2;
controls.panSpeed = 0.8;
controls.noPan = false;
controls.staticMoving = false;
controls.dynamicDampingFactor = 0.15;

window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
  controls.handleResize();
});

// ---------------------------------------------------------------------------
// Graph + strips. The weave graph is topology only; every position lives in
// the strip solver's state, and graph vertices are located through its
// crossing table (crossingPosition). Graph positions exist only once, to
// build the seed's strips.
// ---------------------------------------------------------------------------

const graph = new WeaveGraph();

// Face-on regular hexagon in the XY plane with equilateral boundary
// triangles: the apex radius is the unit hexagon's apothem plus a unit
// triangle's height.
function seedLayout(hexVerts, apexVerts) {
  const positions = new Map();
  const hexR = 1 / (2 * Math.sin(Math.PI / 6)); // = 1 for hexagon (edge length 1)
  for (let i = 0; i < 6; i++) {
    const a = (Math.PI / 3) * i - Math.PI / 2;
    positions.set(hexVerts[i], { x: hexR * Math.cos(a), y: hexR * Math.sin(a), z: 0 });
  }
  const apexR = hexR * Math.cos(Math.PI / 6) + Math.sqrt(3) / 2;
  for (let i = 0; i < 6; i++) {
    const a = (Math.PI / 3) * i - Math.PI / 2 + Math.PI / 6;
    positions.set(apexVerts[i], { x: apexR * Math.cos(a), y: apexR * Math.sin(a), z: 0 });
  }
  return positions;
}

const STRIP_STORAGE_KEY = "weaveGraph.stripParams";
const stripParams = { ...DEFAULT_STRIP_PARAMS };
try {
  const raw = JSON.parse(localStorage.getItem(STRIP_STORAGE_KEY) || "{}");
  for (const key of ["width", "thickness"]) {
    if (typeof raw[key] === "number" && Number.isFinite(raw[key])) stripParams[key] = raw[key];
  }
} catch {
  // ignore - defaults are fine
}

const SOLVER_STORAGE_KEY = "weaveGraph.solverParams";
const solverParams = { ...DEFAULT_SOLVER_PARAMS };
try {
  const raw = JSON.parse(localStorage.getItem(SOLVER_STORAGE_KEY) || "{}");
  for (const key of Object.keys(DEFAULT_SOLVER_PARAMS)) {
    if (typeof raw[key] === "number" && Number.isFinite(raw[key])) solverParams[key] = raw[key];
  }
} catch {
  // ignore - defaults are fine
}

let stripModel = null;
let stripState = null;
let ribbonLayout = null;
const ribbonMaterial = new THREE.MeshStandardMaterial({
  vertexColors: true,
  roughness: 0.55,
  metalness: 0.0,
  side: THREE.DoubleSide,
});
const ribbonMesh = new THREE.Mesh(new THREE.BufferGeometry(), ribbonMaterial);
ribbonMesh.frustumCulled = false; // vertices move every frame; bounds go stale
scene.add(ribbonMesh);

// Replace the strip model and state after a topology edit. `seeded` gives
// positions for vertices the current state can't locate (fresh vertices;
// all of them when starting over).
function rebuildStrips(seeded, fromScratch = false) {
  const previous = stripModel;
  stripModel = buildStripModel(graph, previous, stripParams.samplesPerEdge);
  stripState = fromScratch
    ? reconcileStripState(null, null, stripModel, seeded, stripParams)
    : reconcileStripState(previous, stripState, stripModel, seeded, stripParams);
  rebuildRibbonBuffers();
}

// Ribbon buffers depend on node counts, which change on edits and when
// tails grow or trim.
function rebuildRibbonBuffers() {
  ribbonLayout = createRibbonLayout(stripModel, stripState);
  const geo = new THREE.BufferGeometry();
  const position = new THREE.BufferAttribute(new Float32Array(3 * ribbonLayout.vertexCount), 3);
  const normal = new THREE.BufferAttribute(new Float32Array(3 * ribbonLayout.vertexCount), 3);
  position.setUsage(THREE.DynamicDrawUsage);
  normal.setUsage(THREE.DynamicDrawUsage);
  geo.setAttribute("position", position);
  geo.setAttribute("normal", normal);
  geo.setAttribute("color", new THREE.BufferAttribute(ribbonLayout.color, 3));
  geo.setIndex(new THREE.BufferAttribute(ribbonLayout.index, 1));
  ribbonMesh.geometry.dispose();
  ribbonMesh.geometry = geo;
}

function writeRibbons() {
  const geo = ribbonMesh.geometry;
  writeRibbonMesh(stripModel, ribbonLayout, stripState, stripParams, geo.attributes.position.array, geo.attributes.normal.array);
  geo.attributes.position.needsUpdate = true;
  geo.attributes.normal.needsUpdate = true;
}

function seed() {
  const { hexVerts, apexVerts } = graph.seedHexagon();
  stripModel = null;
  rebuildStrips(seedLayout(hexVerts, apexVerts), true);
}
seed();

let currentMode = { type: "place", n: 6 };

// ---------------------------------------------------------------------------
// Rendering: spheres for concave (placement-target) vertices; the graph
// itself is shown only through the woven strips
// ---------------------------------------------------------------------------

const targetGeometry = new THREE.SphereGeometry(0.164, 16, 12);
const targetMaterial = new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xffffff, emissiveIntensity: 0.5, roughness: 0.4 });
const targetMaterialHover = new THREE.MeshStandardMaterial({ color: 0xffd24c, emissive: 0x6b4a1a, roughness: 0.4 });
const deleteTargetMaterial = new THREE.MeshStandardMaterial({ color: 0xff5c5c, emissive: 0x6b1a1a, roughness: 0.4 });

const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
dirLight.position.set(2, 3, 4);
scene.add(dirLight);

let targetMeshes = new Map(); // vertexId -> Mesh
const targetsGroup = new THREE.Group();
scene.add(targetsGroup);

function rebuildTargets() {
  for (const mesh of targetMeshes.values()) targetsGroup.remove(mesh);
  targetMeshes.clear();

  const isDeleteMode = currentMode.type === "delete";
  const candidateVerts = isDeleteMode
    ? graph.boundary.filter((v) => graph.vertexType(v) !== "unknown" && graph.polygonAt(v) && graph.polygonAt(v).record)
    : graph.concaveVertices();

  for (const v of candidateVerts) {
    const mat = isDeleteMode ? deleteTargetMaterial : targetMaterial;
    const mesh = new THREE.Mesh(targetGeometry, mat.clone());
    mesh.userData.vertexId = v;
    targetsGroup.add(mesh);
    targetMeshes.set(v, mesh);
  }
}
rebuildTargets();

// Graph vertices are located through the strips' crossing table.
function updateRenderGeometry() {
  writeRibbons();

  for (const [v, mesh] of targetMeshes) {
    const p = crossingPosition(stripModel, stripState, v);
    if (p) mesh.position.set(p.x, p.y, p.z);
  }
}

// ---------------------------------------------------------------------------
// Mode chooser UI
// ---------------------------------------------------------------------------

const polygonGroup = document.getElementById("polygon-group");
const deleteBtn = document.getElementById("delete-mode");
const hint = document.getElementById("hint");

function setModePlace(n) {
  currentMode = { type: "place", n };
  deleteBtn.classList.remove("active");
  for (const btn of polygonGroup.querySelectorAll(".mode-btn")) {
    btn.classList.toggle("active", parseInt(btn.dataset.n, 10) === n);
  }
  const label = n === 3 ? "triangle" : n === 4 ? "square" : n === 5 ? "pentagon" : n === 6 ? "hexagon" : n + "-gon";
  hint.textContent = `Click a highlighted vertex to place a ${label}. Drag to rotate, scroll to zoom.`;
  rebuildTargets();
}

function setModeDelete() {
  currentMode = { type: "delete" };
  deleteBtn.classList.add("active");
  for (const btn of polygonGroup.querySelectorAll(".mode-btn")) btn.classList.remove("active");
  hint.textContent = "Click a boundary vertex to delete its cell (only removable if nothing was built on it).";
  rebuildTargets();
}

for (const btn of polygonGroup.querySelectorAll(".mode-btn[data-n]")) {
  btn.addEventListener("click", () => setModePlace(parseInt(btn.dataset.n, 10)));
}
deleteBtn.addEventListener("click", setModeDelete);

const flipBtn = document.getElementById("flip-weave");
// Flipping over/under can't be a continuous motion (every crossing would
// have to pass through itself), so rebuild the strips from the current
// crossing positions with the other sigma.
flipBtn.addEventListener("click", () => {
  const positions = new Map([...stripModel.crossings.keys()].map((v) => [v, crossingPosition(stripModel, stripState, v)]));
  stripModel.sigma = -stripModel.sigma;
  rebuildStrips(positions, true);
});

// ---------------------------------------------------------------------------
// Interaction: raycast against target spheres on click/tap
// ---------------------------------------------------------------------------

const raycaster = new THREE.Raycaster();
raycaster.params.Points.threshold = 0.15;
const pointer = new THREE.Vector2();

let pointerDownPos = null;

function onPointerDown(e) {
  pointerDownPos = { x: e.clientX, y: e.clientY };
}

function onPointerUp(e) {
  if (!pointerDownPos) return;
  const dx = e.clientX - pointerDownPos.x;
  const dy = e.clientY - pointerDownPos.y;
  pointerDownPos = null;
  if (Math.hypot(dx, dy) > 6) return; // was a drag (trackball gesture), not a tap

  pointer.x = (e.clientX / window.innerWidth) * 2 - 1;
  pointer.y = -(e.clientY / window.innerHeight) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);

  const meshes = Array.from(targetMeshes.values());
  const hits = raycaster.intersectObjects(meshes, false);
  if (hits.length === 0) return;

  const vertexId = hits[0].object.userData.vertexId;
  handleTargetClick(vertexId);
}

function handleTargetClick(vertexId) {
  if (currentMode.type === "place") {
    const record = graph.placePolygon(vertexId, currentMode.n);
    if (!record) return;
    const existingChain =
      record.kind === "fuse" ? [record.u, ...record.siteVerts] : [record.u, ...record.siteVerts, record.w];
    // The strip model hasn't been rebuilt yet, so it still has every chain
    // vertex (even the one a fuse just merged away).
    const knownPts = existingChain.map((v) => crossingPosition(stripModel, stripState, v));
    const { mid, apex } = seedPlacementPositions(knownPts, record.midVerts.length, record.apexVerts.length);
    const seeded = new Map();
    record.midVerts.forEach((v, i) => seeded.set(v, mid[i]));
    record.apexVerts.forEach((v, i) => seeded.set(v, apex[i]));
    rebuildStrips(seeded);
  } else if (currentMode.type === "delete") {
    if (!graph.deleteAt(vertexId)) return;
    rebuildStrips(new Map());
  }
  rebuildTargets();
}

renderer.domElement.addEventListener("pointerdown", onPointerDown);
renderer.domElement.addEventListener("pointerup", onPointerUp);

// ---------------------------------------------------------------------------
// Tuning panel: solver parameters and strip geometry
// ---------------------------------------------------------------------------

const tuningPanel = document.getElementById("tuning-panel");
const tuningHeader = document.getElementById("tuning-header");
const tuningReset = document.getElementById("tuning-reset");

tuningHeader.addEventListener("click", () => tuningPanel.classList.toggle("collapsed"));

// slider id -> [params object, key, defaults, decimals, storage key]
const sliders = [
  ["substeps", solverParams, "substeps", DEFAULT_SOLVER_PARAMS, 0],
  ["iterations", solverParams, "iterations", DEFAULT_SOLVER_PARAMS, 0],
  ["planarStiffness", solverParams, "planarStiffness", DEFAULT_SOLVER_PARAMS, 2],
  ["twistStiffness", solverParams, "twistStiffness", DEFAULT_SOLVER_PARAMS, 2],
  ["bendStiffness", solverParams, "bendStiffness", DEFAULT_SOLVER_PARAMS, 2],
  ["friction", solverParams, "friction", DEFAULT_SOLVER_PARAMS, 2],
  ["apexGrip", solverParams, "apexGrip", DEFAULT_SOLVER_PARAMS, 2],
  ["crossingGrip", solverParams, "crossingGrip", DEFAULT_SOLVER_PARAMS, 2],
  ["damping", solverParams, "damping", DEFAULT_SOLVER_PARAMS, 2],
  ["strip-width", stripParams, "width", DEFAULT_STRIP_PARAMS, 2],
  ["strip-thickness", stripParams, "thickness", DEFAULT_STRIP_PARAMS, 2],
];

function saveParams() {
  try {
    localStorage.setItem(SOLVER_STORAGE_KEY, JSON.stringify(solverParams));
    localStorage.setItem(STRIP_STORAGE_KEY, JSON.stringify({ width: stripParams.width, thickness: stripParams.thickness }));
  } catch {
    // ignore (private browsing, quota, etc.) - tuning still works this session
  }
}

function showSlider([id, params, key, , decimals]) {
  document.getElementById(id).value = params[key];
  document.querySelector(`.val[data-val="${id}"]`).textContent = params[key].toFixed(decimals);
}

for (const slider of sliders) {
  const [id, params, key] = slider;
  showSlider(slider);
  document.getElementById(id).addEventListener("input", (e) => {
    params[key] = parseFloat(e.target.value);
    showSlider(slider);
    saveParams();
  });
}

tuningReset.addEventListener("click", () => {
  for (const slider of sliders) {
    const [, params, key, defaults] = slider;
    params[key] = defaults[key];
    showSlider(slider);
  }
  saveParams();
});

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------

function animate() {
  requestAnimationFrame(animate);

  stepSolver(stripModel, stripState, solverParams, stripParams);
  if (adjustTails(stripModel, stripState, stripParams)) rebuildRibbonBuffers();

  updateRenderGeometry();
  controls.update();
  renderer.render(scene, camera);
}
animate();
