import * as THREE from "three";
import { TrackballControls } from "three/addons/controls/TrackballControls.js";
import { WeaveGraph } from "./weaveGraph.js";
import { Physics, DEFAULT_PARAMS } from "./physics.js";
import {
  DEFAULT_STRIP_PARAMS,
  buildStripModel,
  createStripState,
  deriveStripState,
  createRibbonLayout,
  writeRibbonMesh,
} from "./strips.js";

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
// Graph + physics state
// ---------------------------------------------------------------------------

const graph = new WeaveGraph();
const physics = new Physics();

function initialLayout(hexVerts, apexVerts) {
  // Face-on regular hexagon in the XY plane, apexes pushed outward.
  const hexR = 1 / (2 * Math.sin(Math.PI / 6)); // = 1 for hexagon (edge length 1)
  for (let i = 0; i < 6; i++) {
    const a = (Math.PI / 3) * i - Math.PI / 2;
    physics.setPosition(hexVerts[i], hexR * Math.cos(a), hexR * Math.sin(a), 0);
  }
  // apex_i sits between hexVert_i and hexVert_{i+1}, pushed further out and
  // nudged slightly in z (small independent random sign per apex) so the
  // dihedral-angle relaxation has a direction to fold into (an exactly flat
  // start is a symmetric saddle it can't break out of on its own). Random
  // sign, not a uniform offset, so triangles fold outward independently
  // instead of the whole ring curling coherently toward/away from camera.
  // apothem (center-to-edge-midpoint distance) of the unit-edge hexagon,
  // plus the height of a unit-edge equilateral triangle, gives the radius
  // that makes every boundary triangle equilateral (all 3 edges unit
  // length) rather than isosceles.
  const apexR = hexR * Math.cos(Math.PI / 6) + Math.sqrt(3) / 2;
  for (let i = 0; i < 6; i++) {
    const a = (Math.PI / 3) * i - Math.PI / 2 + Math.PI / 6;
    const jitterZ = (Math.random() < 0.5 ? -1 : 1) * 0.05;
    physics.setPosition(apexVerts[i], apexR * Math.cos(a), apexR * Math.sin(a), jitterZ);
  }
}

function seed() {
  const { hexVerts, apexVerts } = graph.seedHexagon();
  initialLayout(hexVerts, apexVerts);
}
seed();

// ---------------------------------------------------------------------------
// Woven strips, derived from the graph (see strips.js). The model is rebuilt
// only on topology changes; its node state is re-derived from the physics
// positions every frame, and the ribbon mesh is written from that state.
// ---------------------------------------------------------------------------

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

function rebuildStrips() {
  stripModel = buildStripModel(graph, stripModel, stripParams.samplesPerEdge);
  stripState = createStripState(stripModel);
  ribbonLayout = createRibbonLayout(stripModel);
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
rebuildStrips();

function updateStrips() {
  deriveStripState(stripModel, physics.positions, stripParams, stripState);
  const geo = ribbonMesh.geometry;
  writeRibbonMesh(stripModel, ribbonLayout, stripState, stripParams, geo.attributes.position.array, geo.attributes.normal.array);
  geo.attributes.position.needsUpdate = true;
  geo.attributes.normal.needsUpdate = true;
}

let currentMode = { type: "place", n: 6 };

// ---------------------------------------------------------------------------
// Rendering: spheres for concave (placement-target) vertices; the graph
// itself is shown only through the woven strips
// ---------------------------------------------------------------------------

const targetGeometry = new THREE.SphereGeometry(0.09, 16, 12);
const targetMaterial = new THREE.MeshStandardMaterial({ color: 0x4c8dff, emissive: 0x1a3a6b, roughness: 0.4 });
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

function updateRenderGeometry() {
  for (const [v, mesh] of targetMeshes) {
    const p = physics.positions.get(v);
    if (p) mesh.position.set(p.x, p.y, p.z);
  }

  updateStrips();
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
flipBtn.addEventListener("click", () => {
  stripModel.sigma = -stripModel.sigma;
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
    physics.seedPlacement(existingChain, record.midVerts, record.apexVerts);
  } else if (currentMode.type === "delete") {
    graph.deleteAt(vertexId);
    physics.pruneTo(new Set(graph.vertices.keys()));
  }
  rebuildTargets();
  rebuildStrips();
}

renderer.domElement.addEventListener("pointerdown", onPointerDown);
renderer.domElement.addEventListener("pointerup", onPointerUp);

// ---------------------------------------------------------------------------
// Physics tuning panel
// ---------------------------------------------------------------------------

const TUNING_STORAGE_KEY = "weaveGraph.physicsParams";

function loadStoredParams() {
  try {
    const raw = localStorage.getItem(TUNING_STORAGE_KEY);
    if (!raw) return {};
    const parsed = JSON.parse(raw);
    return parsed && typeof parsed === "object" ? parsed : {};
  } catch {
    return {};
  }
}

function saveStoredParams() {
  try {
    const toStore = {};
    for (const key of sliderIds) toStore[key] = physics[key];
    localStorage.setItem(TUNING_STORAGE_KEY, JSON.stringify(toStore));
  } catch {
    // ignore (private browsing, quota, etc.) - tuning still works this session
  }
}

const tuningPanel = document.getElementById("tuning-panel");
const tuningHeader = document.getElementById("tuning-header");
const tuningReset = document.getElementById("tuning-reset");

tuningHeader.addEventListener("click", () => tuningPanel.classList.toggle("collapsed"));

const sliderIds = Object.keys(DEFAULT_PARAMS);
const stored = loadStoredParams();
for (const key of sliderIds) {
  if (typeof stored[key] === "number" && Number.isFinite(stored[key])) {
    physics[key] = stored[key];
  }
}

function setSliderDisplay(key, value) {
  const input = document.getElementById(key);
  const valLabel = document.querySelector(`.val[data-val="${key}"]`);
  input.value = value;
  // damping's range is much finer (0-0.1) than the other 0-1ish sliders, so
  // it needs an extra decimal of display precision or small adjustments all
  // show as "0.02" -> "0.02" with no visible change.
  const decimals = key === "damping" ? 3 : 2;
  valLabel.textContent = value.toFixed(decimals);
}

for (const key of sliderIds) {
  setSliderDisplay(key, physics[key]);
  const input = document.getElementById(key);
  input.addEventListener("input", () => {
    const v = parseFloat(input.value);
    physics[key] = v;
    setSliderDisplay(key, v);
    saveStoredParams();
  });
}

// strip geometry sliders (render-only for now; not physics params)
const stripSliderIds = ["width", "thickness"];
function setStripSliderDisplay(key) {
  document.getElementById("strip-" + key).value = stripParams[key];
  document.querySelector(`.val[data-val="strip-${key}"]`).textContent = stripParams[key].toFixed(2);
}
function saveStripParams() {
  try {
    localStorage.setItem(STRIP_STORAGE_KEY, JSON.stringify({ width: stripParams.width, thickness: stripParams.thickness }));
  } catch {
    // ignore
  }
}
for (const key of stripSliderIds) {
  setStripSliderDisplay(key);
  document.getElementById("strip-" + key).addEventListener("input", (e) => {
    stripParams[key] = parseFloat(e.target.value);
    setStripSliderDisplay(key);
    saveStripParams();
  });
}

tuningReset.addEventListener("click", () => {
  for (const key of sliderIds) {
    physics[key] = DEFAULT_PARAMS[key];
    setSliderDisplay(key, DEFAULT_PARAMS[key]);
  }
  saveStoredParams();
  for (const key of stripSliderIds) {
    stripParams[key] = DEFAULT_STRIP_PARAMS[key];
    setStripSliderDisplay(key);
  }
  saveStripParams();
});

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------

function animate() {
  requestAnimationFrame(animate);

  const edges = graph.edges().map(([a, b]) => [a, b]);
  const faces = graph.allFaces();
  const edgeFaces = graph.edgesWithAdjacentFaces();
  const degree4Vertices = graph.degree4VertexNeighbors();
  physics.step(edges, faces, edgeFaces, degree4Vertices);

  updateRenderGeometry();
  controls.update();
  renderer.render(scene, camera);
}
animate();
