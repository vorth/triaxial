import * as THREE from "three";
import { TrackballControls } from "three/addons/controls/TrackballControls.js";
import { WeaveGraph } from "./weaveGraph.js";
import { Physics, DEFAULT_PARAMS } from "./physics.js";

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

let currentMode = { type: "place", n: 6 };

// ---------------------------------------------------------------------------
// Rendering: lines for edges, spheres for concave (placement-target) vertices
// ---------------------------------------------------------------------------

const edgeMaterial = new THREE.LineBasicMaterial({ color: 0x8fa3c8, linewidth: 1 });
let edgeLines = new THREE.LineSegments(new THREE.BufferGeometry(), edgeMaterial);
scene.add(edgeLines);

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
  const edges = graph.edges();
  const positions = new Float32Array(edges.length * 2 * 3);
  let i = 0;
  for (const [a, b] of edges) {
    const pa = physics.positions.get(a);
    const pb = physics.positions.get(b);
    if (!pa || !pb) continue;
    positions[i++] = pa.x;
    positions[i++] = pa.y;
    positions[i++] = pa.z;
    positions[i++] = pb.x;
    positions[i++] = pb.y;
    positions[i++] = pb.z;
  }
  edgeLines.geometry.dispose();
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(positions.subarray(0, i), 3));
  edgeLines.geometry = geo;

  for (const [v, mesh] of targetMeshes) {
    const p = physics.positions.get(v);
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

tuningReset.addEventListener("click", () => {
  for (const key of sliderIds) {
    physics[key] = DEFAULT_PARAMS[key];
    setSliderDisplay(key, DEFAULT_PARAMS[key]);
  }
  saveStoredParams();
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
