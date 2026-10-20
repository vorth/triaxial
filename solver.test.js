// solver.test.js
// Run with: node --test
//
// Covers the XPBD strip solver (solver.js): a flat seed stays at rest, a
// grown hexagonal mesh settles with constraints satisfied, over/under is
// never lost (including on a pentagon mesh, which rigid planarity can't
// fully satisfy), re-tracking follows a strip that slides, friction holds
// crossings that are pressed or clipped, tails grow and trim, and
// simulation survives edits.

import { test, describe } from "node:test";
import assert from "node:assert/strict";
import { WeaveGraph } from "./weaveGraph.js";
import { DEFAULT_STRIP_PARAMS, buildStripModel, crossingPosition, stripPointAt } from "./strips.js";
import { seedPlacementPositions } from "./seeding.js";
import { reconcileStripState } from "./stripState.js";
import { DEFAULT_SOLVER_PARAMS, stepSolver, adjustTails } from "./solver.js";

const params = DEFAULT_STRIP_PARAMS;
const ext = params.apexExtension;

// ---- shared helpers ---------------------------------------------------

function seedWithStrips() {
  const g = new WeaveGraph();
  const { hexVerts, apexVerts } = g.seedHexagon();
  const seeded = new Map();
  for (let i = 0; i < 6; i++) {
    const a = (Math.PI / 3) * i - Math.PI / 2;
    seeded.set(hexVerts[i], { x: Math.cos(a), y: Math.sin(a), z: 0 });
  }
  const apexR = Math.cos(Math.PI / 6) + Math.sqrt(3) / 2;
  for (let i = 0; i < 6; i++) {
    const a = (Math.PI / 3) * i - Math.PI / 2 + Math.PI / 6;
    seeded.set(apexVerts[i], { x: apexR * Math.cos(a), y: apexR * Math.sin(a), z: 0 });
  }
  const model = buildStripModel(g);
  return { g, w: { model, state: reconcileStripState(null, null, model, seeded, params) } };
}

function place(g, w, v, n) {
  const record = g.placePolygon(v, n);
  if (!record) return null;
  const chain = record.kind === "fuse" ? [record.u, ...record.siteVerts] : [record.u, ...record.siteVerts, record.w];
  const known = chain.map((id) => crossingPosition(w.model, w.state, id));
  const { mid, apex } = seedPlacementPositions(known, record.midVerts.length, record.apexVerts.length);
  const seeded = new Map();
  record.midVerts.forEach((id, i) => seeded.set(id, mid[i]));
  record.apexVerts.forEach((id, i) => seeded.set(id, apex[i]));
  const model = buildStripModel(g, w.model);
  return { model, state: reconcileStripState(w.model, w.state, model, seeded, params) };
}

function grow(g, w, n, times) {
  for (let i = 0; i < times; i++) {
    const c = g.concaveVertices();
    w = place(g, w, c[(i * 7) % c.length], n) || w;
  }
  return w;
}

function run(w, frames, solverParams = DEFAULT_SOLVER_PARAMS) {
  for (let f = 0; f < frames; f++) {
    stepSolver(w.model, w.state, solverParams, params);
    adjustTails(w.model, w.state, params);
  }
}

// Constraint residuals and over/under status.
function measure(w) {
  const { model, state } = w;
  const x = state.position;
  let stretch = 0;
  let planar = 0;
  let zMin = Infinity;
  let zMax = -Infinity;
  for (const st of state.strips) {
    const m = st.nodeCount;
    const segs = st.closed ? m : m - 1;
    for (let i = 0; i < segs; i++) {
      const a = 3 * (st.nodeOffset + i);
      const b = 3 * (st.nodeOffset + ((i + 1) % m));
      const d = [x[b] - x[a], x[b + 1] - x[a + 1], x[b + 2] - x[a + 2]];
      stretch = Math.max(stretch, Math.abs(Math.hypot(...d) - st.h) / st.h);
    }
    // in-plane bending: second difference along the node's width vector,
    // relative to h (an angle, roughly)
    const wd = state.width;
    for (let i = 1; i + 1 < m; i++) {
      const p = 3 * (st.nodeOffset + i - 1);
      const c = 3 * (st.nodeOffset + i);
      const q = 3 * (st.nodeOffset + i + 1);
      const sg = wd[p] * wd[c] + wd[p + 1] * wd[c + 1] + wd[p + 2] * wd[c + 2] < 0 ? -1 : 1;
      const b = [wd[c] + sg * wd[p], wd[c + 1] + sg * wd[p + 1], wd[c + 2] + sg * wd[p + 2]];
      const bl = Math.hypot(...b);
      const k = [0, 1, 2].map((a) => x[q + a] - 2 * x[c + a] + x[p + a]);
      planar = Math.max(planar, Math.abs(k[0] * b[0] + k[1] * b[1] + k[2] * b[2]) / bl / st.h);
    }
  }
  for (let i = 2; i < x.length; i += 3) {
    zMin = Math.min(zMin, x[i]);
    zMax = Math.max(zMax, x[i]);
  }
  let flipped = 0;
  for (const c of model.crossings.values()) {
    const [p, q] = c.passes.map((pass, k) => {
      const s = state.passS[2 * c.index + k];
      const a = stripPointAt(state, pass.strip, s - 0.05);
      const b = stripPointAt(state, pass.strip, s + 0.05);
      return { pt: stripPointAt(state, pass.strip, s), t: [b[0] - a[0], b[1] - a[1], b[2] - a[2]] };
    });
    const n = [
      p.t[1] * q.t[2] - p.t[2] * q.t[1],
      p.t[2] * q.t[0] - p.t[0] * q.t[2],
      p.t[0] * q.t[1] - p.t[1] * q.t[0],
    ];
    const sep = state.contactSign[c.index] * ((p.pt[0] - q.pt[0]) * n[0] + (p.pt[1] - q.pt[1]) * n[1] + (p.pt[2] - q.pt[2]) * n[2]);
    if (sep < 0) flipped++;
  }
  const spans = [];
  for (const s of model.strips) {
    const n = s.verts.length;
    for (let j = 0; j < (s.closed ? n : n - 1); j++) {
      const p = crossingPosition(model, state, s.verts[j]);
      const q = crossingPosition(model, state, s.verts[(j + 1) % n]);
      spans.push(Math.hypot(p.x - q.x, p.y - q.y, p.z - q.z));
    }
  }
  return {
    finite: state.position.every(Number.isFinite),
    stretch,
    planar,
    flipped,
    zRange: zMax - zMin,
    spanMin: Math.min(...spans),
    spanMax: Math.max(...spans),
  };
}

function passCoord(w, v, stripId) {
  const c = w.model.crossings.get(v);
  return w.state.passS[2 * c.index + c.passes.findIndex((p) => p.strip === stripId)];
}

function tails(w) {
  const out = [];
  w.model.strips.forEach((s, i) => {
    if (s.closed) return;
    const st = w.state.strips[i];
    out.push(passCoord(w, s.verts[0], i) - st.sStart);
    out.push(st.sStart + (st.nodeCount - 1) * st.h - passCoord(w, s.verts[s.verts.length - 1], i));
  });
  return out;
}

// ---- tests ------------------------------------------------------------

describe("solver", () => {
  test("the flat seed stays at rest", () => {
    const { w } = seedWithStrips();
    run(w, 200);
    const m = measure(w);
    assert.ok(m.finite);
    assert.ok(m.stretch < 1e-3, `stretch ${m.stretch}`);
    assert.ok(m.planar < 1e-3, `planar ${m.planar}`);
    assert.equal(m.flipped, 0);
    assert.ok(m.zRange < 0.05, `z range ${m.zRange}`);
    assert.ok(m.spanMin > 0.98 && m.spanMax < 1.02);
  });

  test("a grown hexagonal mesh settles with its constraints satisfied", () => {
    const { g, w: w0 } = seedWithStrips();
    const w = grow(g, w0, 6, 6);
    run(w, 300);
    const m = measure(w);
    assert.ok(m.finite);
    assert.ok(m.stretch < 0.01, `stretch ${m.stretch}`);
    assert.ok(m.planar < 0.01, `planar ${m.planar}`);
    assert.equal(m.flipped, 0);
    assert.ok(m.zRange < 0.5, `z range ${m.zRange}`);
    for (const t of tails(w)) assert.ok(t >= ext - 0.13 && t <= 2 * ext + 1e-9, `tail ${t}`);
  });

  test("over/under survives a pentagon (which rigid planarity can't fully satisfy)", () => {
    const { g, w: w0 } = seedWithStrips();
    let w = grow(g, w0, 6, 6);
    w = place(g, w, g.concaveVertices()[2], 5);
    run(w, 300);
    const m = measure(w);
    assert.ok(m.finite);
    assert.equal(m.flipped, 0);
  });

  test("with planarity off, a pentagon mesh settles", () => {
    const { g, w: w0 } = seedWithStrips();
    let w = grow(g, w0, 6, 6);
    w = place(g, w, g.concaveVertices()[2], 5);
    run(w, 300, { ...DEFAULT_SOLVER_PARAMS, off: ["planarity", "refit"] });
    const m = measure(w);
    assert.ok(m.stretch < 0.02, `stretch ${m.stretch}`);
    assert.equal(m.flipped, 0);
  });

  test("apex clips hold: a strip pulled along itself drags its apex crossing with it", () => {
    for (const [grip, expectSlide] of [
      [1, false],
      [0, true],
    ]) {
      const { w } = seedWithStrips();
      const strip = 0;
      const verts = w.model.strips[strip].verts;
      const apex = verts[0]; // open strips start at an apex
      const before = passCoord(w, apex, strip);
      // give strip 0 a velocity along itself (prev behind position)
      const st = w.state.strips[strip];
      const a = 3 * st.nodeOffset;
      const b = 3 * (st.nodeOffset + st.nodeCount - 1);
      const len = Math.hypot(w.state.position[b] - w.state.position[a], w.state.position[b + 1] - w.state.position[a + 1]);
      const T = [(w.state.position[b] - w.state.position[a]) / len, (w.state.position[b + 1] - w.state.position[a + 1]) / len, 0];
      for (let k = st.nodeOffset; k < st.nodeOffset + st.nodeCount; k++) {
        for (let i = 0; i < 3; i++) w.state.prev[3 * k + i] -= 0.01 * T[i];
      }
      const P = { ...DEFAULT_SOLVER_PARAMS, damping: 0, apexGrip: grip };
      stepSolver(w.model, w.state, P, params);
      const slid = Math.abs(passCoord(w, apex, strip) - before);
      if (expectSlide) assert.ok(slid > 0.005, `grip ${grip}: slid ${slid}`);
      else assert.ok(slid < 0.002, `grip ${grip}: slid ${slid}`);
    }
  });

  test("re-tracking follows a strip that slides", () => {
    const { w } = seedWithStrips();
    const strip = 0;
    const st = w.state.strips[strip];
    const ends = [st.nodeOffset, st.nodeOffset + st.nodeCount - 1].map((k) => [0, 1, 2].map((a) => w.state.position[3 * k + a]));
    const len = Math.hypot(...ends[1].map((v, a) => v - ends[0][a]));
    const T = ends[1].map((v, a) => (v - ends[0][a]) / len);
    const before = w.model.strips[strip].verts.map((v) => passCoord(w, v, strip));
    // slide the whole strip 0.2 along itself; material at a fixed point in
    // space is now 0.2 earlier in material coordinates
    for (let k = st.nodeOffset; k < st.nodeOffset + st.nodeCount; k++) {
      for (let a = 0; a < 3; a++) {
        w.state.position[3 * k + a] += 0.2 * T[a];
        w.state.prev[3 * k + a] += 0.2 * T[a];
      }
    }
    // only re-tracking runs, so nothing else moves
    const only = { ...DEFAULT_SOLVER_PARAMS, substeps: 4, off: ["stretch", "planarity", "bending", "contact", "friction", "gap", "refit"] };
    stepSolver(w.model, w.state, only, params);
    const after = w.model.strips[strip].verts.map((v) => passCoord(w, v, strip));
    before.forEach((s, i) => assert.ok(Math.abs(after[i] - (s - 0.2)) < 0.01, `crossing ${i}: ${s} -> ${after[i]}`));
  });

  test("tails grow when a crossing slides toward the end, keeping existing nodes", () => {
    const { w } = seedWithStrips();
    const strip = 0;
    const s0 = w.model.strips[strip].verts[0];
    const c = w.model.crossings.get(s0);
    const k = c.passes.findIndex((p) => p.strip === strip);
    // pretend the first crossing slid 0.25 toward the start
    w.state.passS[2 * c.index + k] -= 0.25;
    const st = w.state.strips[strip];
    const firstNode = Array.from(w.state.position.subarray(3 * st.nodeOffset, 3 * st.nodeOffset + 3));
    const sStartBefore = st.sStart;
    assert.ok(adjustTails(w.model, w.state, params));
    const st2 = w.state.strips[strip];
    // nodes added at the start end (the other end may adjust too)
    const grown = Math.round((sStartBefore - st2.sStart) / st2.h);
    assert.ok(grown > 0);
    assert.deepEqual(Array.from(w.state.position.subarray(3 * (st2.nodeOffset + grown), 3 * (st2.nodeOffset + grown) + 3)), firstNode);
    const tail0 = w.state.passS[2 * c.index + k] - st2.sStart;
    assert.ok(tail0 >= ext && tail0 < ext + st2.h + 1e-9, `tail ${tail0}`);
  });

  test("tails trim when they grow past twice the extension", () => {
    const { w } = seedWithStrips();
    const strip = 0;
    const s0 = w.model.strips[strip].verts[0];
    const c = w.model.crossings.get(s0);
    const k = c.passes.findIndex((p) => p.strip === strip);
    w.state.passS[2 * c.index + k] += 0.5; // crossing slid away from the start
    const countBefore = w.state.strips[strip].nodeCount;
    assert.ok(adjustTails(w.model, w.state, params));
    const st = w.state.strips[strip];
    assert.ok(st.nodeCount < countBefore);
    const tail0 = w.state.passS[2 * c.index + k] - st.sStart;
    assert.ok(tail0 >= ext && tail0 <= 2 * ext, `tail ${tail0}`);
  });

  test("simulation continues cleanly across edits", () => {
    const { g, w: w0 } = seedWithStrips();
    let w = grow(g, w0, 6, 3);
    for (let i = 0; i < 4; i++) {
      run(w, 40);
      const c = g.concaveVertices();
      w = place(g, w, c[(i * 5) % c.length], 6) || w;
    }
    run(w, 150);
    const m = measure(w);
    assert.ok(m.finite);
    assert.equal(m.flipped, 0);
    assert.ok(m.stretch < 0.02, `stretch ${m.stretch}`);
  });
});
