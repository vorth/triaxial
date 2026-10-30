// rapierWeave.test.js
// Run with: node --test
//
// Strip mechanics on Rapier (rapierWeave.js), on meshes grown from the seed
// hexagon: inextensible strips, pinned crossings with fixed over/under, and
// rebuilding the world across edits.

import { test, describe, before } from "node:test";
import assert from "node:assert/strict";
import { WeaveGraph } from "./weaveGraph.js";
import { DEFAULT_STRIP_PARAMS as params, buildStripModel, crossingPosition, stripPointAt } from "./strips.js";
import { seedPlacementPositions } from "./seeding.js";
import { reconcileStripState } from "./stripState.js";
import { DEFAULT_RAPIER_PARAMS as rapierParams, initRapier, createWeave, stepWeave, disposeWeave } from "./rapierWeave.js";

before(() => initRapier());

// ---- shared helpers ---------------------------------------------------

function seed() {
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
  const state = reconcileStripState(null, null, model, seeded, params);
  return { g, w: { model, state, weave: createWeave(model, state, rapierParams, params) } };
}

// Place, reconcile and rebuild the world, as main.js does.
function place(g, w, v, n) {
  const record = g.placePolygon(v, n);
  if (!record) return w;
  const chain = record.kind === "fuse" ? [record.u, ...record.siteVerts] : [record.u, ...record.siteVerts, record.w];
  const known = chain.map((id) => crossingPosition(w.model, w.state, id));
  const { mid, apex } = seedPlacementPositions(known, record.midVerts.length, record.apexVerts.length);
  const seeded = new Map();
  record.midVerts.forEach((id, i) => seeded.set(id, mid[i]));
  record.apexVerts.forEach((id, i) => seeded.set(id, apex[i]));
  const model = buildStripModel(g, w.model);
  const state = reconcileStripState(w.model, w.state, model, seeded, params);
  disposeWeave(w.weave);
  return { model, state, weave: createWeave(model, state, rapierParams, params) };
}

function session(g, w, polygons, frames) {
  polygons.forEach((n, i) => {
    const c = g.concaveVertices();
    w = place(g, w, c[(i * 7) % c.length], n);
    run(w, frames);
  });
  return w;
}

function run(w, frames) {
  for (let f = 0; f < frames; f++) stepWeave(w.weave, rapierParams);
}

function measure(w) {
  const { model, state } = w;
  const x = state.position;
  let stretch = 0;
  let finite = true;
  let zMin = Infinity;
  let zMax = -Infinity;
  for (const st of state.strips) {
    const m = st.nodeCount;
    const segs = st.closed ? m : m - 1;
    for (let i = 0; i < segs; i++) {
      const a = 3 * (st.nodeOffset + i);
      const b = 3 * (st.nodeOffset + ((i + 1) % m));
      stretch = Math.max(stretch, Math.abs(Math.hypot(x[b] - x[a], x[b + 1] - x[a + 1], x[b + 2] - x[a + 2]) - st.h) / st.h);
    }
    for (let k = 0; k < 3 * m; k++) finite &&= Number.isFinite(x[3 * st.nodeOffset + k]);
    for (let k = 0; k < m; k++) {
      zMin = Math.min(zMin, x[3 * (st.nodeOffset + k) + 2]);
      zMax = Math.max(zMax, x[3 * (st.nodeOffset + k) + 2]);
    }
  }
  const spans = [];
  for (const key of model.spanIndex.keys()) {
    const [a, b] = key.split("_").map(Number);
    const pa = crossingPosition(model, state, a);
    const pb = crossingPosition(model, state, b);
    spans.push(Math.hypot(pa.x - pb.x, pa.y - pb.y, pa.z - pb.z));
  }
  return { stretch, finite, zMin, zMax, spanMin: Math.min(...spans), spanMax: Math.max(...spans) };
}

// Crossings whose two passes have swapped sides relative to the over/under
// the reconciled state was built with (contactSign, about n = tA x tB).
function flippedCrossings(w) {
  const { model, state } = w;
  let flipped = 0;
  const tangent = (pass, s) => {
    const p0 = stripPointAt(state, pass.strip, s - 0.05);
    const p1 = stripPointAt(state, pass.strip, s + 0.05);
    return [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
  };
  for (const c of model.crossings.values()) {
    if (c.passes.length !== 2) continue;
    const [sA, sB] = [state.passS[2 * c.index], state.passS[2 * c.index + 1]];
    const pA = stripPointAt(state, c.passes[0].strip, sA);
    const pB = stripPointAt(state, c.passes[1].strip, sB);
    const tA = tangent(c.passes[0], sA);
    const tB = tangent(c.passes[1], sB);
    const n = [tA[1] * tB[2] - tA[2] * tB[1], tA[2] * tB[0] - tA[0] * tB[2], tA[0] * tB[1] - tA[1] * tB[0]];
    const d = (pA[0] - pB[0]) * n[0] + (pA[1] - pB[1]) * n[1] + (pA[2] - pB[2]) * n[2];
    if (Math.sign(d) !== state.contactSign[c.index]) flipped++;
  }
  return flipped;
}

// ---- tests ----------------------------------------------------------------

describe("rapier weave", () => {
  test("the flat seed stays at rest with unit spans", () => {
    const { w } = seed();
    run(w, 120);
    const m = measure(w);
    assert.ok(m.finite);
    assert.ok(m.stretch < 0.005, `stretch ${m.stretch}`);
    // over/under alone spans one thickness; allow as much again for curl
    assert.ok(m.zMax - m.zMin < 2 * params.thickness, `z extent ${m.zMax - m.zMin}`);
    assert.ok(m.spanMin > 0.99 && m.spanMax < 1.01, `spans ${m.spanMin}..${m.spanMax}`);
    assert.equal(flippedCrossings(w), 0);
    disposeWeave(w.weave);
  });

  test("a grown hexagonal mesh keeps strips inextensible and spans at unit length", () => {
    let { g, w } = seed();
    w = session(g, w, [6, 6, 6, 6, 6, 6], 60);
    run(w, 120);
    const m = measure(w);
    assert.ok(m.finite);
    assert.ok(m.stretch < 0.005, `stretch ${m.stretch}`);
    assert.ok(m.spanMin > 0.99 && m.spanMax < 1.01, `spans ${m.spanMin}..${m.spanMax}`);
    assert.equal(flippedCrossings(w), 0);
    disposeWeave(w.weave);
  });

  test("a pentagon session stays finite, inextensible and woven", () => {
    let { g, w } = seed();
    w = session(g, w, [6, 6, 6, 6, 6, 6, 5, 6, 6, 6], 60);
    run(w, 300);
    const m = measure(w);
    assert.ok(m.finite);
    assert.ok(m.stretch < 0.005, `stretch ${m.stretch}`);
    // spans are unit material between pinned crossings: the chord can only
    // shorten where a strip curls
    assert.ok(m.spanMax < 1.01, `longest span ${m.spanMax}`);
    assert.equal(flippedCrossings(w), 0);
    disposeWeave(w.weave);
  });

  test("new spans get unit material length whatever the seeded geometry", () => {
    let { g, w } = seed();
    w = session(g, w, [5], 0);
    const { model, state } = w;
    for (const s of model.strips) {
      const n = s.verts.length;
      const spans = s.closed ? n : n - 1;
      for (let i = 0; i < spans; i++) {
        const slot = (v, index) => {
          const c = model.crossings.get(v);
          return 2 * c.index + c.passes.findIndex((p) => p.strip === s.id && p.index === index);
        };
        let ds = state.passS[slot(s.verts[(i + 1) % n], (i + 1) % n)] - state.passS[slot(s.verts[i], i)];
        if (ds < 0) ds += state.strips[s.id].length;
        assert.ok(Math.abs(ds - params.edgeLength) < 1e-9, `strip ${s.id} span ${i}: ${ds}`);
      }
    }
    disposeWeave(w.weave);
  });

  test("rebuilding the world from its own state moves nothing", () => {
    let { g, w } = seed();
    w = session(g, w, [6, 6, 5], 60);
    const before = Float32Array.from(w.state.position);
    disposeWeave(w.weave);
    w.weave = createWeave(w.model, w.state, rapierParams, params);
    let moved = 0;
    for (let k = 0; k < before.length; k++) moved = Math.max(moved, Math.abs(w.state.position[k] - before[k]));
    assert.ok(moved < 0.01, `moved ${moved}`);
    disposeWeave(w.weave);
  });
});
