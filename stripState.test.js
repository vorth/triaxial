// stripState.test.js
// Run with: node --test
//
// Covers the solver-owned strip state (stripState.js): building it from
// scratch, and span-by-span reconciliation across placements and deletions -
// untouched strips carry over bit-for-bit (positions, prev, width vectors,
// crossing table), existing crossings stay put, place-then-delete restores
// the original crossings, tails stay within bounds, closed strips get a
// uniform grid, and random edit sequences never produce a broken state.

import { test, describe } from "node:test";
import assert from "node:assert/strict";
import { WeaveGraph } from "./weaveGraph.js";
import { DEFAULT_STRIP_PARAMS, buildStripModel, crossingPosition } from "./strips.js";
import { seedPlacementPositions } from "./seeding.js";
import { reconcileStripState } from "./stripState.js";

const params = DEFAULT_STRIP_PARAMS;
const ext = params.apexExtension;

// ---- shared helpers ---------------------------------------------------

// Seed hexagon with the strips built from scratch (the only time graph
// positions are used).
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
  const state = reconcileStripState(null, null, model, seeded, params);
  return { g, w: { model, state }, seeded };
}

// Place an n-gon the way the app will: seed fresh vertices from the strips'
// crossing positions, rebuild the model, reconcile the state.
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
  const state = reconcileStripState(w.model, w.state, model, seeded, params);
  return { model, state, record };
}

function remove(g, w, v) {
  if (!g.deleteAt(v)) return null;
  const model = buildStripModel(g, w.model);
  return { model, state: reconcileStripState(w.model, w.state, model, new Map(), params) };
}

function grow(g, w, n, times) {
  for (let i = 0; i < times; i++) {
    const c = g.concaveVertices();
    if (!c.length) break;
    const next = place(g, w, c[(i * 7) % c.length], n);
    if (next) w = next;
  }
  return w;
}

function stripSlice(state, i, array = state.position) {
  const st = state.strips[i];
  return Array.from(array.subarray(3 * st.nodeOffset, 3 * (st.nodeOffset + st.nodeCount)));
}

const sameKeys = (a, b) => a.length === b.length && a.every((k) => b.includes(k));

function dist(p, q) {
  return Math.hypot(p.x - q.x, p.y - q.y, p.z - q.z);
}

// Vertices with an incident edge that is a new span (not a span of the
// previous model): crossings where old and new material meet.
function editBoundary(oldModel, newModel) {
  const verts = new Set();
  for (const s of newModel.strips) {
    s.edgeKeys.forEach((k, i) => {
      if (oldModel.spanIndex.has(k)) return;
      verts.add(s.verts[i]);
      verts.add(s.verts[(i + 1) % s.verts.length]);
    });
  }
  return verts;
}

function checkStateSane(w) {
  const { model, state } = w;
  assert.ok(state.position.every(Number.isFinite));
  assert.ok(state.prev.every(Number.isFinite));
  state.strips.forEach((st, i) => {
    assert.ok(st.nodeCount >= 2);
    // unit width vectors, perpendicular to their segments
    for (let j = 0; j < st.nodeCount; j++) {
      const o = 3 * (st.nodeOffset + j);
      assert.ok(Math.abs(Math.hypot(state.width[o], state.width[o + 1], state.width[o + 2]) - 1) < 1e-5);
    }
    // material coordinates are the uniform grid sStart + k*h
    for (let k = 0; k < st.nodeCount; k++) {
      assert.ok(Math.abs(state.arc[st.nodeOffset + k] - (st.sStart + k * st.h)) < 1e-9);
    }
    // Segments: resampling a curve only shortens chords, and where old and
    // new material meet the gap is at most the over/under offset. Kinks
    // (sharpest after a fuse, which joins two vertices that were apart) can
    // make segments short; the solver restores them. Nodes never coincide.
    const pos = state.position;
    for (let k = 0; k + 1 < st.nodeCount; k++) {
      const o = 3 * (st.nodeOffset + k);
      const d = Math.hypot(pos[o + 3] - pos[o], pos[o + 4] - pos[o + 1], pos[o + 5] - pos[o + 2]);
      assert.ok(d <= st.h * 1.05 + params.thickness && d > 1e-6, `strip ${i} segment ${k}: ${d}`);
    }
    // every pass lies within its strip's material
    const first = state.arc[st.nodeOffset];
    const last = state.arc[st.nodeOffset + st.nodeCount - 1];
    for (const c of model.crossings.values()) {
      c.passes.forEach((pass, k) => {
        if (pass.strip !== i) return;
        const s = state.passS[2 * c.index + k];
        if (st.closed) assert.ok(s >= st.sStart - 1e-9 && s < st.sStart + st.length + 1e-9);
        else assert.ok(s > first && s < last);
      });
    }
  });
}

// ---- tests ------------------------------------------------------------

describe("building from scratch", () => {
  test("seed: crossings land on the seeded vertex positions", () => {
    const { g, w, seeded } = seedWithStrips();
    checkStateSane(w);
    for (const v of g.vertices.keys()) {
      assert.ok(dist(crossingPosition(w.model, w.state, v), seeded.get(v)) < 0.01, `vertex ${v}`);
    }
  });

  test("open strips get tails between apexExtension and twice that", () => {
    const { w } = seedWithStrips();
    for (const [i, s] of w.model.strips.entries()) {
      const st = w.state.strips[i];
      const c0 = w.model.crossings.get(s.verts[0]);
      const s0 = w.state.passS[2 * c0.index + c0.passes.findIndex((p) => p.strip === i)];
      const tail = s0 - st.sStart;
      assert.ok(tail >= ext - st.h - 1e-9 && tail <= 2 * ext + 1e-9, `tail ${tail}`);
    }
  });
});

describe("reconciliation", () => {
  test("strips untouched by a placement carry over bit-for-bit", () => {
    const { g, w: w0 } = seedWithStrips();
    let w = grow(g, w0, 6, 4);
    // give the old state some velocity, so prev is distinguishable
    for (let i = 0; i < w.state.prev.length; i++) w.state.prev[i] = w.state.position[i] - 0.01 * ((i % 7) - 3);
    const before = w;
    const after = place(g, w, g.concaveVertices()[0], 6);
    let untouched = 0;
    after.model.strips.forEach((s, i) => {
      const j = before.model.strips.findIndex((o) => sameKeys(o.edgeKeys, s.edgeKeys));
      if (j < 0) return;
      untouched++;
      assert.deepEqual(stripSlice(after.state, i), stripSlice(before.state, j));
      assert.deepEqual(stripSlice(after.state, i, after.state.prev), stripSlice(before.state, j, before.state.prev));
      assert.deepEqual(stripSlice(after.state, i, after.state.width), stripSlice(before.state, j, before.state.width));
      assert.equal(after.state.strips[i].sStart, before.state.strips[j].sStart);
      for (const v of s.verts) {
        const cA = after.model.crossings.get(v);
        const cB = before.model.crossings.get(v);
        const sA = after.state.passS[2 * cA.index + cA.passes.findIndex((p) => p.strip === i)];
        const sB = before.state.passS[2 * cB.index + cB.passes.findIndex((p) => p.strip === j)];
        assert.equal(sA, sB);
      }
    });
    assert.ok(untouched > 0);
    checkStateSane(after);
  });

  test("existing crossings stay put through placements of every kind", () => {
    const { g, w: w0 } = seedWithStrips();
    let w = grow(g, w0, 6, 5);
    for (const n of [6, 5, 4, 3, 7]) {
      const c = g.concaveVertices();
      const v = c[c.length >> 1];
      const site = g.nestingSiteAt(v);
      const k = site.siteVerts.length;
      for (const size of [n, k + 2, k + 1]) {
        // normal / joinEdge / fuse, whichever applies first
        const old = new Map([...w.model.crossings.keys()].map((id) => [id, crossingPosition(w.model, w.state, id)]));
        const next = place(g, w, v, Math.max(3, size));
        if (!next) continue;
        const boundary = editBoundary(w.model, next.model);
        for (const [id, p] of old) {
          if (!next.model.crossings.has(id)) continue; // fused away
          const d = dist(crossingPosition(next.model, next.state, id), p);
          if (boundary.has(id)) {
            // where old material meets new, the resampled crossing moves by
            // at most a small kink's worth (not for the fused vertex itself,
            // which the merge genuinely moves)
            if (!(next.record.kind === "fuse" && id === next.record.u)) assert.ok(d < 0.05, `${next.record.kind}: edge vertex ${id}: ${d}`);
          } else {
            // away from the edit only resampling (when merged strips need a
            // new node grid) can move a crossing, and only slightly
            assert.ok(d < 2e-3, `${next.record.kind}: vertex ${id}: ${d}`);
          }
        }
        checkStateSane(next);
        w = next;
        break;
      }
    }
  });

  test("place then delete restores the original crossings", () => {
    const { g, w: w0 } = seedWithStrips();
    const w = grow(g, w0, 6, 5);
    const keysBefore = w.model.strips.map((s) => s.edgeKeys.slice().sort().join("|")).sort();
    const posBefore = new Map([...g.vertices.keys()].map((v) => [v, crossingPosition(w.model, w.state, v)]));
    const v = g.concaveVertices()[0];
    const placed = place(g, w, v, 6);
    const deleted = remove(g, placed, placed.record.midVerts[0]);
    assert.ok(deleted, "deletion succeeded");
    const keysAfter = deleted.model.strips.map((s) => s.edgeKeys.slice().sort().join("|")).sort();
    assert.deepEqual(keysAfter, keysBefore);
    const boundary = editBoundary(placed.model, deleted.model);
    for (const [id, p] of posBefore) {
      const d = dist(crossingPosition(deleted.model, deleted.state, id), p);
      assert.ok(d < (boundary.has(id) || placed.record.u === id || placed.record.w === id ? 0.05 : 2e-3), `vertex ${id}: ${d}`);
    }
    checkStateSane(deleted);
  });

  test("a fully closed mesh gets closed strips on a uniform grid", () => {
    const { g, w: w0 } = seedWithStrips();
    let w = w0;
    for (const n of [6, 5, 4, 3, 7]) w = grow(g, w, n, [8, 6, 5, 4, 3][[6, 5, 4, 3, 7].indexOf(n)]);
    for (let it = 0; it < 500; it++) {
      const c = g.concaveVertices();
      if (!c.length) break;
      const k = g.nestingSiteAt(c[0]).siteVerts.length;
      w = place(g, w, c[0], Math.max(3, k + 1)) || place(g, w, c[0], k + 2) || w;
    }
    assert.ok(w.model.strips.every((s) => s.closed));
    for (const st of w.state.strips) assert.ok(Math.abs(st.length - st.nodeCount * st.h) < 1e-9);
    checkStateSane(w);
  });

  test("random edit sequences always yield a sane state", () => {
    for (let trial = 0; trial < 40; trial++) {
      let seed = trial + 1;
      const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
      const { g, w: w0 } = seedWithStrips();
      let w = w0;
      for (let op = 0; op < 25; op++) {
        try {
          if (rnd() < 0.75) {
            const c = g.concaveVertices();
            if (!c.length) break;
            const v = c[Math.floor(rnd() * c.length)];
            // known weaveGraph gap (see CLAUDE.md): with one apex left, both
            // flanks of the site are the same vertex and placement builds
            // degenerate faces
            const site = g.nestingSiteAt(v);
            if (site.apexLeft === site.apexRight) break;
            w = place(g, w, v, 3 + Math.floor(rnd() * 5)) || w;
          } else {
            w = remove(g, w, g.boundary[Math.floor(rnd() * g.boundary.length)]) || w;
          }
        } catch {
          // known weaveGraph fuse bugs (see CLAUDE.md) can corrupt the
          // graph mid-placement; this test is about the strip state
          break;
        }
        if ([...w.model.crossings.values()].some((c) => c.kind === "irregular")) break;
        checkStateSane(w);
      }
    }
  });
});
