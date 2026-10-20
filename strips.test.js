// strips.test.js
// Run with: node --test
//
// Covers the strip model derived from a WeaveGraph (strips.js): tracing
// (edge partition, open/closed, apex endpoints), the over/under rule
// (exactly one positive pass per crossing, alternation along every strip,
// global sigma flip), stable color inheritance across edits, and the
// derived node state / ribbon mesh buffers.

import { test, describe } from "node:test";
import assert from "node:assert/strict";
import { WeaveGraph } from "./weaveGraph.js";
import { Physics } from "./physics.js";
import {
  DEFAULT_STRIP_PARAMS,
  buildStripModel,
  createStripState,
  deriveStripState,
  crossingNormals,
  passSide,
  createRibbonLayout,
  writeRibbonMesh,
  crossingPosition,
  stripPointAt,
  stripLength,
} from "./strips.js";
import { seedPlacementPositions } from "./seeding.js";

// ---- shared helpers ---------------------------------------------------

function seedFlat() {
  const g = new WeaveGraph();
  const p = new Physics();
  const { hexVerts, apexVerts } = g.seedHexagon();
  for (let i = 0; i < 6; i++) {
    const a = (Math.PI / 3) * i - Math.PI / 2;
    p.setPosition(hexVerts[i], Math.cos(a), Math.sin(a), 0);
  }
  const apexR = Math.cos(Math.PI / 6) + Math.sqrt(3) / 2;
  for (let i = 0; i < 6; i++) {
    const a = (Math.PI / 3) * i - Math.PI / 2 + Math.PI / 6;
    p.setPosition(apexVerts[i], apexR * Math.cos(a), apexR * Math.sin(a), 0);
  }
  return { g, p };
}

function place(g, p, v, n) {
  const record = g.placePolygon(v, n);
  if (!record) return null;
  const existingChain =
    record.kind === "fuse" ? [record.u, ...record.siteVerts] : [record.u, ...record.siteVerts, record.w];
  p.seedPlacement(existingChain, record.midVerts, record.apexVerts);
  return record;
}

function grow(g, p, n, times) {
  for (let i = 0; i < times; i++) {
    const c = g.concaveVertices();
    if (!c.length) return;
    place(g, p, c[(i * 7) % c.length], n);
  }
}

function closeGreedily(g, p) {
  for (let it = 0; it < 500; it++) {
    const c = g.concaveVertices();
    if (!c.length) return;
    const k = g.nestingSiteAt(c[0]).siteVerts.length;
    place(g, p, c[0], Math.max(3, k + 1)) || place(g, p, c[0], k + 2);
  }
}

// Every edge must border at most one weave triangle (a triangle-triangle
// edge means the graph itself is malformed, not the strips).
function assertWellFormed(g) {
  const count = new Map();
  for (const t of g.triangles.values()) {
    for (let i = 0; i < 3; i++) {
      const a = t.verts[i];
      const b = t.verts[(i + 1) % 3];
      const k = a < b ? a + "_" + b : b + "_" + a;
      count.set(k, (count.get(k) || 0) + 1);
    }
  }
  for (const [k, c] of count) assert.ok(c === 1, `edge ${k} borders ${c} weave triangles`);
}

function apexCount(g) {
  return [...g.vertices.keys()].filter((v) => g.faceCount(v) === 1).length;
}

function checkTopology(g, model) {
  // every graph edge in exactly one strip
  const graphEdges = new Set(g.edges().map(([a, b]) => (a < b ? a + "_" + b : b + "_" + a)));
  const stripEdges = model.strips.flatMap((s) => s.edgeKeys);
  assert.equal(stripEdges.length, graphEdges.size);
  assert.deepEqual(new Set(stripEdges), graphEdges);
  // open strips end at apexes, and there is one open strip per apex
  const open = model.strips.filter((s) => !s.closed);
  for (const s of open) {
    assert.equal(g.faceCount(s.verts[0]), 1);
    assert.equal(g.faceCount(s.verts[s.verts.length - 1]), 1);
  }
  assert.equal(open.length, apexCount(g));
  // every vertex is a crossing with exactly two passes, one Tait-positive
  assert.equal(model.crossings.size, g.vertices.size);
  for (const c of model.crossings.values()) {
    assert.equal(c.passes.length, 2, `vertex ${c.v}`);
    assert.deepEqual(c.passes.map((p) => p.tait).sort(), [-1, 1]);
  }
}

// Alternation, read off the model: consecutive crossings along a strip put
// it on opposite sides, measured geometrically (displacement directions).
function checkAlternation(model, positions) {
  const normals = crossingNormals(model, positions);
  const disp = (s, i) => {
    const pass = model.crossings.get(s.verts[i]).passes.find((p) => p.strip === s.id && p.index === i);
    const N = normals.get(s.verts[i]);
    return N.map((x) => x * passSide(model, pass));
  };
  let count = 0;
  for (const s of model.strips) {
    const n = s.verts.length;
    const spans = s.closed ? n : n - 1;
    for (let i = 0; i < spans; i++) {
      const a = disp(s, i);
      const b = disp(s, (i + 1) % n);
      assert.ok(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] < 0, `strip ${s.id} span ${i}`);
      count++;
    }
  }
  return count;
}

// ---- tests ------------------------------------------------------------

describe("strip tracing", () => {
  test("seed hexagon: 6 open strips of 3 spans each", () => {
    const { g } = seedFlat();
    const model = buildStripModel(g);
    checkTopology(g, model);
    assert.equal(model.strips.length, 6);
    for (const s of model.strips) {
      assert.equal(s.closed, false);
      assert.equal(s.verts.length, 4);
    }
  });

  test("grown meshes with mixed polygons partition edges into strips", () => {
    const { g, p } = seedFlat();
    grow(g, p, 6, 8);
    checkTopology(g, buildStripModel(g));
    grow(g, p, 5, 6);
    checkTopology(g, buildStripModel(g));
    grow(g, p, 4, 5);
    grow(g, p, 3, 4);
    grow(g, p, 7, 3);
    checkTopology(g, buildStripModel(g));
  });

  test("a fully closed mesh has only closed strips", () => {
    const { g, p } = seedFlat();
    grow(g, p, 6, 8);
    grow(g, p, 5, 6);
    grow(g, p, 4, 5);
    grow(g, p, 3, 4);
    grow(g, p, 7, 3);
    closeGreedily(g, p);
    assertWellFormed(g);
    assert.equal(apexCount(g), 0);
    const model = buildStripModel(g);
    checkTopology(g, model);
    assert.ok(model.strips.length > 0);
    for (const s of model.strips) assert.equal(s.closed, true);
  });
});

describe("over/under", () => {
  test("alternates along every strip on the flat seed", () => {
    const { g, p } = seedFlat();
    const model = buildStripModel(g);
    assert.equal(checkAlternation(model, p.positions), 18);
  });

  test("alternates along every strip on a grown hexagonal mesh", () => {
    const { g, p } = seedFlat();
    grow(g, p, 6, 10);
    const model = buildStripModel(g);
    assert.equal(checkAlternation(model, p.positions), g.edges().length);
  });

  test("flipping sigma puts every pass on the other side", () => {
    const { g } = seedFlat();
    const model = buildStripModel(g);
    const before = [...model.crossings.values()].flatMap((c) => c.passes.map((p) => passSide(model, p)));
    model.sigma = -model.sigma;
    const after = [...model.crossings.values()].flatMap((c) => c.passes.map((p) => passSide(model, p)));
    assert.deepEqual(
      after,
      before.map((s) => -s),
    );
  });

  test("sigma survives a rebuild", () => {
    const { g, p } = seedFlat();
    const model = buildStripModel(g);
    model.sigma = -1;
    place(g, p, g.concaveVertices()[0], 6);
    assert.equal(buildStripModel(g, model).sigma, -1);
  });
});

describe("colors", () => {
  test("strips get distinct colors", () => {
    const { g, p } = seedFlat();
    grow(g, p, 6, 6);
    const model = buildStripModel(g);
    assert.equal(new Set(model.strips.map((s) => s.color)).size, model.strips.length);
  });

  test("strips untouched by a placement keep their colors", () => {
    const { g, p } = seedFlat();
    grow(g, p, 6, 3);
    const before = buildStripModel(g);
    const colorOf = new Map(before.strips.map((s) => [s.edgeKeys.slice().sort().join("|"), s.color]));
    place(g, p, g.concaveVertices()[0], 6);
    const after = buildStripModel(g, before);
    let unchanged = 0;
    for (const s of after.strips) {
      const key = s.edgeKeys.slice().sort().join("|");
      if (colorOf.has(key)) {
        assert.equal(s.color, colorOf.get(key));
        unchanged++;
      }
    }
    assert.ok(unchanged > 0);
    // every strip that existed before and only grew keeps its color too
    for (const s of after.strips) {
      const old = before.strips.filter((o) => o.edgeKeys.every((k) => s.edgeKeys.includes(k)));
      if (old.length === 1) assert.equal(s.color, old[0].color);
    }
  });
});

describe("derived state and ribbon mesh", () => {
  function build(g, p) {
    const model = buildStripModel(g);
    const state = deriveStripState(model, p.positions, DEFAULT_STRIP_PARAMS, createStripState(model));
    return { model, state };
  }

  test("crossing nodes sit thickness/2 off the graph vertex, on opposite sides", () => {
    const { g, p } = seedFlat();
    grow(g, p, 6, 4);
    const { model, state } = build(g, p);
    for (const c of model.crossings.values()) {
      const pv = p.positions.get(c.v);
      const offs = c.passes.map((pass) => {
        const s = model.strips[pass.strip];
        const o = 3 * (s.nodeOffset + pass.node);
        return [state.position[o] - pv.x, state.position[o + 1] - pv.y, state.position[o + 2] - pv.z];
      });
      for (const d of offs) assert.ok(Math.abs(Math.hypot(...d) - DEFAULT_STRIP_PARAMS.thickness / 2) < 1e-5);
      const dotp = offs[0][0] * offs[1][0] + offs[0][1] * offs[1][1] + offs[0][2] * offs[1][2];
      assert.ok(dotp < 0);
    }
  });

  test("open strips extend apexExtension past their end apexes", () => {
    const { g, p } = seedFlat();
    const { model, state } = build(g, p);
    for (const s of model.strips) {
      const o0 = 3 * s.nodeOffset;
      const o1 = 3 * (s.nodeOffset + 1);
      const d = Math.hypot(
        state.position[o0] - state.position[o1],
        state.position[o0 + 1] - state.position[o1 + 1],
        state.position[o0 + 2] - state.position[o1 + 2],
      );
      assert.ok(Math.abs(d - DEFAULT_STRIP_PARAMS.apexExtension) < 1e-5);
    }
  });

  test("directors are unit length and continuous along each strip", () => {
    const { g, p } = seedFlat();
    grow(g, p, 6, 6);
    grow(g, p, 5, 3);
    const { model, state } = build(g, p);
    const d = state.director;
    for (const s of model.strips) {
      for (let k = 0; k < s.nodeCount; k++) {
        const o = 3 * (s.nodeOffset + k);
        assert.ok(Math.abs(Math.hypot(d[o], d[o + 1], d[o + 2]) - 1) < 1e-4);
        if (k > 0) assert.ok(d[o] * d[o - 3] + d[o + 1] * d[o - 2] + d[o + 2] * d[o - 1] > 0);
      }
    }
  });

  test("ribbon buffers are fully written and finite, including closed strips", () => {
    const { g, p } = seedFlat();
    grow(g, p, 6, 8);
    grow(g, p, 5, 6);
    grow(g, p, 4, 5);
    grow(g, p, 3, 4);
    grow(g, p, 7, 3);
    closeGreedily(g, p);
    assertWellFormed(g);
    const { model, state } = build(g, p);
    const layout = createRibbonLayout(model, state);
    const position = new Float32Array(3 * layout.vertexCount).fill(NaN);
    const normal = new Float32Array(3 * layout.vertexCount).fill(NaN);
    writeRibbonMesh(model, layout, state, DEFAULT_STRIP_PARAMS, position, normal);
    assert.ok(position.every(Number.isFinite));
    assert.ok(normal.every(Number.isFinite));
    assert.ok(layout.index.every((i) => i < layout.vertexCount));
  });
});

describe("crossing table", () => {
  function build(g, p) {
    const model = buildStripModel(g);
    const state = deriveStripState(model, p.positions, DEFAULT_STRIP_PARAMS, createStripState(model));
    return { model, state };
  }

  function grownAndClosed() {
    const { g, p } = seedFlat();
    grow(g, p, 6, 8);
    grow(g, p, 5, 6);
    grow(g, p, 4, 5);
    grow(g, p, 3, 4);
    grow(g, p, 7, 3);
    closeGreedily(g, p);
    assertWellFormed(g);
    return { g, p };
  }

  test("every crossing has a stable index and kind", () => {
    const { g, p } = seedFlat();
    grow(g, p, 6, 5);
    const { model } = build(g, p);
    const indices = [...model.crossings.values()].map((c) => c.index).sort((a, b) => a - b);
    assert.deepEqual(
      indices,
      indices.map((_, i) => i),
    );
    for (const c of model.crossings.values()) {
      const fc = g.faceCount(c.v);
      assert.equal(c.kind, fc === 4 ? "interior" : fc === 1 ? "apex" : "nesting");
    }
  });

  test("crossingPosition locates every graph vertex (open and closed strips)", () => {
    for (const { g, p } of [
      (() => {
        const m = seedFlat();
        grow(m.g, m.p, 6, 8);
        grow(m.g, m.p, 5, 4);
        return m;
      })(),
      grownAndClosed(),
    ]) {
      const { model, state } = build(g, p);
      for (const v of g.vertices.keys()) {
        const q = crossingPosition(model, state, v);
        const pv = p.positions.get(v);
        assert.ok(Math.hypot(q.x - pv.x, q.y - pv.y, q.z - pv.z) < 1e-5, `vertex ${v}`);
      }
    }
  });

  test("material coordinates: monotone along strips, passes within strip length", () => {
    const { g, p } = grownAndClosed();
    const { model, state } = build(g, p);
    for (const s of model.strips) {
      for (let k = 1; k < s.nodeCount; k++) {
        assert.ok(state.arc[s.nodeOffset + k] > state.arc[s.nodeOffset + k - 1]);
      }
    }
    for (const c of model.crossings.values()) {
      c.passes.forEach((pass, k) => {
        const sv = state.passS[2 * c.index + k];
        assert.ok(sv >= 0 && sv <= stripLength(state, pass.strip));
      });
    }
  });

  test("stripPointAt hits nodes exactly and wraps on closed strips", () => {
    const { g, p } = grownAndClosed();
    const { model, state } = build(g, p);
    const s = model.strips[0];
    assert.ok(s.closed);
    const len = stripLength(state, s.id);
    for (const k of [0, 3, s.nodeCount - 1]) {
      const sv = state.arc[s.nodeOffset + k];
      const o = 3 * (s.nodeOffset + k);
      for (const shift of [0, len, -len]) {
        const q = stripPointAt(state, s.id, sv + shift);
        assert.ok(Math.hypot(q[0] - state.position[o], q[1] - state.position[o + 1], q[2] - state.position[o + 2]) < 1e-4);
      }
    }
  });

  test("seeding from crossing positions matches seeding from graph positions", () => {
    const { g, p } = seedFlat();
    grow(g, p, 6, 6);
    const { model, state } = build(g, p);
    for (const v of g.concaveVertices().slice(0, 6)) {
      const { apexLeft, siteVerts, apexRight } = g.nestingSiteAt(v);
      const chain = [apexLeft, ...siteVerts, apexRight];
      const fromGraph = seedPlacementPositions(
        chain.map((id) => p.positions.get(id)),
        2,
        3,
      );
      const fromStrips = seedPlacementPositions(
        chain.map((id) => crossingPosition(model, state, id)),
        2,
        3,
      );
      [...fromGraph.mid, ...fromGraph.apex].forEach((a, i) => {
        const b = [...fromStrips.mid, ...fromStrips.apex][i];
        assert.ok(Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z) < 1e-4);
      });
    }
  });
});
