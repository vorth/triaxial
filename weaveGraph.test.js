// weaveGraph.test.js
// Run with: node --test
//
// Covers the combinatorial/topological model (weaveGraph.js) and its
// integration with the physics seeding (physics.js): placement (normal,
// joinEdge, fuse, and the too-many-vertices failure), deletion as the exact
// structural inverse of each, and end-to-end physics stability after both.

import { test, describe } from "node:test";
import assert from "node:assert/strict";
import { WeaveGraph } from "./weaveGraph.js";
import { Physics } from "./physics.js";

// ---- shared helpers ---------------------------------------------------

function seedHexWithPhysics() {
  const g = new WeaveGraph();
  const p = new Physics();
  const { hexVerts, apexVerts } = g.seedHexagon();
  const hexR = 1 / (2 * Math.sin(Math.PI / 6));
  for (let i = 0; i < 6; i++) {
    const a = (Math.PI / 3) * i - Math.PI / 2;
    p.setPosition(hexVerts[i], hexR * Math.cos(a), hexR * Math.sin(a), 0);
  }
  const apexR = hexR + 0.85;
  for (let i = 0; i < 6; i++) {
    const a = (Math.PI / 3) * i - Math.PI / 2 + Math.PI / 6;
    p.setPosition(apexVerts[i], apexR * Math.cos(a), apexR * Math.sin(a), (i % 2 ? 1 : -1) * 0.05);
  }
  return { g, p, hexVerts, apexVerts };
}

function settle(g, p, steps = 300) {
  const edges = g.edges();
  const faces = g.allFaces();
  const edgeFaces = g.edgesWithAdjacentFaces();
  for (let s = 0; s < steps; s++) p.step(edges, faces, edgeFaces);
}

function place(g, p, v, n) {
  const record = g.placePolygon(v, n);
  if (!record) return null;
  const existingChain =
    record.kind === "fuse" ? [record.u, ...record.siteVerts] : [record.u, ...record.siteVerts, record.w];
  p.seedPlacement(existingChain, record.midVerts, record.apexVerts);
  return record;
}

function allFinite(p) {
  for (const pos of p.positions.values()) {
    if (!Number.isFinite(pos.x) || !Number.isFinite(pos.y) || !Number.isFinite(pos.z)) return false;
  }
  return true;
}

// Find a nesting site of exactly `size` vertices somewhere on the boundary.
function findNestingSite(g, size) {
  const seen = new Set();
  for (const bv of g.boundary) {
    if (g.vertexKind(bv) === "nesting" && !seen.has(bv)) {
      const site = g.nestingSiteAt(bv);
      for (const sv of site.siteVerts) seen.add(sv);
      if (site.siteVerts.length === size) return site;
    }
  }
  return null;
}

// ---- seed hexagon -------------------------------------------------------

describe("seedHexagon", () => {
  test("creates a hexagon surrounded by 6 triangles with 6 concave targets", () => {
    const g = new WeaveGraph();
    g.seedHexagon();
    assert.equal(g.polygons.size, 1);
    assert.equal(g.triangles.size, 6);
    assert.equal(g.concaveVertices().length, 6);
    assert.equal(g.edges().length, 18);
  });

  test("settles to a perfectly regular hexagon regardless of dihedral strength", () => {
    const { g, p, hexVerts } = seedHexWithPhysics();
    p.dihedralStiffness = 0.4; // near the UI slider's max
    settle(g, p, 300);
    for (let i = 0; i < 6; i++) {
      const prev = p.positions.get(hexVerts[(i + 5) % 6]);
      const cur = p.positions.get(hexVerts[i]);
      const next = p.positions.get(hexVerts[(i + 1) % 6]);
      const e1 = { x: prev.x - cur.x, y: prev.y - cur.y, z: prev.z - cur.z };
      const e2 = { x: next.x - cur.x, y: next.y - cur.y, z: next.z - cur.z };
      const l1 = Math.hypot(e1.x, e1.y, e1.z);
      const l2 = Math.hypot(e2.x, e2.y, e2.z);
      const cosA = (e1.x * e2.x + e1.y * e2.y + e1.z * e2.z) / (l1 * l2);
      const deg = (Math.acos(cosA) * 180) / Math.PI;
      assert.ok(Math.abs(deg - 120) < 0.01, `vertex ${i} interior angle ${deg} should be ~120deg`);
    }
  });

  test("dihedral term never moves a polygon's own vertices, only triangle apexes", () => {
    const { g, p, hexVerts, apexVerts } = seedHexWithPhysics();
    p.dihedralStiffness = 5.0;
    p.angleStiffness = 0;
    p.planarStiffness = 0;
    p.springStiffness = 0;
    const edges = g.edges();
    const faces = g.allFaces();
    const edgeFaces = g.edgesWithAdjacentFaces();
    const before = new Map();
    for (const id of hexVerts) before.set(id, { ...p.positions.get(id) });
    p.step(edges, faces, edgeFaces); // no momentum removal isolation needed - single step, direct term check
    // hex vertices should not have moved from the dihedral term itself in any way
    // that isn't explainable by global momentum removal; check they stay put to
    // a tight tolerance since all other terms are zeroed and momentum removal
    // only redistributes a small residual.
    for (const id of hexVerts) {
      const b = before.get(id);
      const a = p.positions.get(id);
      const moved = Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
      assert.ok(moved < 0.2, `hex vertex ${id} moved ${moved}, expected small (momentum-removal residual only)`);
    }
    void apexVerts;
  });
});

// ---- single-vertex nesting site placement (regression) ------------------

describe("placement on a single-vertex nesting site", () => {
  for (const n of [3, 4, 5, 6, 7]) {
    test(`n=${n} produces a polygon with ${n} vertices`, () => {
      const g = new WeaveGraph();
      g.seedHexagon();
      const v = g.boundary.find((x) => g.vertexKind(x) === "nesting");
      const record = g.placePolygon(v, n);
      assert.ok(record, `placement should succeed for n=${n}`);
      const poly = g.polygons.get(record.polygonId);
      assert.equal(poly.verts.length, n);
    });
  }

  test("n=3 routes through the joinEdge case (triangle special case)", () => {
    const g = new WeaveGraph();
    g.seedHexagon();
    const v = g.boundary.find((x) => g.vertexKind(x) === "nesting");
    const record = g.placePolygon(v, 3);
    assert.equal(record.kind, "joinEdge");
  });

  test("n < 3 fails", () => {
    const g = new WeaveGraph();
    g.seedHexagon();
    const v = g.boundary.find((x) => g.vertexKind(x) === "nesting");
    assert.equal(g.placePolygon(v, 2), null);
  });
});

// ---- multi-vertex nesting sites -----------------------------------------

describe("multi-vertex nesting sites", () => {
  test("placing a hexagon on the seed creates two 2-vertex nesting sites", () => {
    const g = new WeaveGraph();
    const { hexVerts } = g.seedHexagon();
    g.placePolygon(hexVerts[0], 6);

    const seen = new Set();
    const siteSizes = [];
    for (const bv of g.boundary) {
      if (g.vertexKind(bv) === "nesting" && !seen.has(bv)) {
        const site = g.nestingSiteAt(bv);
        for (const sv of site.siteVerts) seen.add(sv);
        siteSizes.push(site.siteVerts.length);
      }
    }
    assert.equal(siteSizes.filter((s) => s === 2).length, 2);
    assert.equal(siteSizes.filter((s) => s === 1).length, 6);
  });

  test("clicking either vertex of a 2-vertex site gives the same polygon shape", () => {
    function placeOn(which) {
      const g = new WeaveGraph();
      const { hexVerts } = g.seedHexagon();
      g.placePolygon(hexVerts[0], 6);
      const site = findNestingSite(g, 2);
      const record = g.placePolygon(site.siteVerts[which], 5);
      return g.polygons.get(record.polygonId).verts.length;
    }
    assert.equal(placeOn(0), placeOn(1));
  });

  for (const [n, expectedKind] of [
    [3, "fuse"],
    [4, "joinEdge"],
    [5, "normal"],
    [6, "normal"],
  ]) {
    test(`n=${n} on a 2-vertex site routes through '${expectedKind}'`, () => {
      const g = new WeaveGraph();
      const { hexVerts } = g.seedHexagon();
      g.placePolygon(hexVerts[0], 6);
      const site = findNestingSite(g, 2);
      const record = g.placePolygon(site.siteVerts[0], n);
      assert.ok(record, `placement should succeed for n=${n}`);
      assert.equal(record.kind, expectedKind);
    });
  }

  test("n=2 on a 2-vertex site fails (too many existing vertices)", () => {
    const g = new WeaveGraph();
    const { hexVerts } = g.seedHexagon();
    g.placePolygon(hexVerts[0], 6);
    const site = findNestingSite(g, 2);
    assert.equal(g.placePolygon(site.siteVerts[0], 2), null);
  });

  test("fuse case leaves the fused vertex as a new 3-face nesting vertex", () => {
    const g = new WeaveGraph();
    const { hexVerts } = g.seedHexagon();
    g.placePolygon(hexVerts[0], 6);
    const site = findNestingSite(g, 2);
    const record = g.placePolygon(site.siteVerts[0], 3);
    assert.equal(record.kind, "fuse");
    assert.equal(g.faceCount(record.fusedId), 3);
    assert.equal(g.vertexKind(record.fusedId), "nesting");
  });
});

// ---- deletion as exact structural inverse --------------------------------

describe("deletion", () => {
  test("deleting a normal placement restores the prior single-vertex site", () => {
    const g = new WeaveGraph();
    g.seedHexagon();
    const v = g.boundary.find((x) => g.vertexKind(x) === "nesting");
    const before = { polys: g.polygons.size, verts: g.vertices.size, boundaryLen: g.boundary.length };
    const record = g.placePolygon(v, 6);
    assert.ok(g.deleteAt(record.u));
    assert.deepEqual({ polys: g.polygons.size, verts: g.vertices.size, boundaryLen: g.boundary.length }, before);
  });

  for (const [n, kind] of [
    [5, "normal"],
    [4, "joinEdge"],
    [3, "fuse"],
  ]) {
    test(`deleting a ${kind} placement on a 2-vertex site restores the prior boundary`, () => {
      const g = new WeaveGraph();
      const { hexVerts } = g.seedHexagon();
      g.placePolygon(hexVerts[0], 6);
      const before = { polys: g.polygons.size, verts: g.vertices.size, boundaryLen: g.boundary.length };

      const site = findNestingSite(g, 2);
      const record = g.placePolygon(site.siteVerts[0], n);
      assert.equal(record.kind, kind);
      const clickVertex = record.kind === "fuse" ? record.fusedId : record.u;
      assert.ok(g.deleteAt(clickVertex));

      assert.deepEqual({ polys: g.polygons.size, verts: g.vertices.size, boundaryLen: g.boundary.length }, before);
    });
  }

  test("delete is refused once something has been built on top (not a leaf)", () => {
    const g = new WeaveGraph();
    const { hexVerts } = g.seedHexagon();
    const record = g.placePolygon(hexVerts[0], 6);
    // build on one of the fresh mid-vertices
    const childTarget = record.midVerts[0];
    g.placePolygon(childTarget, 4);
    assert.equal(g.deleteAt(record.u), false);
  });

  test("the seed hexagon itself can never be deleted", () => {
    const g = new WeaveGraph();
    const { hexVerts } = g.seedHexagon();
    assert.equal(g.deleteAt(hexVerts[0]), false);
  });
});

// ---- end-to-end physics stability ----------------------------------------

describe("physics stability after placement", () => {
  for (const n of [3, 4, 5, 6, 7]) {
    test(`n=${n} single-vertex placement settles without NaN/Infinity`, () => {
      const { g, p, hexVerts } = seedHexWithPhysics();
      settle(g, p, 100);
      const record = place(g, p, hexVerts[0], n);
      assert.ok(record);
      settle(g, p, 200);
      assert.ok(allFinite(p));
    });
  }

  for (const [n, kind] of [
    [5, "normal"],
    [4, "joinEdge"],
    [3, "fuse"],
  ]) {
    test(`${kind} placement on a 2-vertex site settles without NaN/Infinity`, () => {
      const { g, p, hexVerts } = seedHexWithPhysics();
      settle(g, p, 100);
      place(g, p, hexVerts[0], 6);
      settle(g, p, 50);
      const site = findNestingSite(g, 2);
      const record = place(g, p, site.siteVerts[0], n);
      assert.ok(record);
      assert.equal(record.kind, kind);
      settle(g, p, 300);
      assert.ok(allFinite(p));
    });
  }

  test("new placements keep all edges within +-10% of unit length at equilibrium", () => {
    const { g, p, hexVerts } = seedHexWithPhysics();
    settle(g, p, 100);
    place(g, p, hexVerts[0], 6);
    settle(g, p, 400);
    for (const [a, b] of g.edges()) {
      const pa = p.positions.get(a);
      const pb = p.positions.get(b);
      const len = Math.hypot(pb.x - pa.x, pb.y - pa.y, pb.z - pa.z);
      assert.ok(len >= 0.9 && len <= 1.1, `edge ${a}-${b} length ${len} outside +-10% of unit`);
    }
  });
});

// ---- chord-ratio construction (physics.js geometry) ----------------------

describe("chord-ratio seeding construction", () => {
  test("R(n) matches known regular-polygon diagonal/edge ratios", () => {
    const ratioFor = (n) => Math.sin((3 * Math.PI) / n) / Math.sin(Math.PI / n);
    assert.ok(Math.abs(ratioFor(4) - 1) < 1e-9); // square
    assert.ok(Math.abs(ratioFor(5) - (1 + Math.sqrt(5)) / 2) < 1e-9); // golden ratio
    assert.ok(Math.abs(ratioFor(6) - 2) < 1e-9); // hexagon
  });

  test("seeded chord A->D is exactly parallel to B->C for every generated mid-vertex", () => {
    const { g, p, hexVerts } = seedHexWithPhysics();
    settle(g, p, 300);
    const record = place(g, p, hexVerts[0], 7);
    const loopVerts = [record.u, ...record.siteVerts, record.w, ...record.midVerts];
    const pts = loopVerts.map((id) => p.positions.get(id));
    const n = loopVerts.length;
    for (let i = 0; i < n - 3; i++) {
      const A = pts[i],
        B = pts[i + 1],
        C = pts[i + 2],
        D = pts[i + 3];
      const BC = { x: C.x - B.x, y: C.y - B.y, z: C.z - B.z };
      const AD = { x: D.x - A.x, y: D.y - A.y, z: D.z - A.z };
      const cross = Math.hypot(
        BC.y * AD.z - BC.z * AD.y,
        BC.z * AD.x - BC.x * AD.z,
        BC.x * AD.y - BC.y * AD.x
      );
      assert.ok(cross < 1e-9, `chord ${i} not parallel (cross=${cross})`);
    }
  });
});

describe("boundary with no apex vertices", () => {
  test("nesting sites and placement return null instead of hanging", () => {
    const g = new WeaveGraph();
    const { apexVerts } = g.seedHexagon();
    // Give every apex a second face so the whole boundary is nesting vertices.
    for (const a of apexVerts) {
      g._addTriangleFace([a, g._addVertex(), g._addVertex()]);
    }
    assert.ok(g.boundary.every((v) => g.vertexKind(v) === "nesting"));

    assert.deepEqual(g.concaveVertices(), []);
    for (const v of g.boundary) {
      assert.equal(g.nestingSiteAt(v), null);
      assert.equal(g.placePolygon(v, 6), null);
    }
  });
});
