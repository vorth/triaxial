// weaveGraph.test.js
// Run with: node --test
//
// Covers the combinatorial/topological model (weaveGraph.js) and placement
// seeding (seeding.js): placement (normal, joinEdge, fuse, and the
// too-many-vertices failure) and deletion as the exact structural inverse of
// each. Strip mechanics are covered in rapierWeave.test.js.

import { test, describe } from "node:test";
import assert from "node:assert/strict";
import { WeaveGraph } from "./weaveGraph.js";
import { SeededPositions } from "./testSupport.js";

// ---- shared helpers ---------------------------------------------------

function seedHex() {
  const g = new WeaveGraph();
  const p = new SeededPositions();
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

function place(g, p, v, n) {
  const record = g.placePolygon(v, n);
  if (!record) return null;
  const existingChain =
    record.kind === "fuse" ? [record.u, ...record.siteVerts] : [record.u, ...record.siteVerts, record.w];
  p.seedPlacement(existingChain, record.midVerts, record.apexVerts);
  return record;
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

// ---- chord-ratio construction (seeding.js geometry) ----------------------

describe("chord-ratio seeding construction", () => {
  test("R(n) matches known regular-polygon diagonal/edge ratios", () => {
    const ratioFor = (n) => Math.sin((3 * Math.PI) / n) / Math.sin(Math.PI / n);
    assert.ok(Math.abs(ratioFor(4) - 1) < 1e-9); // square
    assert.ok(Math.abs(ratioFor(5) - (1 + Math.sqrt(5)) / 2) < 1e-9); // golden ratio
    assert.ok(Math.abs(ratioFor(6) - 2) < 1e-9); // hexagon
  });

  test("seeded chord A->D is exactly parallel to B->C for every generated mid-vertex", () => {
    const { g, p, hexVerts } = seedHex();
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

describe("serialization", () => {
  // A few placements of every kind, through JSON text and back.
  function grownGraph() {
    const g = new WeaveGraph();
    const { hexVerts } = g.seedHexagon();
    const first = g.placePolygon(hexVerts[0], 6);
    assert.ok(g.placePolygon(first.midVerts[0], 5));
    assert.equal(g.placePolygon(findNestingSite(g, 2).siteVerts[0], 3).kind, "fuse");
    return g;
  }
  const roundTrip = (g) => {
    const h = new WeaveGraph();
    h.loadJSON(JSON.parse(JSON.stringify(g.toJSON())));
    return h;
  };

  test("a round trip reproduces the graph exactly", () => {
    const g = grownGraph();
    const h = roundTrip(g);
    assert.deepEqual(h.toJSON(), g.toJSON());
    assert.deepEqual(h.concaveVertices(), g.concaveVertices());
    for (const v of g.vertices.keys()) assert.equal(h.faceCount(v), g.faceCount(v));
  });

  test("placement provenance survives: leaf deletion and the non-leaf guard", () => {
    const g = new WeaveGraph();
    const { hexVerts } = g.seedHexagon();
    const first = g.placePolygon(hexVerts[0], 6);
    const second = g.placePolygon(first.midVerts[0], 4);
    const h = roundTrip(g);
    assert.equal(h.deleteAt(first.u), false); // has a child
    assert.ok(h.deleteAt(second.kind === "fuse" ? second.fusedId : second.u));
    assert.ok(h.deleteAt(first.u)); // a leaf again once its child is gone
  });

  test("placements after loading never reuse an existing id", () => {
    const h = roundTrip(grownGraph());
    const before = new Set([...h.vertices.keys(), ...h.polygons.keys(), ...h.triangles.keys()]);
    const record = h.placePolygon(h.concaveVertices()[0], 6);
    assert.ok(record);
    for (const id of [...record.midVerts, ...record.apexVerts, record.polygonId, ...record.triangleIds]) {
      assert.ok(!before.has(id));
    }
  });
});
