// weaveGraph.js
// Pure combinatorial + topological model of a weave graph:
// polygons and triangles glued vertex-to-vertex, one triangle per polygon edge,
// interior vertices degree-4 (2 triangles + 2 polygons alternating), boundary is
// a cycle alternating convex triangle-apex vertices (1 face) and "nesting sites"
// (the valid placement targets) - runs of one or more consecutive boundary
// vertices, each with 2 or 3 faces already, that still need more faces to
// close to the interior degree-4 profile.
//
// Representation:
//  - vertices: Map<id, { id }>                       (geometry lives elsewhere, keyed by id)
//  - polygons: Map<id, { id, verts: [id...], record }> (N >= 3, the user-placed cells)
//  - triangles: Map<id, { id, verts: [id,id,id] }>    (auto-managed filler faces)
//  - boundary: array of vertex ids in cyclic order around the open boundary
//
// Placement provenance: each placed polygon keeps a PlacementRecord describing
// exactly the boundary edit it made, so deletion can be its exact structural
// inverse. Deleting a polygon is only permitted while it is still a "leaf" —
// none of the fresh vertices/merges it created have since had another
// polygon placed on them. We track this with a simple child-count on each
// record; placing on a fresh vertex (or a fused vertex) increments the
// parent's childCount, and deletion is only allowed when childCount === 0.
//
// Each boundary vertex's "kind" is derived on demand from how many faces
// currently reference it:
//   1 face        -> apex (pure separator between nesting sites, never a
//                     click target itself)
//   2 or 3 faces  -> nesting vertex (part of some nesting site, a click target)
// (0 or 4 faces cannot occur on the boundary: 0 would mean not yet part of
// any face, which never happens for a vertex we've created; 4 means fully
// closed, i.e. interior, which removes it from the boundary loop.)

let nextId = 1;
function freshId() {
  return nextId++;
}

export class WeaveGraph {
  constructor() {
    this.vertices = new Map(); // id -> {id}
    this.polygons = new Map(); // id -> {id, verts:[...], record}
    this.triangles = new Map(); // id -> {id, verts:[a,b,c]}
    this.boundary = []; // array of vertex ids, cyclic order

    this._faceCountAt = new Map(); // vertex id -> number of faces touching it
    this._facesAt = new Map(); // vertex id -> Set of face keys ("p"+id / "t"+id)

    // provenance: vertex id -> the PlacementRecord that created it (undefined for seed verts)
    this._createdBy = new Map();
  }

  reset() {
    this.vertices.clear();
    this.polygons.clear();
    this.triangles.clear();
    this.boundary = [];
    this._faceCountAt.clear();
    this._facesAt.clear();
    this._createdBy.clear();
  }

  // ---- internal bookkeeping -------------------------------------------------

  _addVertex(createdBy) {
    const id = freshId();
    this.vertices.set(id, { id });
    this._faceCountAt.set(id, 0);
    this._facesAt.set(id, new Set());
    if (createdBy) this._createdBy.set(id, createdBy);
    return id;
  }

  _touchFace(vertId, faceKey) {
    const set = this._facesAt.get(vertId);
    if (!set.has(faceKey)) {
      set.add(faceKey);
      this._faceCountAt.set(vertId, set.size);
    }
  }

  _untouchFace(vertId, faceKey) {
    const set = this._facesAt.get(vertId);
    if (set && set.has(faceKey)) {
      set.delete(faceKey);
      this._faceCountAt.set(vertId, set.size);
    }
  }

  _addPolygonFace(verts, record) {
    const id = freshId();
    const key = "p" + id;
    this.polygons.set(id, { id, verts: verts.slice(), record });
    for (const v of verts) this._touchFace(v, key);
    return id;
  }

  _addTriangleFace(verts) {
    const id = freshId();
    const key = "t" + id;
    this.triangles.set(id, { id, verts: verts.slice() });
    for (const v of verts) this._touchFace(v, key);
    return id;
  }

  _removePolygonFace(id) {
    const poly = this.polygons.get(id);
    if (!poly) return;
    const key = "p" + id;
    for (const v of poly.verts) this._untouchFace(v, key);
    this.polygons.delete(id);
  }

  _removeTriangleFace(id) {
    const tri = this.triangles.get(id);
    if (!tri) return;
    const key = "t" + id;
    for (const v of tri.verts) this._untouchFace(v, key);
    this.triangles.delete(id);
  }

  // Replace every occurrence of `oldId` with `newId` across all faces (used
  // when fusing two apex vertices into one). Both vertices must currently
  // exist; oldId is left in `vertices` (caller deletes it) but its face
  // bookkeeping is merged into newId's.
  _mergeVertexInto(oldId, newId) {
    const oldFaces = this._facesAt.get(oldId);
    if (!oldFaces) return;
    for (const key of oldFaces) {
      const isPoly = key[0] === "p";
      const id = parseInt(key.slice(1), 10);
      const face = isPoly ? this.polygons.get(id) : this.triangles.get(id);
      if (!face) continue;
      face.verts = face.verts.map((v) => (v === oldId ? newId : v));
      this._touchFace(newId, key);
    }
  }

  faceCount(vertId) {
    return this._faceCountAt.get(vertId) || 0;
  }

  // vertex kind: 'apex' | 'nesting' | 'interior'
  vertexKind(vertId) {
    const idx = this.boundary.indexOf(vertId);
    if (idx === -1) return "interior";
    const fc = this.faceCount(vertId);
    if (fc <= 1) return "apex";
    return "nesting"; // 2 or 3 faces
  }

  // Backwards-compatible alias used by the UI: any nesting vertex is a valid
  // click target ("concave" in the old single-vertex-only vocabulary).
  vertexType(vertId) {
    const kind = this.vertexKind(vertId);
    if (kind === "nesting") return "concave";
    if (kind === "apex") return "convex";
    return "interior";
  }

  concaveVertices() {
    // With no apex anywhere on the boundary there are no flanking apexes,
    // so no nesting site can be formed and nothing is placeable.
    if (!this.boundary.some((v) => this.vertexKind(v) === "apex")) return [];
    return this.boundary.filter((v) => this.vertexKind(v) === "nesting");
  }

  polygonAt(vertId) {
    const set = this._facesAt.get(vertId);
    if (!set) return null;
    for (const key of set) {
      if (key[0] === "p") return this.polygons.get(parseInt(key.slice(1), 10));
    }
    return null;
  }

  // Find the maximal run of consecutive nesting-vertices in the boundary
  // loop containing `v` (a "nesting site"), plus its two flanking apexes.
  // Returns { apexLeft, siteVerts: [s1..sk], apexRight } or null if `v`
  // isn't currently a nesting vertex, or if the boundary has no apex at all
  // (every boundary vertex is a nesting vertex, so there are no flanks).
  nestingSiteAt(v) {
    if (this.vertexKind(v) !== "nesting") return null;
    const len = this.boundary.length;
    const idx = this.boundary.indexOf(v);

    let lo = idx;
    let steps = 0;
    while (this.vertexKind(this.boundary[(lo - 1 + len) % len]) === "nesting") {
      lo = (lo - 1 + len) % len;
      if (++steps >= len) return null; // wrapped around: no apex anywhere
    }
    let hi = idx;
    while (this.vertexKind(this.boundary[(hi + 1) % len]) === "nesting") {
      hi = (hi + 1) % len;
    }

    const siteVerts = [];
    for (let k = lo; ; k = (k + 1) % len) {
      siteVerts.push(this.boundary[k]);
      if (k === hi) break;
    }
    const apexLeft = this.boundary[(lo - 1 + len) % len];
    const apexRight = this.boundary[(hi + 1) % len];
    return { apexLeft, siteVerts, apexRight };
  }

  // All unique edges across polygons+triangles, for rendering & physics.
  edges() {
    const seen = new Set();
    const result = [];
    const addLoop = (verts) => {
      const n = verts.length;
      for (let i = 0; i < n; i++) {
        const a = verts[i];
        const b = verts[(i + 1) % n];
        const key = a < b ? a + "_" + b : b + "_" + a;
        if (!seen.has(key)) {
          seen.add(key);
          result.push([a, b]);
        }
      }
    };
    for (const poly of this.polygons.values()) addLoop(poly.verts);
    for (const tri of this.triangles.values()) addLoop(tri.verts);
    return result;
  }

  // All faces (for angle-regularization physics term).
  allFaces() {
    const faces = [];
    for (const poly of this.polygons.values()) faces.push(poly.verts);
    for (const tri of this.triangles.values()) faces.push(tri.verts);
    return faces;
  }

  // Each edge that currently has two adjacent faces, together with those
  // faces - this is every edge in the graph EXCEPT the open 1-D edges of
  // boundary triangles (their two "free" legs, which have no face on the
  // outside yet and so can't have a dihedral angle at all). In particular
  // this includes every edge shared between a boundary triangle and the
  // polygon cell it's attached to - "boundary" in the app's sense (a
  // triangle at the mesh's open edge) is not the same as "has only one
  // face": a boundary triangle still has two of its three edges fully
  // double-sided, and both of those are included here.
  // Returns: [{ a, b, faceA: [verts...], faceB: [verts...] }, ...]
  edgesWithAdjacentFaces() {
    const byEdge = new Map(); // key -> { a, b, faces: [verts,...] }
    const addLoop = (verts) => {
      const n = verts.length;
      for (let i = 0; i < n; i++) {
        const a = verts[i];
        const b = verts[(i + 1) % n];
        const key = a < b ? a + "_" + b : b + "_" + a;
        let entry = byEdge.get(key);
        if (!entry) {
          entry = { a, b, faces: [] };
          byEdge.set(key, entry);
        }
        entry.faces.push(verts);
      }
    };
    for (const poly of this.polygons.values()) addLoop(poly.verts);
    for (const tri of this.triangles.values()) addLoop(tri.verts);

    const result = [];
    for (const entry of byEdge.values()) {
      if (entry.faces.length === 2) {
        result.push({ a: entry.a, b: entry.b, faceA: entry.faces[0], faceB: entry.faces[1] });
      }
    }
    return result;
  }

  // For every degree-4 vertex (4 distinct edges - true for every vertex in
  // this graph except fresh triangle apexes, which only ever have 2), its 4
  // neighbors in true cyclic ANGULAR order around it. A fully-interior
  // vertex (4 faces) has this order fully backed by faces on every side; a
  // boundary "nesting" vertex (2 or 3 faces) is degree-4 too - it always
  // has all 4 of its edges already, just with one angular sector still an
  // open gap (no face there yet) rather than four - and the 4 neighbors
  // still have a well-defined cyclic order, it's just an open chain instead
  // of a closed one at the gap.
  //
  // Used by the physics collinearity term: at such a vertex, each pair of
  // OPPOSITE neighbors (2 apart in this cyclic order, i.e.
  // neighbors[0]&neighbors[2] and neighbors[1]&neighbors[3]) should end up
  // collinear through the vertex for the mesh to be locally flat there -
  // including at the boundary, where "this vertex isn't a kink" is just as
  // meaningful even though one of its four angular sectors is still open.
  // Returns: [{ v, neighbors: [n0,n1,n2,n3], firstSectorIsPolygon }, ...]
  degree4VertexNeighbors() {
    const result = [];
    for (const [v, faceKeys] of this._facesAt) {
      const faceCount = faceKeys.size;
      if (faceCount < 2 || faceCount > 4) continue; // 1 face = pure apex (degree 2), not degree-4

      // Collect, per face touching v, the pair of v's neighbors within that
      // face's own loop (the two edges of that face incident to v) - each
      // pair means those two edges are angularly CONSECUTIVE around v, with
      // this face between them.
      const faceNeighborPairs = []; // [[na,nb], ...] one per face
      for (const key of faceKeys) {
        const isPoly = key[0] === "p";
        const id = parseInt(key.slice(1), 10);
        const face = isPoly ? this.polygons.get(id) : this.triangles.get(id);
        if (!face) continue;
        const verts = face.verts;
        const n = verts.length;
        const iv = verts.indexOf(v);
        if (iv === -1) continue;
        const na = verts[(iv - 1 + n) % n];
        const nb = verts[(iv + 1) % n];
        faceNeighborPairs.push([na, nb, isPoly]);
      }
      if (faceNeighborPairs.length !== faceCount) continue;

      // Chain the pairs together via shared endpoints (a half-edge walk).
      // With 4 faces this closes into a 4-cycle; with 2 or 3 faces it forms
      // an open path whose two endpoints are exactly the vertices flanking
      // the still-open gap. Either way, walking outward from an arbitrary
      // starting pair visits all of v's neighbors in true cyclic order.
      const neighbors = [faceNeighborPairs[0][0], faceNeighborPairs[0][1]];
      const used = new Set([0]);
      // extend forward from the current tail - capped at 4 total neighbors
      // (not just "one fewer step than there are pairs"), since a CLOSED
      // 4-face cycle would otherwise wrap back around and re-append its own
      // starting vertex as a spurious 5th entry, which then failed the
      // length-4 sanity check below and silently dropped every fully
      // interior vertex from the result.
      for (let step = 0; step < faceNeighborPairs.length - 1 && neighbors.length < 4; step++) {
        const current = neighbors[neighbors.length - 1];
        let found = false;
        for (let i = 0; i < faceNeighborPairs.length; i++) {
          if (used.has(i)) continue;
          const [na, nb] = faceNeighborPairs[i];
          if (na === current) {
            neighbors.push(nb);
            used.add(i);
            found = true;
            break;
          }
          if (nb === current) {
            neighbors.push(na);
            used.add(i);
            found = true;
            break;
          }
        }
        if (!found) break;
      }
      // extend backward from the current head (only needed for open chains,
      // where the initial pair might not be at either true end)
      for (let step = 0; step < faceNeighborPairs.length - 1 && neighbors.length < 4; step++) {
        const current = neighbors[0];
        let found = false;
        for (let i = 0; i < faceNeighborPairs.length; i++) {
          if (used.has(i)) continue;
          const [na, nb] = faceNeighborPairs[i];
          if (na === current) {
            neighbors.unshift(nb);
            used.add(i);
            found = true;
            break;
          }
          if (nb === current) {
            neighbors.unshift(na);
            used.add(i);
            found = true;
            break;
          }
        }
        if (!found) break;
      }
      if (neighbors.length === 4 && new Set(neighbors).size === 4) {
        // The sector between neighbors[0] and neighbors[1] is always backed
        // by a face (an open chain's gap is between neighbors[3] and
        // neighbors[0]); report whether it is a polygon or a weave triangle,
        // looked up by face class, never by vertex count (a 3-gon polygon
        // is still a polygon). Strips use this for the over/under rule.
        const first = faceNeighborPairs.find(
          ([na, nb]) => (na === neighbors[0] && nb === neighbors[1]) || (na === neighbors[1] && nb === neighbors[0]),
        );
        result.push({ v, neighbors, firstSectorIsPolygon: first ? first[2] : null });
      }
    }
    return result;
  }

  // ---- seed -------------------------------------------------------------

  // Single hexagonal cell surrounded by six triangles; all six hex vertices
  // are boundary (nesting) targets.
  seedHexagon() {
    this.reset();
    const hexVerts = [];
    for (let i = 0; i < 6; i++) hexVerts.push(this._addVertex());
    this._addPolygonFace(hexVerts, null);

    const apexVerts = [];
    for (let i = 0; i < 6; i++) apexVerts.push(this._addVertex());

    for (let i = 0; i < 6; i++) {
      const v0 = hexVerts[i];
      const v1 = hexVerts[(i + 1) % 6];
      const a = apexVerts[i];
      this._addTriangleFace([v0, v1, a]);
    }

    // Boundary loop alternates apex, hexVert, apex, hexVert...
    // apex_i is flanked by hexVert_i and hexVert_{i+1}
    const loop = [];
    for (let i = 0; i < 6; i++) {
      loop.push(apexVerts[(i + 5) % 6]); // apex_{i-1}
      loop.push(hexVerts[i]);
    }
    this.boundary = loop;

    return { hexVerts, apexVerts };
  }

  // ---- placement ----------------------------------------------------------

  // Place an N-gon (N>=3) at the nesting site containing boundary vertex
  // `v` (any vertex within a multi-vertex nesting site has the same effect).
  // The new polygon always uses the site's flanking apexes plus all of its
  // existing nesting vertices as its first (k+2) vertices, where k is the
  // site's length; the remaining n-(k+2) vertices are freshly created via
  // the chord-ratio construction, EXCEPT for two special cases:
  //   n == k+2  : no fresh vertices - the two flanking apexes become
  //               directly adjacent via one new edge (generalizes the old
  //               "triangle cell = insert an edge between the two apexes").
  //   n == k+1  : one too many existing vertices - fuse the two flanking
  //               apexes into a single vertex, which becomes adjacent to
  //               both ends of the existing chain.
  //   n < k+1   : impossible to build (too many existing vertices for the
  //               requested polygon) - returns null.
  // Returns the PlacementRecord, or null if invalid.
  placePolygon(v, n) {
    if (n < 3) return null;
    const site = this.nestingSiteAt(v);
    if (!site) return null;
    const { apexLeft, siteVerts, apexRight } = site;
    const k = siteVerts.length;

    if (n < k + 1) return null; // too many existing vertices - fail

    if (n === k + 1) return this._placeFusing(site, n);
    if (n === k + 2) return this._placeJoiningEdge(site, n);
    return this._placeNormal(site, n);
  }

  // Normal case: n > k+2, generate (n-k-2) fresh mid-vertices continuing
  // the chain apexLeft, s1..sk, apexRight, then fan triangles around the
  // remaining open edges (apexRight -> mid_1 -> ... -> mid_last -> apexLeft).
  _placeNormal(site, n) {
    const { apexLeft: u, siteVerts, apexRight: w } = site;
    const k = siteVerts.length;
    const midCount = n - k - 2;

    const record = {
      kind: "normal",
      polygonId: null,
      u,
      siteVerts,
      w,
      midVerts: [],
      apexVerts: [],
      triangleIds: [],
      childCount: 0,
      parents: this._parentsOf([u, ...siteVerts, w]),
    };

    const midVerts = [];
    for (let i = 0; i < midCount; i++) midVerts.push(this._addVertex(record));
    record.midVerts = midVerts;

    const polyLoop = [u, ...siteVerts, w, ...midVerts];
    const polygonId = this._addPolygonFace(polyLoop, record);
    record.polygonId = polygonId;

    // Fresh triangles fill the chain w -> mid_1 -> ... -> mid_last -> u
    // (every edge of the new polygon except the k+1 reused ones along
    // apexLeft..apexRight, which already have their triangle from before).
    const chain = [w, ...midVerts, u];
    const newBoundarySegment = [];
    for (let i = 0; i < chain.length - 1; i++) {
      const a = chain[i];
      const b = chain[i + 1];
      const apex = this._addVertex(record);
      record.apexVerts.push(apex);
      const triId = this._addTriangleFace([a, b, apex]);
      record.triangleIds.push(triId);
      newBoundarySegment.push(a, apex);
    }
    newBoundarySegment.push(chain[chain.length - 1]); // final vertex u

    this._spliceBoundary(u, w, newBoundarySegment.slice().reverse());
    this._bumpChildCounts(record);
    return record;
  }

  // n == k+2: exact fit, no fresh vertices - close the polygon directly
  // through [apexLeft, s1..sk, apexRight] and add exactly one new triangle
  // bridging the new (apexRight -> apexLeft) edge with a fresh apex. This
  // generalizes the original triangle-cell behavior (k=1, n=3).
  _placeJoiningEdge(site) {
    const { apexLeft: u, siteVerts, apexRight: w } = site;

    const record = {
      kind: "joinEdge",
      polygonId: null,
      u,
      siteVerts,
      w,
      midVerts: [],
      apexVerts: [],
      triangleIds: [],
      childCount: 0,
      parents: this._parentsOf([u, ...siteVerts, w]),
    };

    const polyLoop = [u, ...siteVerts, w];
    const polygonId = this._addPolygonFace(polyLoop, record);
    record.polygonId = polygonId;

    const apex = this._addVertex(record);
    record.apexVerts.push(apex);
    const triId = this._addTriangleFace([w, u, apex]);
    record.triangleIds.push(triId);

    // Boundary segment replacing [u, s1..sk, w] is just [u, apex, w].
    this._spliceBoundary(u, w, [u, apex, w]);
    this._bumpChildCounts(record);
    return record;
  }

  // n == k+1: one too many existing vertices for a plain closure - fuse
  // apexLeft and apexRight into a single vertex, which becomes adjacent to
  // both s1 and sk. The fused vertex ends up with 3 faces (apexLeft's old
  // triangle, apexRight's old triangle, and the new polygon), i.e. it
  // becomes a new single-vertex nesting site itself.
  _placeFusing(site) {
    const { apexLeft: u, siteVerts, apexRight: w } = site;

    const record = {
      kind: "fuse",
      polygonId: null,
      u,
      siteVerts,
      w,
      fusedId: null,
      removedId: null, // the apex identity that was merged away (w)
      midVerts: [],
      apexVerts: [],
      triangleIds: [],
      childCount: 0,
      parents: this._parentsOf([u, ...siteVerts, w]),
    };

    // Splice the boundary FIRST (while both u and w are still distinct,
    // findable ids), replacing [u, s1..sk, w] with just [u] - w no longer
    // exists as a separate boundary entry once fused.
    this._spliceBoundary(u, w, [u]);

    // Now merge w's faces onto u and delete w's identity.
    this._mergeVertexInto(w, u);
    this.vertices.delete(w);
    this._faceCountAt.delete(w);
    this._facesAt.delete(w);
    this._createdBy.delete(w);
    record.fusedId = u;
    record.removedId = w;

    const polyLoop = [u, ...siteVerts];
    const polygonId = this._addPolygonFace(polyLoop, record);
    record.polygonId = polygonId;

    this._bumpChildCounts(record);
    return record;
  }

  // Splice the boundary loop, replacing the contiguous block from vertex
  // `u` to vertex `w` (inclusive on both ends) with `replacement`. Callers
  // that need to fuse u and w into one id must call this BEFORE performing
  // the fusion, while both are still distinct, findable boundary entries.
  _spliceBoundary(u, w, replacement) {
    const len = this.boundary.length;
    const idxU = this.boundary.indexOf(u);
    const idxW = this.boundary.indexOf(w);
    if (idxU === -1 || idxW === -1) return;
    const rest = [];
    for (let k = 1; k <= len - 1; k++) {
      const pos = (idxW + k) % len;
      if (pos === idxU) break;
      rest.push(this.boundary[pos]);
    }
    this.boundary = [...replacement, ...rest];
  }

  _parentsOf(vertIds) {
    const parents = new Set();
    for (const id of vertIds) {
      const p = this._createdBy.get(id);
      if (p) parents.add(p);
    }
    return parents;
  }

  _bumpChildCounts(record) {
    for (const p of record.parents) p.childCount++;
  }

  // ---- deletion -------------------------------------------------------------

  // Delete the polygon that "caps" the given boundary vertex, but only if
  // it is still a leaf placement (childCount === 0). Returns true if deleted.
  deleteAt(vertId) {
    const poly = this.polygonAt(vertId);
    if (!poly || !poly.record) return false; // can't delete the seed hexagon
    return this._deleteRecord(poly.record);
  }

  _deleteRecord(record) {
    if (record.childCount > 0) return false;

    if (record.kind === "fuse") return this._undoFuse(record);
    if (record.kind === "joinEdge") return this._undoJoinEdge(record);
    return this._undoNormal(record);
  }

  _verifyFreshUntouched(record) {
    for (const mv of record.midVerts) {
      if (!this.vertices.has(mv)) return false;
      if (this.faceCount(mv) !== 3) return false;
    }
    for (const av of record.apexVerts) {
      if (!this.vertices.has(av)) return false;
      if (this.faceCount(av) !== 1) return false;
    }
    return true;
  }

  _removeFreshVerts(record) {
    for (const mv of record.midVerts) {
      this.vertices.delete(mv);
      this._faceCountAt.delete(mv);
      this._facesAt.delete(mv);
      this._createdBy.delete(mv);
    }
    for (const av of record.apexVerts) {
      this.vertices.delete(av);
      this._faceCountAt.delete(av);
      this._facesAt.delete(av);
      this._createdBy.delete(av);
    }
  }

  _unbumpChildCounts(record) {
    for (const p of record.parents) p.childCount--;
  }

  _undoNormal(record) {
    const { u, siteVerts, w, midVerts, apexVerts } = record;
    if (!this._verifyFreshUntouched(record)) return false;

    const freshSet = new Set([...midVerts, ...apexVerts]);
    const idxU = this.boundary.indexOf(u);
    const idxW = this.boundary.indexOf(w);
    if (idxU === -1 || idxW === -1) return false;

    const len = this.boundary.length;
    let k = idxU;
    let steps = 0;
    let ok = false;
    while (steps < len) {
      k = (k + 1) % len;
      steps++;
      const vv = this.boundary[k];
      if (vv === w) {
        ok = true;
        break;
      }
      if (!freshSet.has(vv)) {
        ok = false;
        break;
      }
    }
    if (!ok) return false;

    for (const triId of record.triangleIds) this._removeTriangleFace(triId);
    this._removePolygonFace(record.polygonId);
    this._removeFreshVerts(record);

    // Restore boundary: replace the [u, ...fresh..., w] block with
    // [u, s1..sk, w].
    const p0 = this.boundary.indexOf(w);
    const survivors = [];
    let p = p0;
    while (true) {
      survivors.push(this.boundary[p]);
      if (p === idxU) break;
      p = (p + 1) % len;
    }
    survivors.shift(); // drop leading w
    survivors.pop(); // drop trailing u
    this.boundary = [u, ...siteVerts, w, ...survivors];

    this._unbumpChildCounts(record);
    return true;
  }

  _undoJoinEdge(record) {
    const { u, siteVerts, w, apexVerts } = record;
    if (!this._verifyFreshUntouched(record)) return false;
    const apex = apexVerts[0];

    const idxU = this.boundary.indexOf(u);
    // boundary segment is currently [..., u, apex, w, ...]
    const idxApex = this.boundary.indexOf(apex);
    const idxW = this.boundary.indexOf(w);
    if (idxU === -1 || idxApex === -1 || idxW === -1) return false;
    const len = this.boundary.length;
    if ((idxU + 1) % len !== idxApex || (idxApex + 1) % len !== idxW) return false;

    for (const triId of record.triangleIds) this._removeTriangleFace(triId);
    this._removePolygonFace(record.polygonId);
    this._removeFreshVerts(record);

    const survivors = [];
    let p = idxW;
    while (true) {
      survivors.push(this.boundary[p]);
      if (p === idxU) break;
      p = (p + 1) % len;
    }
    survivors.shift();
    survivors.pop();
    this.boundary = [u, ...siteVerts, w, ...survivors];

    this._unbumpChildCounts(record);
    return true;
  }

  _undoFuse(record) {
    const { u, siteVerts, w, fusedId, removedId } = record;
    // fusedId (== u) must still exist and have exactly the 3 faces this
    // placement left it with (its own old triangle, the ex-apexRight's old
    // triangle, and this polygon) - if anything else was placed on it since
    // (besides this polygon), childCount would already be > 0 and we
    // wouldn't reach here, but double check face count as a sanity guard.
    if (!this.vertices.has(fusedId)) return false;
    if (this.faceCount(fusedId) !== 3) return false;

    // Recover which two faces at fusedId are the original flanking
    // triangles (everything on fusedId except this polygon).
    const polyKey = "p" + record.polygonId;
    const otherFaceKeys = [...this._facesAt.get(fusedId)].filter((k) => k !== polyKey);
    if (otherFaceKeys.length !== 2) return false;

    this._removePolygonFace(record.polygonId);

    // Recreate `removedId` (w) and repoint one of the two flanking
    // triangles (whichever one originally belonged to apexRight) back onto
    // it. We identify apexRight's triangle as the one that does NOT
    // contain sk-adjacent... simplest robust rule: apexLeft's triangle
    // contains s1 (siteVerts[0]); apexRight's triangle contains sk (last
    // siteVert). Whichever remaining face contains the last siteVert is
    // the one to repoint onto the restored w.
    const sk = siteVerts[siteVerts.length - 1];
    let apexRightFaceKey = null;
    for (const key of otherFaceKeys) {
      const isPoly = key[0] === "p";
      const id = parseInt(key.slice(1), 10);
      const face = isPoly ? this.polygons.get(id) : this.triangles.get(id);
      if (face && face.verts.includes(sk)) {
        apexRightFaceKey = key;
        break;
      }
    }
    if (!apexRightFaceKey) return false;

    this.vertices.set(removedId, { id: removedId });
    this._faceCountAt.set(removedId, 0);
    this._facesAt.set(removedId, new Set());

    const isPoly = apexRightFaceKey[0] === "p";
    const faceId = parseInt(apexRightFaceKey.slice(1), 10);
    const face = isPoly ? this.polygons.get(faceId) : this.triangles.get(faceId);
    face.verts = face.verts.map((v) => (v === fusedId ? removedId : v));
    this._untouchFace(fusedId, apexRightFaceKey);
    this._touchFace(removedId, apexRightFaceKey);

    // Restore boundary: the single fused-vertex entry becomes [u, s1..sk, w] again.
    const idx = this.boundary.indexOf(fusedId);
    if (idx === -1) return false;
    const len = this.boundary.length;
    const before = this.boundary.slice(0, idx);
    const after = this.boundary.slice(idx + 1);
    // rotate so fusedId's position is replaced in place
    this.boundary = [...before, u, ...siteVerts, w, ...after];
    void len;

    this._unbumpChildCounts(record);
    return true;
  }
}
