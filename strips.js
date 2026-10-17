// strips.js
// Woven-strip model derived from a WeaveGraph.
//
// The weave graph already IS a strip diagram: vertices are crossings, edges
// are strip spans between crossings, polygons and weave triangles are the
// holes. A strip runs straight through every crossing (enter on one edge,
// leave on the OPPOSITE edge in the vertex's cyclic order), so strips are
// traced purely combinatorially:
//   - every vertex has 1 face (apex, degree 2) or 3-4 faces (degree 4);
//     strips pass through degree-4 vertices and end only at apexes
//   - every edge belongs to exactly one strip
//   - every vertex carries exactly two strip passes (possibly both from the
//     same strip - a rare self-crossing)
//   - open strips run apex-to-apex; a closed mesh has only closed strips
//
// Over/under: every edge separates a polygon from a weave triangle, so an
// alternating weave always exists and is fixed by ONE global bit `sigma`.
// It is equivalent to a purely local rule (the Tait sign of knot theory):
// at each crossing, the strip that sweeps a POLYGON sector when rotated
// counterclockwise (seen from the ring's own orientation) sits on the +normal
// side when sigma = +1. Each crossing uses only its own ring orientation and
// its own normal, so nothing compares orientations between vertices - this
// stays correct on non-orientable meshes.
//
// Layers (kept separate so the eventual XPBD solver can take over the middle
// one without touching the others):
//   1. StripModel (buildStripModel): topology only - strips, their node
//      chains, crossings as contact pairs with fixed over/under. Rebuilt
//      only when the graph changes.
//   2. StripState (createStripState / deriveStripState): per-node position
//      and director in flat Float32Arrays - the solver's state layout. For
//      now it is DERIVED every frame from weave-graph vertex positions;
//      later the solver will own it.
//   3. Ribbon mesh (createRibbonLayout / writeRibbonMesh): render-only box
//      cross-section built from the state.

export const DEFAULT_STRIP_PARAMS = {
  width: 0.2, // in-surface width of a strip (edge length = 1)
  thickness: 0.02, // along the surface normal; also the over/under separation
  apexExtension: 0.3, // how far an open strip runs straight past its end apex
  samplesPerEdge: 8, // nodes per crossing-to-crossing span (topology-time)
};

const edgeKey = (a, b) => (a < b ? a + "_" + b : b + "_" + a);

// ---------------------------------------------------------------------------
// 1. Model
// ---------------------------------------------------------------------------

// Crossing ring conventions:
//  - degree 4 ('interior' / 'boundary'): ring = [r0,r1,r2,r3] in cyclic order,
//    rotated so the sector (r0,r1) is a POLYGON. The strip through {r0,r2}
//    is Tait-positive. Ring normal = (p[r2]-p[r0]) x (p[r3]-p[r1]).
//  - apex: ring = [x, y], the triangle's other two vertices (the only sector,
//    (x,y), is the weave triangle). Rotating the y-edge counterclockwise
//    sweeps the (future) polygon sector, so the strip through y is
//    Tait-positive. Ring normal = (p[x]-p[a]) x (p[y]-p[a]).
// Reversing a ring flips both its normal and its positive strip, so the
// arbitrary cyclic direction the graph hands back never matters.

function buildCrossings(graph) {
  const crossings = new Map();
  for (const { v, neighbors, firstSectorIsPolygon } of graph.degree4VertexNeighbors()) {
    const ring = firstSectorIsPolygon ? neighbors.slice() : [neighbors[1], neighbors[2], neighbors[3], neighbors[0]];
    crossings.set(v, {
      v,
      kind: graph.faceCount(v) === 4 ? "interior" : "boundary",
      ring,
      positive: [ring[0], ring[2]],
      passes: [],
    });
  }
  for (const tri of graph.triangles.values()) {
    for (const a of tri.verts) {
      if (graph.faceCount(a) !== 1) continue;
      const ring = tri.verts.filter((x) => x !== a);
      crossings.set(a, { v: a, kind: "apex", ring, positive: [ring[1]], passes: [] });
    }
  }
  return crossings;
}

// A vertex the graph can't give a proper ring for only exists in a
// malformed graph (see the fuse bug in CLAUDE.md); give it an empty
// placeholder so strips simply end there instead of throwing.
function crossingAt(crossings, v) {
  let c = crossings.get(v);
  if (!c) {
    c = { v, kind: "irregular", ring: [], positive: [], passes: [] };
    crossings.set(v, c);
  }
  return c;
}

function opposite(crossing, from) {
  const r = crossing.ring;
  if (r.length !== 4) return null;
  return r[(r.indexOf(from) + 2) % 4];
}

function traceStrips(graph, crossings) {
  const seen = new Set();
  const strips = [];
  const walk = (a, b) => {
    const out = [b];
    seen.add(edgeKey(a, b));
    for (;;) {
      const c = opposite(crossingAt(crossings, b), a);
      if (c == null) return { out, closed: false };
      if (seen.has(edgeKey(b, c))) return { out, closed: true };
      seen.add(edgeKey(b, c));
      out.push(c);
      a = b;
      b = c;
    }
  };
  for (const [a0, b0] of graph.edges()) {
    if (seen.has(edgeKey(a0, b0))) continue;
    const fwd = walk(a0, b0);
    let verts = [a0, ...fwd.out];
    if (fwd.closed) {
      verts.pop(); // last entry is a0 again
    } else {
      const back = walk(b0, a0);
      verts = [...back.out.slice(1).reverse(), ...verts];
    }
    strips.push({ verts, closed: fwd.closed });
  }
  return strips;
}

function stripEdgeKeys(strip) {
  const { verts, closed } = strip;
  const keys = [];
  const n = verts.length;
  for (let i = 0; i + 1 < n; i++) keys.push(edgeKey(verts[i], verts[i + 1]));
  if (closed) keys.push(edgeKey(verts[n - 1], verts[0]));
  return keys;
}

// Stable colors: each new strip inherits the color most of its edges had in
// the previous model (largest strips claim first, so when a placement merges
// strips the bigger part keeps its color); anything unclaimed gets a fresh
// color index.
function assignColors(strips, previous) {
  const prevEdgeColor = previous ? previous.edgeColor : new Map();
  let nextColor = previous ? previous.nextColor : 0;
  const order = strips.map((_, i) => i).sort((i, j) => strips[j].edgeKeys.length - strips[i].edgeKeys.length);
  const used = new Set();
  for (const si of order) {
    const tally = new Map();
    for (const k of strips[si].edgeKeys) {
      const c = prevEdgeColor.get(k);
      if (c !== undefined && !used.has(c)) tally.set(c, (tally.get(c) || 0) + 1);
    }
    let best = null;
    for (const [c, count] of tally) if (best === null || count > tally.get(best)) best = c;
    if (best === null) best = nextColor++;
    used.add(best);
    strips[si].color = best;
  }
  const edgeColor = new Map();
  for (const s of strips) for (const k of s.edgeKeys) edgeColor.set(k, s.color);
  return { edgeColor, nextColor };
}

// Build the strip model from the graph. Pass the previous model to keep
// colors (and sigma) stable across edits.
//
// StripModel {
//   sigma: +1 | -1,                       global over/under choice
//   strips: [{ id, verts, closed, color, edgeKeys,
//              nodeOffset, nodeCount,      slice of the global node arrays
//              vertexNode: [i -> node index within this strip] }],
//   crossings: Map<vertexId, { v, kind, ring, positive,
//              passes: [{ strip, index, node, tait }] x2 }>,
//   nodeCount,                            total nodes across all strips
//   edgeColor, nextColor,                 color inheritance bookkeeping
//   samplesPerEdge,
// }
// Node layout per strip (spe = samplesPerEdge, E = number of spans):
//   open:   [end-extension, v0, ...(spe-1 span nodes)..., v1, ..., vLast, end-extension]
//           -> E*spe + 3 nodes, vertex i at node 1 + i*spe
//   closed: [v0, ...span nodes..., v1, ..., (wraps to v0)]
//           -> E*spe nodes, vertex i at node i*spe
// A pass's `tait` is +1 for the Tait-positive strip; the pass sits on the
// +ring-normal side of its crossing iff sigma * tait > 0. The two passes of a
// crossing are the contact pair the solver will keep separated, in this order.
export function buildStripModel(graph, previous = null, samplesPerEdge = DEFAULT_STRIP_PARAMS.samplesPerEdge) {
  const crossings = buildCrossings(graph);
  const strips = traceStrips(graph, crossings);
  strips.forEach((s, i) => {
    s.id = i;
    s.edgeKeys = stripEdgeKeys(s);
  });
  const { edgeColor, nextColor } = assignColors(strips, previous);

  let nodeOffset = 0;
  for (const s of strips) {
    const spans = s.edgeKeys.length;
    const lead = s.closed ? 0 : 1;
    s.nodeCount = s.closed ? spans * samplesPerEdge : spans * samplesPerEdge + 3;
    s.nodeOffset = nodeOffset;
    s.vertexNode = s.verts.map((_, i) => lead + i * samplesPerEdge);
    nodeOffset += s.nodeCount;

    const n = s.verts.length;
    s.verts.forEach((v, i) => {
      const c = crossingAt(crossings, v);
      // either strip-neighbor works: at a degree-4 crossing both lie on the
      // same opposite pair; at an open end only one exists
      const nb = s.closed ? s.verts[(i + 1) % n] : i + 1 < n ? s.verts[i + 1] : s.verts[i - 1];
      const tait = c.positive.includes(nb) ? 1 : -1;
      c.passes.push({ strip: s.id, index: i, node: s.vertexNode[i], tait });
    });
  }

  return {
    sigma: previous ? previous.sigma : 1,
    strips,
    crossings,
    nodeCount: nodeOffset,
    edgeColor,
    nextColor,
    samplesPerEdge,
  };
}

// ---------------------------------------------------------------------------
// 2. State (the solver's layout; derived from the graph for now)
// ---------------------------------------------------------------------------

// StripState {
//   position: Float32Array(3 * nodeCount)   strip centerline nodes
//   director: Float32Array(3 * nodeCount)   unit ribbon normal per node,
//                                           perpendicular to the tangent,
//                                           continuous along each strip
// }
export function createStripState(model) {
  return {
    position: new Float32Array(3 * model.nodeCount),
    director: new Float32Array(3 * model.nodeCount),
  };
}

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function normalize(a, fallback = [0, 0, 1]) {
  const len = Math.hypot(a[0], a[1], a[2]);
  return len > 1e-12 ? [a[0] / len, a[1] / len, a[2] / len] : fallback.slice();
}

// Unit ring normal at each crossing, from current vertex positions.
export function crossingNormals(model, positions) {
  const P = (id) => {
    const p = positions.get(id);
    return p ? [p.x, p.y, p.z] : [0, 0, 0];
  };
  const normals = new Map();
  for (const c of model.crossings.values()) {
    const r = c.ring;
    const raw =
      r.length === 4
        ? cross(sub(P(r[2]), P(r[0])), sub(P(r[3]), P(r[1])))
        : r.length === 2
          ? cross(sub(P(r[0]), P(c.v)), sub(P(r[1]), P(c.v)))
          : [0, 0, 0];
    normals.set(c.v, normalize(raw));
  }
  return normals;
}

// Over/under displacement direction (unit) of one pass: which side of the
// surface this strip sits on at that crossing.
export function passSide(model, pass) {
  return model.sigma * pass.tait;
}

// Fill `state` from weave-graph vertex positions: Catmull-Rom centerline
// through the crossings, offset +-thickness/2 along each crossing's normal
// per the over/under rule with a cosine blend between (alternation makes the
// two ends' displacements opposite, so the offset passes through zero at
// mid-span), plus a straight extension past each open end.
export function deriveStripState(model, positions, params, state) {
  const spe = model.samplesPerEdge;
  const half = params.thickness / 2;
  const normals = crossingNormals(model, positions);
  const pos = state.position;
  const dir = state.director;

  for (const s of model.strips) {
    const { verts, closed } = s;
    const n = verts.length;
    const P = verts.map((v) => {
      const p = positions.get(v);
      return p ? [p.x, p.y, p.z] : [0, 0, 0];
    });
    // displacement per strip vertex, and the surface normal sign-aligned
    // along the strip (local pairwise alignment only - a closed strip on a
    // non-orientable mesh may come back flipped, which a ribbon tolerates)
    const disp = [];
    const surf = [];
    for (let i = 0; i < n; i++) {
      const c = model.crossings.get(verts[i]);
      const pass = c.passes.find((p) => p.strip === s.id && p.index === i);
      const N = normals.get(verts[i]);
      const side = passSide(model, pass);
      disp.push([N[0] * side, N[1] * side, N[2] * side]);
      const flip = i > 0 && dot(N, surf[i - 1]) < 0 ? -1 : 1;
      surf.push([N[0] * flip, N[1] * flip, N[2] * flip]);
    }
    const at = (arr, i) => (closed ? arr[(i + n) % n] : arr[i]);
    const ctrl = (i) => {
      if (closed || (i >= 0 && i < n)) return at(P, i);
      // reflect for one-sided end tangents
      return i < 0 ? sub([2 * P[0][0], 2 * P[0][1], 2 * P[0][2]], P[1]) : sub([2 * P[n - 1][0], 2 * P[n - 1][1], 2 * P[n - 1][2]], P[n - 2]);
    };

    const base = s.nodeOffset;
    const lead = closed ? 0 : 1;
    const setNode = (k, p, d) => {
      const o = 3 * (base + k);
      pos[o] = p[0];
      pos[o + 1] = p[1];
      pos[o + 2] = p[2];
      dir[o] = d[0];
      dir[o + 1] = d[1];
      dir[o + 2] = d[2];
    };

    const spans = s.edgeKeys.length;
    for (let i = 0; i < spans; i++) {
      const p0 = ctrl(i - 1);
      const p1 = ctrl(i);
      const p2 = ctrl(i + 1);
      const p3 = ctrl(i + 2);
      const d1 = at(disp, i);
      const d2 = at(disp, i + 1);
      const n1 = at(surf, i);
      let n2 = at(surf, i + 1);
      if (dot(n1, n2) < 0) n2 = [-n2[0], -n2[1], -n2[2]]; // closing span of a flipped loop
      const last = !closed && i === spans - 1;
      for (let k = 0; k < spe + (last ? 1 : 0); k++) {
        const t = k / spe;
        const t2 = t * t;
        const t3 = t2 * t;
        const h00 = 2 * t3 - 3 * t2 + 1;
        const h10 = t3 - 2 * t2 + t;
        const h01 = -2 * t3 + 3 * t2;
        const h11 = t3 - t2;
        const f = (1 - Math.cos(Math.PI * t)) / 2;
        const p = [0, 0, 0];
        const nrm = [0, 0, 0];
        for (let a = 0; a < 3; a++) {
          const m1 = (p2[a] - p0[a]) / 2;
          const m2 = (p3[a] - p1[a]) / 2;
          p[a] = h00 * p1[a] + h10 * m1 + h01 * p2[a] + h11 * m2 + half * ((1 - f) * d1[a] + f * d2[a]);
          nrm[a] = (1 - t) * n1[a] + t * n2[a];
        }
        setNode(lead + i * spe + k, p, normalize(nrm));
      }
    }

    if (!closed) {
      // straight extensions past the end apexes, along the end tangents
      const extend = (endNode, innerNode, target) => {
        const e = 3 * (base + endNode);
        const q = 3 * (base + innerNode);
        const away = normalize([pos[e] - pos[q], pos[e + 1] - pos[q + 1], pos[e + 2] - pos[q + 2]]);
        const x = params.apexExtension;
        setNode(target, [pos[e] + x * away[0], pos[e + 1] + x * away[1], pos[e + 2] + x * away[2]], [
          dir[e],
          dir[e + 1],
          dir[e + 2],
        ]);
      };
      extend(1, 2, 0);
      extend(s.nodeCount - 2, s.nodeCount - 3, s.nodeCount - 1);
    }

    // orthogonalize directors against the centerline tangent
    const m = s.nodeCount;
    for (let k = 0; k < m; k++) {
      const kp = closed ? (k - 1 + m) % m : Math.max(0, k - 1);
      const kn = closed ? (k + 1) % m : Math.min(m - 1, k + 1);
      const a = 3 * (base + kp);
      const b = 3 * (base + kn);
      const T = normalize([pos[b] - pos[a], pos[b + 1] - pos[a + 1], pos[b + 2] - pos[a + 2]], [1, 0, 0]);
      const o = 3 * (base + k);
      const d = [dir[o], dir[o + 1], dir[o + 2]];
      const proj = dot(d, T);
      const dd = normalize([d[0] - proj * T[0], d[1] - proj * T[1], d[2] - proj * T[2]]);
      dir[o] = dd[0];
      dir[o + 1] = dd[1];
      dir[o + 2] = dd[2];
    }
  }
  return state;
}

// ---------------------------------------------------------------------------
// 3. Ribbon mesh (render only)
// ---------------------------------------------------------------------------

// Golden-angle hue per color index, as linear-ish RGB in [0,1].
export function stripColorRGB(index) {
  const h = (index * 0.61803398875 + 0.08) % 1;
  const s = 0.65;
  const l = 0.55;
  const q = l < 0.5 ? l * (1 + s) : l + s - l * s;
  const p = 2 * l - q;
  const hue = (t) => {
    t = (t + 1) % 1;
    if (t < 1 / 6) return p + (q - p) * 6 * t;
    if (t < 1 / 2) return q;
    if (t < 2 / 3) return p + (q - p) * (2 / 3 - t) * 6;
    return p;
  };
  return [hue(h + 1 / 3), hue(h), hue(h - 1 / 3)];
}

// Box cross-section, 8 mesh vertices per ring (each of the 4 sides gets its
// own 2 vertices so it can carry a flat normal). Closed strips repeat their
// first node as a final ring so the seam closes; open strips get end caps.
// Corners go counterclockwise in the (B, N) plane: +B+N, -B+N, -B-N, +B-N.
export function createRibbonLayout(model) {
  const parts = [];
  let vertexCount = 0;
  let indexCount = 0;
  for (const s of model.strips) {
    const rings = s.closed ? s.nodeCount + 1 : s.nodeCount;
    const verts = rings * 8 + (s.closed ? 0 : 8);
    parts.push({ strip: s, rings, vertexOffset: vertexCount });
    vertexCount += verts;
    indexCount += (rings - 1) * 24 + (s.closed ? 0 : 12);
  }
  const index = new Uint32Array(indexCount);
  const color = new Float32Array(3 * vertexCount);
  let ii = 0;
  for (const part of parts) {
    const { strip, rings, vertexOffset: vo } = part;
    for (let r = 0; r + 1 < rings; r++) {
      for (let side = 0; side < 4; side++) {
        const a = vo + r * 8 + side * 2;
        const b = a + 1;
        const c = a + 9;
        const d = a + 8;
        index.set([a, c, b, a, d, c], ii);
        ii += 6;
      }
    }
    if (!strip.closed) {
      const cap = vo + rings * 8;
      index.set([cap, cap + 1, cap + 2, cap, cap + 2, cap + 3], ii); // start, normal -T
      index.set([cap + 4, cap + 6, cap + 5, cap + 4, cap + 7, cap + 6], ii + 6); // end, normal +T
      ii += 12;
    }
    const rgb = stripColorRGB(strip.color);
    const end = vo + rings * 8 + (strip.closed ? 0 : 8);
    for (let v = vo; v < end; v++) color.set(rgb, 3 * v);
  }
  return { parts, vertexCount, index, color };
}

export function writeRibbonMesh(model, layout, state, params, outPosition, outNormal) {
  const hw = params.width / 2;
  const ht = params.thickness / 2;
  const pos = state.position;
  const dir = state.director;
  for (const { strip: s, rings, vertexOffset: vo } of layout.parts) {
    const m = s.nodeCount;
    let prevB = null;
    const corners = [];
    for (let r = 0; r < rings; r++) {
      const k = r % m;
      const kp = s.closed ? (k - 1 + m) % m : Math.max(0, k - 1);
      const kn = s.closed ? (k + 1) % m : Math.min(m - 1, k + 1);
      const o = 3 * (s.nodeOffset + k);
      const a = 3 * (s.nodeOffset + kp);
      const b = 3 * (s.nodeOffset + kn);
      const C = [pos[o], pos[o + 1], pos[o + 2]];
      const T = normalize([pos[b] - pos[a], pos[b + 1] - pos[a + 1], pos[b + 2] - pos[a + 2]], [1, 0, 0]);
      let N = [dir[o], dir[o + 1], dir[o + 2]];
      let B = normalize(cross(T, N), [0, 1, 0]);
      // keep the frame continuous (matters only on the repeated seam ring of
      // a loop whose directors came back flipped)
      if (prevB && dot(B, prevB) < 0) {
        B = [-B[0], -B[1], -B[2]];
        N = [-N[0], -N[1], -N[2]];
      }
      prevB = B;
      const ring = [
        [1, 1],
        [-1, 1],
        [-1, -1],
        [1, -1],
      ].map(([sb, sn]) => [
        C[0] + sb * hw * B[0] + sn * ht * N[0],
        C[1] + sb * hw * B[1] + sn * ht * N[1],
        C[2] + sb * hw * B[2] + sn * ht * N[2],
      ]);
      const sideNormals = [N, [-B[0], -B[1], -B[2]], [-N[0], -N[1], -N[2]], B];
      for (let side = 0; side < 4; side++) {
        const v = vo + r * 8 + side * 2;
        outPosition.set(ring[side], 3 * v);
        outPosition.set(ring[(side + 1) % 4], 3 * v + 3);
        outNormal.set(sideNormals[side], 3 * v);
        outNormal.set(sideNormals[side], 3 * v + 3);
      }
      if (r === 0 || r === rings - 1) corners.push({ ring, T });
    }
    if (!s.closed) {
      const cap = vo + rings * 8;
      for (let e = 0; e < 2; e++) {
        const { ring, T } = corners[e];
        const n = e === 0 ? [-T[0], -T[1], -T[2]] : T;
        for (let c = 0; c < 4; c++) {
          outPosition.set(ring[c], 3 * (cap + e * 4 + c));
          outNormal.set(n, 3 * (cap + e * 4 + c));
        }
      }
    }
  }
}
