// physics.js
// A lightweight position-based-dynamics relaxer for the weave graph.
// - Edges behave like very stiff springs (rest length 1, clamped to +-10%).
// - Faces (both polygons and triangles) get an angle-regularization term that
//   nudges each vertex toward making its two incident face-angles equal to the
//   face's ideal regular-polygon interior angle, which drives the whole graph
//   toward a locally regular, roughly-planar-per-face embedding in 3D.
//
// Runs as an incremental relaxation you step every animation frame; positions
// are kept in a Float32Array-backed map keyed by vertex id so the caller can
// feed them straight into a THREE.BufferGeometry.

const REST_LENGTH = 1.0;
const MAX_STRAIN = 0.1; // +-10%
const MIN_LEN = REST_LENGTH * (1 - MAX_STRAIN);
const MAX_LEN = REST_LENGTH * (1 + MAX_STRAIN);

// Defaults for the tunable force weights, also used by the UI tuning panel
// (main.js) to populate slider ranges and power its "reset to defaults".
export const DEFAULT_PARAMS = {
  springStiffness: 0.35,
  angleStiffness: 0.4,
  planarStiffness: 0.5,
  // Off by default: dihedral fires on every edge with 2 adjacent faces,
  // including interior ones where a triangle borders 2 polygons - at that
  // point it's redundant with (and actively fights) the collinearity term,
  // which handles general mesh flatness cleanly on its own. Confirmed via
  // long-run tests on real hexagon+triangle topology: collinear+spring (+
  // angle, + planar) converges toward flat; adding dihedral back in
  // destabilizes it. Still available as a slider for boundary-triangle
  // pop-out if wanted.
  dihedralStiffness: 0,
  collinearStiffness: 0.4,
  damping: 0.02,
};

export class Physics {
  constructor() {
    this.positions = new Map(); // id -> THREE.Vector3-like {x,y,z} (plain objects)
    this.velocities = new Map(); // id -> {x,y,z}
    Object.assign(this, DEFAULT_PARAMS);
  }

  hasVertex(id) {
    return this.positions.has(id);
  }

  setPosition(id, x, y, z) {
    this.positions.set(id, { x, y, z });
    if (!this.velocities.has(id)) this.velocities.set(id, { x: 0, y: 0, z: 0 });
  }

  ensureVertex(id, fallback) {
    if (!this.positions.has(id)) {
      const p = fallback || { x: (Math.random() - 0.5) * 0.1, y: (Math.random() - 0.5) * 0.1, z: (Math.random() - 0.5) * 0.1 };
      this.positions.set(id, { x: p.x, y: p.y, z: p.z });
      this.velocities.set(id, { x: 0, y: 0, z: 0 });
    }
  }

  removeVertex(id) {
    this.positions.delete(id);
    this.velocities.delete(id);
  }

  // Place a brand-new vertex near the average of a set of neighbor ids
  // (used right after a topology edit so new verts don't start at the origin).
  seedNear(id, neighborIds) {
    let cx = 0,
      cy = 0,
      cz = 0,
      n = 0;
    for (const nb of neighborIds) {
      const p = this.positions.get(nb);
      if (p) {
        cx += p.x;
        cy += p.y;
        cz += p.z;
        n++;
      }
    }
    if (n === 0) {
      this.setPosition(id, (Math.random() - 0.5) * 0.1, (Math.random() - 0.5) * 0.1, (Math.random() - 0.5) * 0.1);
      return;
    }
    cx /= n;
    cy /= n;
    cz /= n;
    const jitter = 0.15;
    this.setPosition(
      id,
      cx + (Math.random() - 0.5) * jitter,
      cy + (Math.random() - 0.5) * jitter,
      cz + (Math.random() - 0.5) * jitter
    );
  }

  // Seed the fresh vertices created by placing an N-gon at a nesting site,
  // using the chord-ratio construction: given 3 consecutive polygon
  // vertices (A, B, C), the next vertex D satisfies D = A + R(n)*(C - B),
  // i.e. chord A->D is parallel to chord B->C, scaled by a fixed ratio
  // R(n) that depends only on the polygon's side count (R=1 for squares,
  // the golden ratio for pentagons, 2 for hexagons, etc. -
  // R(n) = sin(3*pi/n)/sin(pi/n) in closed form, verified against a true
  // regular n-gon's diagonal/edge ratio). Sliding the window
  // (A,B,C)->(B,C,D) and repeating generates the rest.
  //
  // `existingChain` is [u, ...siteVerts, w] - the nesting site's flanking
  // apexes plus all of its existing boundary vertices, in polygon-loop
  // order, exactly as WeaveGraph.placePolygon used them. These are real,
  // already-placed mesh points, so the angles between them are whatever
  // the existing mesh already has - not necessarily the new polygon's own
  // ideal interior angle. This construction only guarantees each new chord
  // is parallel to the one two-steps-back and scaled by R(n); the result
  // is "affine regular" (same parallel-chord structure as a true regular
  // n-gon, but sheared by the real starting angles), not an exact regular
  // polygon. That's fine as a seed: physics relaxation (angle-regularity +
  // planarity terms) takes it the rest of the way once released.
  //
  // midVerts are the fresh vertices to fill in after existingChain (may be
  // empty - the join-edge and fuse placement cases never need any).
  // apexVerts are the fresh triangle apexes, one per edge of the chain
  // [w, ...midVerts, u] (also may be a single entry for the join-edge case,
  // which bridges w->u directly).
  seedPlacement(existingChain, midVerts, apexVerts) {
    const knownPts = existingChain.map((id) => this.positions.get(id));
    if (knownPts.some((p) => !p)) {
      for (const id of [...midVerts, ...apexVerts]) this.seedNear(id, existingChain);
      return;
    }

    const n = existingChain.length + midVerts.length; // total sides of the new polygon
    const ratio = n === 3 ? 0 : Math.sin((3 * Math.PI) / n) / Math.sin(Math.PI / n);

    // Slide the window over the chain's own last 3 known points to seed
    // each fresh mid-vertex in turn.
    const loopPts = knownPts.slice();
    let A = loopPts[loopPts.length - 3];
    let B = loopPts[loopPts.length - 2];
    let C = loopPts[loopPts.length - 1];
    for (let i = 0; i < midVerts.length; i++) {
      const D = {
        x: A.x + ratio * (C.x - B.x),
        y: A.y + ratio * (C.y - B.y),
        z: A.z + ratio * (C.z - B.z),
      };
      const id = midVerts[i];
      if (id != null && !this.positions.has(id)) this.setPosition(id, D.x, D.y, D.z);
      loopPts.push(D);
      A = B;
      B = C;
      C = D;
    }

    // Local surface normal at this corner (from the chain's first 3 known
    // points, real mesh vertices), used only to give triangle apexes a
    // small fold-direction jitter so the dihedral term has something to
    // break flat-saddle symmetry with - same reasoning as the seed
    // hexagon's own apex jitter.
    const [pu, p2, p3] = knownPts;
    let nx = (pu.y - p2.y) * (p3.z - p2.z) - (pu.z - p2.z) * (p3.y - p2.y);
    let ny = (pu.z - p2.z) * (p3.x - p2.x) - (pu.x - p2.x) * (p3.z - p2.z);
    let nz = (pu.x - p2.x) * (p3.y - p2.y) - (pu.y - p2.y) * (p3.x - p2.x);
    const nlen = Math.hypot(nx, ny, nz) || 1;
    nx /= nlen;
    ny /= nlen;
    nz /= nlen;

    // centroid of the new polygon loop, used to orient each triangle apex
    // "outward" from the cell.
    let cx = 0,
      cy = 0,
      cz = 0;
    for (const p of loopPts) {
      cx += p.x;
      cy += p.y;
      cz += p.z;
    }
    cx /= loopPts.length;
    cy /= loopPts.length;
    cz /= loopPts.length;

    // Triangle apexes: one per edge along the chain [w, mid_1, ..., mid_{n-3}, u]
    // (for the join-edge case, midVerts is empty and this is just [w, u]).
    // w is the last vertex of the known chain; the mid-verts (if any) were
    // appended to loopPts right after it.
    const w = loopPts[existingChain.length - 1];
    const u = loopPts[0];
    const chain = [w, ...loopPts.slice(existingChain.length), u];
    const jitter = 0.02;
    for (let i = 0; i < apexVerts.length; i++) {
      const pa = chain[i];
      const pb = chain[i + 1];
      const apexId = apexVerts[i];
      if (apexId == null || this.positions.has(apexId) || !pa || !pb) continue;
      const mx = (pa.x + pb.x) / 2;
      const my = (pa.y + pb.y) / 2;
      const mz = (pa.z + pb.z) / 2;
      // outward-from-centroid direction at this edge's midpoint, in-plane
      let dx = mx - cx,
        dy = my - cy,
        dz = mz - cz;
      const dlen = Math.hypot(dx, dy, dz) || 1;
      dx /= dlen;
      dy /= dlen;
      dz /= dlen;
      // equilateral-triangle apex height for the actual base edge length
      const edgeLen = Math.hypot(pb.x - pa.x, pb.y - pa.y, pb.z - pa.z) || 1;
      const height = (edgeLen * Math.sqrt(3)) / 2;
      const foldSign = i % 2 === 0 ? -1 : 1; // alternate to break flat-saddle symmetry
      const fx = mx + dx * height + foldSign * jitter * nx;
      const fy = my + dy * height + foldSign * jitter * ny;
      const fz = mz + dz * height + foldSign * jitter * nz;
      this.setPosition(apexId, fx, fy, fz);
    }
  }

  // Advance the simulation by one step given the current graph topology.
  // `edges`: array of [a,b] vertex id pairs.
  // `faces`: array of vertex-id loops (triangles and polygons).
  // `edgeFaces`: array of { a, b, faceA, faceB } for every edge that has two
  // adjacent faces (i.e. all edges except boundary triangles' two open/free
  // legs), used to drive dihedral angles toward 180 degrees.
  // `degree4Vertices`: array of { v, neighbors: [n0,n1,n2,n3] } for every
  // degree-4 vertex (both fully-interior ones and boundary "nesting"
  // vertices, which already have all 4 edges even before their last face
  // slot fills in), neighbors in cyclic order - used to drive each pair of
  // OPPOSITE edges (n0/n2 and n1/n3) toward collinearity through v, which
  // works alongside the dihedral term to keep the mesh flat.
  step(edges, faces, edgeFaces, degree4Vertices) {
    const pos = this.positions;
    const forces = new Map();
    for (const id of pos.keys()) forces.set(id, { x: 0, y: 0, z: 0 });

    // --- spring term: stiff edges clamped to +-10% of rest length ---
    for (const [a, b] of edges) {
      const pa = pos.get(a);
      const pb = pos.get(b);
      if (!pa || !pb) continue;
      let dx = pb.x - pa.x;
      let dy = pb.y - pa.y;
      let dz = pb.z - pa.z;
      let dist = Math.hypot(dx, dy, dz);
      if (dist < 1e-6) {
        dx = (Math.random() - 0.5) * 1e-3;
        dy = (Math.random() - 0.5) * 1e-3;
        dz = (Math.random() - 0.5) * 1e-3;
        dist = Math.hypot(dx, dy, dz);
      }
      const nx = dx / dist;
      const ny = dy / dist;
      const nz = dz / dist;

      // target: pull hard toward REST_LENGTH; additionally hard-clamp handled after.
      const stretch = dist - REST_LENGTH;
      const f = stretch * this.springStiffness;
      const fx = nx * f;
      const fy = ny * f;
      const fz = nz * f;

      const fa = forces.get(a);
      fa.x += fx;
      fa.y += fy;
      fa.z += fz;
      const fb = forces.get(b);
      fb.x -= fx;
      fb.y -= fy;
      fb.z -= fz;
    }

    // --- per-face regularization: two independent sub-terms per face so
    // they drive toward one consistent target instead of fighting each
    // other. (1) planarity: pull vertices into the face's own best-fit
    // plane. (2) interior-angle equalization (below): rotate vertices
    // in-plane so every interior angle matches the ideal regular-polygon
    // angle. Together these make each face flatten and become regular. ---
    for (const face of faces) {
      const n = face.length;
      if (n < 3) continue;
      const pts = face.map((id) => pos.get(id)).filter(Boolean);
      if (pts.length !== n) continue;

      // centroid
      let cx = 0,
        cy = 0,
        cz = 0;
      for (const p of pts) {
        cx += p.x;
        cy += p.y;
        cz += p.z;
      }
      cx /= n;
      cy /= n;
      cz /= n;

      // best-fit normal via Newell's method
      let nx = 0,
        ny = 0,
        nz = 0;
      for (let i = 0; i < n; i++) {
        const p0 = pts[i];
        const p1 = pts[(i + 1) % n];
        nx += (p0.y - cy) * (p1.z - cz) - (p0.z - cz) * (p1.y - cy);
        ny += (p0.z - cz) * (p1.x - cx) - (p0.x - cx) * (p1.z - cz);
        nz += (p0.x - cx) * (p1.y - cy) - (p0.y - cy) * (p1.x - cx);
      }
      const nlen = Math.hypot(nx, ny, nz) || 1;
      nx /= nlen;
      ny /= nlen;
      nz /= nlen;

      // build an in-plane basis (u, v)
      let ux, uy, uz;
      if (Math.abs(nx) < 0.9) {
        ux = 1;
        uy = 0;
        uz = 0;
      } else {
        ux = 0;
        uy = 1;
        uz = 0;
      }
      // u = normalize(u - (u.n)n)
      const dot = ux * nx + uy * ny + uz * nz;
      ux -= dot * nx;
      uy -= dot * ny;
      uz -= dot * nz;
      const ulen = Math.hypot(ux, uy, uz) || 1;
      ux /= ulen;
      uy /= ulen;
      uz /= ulen;
      // v = n x u
      const vx = ny * uz - nz * uy;
      const vy = nz * ux - nx * uz;
      const vz = nx * uy - ny * ux;

      // Planarity term: pull each vertex's out-of-plane component (relative
      // to the face's own best-fit centroid/normal) to zero, leaving its
      // in-plane position untouched. In-plane shape (equal angles, equal
      // edge lengths) is handled entirely by the interior-angle term below;
      // keeping this term purely out-of-plane avoids the two fighting over
      // slightly different targets, which is what kept the seed hexagon
      // from ever settling into a fully regular shape.
      const raw = new Array(n);
      for (let i = 0; i < n; i++) {
        const p = pts[i];
        const rx = p.x - cx;
        const ry = p.y - cy;
        const rz = p.z - cz;
        const along = rx * nx + ry * ny + rz * nz; // out-of-plane distance
        raw[i] = {
          x: -along * nx * this.planarStiffness,
          y: -along * ny * this.planarStiffness,
          z: -along * nz * this.planarStiffness,
        };
      }

      // Strip net translation (mean force).
      let fcx = 0,
        fcy = 0,
        fcz = 0;
      for (const r of raw) {
        fcx += r.x;
        fcy += r.y;
        fcz += r.z;
      }
      fcx /= n;
      fcy /= n;
      fcz /= n;
      for (const r of raw) {
        r.x -= fcx;
        r.y -= fcy;
        r.z -= fcz;
      }

      // Strip net rotation (torque about centroid, projected out via a
      // first-order angular-velocity correction: omega = I^-1 * L, using
      // the scalar/isotropic approximation I ~= sum(|r_i|^2) since face
      // vertices are roughly equidistant from the centroid by construction).
      let lx = 0,
        ly = 0,
        lz = 0,
        inertia = 0;
      const rel = new Array(n);
      for (let i = 0; i < n; i++) {
        const rx = pts[i].x - cx;
        const ry = pts[i].y - cy;
        const rz = pts[i].z - cz;
        rel[i] = { x: rx, y: ry, z: rz };
        lx += ry * raw[i].z - rz * raw[i].y;
        ly += rz * raw[i].x - rx * raw[i].z;
        lz += rx * raw[i].y - ry * raw[i].x;
        inertia += rx * rx + ry * ry + rz * rz;
      }
      if (inertia > 1e-9) {
        const wx = lx / inertia;
        const wy = ly / inertia;
        const wz = lz / inertia;
        for (let i = 0; i < n; i++) {
          // subtract omega x r_i (the rigid-rotation component at this vertex)
          raw[i].x -= wy * rel[i].z - wz * rel[i].y;
          raw[i].y -= wz * rel[i].x - wx * rel[i].z;
          raw[i].z -= wx * rel[i].y - wy * rel[i].x;
        }
      }

      for (let i = 0; i < n; i++) {
        const id = face[i];
        const f = forces.get(id);
        if (!f) continue;
        f.x += raw[i].x;
        f.y += raw[i].y;
        f.z += raw[i].z;
      }

      // --- explicit interior-angle equalization: the radius/angular-spacing
      // pull above equalizes a regular polygon's angles only indirectly (as
      // a byproduct of even radius+spacing), so a shape can drift into an
      // elongated form that still looks "evenly spaced" from the centroid
      // while its actual vertex-to-vertex interior angles differ. Directly
      // measure each vertex's interior angle (between its two edges to
      // face-neighbors) and rotate those edges, in-plane, to push the angle
      // toward the ideal regular-polygon interior angle. ---
      const idealInterior = ((n - 2) * Math.PI) / n;
      const angleRaw = new Array(n).fill(null).map(() => ({ x: 0, y: 0, z: 0 }));
      for (let i = 0; i < n; i++) {
        const prev = pts[(i - 1 + n) % n];
        const cur = pts[i];
        const next = pts[(i + 1) % n];
        let e1x = prev.x - cur.x,
          e1y = prev.y - cur.y,
          e1z = prev.z - cur.z;
        let e2x = next.x - cur.x,
          e2y = next.y - cur.y,
          e2z = next.z - cur.z;
        const l1 = Math.hypot(e1x, e1y, e1z) || 1;
        const l2 = Math.hypot(e2x, e2y, e2z) || 1;
        e1x /= l1;
        e1y /= l1;
        e1z /= l1;
        e2x /= l2;
        e2y /= l2;
        e2z /= l2;
        const cosA = Math.max(-1, Math.min(1, e1x * e2x + e1y * e2y + e1z * e2z));
        const curAngle = Math.acos(cosA);
        const err = idealInterior - curAngle; // >0 means angle too small, need to widen it

        // In-plane direction perpendicular to each edge, pointing away from
        // the other edge (i.e. the direction that widens the angle at `cur`
        // when prev/next move along it). Use the face normal to get an
        // in-plane perpendicular via cross product.
        const perp1x = ny * e1z - nz * e1y,
          perp1y = nz * e1x - nx * e1z,
          perp1z = nx * e1y - ny * e1x;
        const perp2x = ny * e2z - nz * e2y,
          perp2y = nz * e2x - nx * e2z,
          perp2z = nx * e2y - ny * e2x;
        // orient perp1 to point away from e2 (widen angle when err>0 moves prev this way)
        const s1 = perp1x * e2x + perp1y * e2y + perp1z * e2z > 0 ? -1 : 1;
        const s2 = perp2x * e1x + perp2y * e1y + perp2z * e1z > 0 ? -1 : 1;

        const k = this.angleStiffness * err;
        const prevIdx = (i - 1 + n) % n;
        const nextIdx = (i + 1) % n;
        angleRaw[prevIdx].x += s1 * perp1x * k;
        angleRaw[prevIdx].y += s1 * perp1y * k;
        angleRaw[prevIdx].z += s1 * perp1z * k;
        angleRaw[nextIdx].x += s2 * perp2x * k;
        angleRaw[nextIdx].y += s2 * perp2y * k;
        angleRaw[nextIdx].z += s2 * perp2z * k;
      }
      for (let i = 0; i < n; i++) {
        const id = face[i];
        const f = forces.get(id);
        if (!f) continue;
        f.x += angleRaw[i].x;
        f.y += angleRaw[i].y;
        f.z += angleRaw[i].z;
      }
    }

    // --- dihedral term: for every edge that has two adjacent faces (this
    // covers essentially the whole graph, including both edges where a
    // boundary triangle meets its polygon - it's only the triangle's two
    // open/free legs, at the mesh's actual 1-D boundary, that are excluded,
    // since there's no second face there to fold against), push the two
    // faces to a dihedral angle of ~180 degrees (as flat/convex as
    // possible). This is what makes boundary triangles pop outward into 3D
    // relative to their neighboring polygon instead of laying flat.
    //
    // Only the triangle's own hinge vertex (its apex) is ever moved by this
    // term - never the polygon's vertices on the other side. A triangle has
    // no in-plane shape to protect (3 points are always "regular"), but a
    // polygon does, and this term used to also pull on the polygon's own
    // flanking vertices, which directly fought the angle-regularity term
    // and coupled "dihedral fold" strength to polygon regularity - moving
    // the dihedral slider changed how round the hexagon looked, which
    // should never happen since the two are conceptually unrelated. ---
    if (edgeFaces) {
      const blend = this.dihedralStiffness;
      const hingeVertexOf = (verts, a, b) => {
        // The face vertex(es) adjacent to edge (a,b) within this face's own
        // cyclic loop, excluding a and b themselves - for a triangle this is
        // exactly its apex; for an n-gon it's the two neighbors of the edge.
        //
        // MUST determine which of a/b comes first in CYCLIC order (not just
        // by raw array index via min/max) - a face's edge (a,b) can wrap
        // around the array boundary (e.g. verts=[1,2,8] with edge (1,8): 1
        // is at index 0, 8 is at index 2, but they're adjacent via the
        // wrap-around 8->1, not via indices 0->2 "forward"). Using
        // min/max(ia,ib) here previously computed the WRONG neighbors
        // whenever an edge wrapped, silently returning a/b themselves as
        // the "hinge" (filtered out as empty) - which meant this term
        // simply did nothing for any edge that happened to wrap, instead of
        // applying its dihedral force. Once a triangle has more than one
        // polygon-adjacent edge (which happens naturally as a mesh grows -
        // e.g. the seed hexagon's own flanking triangles once a neighbor
        // polygon is placed beside them), at least one of its edges is
        // essentially guaranteed to wrap, so this silently broke dihedral
        // flattening across exactly the edges that most needed it,
        // producing the accordion-fold/curling behavior on any extended mesh.
        const n = verts.length;
        const ia = verts.indexOf(a);
        const ib = verts.indexOf(b);
        if (ia === -1 || ib === -1) return [];
        let loIdx, hiIdx;
        if ((ia + 1) % n === ib) {
          loIdx = ia;
          hiIdx = ib;
        } else if ((ib + 1) % n === ia) {
          loIdx = ib;
          hiIdx = ia;
        } else {
          return []; // a, b aren't actually adjacent in this face's loop
        }
        const result = new Set();
        const before = verts[(loIdx - 1 + n) % n];
        const after = verts[(hiIdx + 1) % n];
        if (before !== a && before !== b) result.add(before);
        if (after !== a && after !== b) result.add(after);
        return [...result];
      };
      // Decompose a hinge vertex's position relative to the edge line into
      // (alongEdge, perpDir, perpDist) so we can build an explicit *target
      // position* for a 180 degree dihedral (reflected to the opposite side
      // of the edge, in-plane with the other face's hinge) rather than
      // applying a constant-magnitude directional force - a directional-only
      // force doesn't shrink near the target and overshoots/spirals under
      // velocity integration.
      const decompose = (id, mx, my, mz, ex, ey, ez) => {
        const p = pos.get(id);
        if (!p) return null;
        let dx = p.x - mx;
        let dy = p.y - my;
        let dz = p.z - mz;
        const along = dx * ex + dy * ey + dz * ez;
        dx -= along * ex;
        dy -= along * ey;
        dz -= along * ez;
        const dist = Math.hypot(dx, dy, dz);
        if (dist < 1e-6) return null;
        return { x: dx / dist, y: dy / dist, z: dz / dist, dist };
      };

      for (const { a, b, faceA, faceB } of edgeFaces) {
        // Only the triangle side of an edge moves for this term - a
        // triangle has no in-plane shape to protect (3 points are always
        // "regular" up to reflection), but a polygon does, and pulling on
        // its own vertices here directly fought the angle-regularity term,
        // coupling "dihedral fold" and "polygon regularity" when they
        // should be fully independent. If an edge somehow isn't exactly one
        // triangle + one polygon (shouldn't happen given this graph's
        // invariants), skip it rather than guess.
        let triFace, otherFace;
        if (faceA.length === 3 && faceB.length !== 3) {
          triFace = faceA;
          otherFace = faceB;
        } else if (faceB.length === 3 && faceA.length !== 3) {
          triFace = faceB;
          otherFace = faceA;
        } else {
          continue;
        }

        const pa = pos.get(a);
        const pb = pos.get(b);
        if (!pa || !pb) continue;
        let ex = pb.x - pa.x;
        let ey = pb.y - pa.y;
        let ez = pb.z - pa.z;
        const elen = Math.hypot(ex, ey, ez);
        if (elen < 1e-6) continue;
        ex /= elen;
        ey /= elen;
        ez /= elen;
        const mx = (pa.x + pb.x) / 2;
        const my = (pa.y + pb.y) / 2;
        const mz = (pa.z + pb.z) / 2;

        const triHinges = hingeVertexOf(triFace, a, b); // the triangle's apex
        const otherHinges = hingeVertexOf(otherFace, a, b); // polygon's flanking verts (read-only reference)
        if (triHinges.length === 0 || otherHinges.length === 0) continue;

        // Reference direction: average of the polygon's flanking-vertex
        // perpendicular directions (read-only - never force-applied), used
        // only to know which way is "away from the polygon".
        let refx = 0,
          refy = 0,
          refz = 0,
          refCount = 0;
        for (const ob of otherHinges) {
          const dec = decompose(ob, mx, my, mz, ex, ey, ez);
          if (!dec) continue;
          refx += dec.x;
          refy += dec.y;
          refz += dec.z;
          refCount++;
        }
        if (refCount === 0) continue;
        const reflen = Math.hypot(refx, refy, refz) || 1;
        refx /= reflen;
        refy /= reflen;
        refz /= reflen;

        for (const ht of triHinges) {
          const decT = decompose(ht, mx, my, mz, ex, ey, ez);
          if (!decT) continue;
          const pht = pos.get(ht);
          const f = forces.get(ht);
          if (!f || !pht) continue;
          // Target: on the opposite side of the edge from the polygon,
          // at the apex's own current distance from the edge line.
          const tx = mx - refx * decT.dist;
          const ty = my - refy * decT.dist;
          const tz = mz - refz * decT.dist;
          f.x += (tx - pht.x) * blend;
          f.y += (ty - pht.y) * blend;
          f.z += (tz - pht.z) * blend;
        }
      }
    }

    // --- collinearity term: at every degree-4 vertex v (fully-interior
    // ones AND boundary "nesting" vertices - the latter already have all 4
    // edges even though one angular sector is still an open gap awaiting
    // its next face), each pair of OPPOSITE neighbors (2 apart in the
    // cyclic order around v, i.e. n0/n2 and n1/n3) should end up collinear
    // through v - that's the direct statement of "this vertex isn't a
    // kink" along each of the two edge-pairs crossing it, and works
    // alongside the dihedral term (which reasons per-edge, about a
    // triangle's apex vs. a neighboring polygon) to pull the whole local
    // vertex star flat, including cases the dihedral term alone doesn't
    // fully resolve (a triangle bordering two different polygons has its
    // dihedral targets computed independently per edge; this term instead
    // directly constrains the vertex where they meet). v itself is left
    // untouched - only n0/n2 and n1/n3 move toward each other's
    // collinear-reflection target, applied symmetrically like the
    // dihedral term's apex-only convention. ---
    if (degree4Vertices) {
      const cBlend = this.collinearStiffness;
      // Neighbors are pulled toward collinearity at HALF strength, and v
      // itself is also corrected (projected onto the current line through
      // its neighbors) at full strength - v is NOT a fixed anchor here.
      // Earlier this term only ever moved a/b to match v's current
      // position, treating v as ground truth. But v is itself a neighbor
      // in OTHER vertices' collinearity constraints (every interior vertex
      // sits at the shared corner of several), so if v had drifted
      // slightly off-plane for any reason, this term would faithfully drag
      // its neighbors to match that error instead of correcting it -
      // confirmed directly with a minimal synthetic test: a single
      // isolated constraint is stable, but two constraints sharing a
      // vertex reliably amplified an initial small perturbation into
      // significant curling, because nothing ever pulled the shared vertex
      // itself back toward consensus with its own neighbors. Splitting the
      // correction three ways (v projected onto line a-b, a/b pulled
      // toward v at reduced strength) makes the term genuinely
      // self-correcting instead of a one-way error channel. ---
      const neighborWeight = 0.5;
      for (const { v, neighbors } of degree4Vertices) {
        const pv = pos.get(v);
        const fv = forces.get(v);
        if (!pv || !fv) continue;
        const pairs = [
          [neighbors[0], neighbors[2]],
          [neighbors[1], neighbors[3]],
        ];
        for (const [ida, idb] of pairs) {
          const pa = pos.get(ida);
          const pb = pos.get(idb);
          const fa = forces.get(ida);
          const fb = forces.get(idb);
          if (!pa || !pb || !fa || !fb) continue;

          // Target for v: its projection onto the current line through a
          // and b (the straight-line "consensus" position).
          let abx = pb.x - pa.x,
            aby = pb.y - pa.y,
            abz = pb.z - pa.z;
          const ablen = Math.hypot(abx, aby, abz) || 1;
          abx /= ablen;
          aby /= ablen;
          abz /= ablen;
          const t = (pv.x - pa.x) * abx + (pv.y - pa.y) * aby + (pv.z - pa.z) * abz;
          const projx = pa.x + abx * t;
          const projy = pa.y + aby * t;
          const projz = pa.z + abz * t;
          fv.x += (projx - pv.x) * cBlend;
          fv.y += (projy - pv.y) * cBlend;
          fv.z += (projz - pv.z) * cBlend;

          // Targets for a,b: reflect-through-v as before (preserving each
          // neighbor's own current distance from v, leaving edge length to
          // the spring term), applied at reduced strength since v is now
          // also correcting itself toward the same goal.
          const da = Math.hypot(pa.x - pv.x, pa.y - pv.y, pa.z - pv.z) || 1;
          const db = Math.hypot(pb.x - pv.x, pb.y - pv.y, pb.z - pv.z) || 1;
          let dax = (pa.x - pv.x) / da,
            day = (pa.y - pv.y) / da,
            daz = (pa.z - pv.z) / da;
          let dbx = (pb.x - pv.x) / db,
            dby = (pb.y - pv.y) / db,
            dbz = (pb.z - pv.z) / db;
          let dirx = dax - dbx,
            diry = day - dby,
            dirz = daz - dbz;
          const dirLen = Math.hypot(dirx, diry, dirz) || 1;
          dirx /= dirLen;
          diry /= dirLen;
          dirz /= dirLen;

          const targetA = { x: pv.x + dirx * da, y: pv.y + diry * da, z: pv.z + dirz * da };
          const targetB = { x: pv.x - dirx * db, y: pv.y - diry * db, z: pv.z - dirz * db };

          fa.x += (targetA.x - pa.x) * cBlend * neighborWeight;
          fa.y += (targetA.y - pa.y) * cBlend * neighborWeight;
          fa.z += (targetA.z - pa.z) * cBlend * neighborWeight;
          fb.x += (targetB.x - pb.x) * cBlend * neighborWeight;
          fb.y += (targetB.y - pb.y) * cBlend * neighborWeight;
          fb.z += (targetB.z - pb.z) * cBlend * neighborWeight;
        }
      }
    }

    // --- global momentum removal: none of the individual terms above are
    // guaranteed to be perfectly zero-sum (the angle-regularization term's
    // torque cancellation uses a scalar-inertia approximation that's only
    // exact for perfectly regular faces, and the dihedral term only acts on
    // hinge vertices, not full edge pairs). Any residual net force or net
    // torque compounds frame over frame into the whole graph visibly
    // translating ("swimming") or spinning, even after per-term fixes. Strip
    // both explicitly here, once, over the whole vertex set, so the graph
    // can only ever deform in place - never drift or spin as a rigid body -
    // regardless of approximation error in any single term. ---
    {
      const ids = Array.from(pos.keys());
      const m = ids.length;
      if (m > 0) {
        // centroid of the whole graph
        let cx = 0,
          cy = 0,
          cz = 0;
        for (const id of ids) {
          const p = pos.get(id);
          cx += p.x;
          cy += p.y;
          cz += p.z;
        }
        cx /= m;
        cy /= m;
        cz /= m;

        // net force -> strip mean (net translation)
        let fcx = 0,
          fcy = 0,
          fcz = 0;
        for (const id of ids) {
          const f = forces.get(id);
          fcx += f.x;
          fcy += f.y;
          fcz += f.z;
        }
        fcx /= m;
        fcy /= m;
        fcz /= m;

        // net torque about centroid -> compute exact 3x3 inertia tensor and
        // solve I * omega = L for the angular-velocity correction, then
        // subtract omega x r_i from each vertex's force.
        let lx = 0,
          ly = 0,
          lz = 0;
        let Ixx = 0,
          Iyy = 0,
          Izz = 0,
          Ixy = 0,
          Ixz = 0,
          Iyz = 0;
        const rel = new Map();
        for (const id of ids) {
          const p = pos.get(id);
          const f = forces.get(id);
          const rx = p.x - cx;
          const ry = p.y - cy;
          const rz = p.z - cz;
          rel.set(id, { x: rx, y: ry, z: rz });
          const fx = f.x - fcx;
          const fy = f.y - fcy;
          const fz = f.z - fcz;
          lx += ry * fz - rz * fy;
          ly += rz * fx - rx * fz;
          lz += rx * fy - ry * fx;
          Ixx += ry * ry + rz * rz;
          Iyy += rx * rx + rz * rz;
          Izz += rx * rx + ry * ry;
          Ixy -= rx * ry;
          Ixz -= rx * rz;
          Iyz -= ry * rz;
        }

        // solve the symmetric 3x3 system [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz] * omega = L
        const det =
          Ixx * (Iyy * Izz - Iyz * Iyz) - Ixy * (Ixy * Izz - Iyz * Ixz) + Ixz * (Ixy * Iyz - Iyy * Ixz);
        let wx = 0,
          wy = 0,
          wz = 0;
        if (Math.abs(det) > 1e-9) {
          const invDet = 1 / det;
          wx =
            (lx * (Iyy * Izz - Iyz * Iyz) - Ixy * (ly * Izz - Iyz * lz) + Ixz * (ly * Iyz - Iyy * lz)) *
            invDet;
          wy =
            (Ixx * (ly * Izz - Iyz * lz) - lx * (Ixy * Izz - Iyz * Ixz) + Ixz * (Ixy * lz - ly * Ixz)) *
            invDet;
          wz =
            (Ixx * (Iyy * lz - ly * Iyz) - Ixy * (Ixy * lz - ly * Ixz) + lx * (Ixy * Iyz - Iyy * Ixz)) *
            invDet;
        }

        for (const id of ids) {
          const f = forces.get(id);
          const r = rel.get(id);
          f.x -= fcx + (wy * r.z - wz * r.y);
          f.y -= fcy + (wz * r.x - wx * r.z);
          f.z -= fcz + (wx * r.y - wy * r.x);
        }
      }
    }

    // --- integrate ---
    const dt = 1;
    for (const [id, f] of forces) {
      const v = this.velocities.get(id);
      const p = pos.get(id);
      v.x = (v.x + f.x * dt) * this.damping;
      v.y = (v.y + f.y * dt) * this.damping;
      v.z = (v.z + f.z * dt) * this.damping;
      p.x += v.x * dt;
      p.y += v.y * dt;
      p.z += v.z * dt;
    }

    // --- hard length clamp pass (keeps the +-10% invariant even under
    // aggressive angle forces): project each edge back within [MIN_LEN, MAX_LEN] ---
    for (let pass = 0; pass < 2; pass++) {
      for (const [a, b] of edges) {
        const pa = pos.get(a);
        const pb = pos.get(b);
        if (!pa || !pb) continue;
        let dx = pb.x - pa.x;
        let dy = pb.y - pa.y;
        let dz = pb.z - pa.z;
        let dist = Math.hypot(dx, dy, dz);
        if (dist < 1e-6) continue;
        let target = null;
        if (dist > MAX_LEN) target = MAX_LEN;
        else if (dist < MIN_LEN) target = MIN_LEN;
        if (target !== null) {
          const diff = (dist - target) / 2;
          const nx = dx / dist;
          const ny = dy / dist;
          const nz = dz / dist;
          pa.x += nx * diff;
          pa.y += ny * diff;
          pa.z += nz * diff;
          pb.x -= nx * diff;
          pb.y -= ny * diff;
          pb.z -= nz * diff;

          // Cancel the along-edge component of each vertex's velocity so the
          // hard correction we just applied isn't immediately re-violated by
          // stale velocity next frame (this is what caused the buzzing).
          const va = this.velocities.get(a);
          const vb = this.velocities.get(b);
          if (va) {
            const vaAlong = va.x * nx + va.y * ny + va.z * nz;
            if (vaAlong < 0) {
              va.x -= vaAlong * nx;
              va.y -= vaAlong * ny;
              va.z -= vaAlong * nz;
            }
          }
          if (vb) {
            const vbAlong = vb.x * nx + vb.y * ny + vb.z * nz;
            if (vbAlong > 0) {
              vb.x -= vbAlong * nx;
              vb.y -= vbAlong * ny;
              vb.z -= vbAlong * nz;
            }
          }
        }
      }
    }
  }

  // Remove any tracked vertices that are no longer present in the graph.
  pruneTo(validIds) {
    for (const id of Array.from(this.positions.keys())) {
      if (!validIds.has(id)) {
        this.positions.delete(id);
        this.velocities.delete(id);
      }
    }
  }
}
