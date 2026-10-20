// seeding.js
// Initial positions for the fresh vertices a placement creates, computed
// from the positions of the nesting site's existing chain. Pure: the caller
// supplies the chain positions (from wherever positions currently live) and
// decides where the results go.
//
// Chord-ratio construction: given 3 consecutive polygon vertices (A, B, C),
// the next vertex D satisfies D = A + R(n)*(C - B), i.e. chord A->D is
// parallel to chord B->C, scaled by R(n) = sin(3*pi/n)/sin(pi/n) (R=1 for
// squares, the golden ratio for pentagons, 2 for hexagons). Sliding the
// window (A,B,C)->(B,C,D) generates the rest. The chain is real mesh
// points, so the result is "affine regular" (sheared by the existing
// angles), not an exact regular polygon - fine as a seed.
//
// `knownPts`: positions {x,y,z} of [u, ...siteVerts, w] (or [u, ...siteVerts]
// for a fuse), in polygon-loop order, exactly as WeaveGraph.placePolygon
// used them. `midCount`: fresh polygon vertices to generate after the
// chain. `apexCount`: fresh triangle apexes, one per edge of the chain
// [w, ...mids, u] (a single entry for the join-edge case, bridging w->u).
// Returns { mid: [{x,y,z}...], apex: [{x,y,z}...] }.
export function seedPlacementPositions(knownPts, midCount, apexCount) {
  const n = knownPts.length + midCount; // total sides of the new polygon
  const ratio = n === 3 ? 0 : Math.sin((3 * Math.PI) / n) / Math.sin(Math.PI / n);

  const loopPts = knownPts.slice();
  const mid = [];
  let A = loopPts[loopPts.length - 3];
  let B = loopPts[loopPts.length - 2];
  let C = loopPts[loopPts.length - 1];
  for (let i = 0; i < midCount; i++) {
    const D = {
      x: A.x + ratio * (C.x - B.x),
      y: A.y + ratio * (C.y - B.y),
      z: A.z + ratio * (C.z - B.z),
    };
    mid.push(D);
    loopPts.push(D);
    A = B;
    B = C;
    C = D;
  }

  // Local surface normal at this corner (from the chain's first 3 known
  // points), used only to give triangle apexes a small alternating fold
  // jitter so a flat start isn't an exact saddle.
  const [pu, p2, p3] = knownPts;
  let nx = (pu.y - p2.y) * (p3.z - p2.z) - (pu.z - p2.z) * (p3.y - p2.y);
  let ny = (pu.z - p2.z) * (p3.x - p2.x) - (pu.x - p2.x) * (p3.z - p2.z);
  let nz = (pu.x - p2.x) * (p3.y - p2.y) - (pu.y - p2.y) * (p3.x - p2.x);
  const nlen = Math.hypot(nx, ny, nz) || 1;
  nx /= nlen;
  ny /= nlen;
  nz /= nlen;

  // centroid of the new polygon loop, used to orient each apex outward
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

  // Apexes: one per edge along [w, mid_1, ..., mid_last, u].
  const w = loopPts[knownPts.length - 1];
  const u = loopPts[0];
  const chain = [w, ...mid, u];
  const jitter = 0.02;
  const apex = [];
  for (let i = 0; i < apexCount; i++) {
    const pa = chain[i];
    const pb = chain[i + 1];
    const mx = (pa.x + pb.x) / 2;
    const my = (pa.y + pb.y) / 2;
    const mz = (pa.z + pb.z) / 2;
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
    const foldSign = i % 2 === 0 ? -1 : 1;
    apex.push({
      x: mx + dx * height + foldSign * jitter * nx,
      y: my + dy * height + foldSign * jitter * ny,
      z: mz + dz * height + foldSign * jitter * nz,
    });
  }
  return { mid, apex };
}
