// stripState.js
// The solver-owned strip state, and its reconciliation across graph edits.
//
// Once the strip solver runs, this state is the ONLY home of positions: the
// weave graph is topology, and a graph vertex is located through the
// crossing table (StripState.passS, see strips.js crossingPosition).
//
// SolverState extends the StripState fields every consumer reads (strips
// layout, position, director, arc, passS - see strips.js) with:
//   strips[i]: + { h, sStart }      node spacing, material coordinate of
//                                   node 0
//   width:     Float32Array(3N)     unit width vector of each segment
//                                   (segment j of a strip, nodes j -> j+1,
//                                   is stored at the slot of node j; an open
//                                   strip's last slot repeats its last
//                                   segment). Perpendicular to its segment;
//                                   varies along the strip only by twist.
//   prev:      Float32Array(3N)     previous positions (XPBD velocity)
//   invMass:   Float32Array(N)
//   contactSign: Int8Array(crossingCount)  over/under at each crossing, as
//                                   the side of n = tA x tB (tangents of its
//                                   first and second pass, in strip
//                                   direction) that pass A must stay on
// Nodes sit at fixed material spacing: node k of strip i is at material
// coordinate sStart + k*h (arc[] stores exactly that). A closed strip's
// length is nodeCount*h; an open strip's is (nodeCount-1)*h.
//
// Reconciliation (reconcileStripState) builds the state for a new strip
// model from the old model+state, one span at a time. A span is the strip
// material between two consecutive crossings:
//   - if the two crossings were adjacent on an old strip, the span's material
//     is copied from there (positions and prev, keeping its arc length);
//   - otherwise it is generated from crossing positions with the over/under
//     offset (strips.js deriveStripState - the same math the renderer used
//     before the solver existed).
// Tails past an end apex copy old material beyond that crossing (up to
// 2*apexExtension) and extrapolate straight if there is less than
// apexExtension. Material coordinates are aligned with the old strip where
// possible, so a strip the edit didn't touch comes back bit-for-bit.

import { DEFAULT_STRIP_PARAMS, createStripState, deriveStripState, crossingPosition, stripPointAt } from "./strips.js";

const GRID_EPS = 1e-6;

function passSlot(model, v, stripId, index) {
  const c = model.crossings.get(v);
  const k = c.passes.findIndex((p) => p.strip === stripId && p.index === index);
  return 2 * c.index + k;
}

const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function normalize(a, fallback = [0, 0, 1]) {
  const len = Math.hypot(a[0], a[1], a[2]);
  return len > 1e-12 ? [a[0] / len, a[1] / len, a[2] / len] : fallback.slice();
}

// Material available on a source strip beyond coordinate `s`, going in
// direction `dir` (+1 / -1). Unlimited on a closed strip.
function materialBeyond(state, stripId, s, dir) {
  const st = state.strips[stripId];
  if (st.closed) return Infinity;
  const first = state.arc[st.nodeOffset];
  const last = state.arc[st.nodeOffset + st.nodeCount - 1];
  return dir > 0 ? last - s : s - first;
}

// Plan one new strip: its pieces (one per span, each mapping new material
// coordinates onto a source strip), crossing coordinates, tails and grid.
function planStrip(strip, oldModel, oldState, newModel, derived, params) {
  const { verts, closed, id } = strip;
  const n = verts.length;
  const spanCount = closed ? n : n - 1;

  const pieces = [];
  for (let j = 0; j < spanCount; j++) {
    const a = verts[j];
    const b = verts[(j + 1) % n];
    const old = oldModel ? oldModel.spanIndex.get(strip.edgeKeys[j]) : null;
    if (old) {
      const os = oldModel.strips[old.strip];
      const on = os.verts.length;
      const forward = os.verts[old.index] === a;
      const ia = forward ? old.index : (old.index + 1) % on;
      const ib = forward ? (old.index + 1) % on : old.index;
      const sa = oldState.passS[passSlot(oldModel, a, old.strip, ia)];
      let sb = oldState.passS[passSlot(oldModel, b, old.strip, ib)];
      const len = oldState.strips[old.strip].length;
      if (forward && sb < sa) sb += len; // across a closed strip's seam
      if (!forward && sb > sa) sb -= len;
      pieces.push({ source: oldState, strip: old.strip, dir: forward ? 1 : -1, sa, sb, len: Math.abs(sb - sa), old: true });
    } else {
      const sa = derived.passS[passSlot(newModel, a, id, j)];
      let sb = derived.passS[passSlot(newModel, b, id, (j + 1) % n)];
      if (sb < sa) sb += derived.strips[id].length;
      pieces.push({ source: derived, strip: id, dir: 1, sa, sb, len: sb - sa, old: false });
    }
  }

  // Crossing coordinates cs[0..spanCount], anchored so the first forward
  // old piece keeps its old coordinates exactly (offset 0).
  let anchor = pieces.findIndex((p) => p.old && p.dir === 1);
  if (anchor < 0) anchor = pieces.findIndex((p) => !p.old);
  if (anchor < 0) anchor = 0;
  const cs = new Array(spanCount + 1);
  cs[anchor] = pieces[anchor].dir === 1 ? pieces[anchor].sa : 0;
  for (let j = anchor; j < spanCount; j++) {
    const p = pieces[j];
    cs[j + 1] = p.dir === 1 && cs[j] === p.sa ? p.sb : cs[j] + p.len;
  }
  for (let j = anchor - 1; j >= 0; j--) {
    const p = pieces[j];
    cs[j] = p.dir === 1 && cs[j + 1] === p.sb ? p.sa : cs[j + 1] - p.len;
  }
  // source coordinate = off + dir * new coordinate
  pieces.forEach((p, j) => (p.off = p.sa - p.dir * cs[j]));

  const ext = params.apexExtension;
  const anchorPiece = pieces[anchor];
  const aligned = anchorPiece.old && anchorPiece.dir === 1 && anchorPiece.off === 0;
  const oldLayout = aligned ? oldState.strips[anchorPiece.strip] : null;
  let h, sStart, nodeCount;

  if (closed) {
    const length = cs[spanCount] - cs[0];
    if (aligned && oldLayout.closed && Math.abs(length - oldLayout.length) < 1e-9) {
      ({ h, sStart, nodeCount } = oldLayout);
    } else {
      nodeCount = Math.max(3, Math.round(length / params.nodeSpacing));
      h = length / nodeCount;
      sStart = cs[0];
    }
  } else {
    const p0 = pieces[0];
    const pl = pieces[spanCount - 1];
    const clampTail = (avail) => Math.min(Math.max(avail, ext), 2 * ext);
    const tail0 = clampTail(materialBeyond(p0.source, p0.strip, p0.sa, -p0.dir));
    const tail1 = clampTail(materialBeyond(pl.source, pl.strip, pl.sb, pl.dir));
    const from = cs[0] - tail0;
    const to = cs[spanCount] + tail1;
    if (aligned) {
      h = oldLayout.h;
      const k0 = Math.ceil((from - oldLayout.sStart) / h - GRID_EPS);
      sStart = k0 === 0 ? oldLayout.sStart : oldLayout.sStart + k0 * h;
    } else {
      h = params.nodeSpacing;
      sStart = from;
    }
    nodeCount = Math.max(2, Math.floor((to - sStart) / h + GRID_EPS) + 1);
  }
  return { strip, pieces, cs, closed, h, sStart, nodeCount, spanCount };
}

// Width vector of the segment containing material coordinate `s` on a source
// strip: copied exactly from a solver state, or tangent x director at the
// nearest node of the derived state.
function widthAt(source, stripId, s) {
  const st = source.strips[stripId];
  const m = st.nodeCount;
  if (source.width) {
    const segs = st.closed ? m : m - 1;
    let j = Math.floor((s - st.sStart) / st.h);
    j = st.closed ? ((j % m) + m) % m : Math.min(Math.max(j, 0), segs - 1);
    const o = 3 * (st.nodeOffset + j);
    return [source.width[o], source.width[o + 1], source.width[o + 2]];
  }
  let lo = 0;
  let hi = m - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (source.arc[st.nodeOffset + mid] <= s) lo = mid;
    else hi = mid;
  }
  const k = Math.min(Math.max(lo, 1), m - 2);
  const pos = source.position;
  const o = 3 * (st.nodeOffset + k);
  const T = normalize([pos[o + 3] - pos[o - 3], pos[o + 4] - pos[o - 2], pos[o + 5] - pos[o - 1]]);
  return normalize(cross(T, [source.director[o], source.director[o + 1], source.director[o + 2]]));
}

// Build the solver state for `newModel`.
//   oldModel/oldState: the model and solver state before the edit (both
//     null to build from scratch, e.g. for the seed)
//   seeded: Map<vertexId, {x,y,z}> positions for vertices the old state
//     can't locate (fresh vertices of a placement; every vertex when
//     building from scratch)
export function reconcileStripState(oldModel, oldState, newModel, seeded, params = DEFAULT_STRIP_PARAMS) {
  // Positions of every crossing of the new model, for generating new spans.
  const positions = new Map();
  for (const v of newModel.crossings.keys()) {
    const p = seeded.get(v) || (oldModel && crossingPosition(oldModel, oldState, v));
    if (p) positions.set(v, p);
  }
  const derived = deriveStripState(newModel, positions, params, createStripState(newModel));

  const plans = newModel.strips.map((s) => planStrip(s, oldModel, oldState, newModel, derived, params));
  const total = plans.reduce((t, p) => t + p.nodeCount, 0);
  const state = {
    strips: [],
    position: new Float32Array(3 * total),
    prev: new Float32Array(3 * total),
    invMass: new Float32Array(total).fill(1),
    director: new Float32Array(3 * total),
    width: new Float32Array(3 * total),
    arc: new Float64Array(total),
    passS: new Float64Array(2 * newModel.crossingCount),
    contactSign: contactSigns(newModel, derived),
  };

  let offset = 0;
  for (const plan of plans) {
    const { strip, pieces, cs, closed, h, sStart, nodeCount, spanCount } = plan;
    const length = closed ? nodeCount * h : (nodeCount - 1) * h;
    const layout = { nodeOffset: offset, nodeCount, closed, length, h, sStart };
    state.strips.push(layout);

    // piece covering new coordinate q (q already wrapped for closed strips)
    const pieceAt = (q) => {
      let lo = 0;
      let hi = spanCount - 1;
      while (lo < hi) {
        const mid = (lo + hi + 1) >> 1;
        if (cs[mid] <= q) lo = mid;
        else hi = mid - 1;
      }
      return pieces[lo];
    };
    for (let k = 0; k < nodeCount; k++) {
      const sNew = k === 0 ? sStart : sStart + k * h;
      state.arc[offset + k] = sNew;
      const q = closed ? cs[0] + ((((sNew - cs[0]) % length) + length) % length) : sNew;
      const p = pieceAt(q);
      const src = p.off + p.dir * q;
      const x = stripPointAt(p.source, p.strip, src);
      const xp = p.old ? stripPointAt(p.source, p.strip, src, p.source.prev) : x;
      state.position.set(x, 3 * (offset + k));
      state.prev.set(xp, 3 * (offset + k));
    }

    // segment widths, sampled at segment midpoints: copied exactly from old
    // material, otherwise taken from the derived ribbon and made
    // perpendicular to the new segment; signs kept continuous
    const segs = closed ? nodeCount : nodeCount - 1;
    let prevW = null;
    for (let j = 0; j < segs; j++) {
      const sMid = sStart + (j + 0.5) * h;
      const q = closed ? cs[0] + ((((sMid - cs[0]) % length) + length) % length) : sMid;
      const p = pieceAt(q);
      let wv = widthAt(p.source, p.strip, p.off + p.dir * q);
      if (!p.old) {
        const a = 3 * (offset + j);
        const b = 3 * (offset + ((j + 1) % nodeCount));
        const pos = state.position;
        const d = normalize([pos[b] - pos[a], pos[b + 1] - pos[a + 1], pos[b + 2] - pos[a + 2]], [1, 0, 0]);
        const dd = dot(wv, d);
        wv = normalize([wv[0] - dd * d[0], wv[1] - dd * d[1], wv[2] - dd * d[2]], prevW || [0, 1, 0]);
      }
      if (prevW && dot(wv, prevW) < 0) wv = [-wv[0], -wv[1], -wv[2]];
      state.width.set(wv, 3 * (offset + j));
      prevW = wv;
    }
    if (!closed) state.width.set(prevW, 3 * (offset + nodeCount - 1));

    // crossing table
    const n = strip.verts.length;
    for (let j = 0; j < n; j++) {
      let s = cs[j];
      if (closed) s = sStart + ((((s - sStart) % length) + length) % length);
      state.passS[passSlot(newModel, strip.verts[j], strip.id, j)] = s;
    }
    offset += nodeCount;
  }
  updateDirectors(state);
  return state;
}

// Over/under per crossing, read off the derived geometry, where each pass is
// displaced exactly +-thickness/2 along the crossing normal by the Tait
// rule. Expressed relative to n = tA x tB, which needs no global
// orientation and which the solver can recompute from strip tangents alone.
function contactSigns(model, derived) {
  const signs = new Int8Array(model.crossingCount);
  const pos = derived.position;
  const at = (pass) => {
    const st = derived.strips[pass.strip];
    const m = st.nodeCount;
    const k = pass.node;
    const kp = st.closed ? (k - 1 + m) % m : Math.max(0, k - 1);
    const kn = st.closed ? (k + 1) % m : Math.min(m - 1, k + 1);
    const o = 3 * (st.nodeOffset + k);
    const a = 3 * (st.nodeOffset + kp);
    const b = 3 * (st.nodeOffset + kn);
    return {
      p: [pos[o], pos[o + 1], pos[o + 2]],
      t: [pos[b] - pos[a], pos[b + 1] - pos[a + 1], pos[b + 2] - pos[a + 2]],
    };
  };
  for (const c of model.crossings.values()) {
    if (c.passes.length !== 2) continue;
    const A = at(c.passes[0]);
    const B = at(c.passes[1]);
    const n = cross(A.t, B.t);
    signs[c.index] = dot([A.p[0] - B.p[0], A.p[1] - B.p[1], A.p[2] - B.p[2]], n) < 0 ? -1 : 1;
  }
  return signs;
}

// Ribbon normal per node, N = b x T, with b the average of the node's
// adjacent segment widths.
export function updateDirectors(state) {
  const pos = state.position;
  const dir = state.director;
  const wd = state.width;
  for (const st of state.strips) {
    const m = st.nodeCount;
    for (let k = 0; k < m; k++) {
      const kp = st.closed ? (k - 1 + m) % m : Math.max(0, k - 1);
      const kn = st.closed ? (k + 1) % m : Math.min(m - 1, k + 1);
      const a = 3 * (st.nodeOffset + kp);
      const b = 3 * (st.nodeOffset + kn);
      const T = normalize([pos[b] - pos[a], pos[b + 1] - pos[a + 1], pos[b + 2] - pos[a + 2]], [1, 0, 0]);
      const w0 = 3 * (st.nodeOffset + kp);
      const w1 = 3 * (st.nodeOffset + k);
      const sgn = wd[w0] * wd[w1] + wd[w0 + 1] * wd[w1 + 1] + wd[w0 + 2] * wd[w1 + 2] < 0 ? -1 : 1;
      const B = [wd[w1] + sgn * wd[w0], wd[w1 + 1] + sgn * wd[w0 + 1], wd[w1 + 2] + sgn * wd[w0 + 2]];
      const N = normalize(cross(normalize(B, [wd[w1], wd[w1 + 1], wd[w1 + 2]]), T));
      dir.set(N, 3 * (st.nodeOffset + k));
    }
  }
}
