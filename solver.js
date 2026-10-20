// solver.js
// XPBD solver for woven strips (see triaxial-weave-solver-design.md,
// "Agreed design for this app"). It owns and advances the solver state built
// by stripState.js; the weave graph contributes only topology (which strips
// cross where, and which spans join consecutive crossings).
//
// Per substep: integrate (semi-implicit, velocity from x - prev), one
// Gauss-Seidel pass over the constraints, refit each strip's width vector,
// then re-track every crossing's material coordinates. Small substeps with a
// single iteration each is the "small steps" XPBD variant, so Lagrange
// multipliers start at zero every substep and need no storage.
//
// Constraints:
//   stretch      |x[i+1] - x[i]| = h                         rigid
//   swerve       (x[i+k] - 2x[i] + x[i-k]) . X_i = 0, k = 1, 2, 4, with X_i
//                the node's width vector (average of its two segments')
//                (bending around Z: never wanted)            rigid by default
//   curl         (x[i+1] - 2x[i] + x[i-1]) . Z_i = 0, Z = T x X
//                (bending around X: the paper-like bend)     soft; 1 = rigid
//   contact      sign * (pA - pB) . n - thickness  (n = tA x tB, unit)
//                  apex: = 0 (two-sided); interior/nesting: >= 0 (one-sided)
//   friction     at a crossing that is pressed together, the tangential slip
//                of its two passes' material points this substep is undone
//                up to friction * (normal correction) - Coulomb friction
//   apex clip    at an apex crossing (always in contact) a fraction apexGrip
//                of the slip is undone regardless of pressure, as if the two
//                strips were clipped together; at apexGrip 1 the crossing is
//                pinned in material coordinates (not re-tracked), so it can't
//                creep
//   grip         at an ordinary crossing while touching, slip is undone
//                completely up to a cap (crossingGrip, plus Coulomb friction
//                from pressure); only the excess slides
//   min gap      |m_a - m_b| >= minGap between consecutive crossings
//                (m = crossing midpoint)                     rigid, one-sided
// Each segment's width vector b is updated after each pass as the
// direction that best satisfies, together: perpendicular to its own and the
// neighbouring segments (so the strip bends about its easy axis), and
// aligned with its neighbours' width vectors (twist stiffness). At every
// crossing, touching or not, the two passes' segments then get b made
// exactly perpendicular to the crossing normal n = tA x tB, so both strips'
// thickness axes Z = T x X lie along n: the Z axes of crossing strips align. A pass's material coordinate
// is re-found by closest-point search every substep, so whatever slip
// friction allows becomes sliding. (Springs pulling spans toward unit length
// were tried and removed: compressing a rigid-planar strip buckles it.)
//
// Tails: adjustTails() runs once per frame, growing an open strip's end when
// its last crossing comes within apexExtension of it and trimming it when
// the tail exceeds 2*apexExtension. It changes node counts, so it returns
// whether the layout changed (the renderer must rebuild its buffers).

import { updateDirectors } from "./stripState.js";

export const DEFAULT_SOLVER_PARAMS = {
  substeps: 10,
  iterations: 1, // constraint passes per substep; multipliers accumulate
  // across them. More iterations stop stiff rules from fighting (curl at
  // spacings 2 and 4 no longer wrecks the weave at 4) but cost proportionally
  // and don't by themselves produce a dome; see the design doc.
  planarStiffness: 1, // swerve stiffness 0..1, log-mapped; 1 = rigid (no swerve)
  twistStiffness: 0.8, // 0..1, log-mapped weight pulling neighbouring width vectors together
  bendStiffness: 0.6, // curl stiffness 0..1, log-mapped; 1 = rigid
  friction: 0.5, // Coulomb coefficient where strips are pressed together
  apexGrip: 1, // 0..1: fraction of slip undone per substep at apex crossings
  // (1 = clipped together)
  crossingGrip: 0, // 0..1: static grip at ordinary crossings while
  // touching, independent of pressure. It holds hole sizes (without it strips
  // slide until a pentagon is small enough to lie flat), but in this solver
  // any grip lets strips curve sideways again (measured: 0.5 holds spans at
  // 0.96-1.02 but sideways curvature p95 rises from 0.015 to ~1), so it is
  // off by default.
  damping: 0.95, // fraction of velocity removed per frame; high = near
  // quasi-static relaxation. Conflicting rigid constraints turn their
  // corrections into velocity, so low damping pumps energy in (measured:
  // 0.02 and 0.3 blew up; 0.95 settles hexagonal meshes cleanly).
  minGap: 0.2, // ~ strip width: closest two crossings along a strip may get
};

// Stiffness slider (0..1) -> XPBD compliance: 1 is ~rigid, 0 very soft.
export function complianceFor(stiffness) {
  return Math.pow(10, -9 + 7 * (1 - stiffness));
}

const FACE_CONTACT_WEIGHT = 4; // a touching crossing counts like 4 segments
// Crossing grip is static friction that doesn't depend on pressure: slip up
// to crossingGrip * GRIP_HOLD per substep is undone completely; only the
// excess slides. (A fraction-of-slip grip was tried first: it acts like
// viscosity, and a steady push creeps through it - a pentagon still shrank
// to sides of 0.3 at any fraction below 1. A cap of 0.002 was too weak to
// resist the rigid in-plane rule.)
const GRIP_HOLD = 0.05;
const BEND_SCALES = [1, 2, 4]; // node spacings for the swerve and curl rules
// Curl uses only adjacent nodes. Applying it at spacings 2 and 4 as well
// (so stiff curl would also resist gentle curls) wrecked the weave: the
// over/under contacts require curl at every crossing, and the stiff curl rule
// fought them (pentagon session: sideways curvature p95 0.02 -> 2.8, stretch
// error 1% -> 17% at curl 0.6).
const CURL_MAX_SCALE = 1;
const CURL_REFERENCE_SPACING = 0.125; // node spacing the curl slider was tuned at

let frameBuffer = new Float64Array(0);
function frameScratch(m) {
  if (frameBuffer.length < 6 * m) frameBuffer = new Float64Array(6 * m);
  return frameBuffer;
}

// Drive the component of the second difference x[kn] - 2x[ki] + x[kp] along
// unit direction u to zero (alphaTilde = 0: rigid). lam[li] is this
// constraint's multiplier accumulated over the substep's iterations (XPBD),
// which makes a soft constraint converge to its real stiffness instead of
// being re-applied from scratch on every pass.
function secondDifference(x, w, kp, ki, kn, ux, uy, uz, alphaTilde, lam, li) {
  const p = 3 * kp;
  const c = 3 * ki;
  const q = 3 * kn;
  const C = (x[q] - 2 * x[c] + x[p]) * ux + (x[q + 1] - 2 * x[c + 1] + x[p + 1]) * uy + (x[q + 2] - 2 * x[c + 2] + x[p + 2]) * uz;
  const W = w[kp] + 4 * w[ki] + w[kn] + alphaTilde;
  if (W <= 1e-18) return;
  const dl = (-C - alphaTilde * lam[li]) / W;
  lam[li] += dl;
  x[p] += w[kp] * dl * ux;
  x[p + 1] += w[kp] * dl * uy;
  x[p + 2] += w[kp] * dl * uz;
  x[c] -= 2 * w[ki] * dl * ux;
  x[c + 1] -= 2 * w[ki] * dl * uy;
  x[c + 2] -= 2 * w[ki] * dl * uz;
  x[q] += w[kn] * dl * ux;
  x[q + 1] += w[kn] * dl * uy;
  x[q + 2] += w[kn] * dl * uz;
}
const OWN_SEGMENT_WEIGHT = 4; // keeps a width vector perpendicular to its own segment

// Twist slider (0..1) -> weight of each neighbour's width vector in a
// segment's width update, relative to weight 1 per neighbouring segment.
export function twistWeightFor(stiffness) {
  return Math.pow(10, 3 * stiffness - 1);
}
const IN_CONTACT = 0.5; // a crossing "touches" while separation < (1 + this) * thickness

// ---- small vector helpers on flat arrays ---------------------------------

function cross(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

// Node indices and blend weight of material coordinate s on a strip:
// point = (1-t) * x[k0] + t * x[k1].
function locate(st, s) {
  const m = st.nodeCount;
  let u = (s - st.sStart) / st.h;
  if (st.closed) {
    u = ((u % m) + m) % m;
    let k = Math.floor(u);
    if (k >= m) k = m - 1;
    return [st.nodeOffset + k, st.nodeOffset + ((k + 1) % m), u - k];
  }
  let k = Math.floor(u);
  if (k < 0) k = 0;
  if (k > m - 2) k = m - 2;
  return [st.nodeOffset + k, st.nodeOffset + k + 1, Math.min(1, Math.max(0, u - k))];
}

function pointAt(x, k0, k1, t) {
  const a = 3 * k0;
  const b = 3 * k1;
  return [x[a] + t * (x[b] - x[a]), x[a + 1] + t * (x[b + 1] - x[a + 1]), x[a + 2] + t * (x[b + 2] - x[a + 2])];
}

function segDir(x, k0, k1) {
  const a = 3 * k0;
  const b = 3 * k1;
  return [x[b] - x[a], x[b + 1] - x[a + 1], x[b + 2] - x[a + 2]];
}

// Generic XPBD update for a constraint whose gradient is `grad` (a 3-vector
// direction) scaled per node by `coef[i]` on node `idx[i]`.
function project(x, w, idx, coef, grad, C, alphaTilde) {
  const g2 = grad[0] * grad[0] + grad[1] * grad[1] + grad[2] * grad[2];
  let W = 0;
  for (let i = 0; i < idx.length; i++) W += w[idx[i]] * coef[i] * coef[i];
  W = W * g2 + alphaTilde;
  if (W <= 1e-18) return;
  const dl = -C / W;
  for (let i = 0; i < idx.length; i++) {
    const s = w[idx[i]] * coef[i] * dl;
    const o = 3 * idx[i];
    x[o] += s * grad[0];
    x[o + 1] += s * grad[1];
    x[o + 2] += s * grad[2];
  }
}

// Allocation-free XPBD update for the common two-node case: gradient -g on
// node i0 and +g on node i1, with g a unit vector.
function project2(x, w, i0, i1, gx, gy, gz, C, alphaTilde) {
  const w0 = w[i0];
  const w1 = w[i1];
  const W = w0 + w1 + alphaTilde;
  if (W <= 1e-18) return;
  const dl = -C / W;
  const a = 3 * i0;
  const b = 3 * i1;
  x[a] -= w0 * dl * gx;
  x[a + 1] -= w0 * dl * gy;
  x[a + 2] -= w0 * dl * gz;
  x[b] += w1 * dl * gx;
  x[b + 1] += w1 * dl * gy;
  x[b + 2] += w1 * dl * gz;
}

// ---- constraint lists (cached per model+state) ---------------------------

function prepare(model, state) {
  const crossings = [];
  const byVertex = new Map();
  for (const c of model.crossings.values()) {
    if (c.passes.length !== 2 || c.kind === "irregular") continue;
    const entry = {
      index: c.index,
      stripA: c.passes[0].strip,
      stripB: c.passes[1].strip,
      twoSided: c.kind === "apex",
      sign: state.contactSign[c.index],
    };
    crossings.push(entry);
    byVertex.set(c.v, entry);
  }
  const spans = [];
  for (const s of model.strips) {
    const n = s.verts.length;
    const count = s.closed ? n : n - 1;
    for (let j = 0; j < count; j++) {
      const a = byVertex.get(s.verts[j]);
      const b = byVertex.get(s.verts[(j + 1) % n]);
      if (a && b && a !== b) spans.push([a, b]);
    }
  }
  return { model, state, crossings, spans };
}

// Crossing midpoint and its node stencil: 4 nodes with weights 1/2 * blend.
function crossingStencil(state, x, c) {
  const A = locate(state.strips[c.stripA], state.passS[2 * c.index]);
  const B = locate(state.strips[c.stripB], state.passS[2 * c.index + 1]);
  const idx = [A[0], A[1], B[0], B[1]];
  const coef = [(1 - A[2]) / 2, A[2] / 2, (1 - B[2]) / 2, B[2] / 2];
  const m = [0, 0, 0];
  for (let i = 0; i < 4; i++) {
    const o = 3 * idx[i];
    m[0] += coef[i] * x[o];
    m[1] += coef[i] * x[o + 1];
    m[2] += coef[i] * x[o + 2];
  }
  return { idx, coef, m };
}

// ---- one substep ------------------------------------------------------------

// params.off: optional array of term names to skip (debugging/tests):
// "stretch", "planarity", "bending", "contact", "friction", "gap", "refit",
// "retrack".
const isOff = (params, term) => Array.isArray(params.off) && params.off.includes(term);

function solveConstraints(prep, params, stripParams, dt) {
  const { state } = prep;
  const x = state.position;
  const w = state.invMass;
  const dt2 = dt * dt;
  // Curl compliance is scaled by (h / h0)^3 per strip, so a slider value means
  // the same physical bending stiffness at any node spacing: the constraint
  // is a second difference (~ curvature * h^2) and bending energy per node is
  // ~ curvature^2 * h. Without this, coarsening nodes 4x made curl ~64x
  // stiffer and it fought the over/under contacts until the solver blew up.
  const bendAlphaRef = params.bendStiffness >= 1 ? 0 : complianceFor(params.bendStiffness) / dt2;
  const planarAlpha = params.planarStiffness >= 1 ? 0 : complianceFor(params.planarStiffness) / dt2;
  const thickness = stripParams.thickness;

  const doStretch = !isOff(params, "stretch");
  const doPlanar = !isOff(params, "planarity");
  const wd = state.width;
  const doBend = !isOff(params, "bending");
  for (const st of state.strips) {
    const m = st.nodeCount;
    const segs = st.closed ? m : m - 1;
    const h = st.h;
    for (let i = 0; i < segs; i++) {
      const k0 = st.nodeOffset + i;
      const k1 = st.nodeOffset + ((i + 1) % m);
      const a = 3 * k0;
      const b = 3 * k1;
      if (doStretch) {
        const dx = x[b] - x[a];
        const dy = x[b + 1] - x[a + 1];
        const dz = x[b + 2] - x[a + 2];
        const len = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (len > 1e-12) project2(x, w, k0, k1, dx / len, dy / len, dz / len, len - h, 0);
      }
    }
    // Swerve and curl, at node spacings k = 1, 2, 4. At node i the second
    // difference x[i+k] - 2x[i] + x[i-k] is split along the node's frame:
    //   swerve: its component along X (the width; bending around Z) - none
    //           allowed, rigid by default;
    //   curl:   its component along Z = T x X (bending around X, out of the
    //           flat strip's plane) - resisted by the curl stiffness.
    // k = 1 alone removes kinks quickly but undoes a gentle curve only very
    // slowly (Gauss-Seidel moves one neighbour at a time), so a "rigid" strip
    // could still swerve like a road; the larger spacings reach those long
    // curves directly. (8 and 16 fought the width updates.) Curl stays at
    // k = 1; see CURL_MAX_SCALE.
    // The frame (X, Z) is taken once per node per pass.
    const frames = frameScratch(m);
    const bendAlpha = bendAlphaRef * Math.pow(st.h / CURL_REFERENCE_SPACING, 3);
    for (let i = 0; i < m; i++) {
      const ip = st.closed ? (i - 1 + m) % m : Math.max(0, i - 1);
      const inx = st.closed ? (i + 1) % m : Math.min(m - 1, i + 1);
      const wa = 3 * (st.nodeOffset + ip);
      const wb = 3 * (st.nodeOffset + i);
      const sg = wd[wa] * wd[wb] + wd[wa + 1] * wd[wb + 1] + wd[wa + 2] * wd[wb + 2] < 0 ? -1 : 1;
      let bx = wd[wb] + sg * wd[wa];
      let by = wd[wb + 1] + sg * wd[wa + 1];
      let bz = wd[wb + 2] + sg * wd[wa + 2];
      const bl = Math.sqrt(bx * bx + by * by + bz * bz) || 1;
      bx /= bl;
      by /= bl;
      bz /= bl;
      const p = 3 * (st.nodeOffset + ip);
      const q = 3 * (st.nodeOffset + inx);
      const tx = x[q] - x[p];
      const ty = x[q + 1] - x[p + 1];
      const tz = x[q + 2] - x[p + 2];
      let zx = ty * bz - tz * by;
      let zy = tz * bx - tx * bz;
      let zz = tx * by - ty * bx;
      const zl = Math.sqrt(zx * zx + zy * zy + zz * zz) || 1;
      frames.set([bx, by, bz, zx / zl, zy / zl, zz / zl], 6 * i);
    }
    for (const [si, k] of BEND_SCALES.entries()) {
      const count = st.closed ? m : m - 2 * k;
      for (let n = 0; n < count; n++) {
        const i = st.closed ? n : n + k;
        const kp = st.nodeOffset + ((i - k + m) % m);
        const ki = st.nodeOffset + i;
        const kn = st.nodeOffset + ((i + k) % m);
        const f = 6 * i;
        if (doPlanar) secondDifference(x, w, kp, ki, kn, frames[f], frames[f + 1], frames[f + 2], planarAlpha, prep.lamSwerve, 3 * ki + si);
        if (doBend && k <= CURL_MAX_SCALE) {
          secondDifference(x, w, kp, ki, kn, frames[f + 3], frames[f + 4], frames[f + 5], bendAlpha, prep.lamCurl, 3 * ki + si);
        }
      }
    }
  }

  // contacts (over/under, with fixed sign)
  for (const c of prep.crossings) {
    const stA = state.strips[c.stripA];
    const stB = state.strips[c.stripB];
    const [a0, a1, ta] = locate(stA, state.passS[2 * c.index]);
    const [b0, b1, tb] = locate(stB, state.passS[2 * c.index + 1]);
    const nRaw = cross(segDir(x, a0, a1), segDir(x, b0, b1));
    const nLen = Math.hypot(nRaw[0], nRaw[1], nRaw[2]);
    c.inContact = false;
    if (nLen < 1e-12) continue;
    const n = [nRaw[0] / nLen, nRaw[1] / nLen, nRaw[2] / nLen];
    c.n = n;
    c.segA = a0; // width slot of the segment each pass lies on
    c.segB = b0;
    const pA = pointAt(x, a0, a1, ta);
    const pB = pointAt(x, b0, b1, tb);
    const sep = c.sign * ((pA[0] - pB[0]) * n[0] + (pA[1] - pB[1]) * n[1] + (pA[2] - pB[2]) * n[2]);
    const C = sep - thickness;
    c.inContact = C < IN_CONTACT * thickness;
    if (!isOff(params, "contact") && (c.twoSided || C < 0)) {
      const s = c.sign;
      project(x, w, [a0, a1, b0, b1], [s * (1 - ta), s * ta, -s * (1 - tb), -s * tb], n, C, 0);
    }

    // friction: undo tangential slip of the two material points (their
    // coordinates are from the start of this substep). Apex clips undo the
    // fraction apexGrip; ordinary crossings, while touching, undo slip up to
    // a static cap: crossingGrip (independent of pressure) plus Coulomb
    // friction * the normal correction.
    if (isOff(params, "friction") || !c.inContact) continue;
    const qA = pointAt(x, a0, a1, ta);
    const qB = pointAt(x, b0, b1, tb);
    const d = [qA[0] - qB[0], qA[1] - qB[1], qA[2] - qB[2]];
    const dn = d[0] * n[0] + d[1] * n[1] + d[2] * n[2];
    const tg = [d[0] - dn * n[0], d[1] - dn * n[1], d[2] - dn * n[2]];
    const slip = Math.hypot(tg[0], tg[1], tg[2]);
    if (slip < 1e-12) continue;
    const corr = c.twoSided
      ? params.apexGrip * slip
      : Math.min(slip, params.crossingGrip * GRIP_HOLD + (C < 0 ? params.friction * -C : 0));
    if (corr <= 0) continue;
    project(x, w, [a0, a1, b0, b1], [1 - ta, ta, -(1 - tb), -tb], [tg[0] / slip, tg[1] / slip, tg[2] / slip], corr, 0);
  }

  // minimum gap between consecutive crossings (keeps holes from inverting)
  for (const [a, b] of isOff(params, "gap") ? [] : prep.spans) {
    const A = crossingStencil(state, x, a);
    const B = crossingStencil(state, x, b);
    const d = [A.m[0] - B.m[0], A.m[1] - B.m[1], A.m[2] - B.m[2]];
    const len = Math.hypot(d[0], d[1], d[2]);
    if (len < 1e-12) continue;
    const n = [d[0] / len, d[1] / len, d[2] / len];
    const idx = [...A.idx, ...B.idx];
    const coef = [...A.coef, ...B.coef.map((v) => -v)];
    if (len < params.minGap) project(x, w, idx, coef, n, len - params.minGap, 0);
  }
}

// Update every segment's width vector b: the unit vector minimizing
//   sum over own + neighbouring segments of weight * (b . d)^2      (bend about b)
//   + FACE_CONTACT_WEIGHT * (b . n)^2 for contacts on this segment  (face contact)
//   - twistWeight * (b . b_neighbour)^2 for both neighbours          (twist)
// i.e. the smallest-eigenvalue direction of the corresponding 3x3 matrix,
// found by a few power iterations of (c I - M) from the current b, which
// keeps it continuous and sign-stable. Then made exactly perpendicular to
// its own segment. Gauss-Seidel along each strip.
function updateWidths(prep, params) {
  const { state } = prep;
  const x = state.position;
  const wd = state.width;
  const twist = twistWeightFor(params.twistStiffness);
  // Z axes align at every crossing, touching or not: each pass's segment
  // gets its width vector made exactly perpendicular to the crossing normal
  // n = tA x tB (and to its own segment), so its Z = T x X lies along n for
  // both strips. Twist stiffness carries that orientation between crossings.
  const contactsAt = new Map(); // width slot -> [n, ...]
  for (const c of prep.crossings) {
    if (!c.n) continue;
    for (const slot of [c.segA, c.segB]) {
      if (!contactsAt.has(slot)) contactsAt.set(slot, []);
      contactsAt.get(slot).push(c.n);
    }
  }
  const M = [0, 0, 0, 0, 0, 0]; // xx yy zz xy xz yz
  const add = (v, wgt) => {
    M[0] += wgt * v[0] * v[0];
    M[1] += wgt * v[1] * v[1];
    M[2] += wgt * v[2] * v[2];
    M[3] += wgt * v[0] * v[1];
    M[4] += wgt * v[0] * v[2];
    M[5] += wgt * v[1] * v[2];
  };
  const seg = (st, j) => {
    const m = st.nodeCount;
    const a = 3 * (st.nodeOffset + j);
    const b = 3 * (st.nodeOffset + ((j + 1) % m));
    const d = [x[b] - x[a], x[b + 1] - x[a + 1], x[b + 2] - x[a + 2]];
    const l = Math.hypot(d[0], d[1], d[2]);
    return l > 1e-12 ? [d[0] / l, d[1] / l, d[2] / l] : null;
  };
  const widthOf = (st, j) => {
    const o = 3 * (st.nodeOffset + j);
    return [wd[o], wd[o + 1], wd[o + 2]];
  };
  for (const st of state.strips) {
    const m = st.nodeCount;
    const segs = st.closed ? m : m - 1;
    for (let j = 0; j < segs; j++) {
      M.fill(0);
      let c = 0;
      const own = seg(st, j);
      if (!own) continue;
      add(own, OWN_SEGMENT_WEIGHT);
      c += OWN_SEGMENT_WEIGHT;
      for (const k of [j - 1, j + 1]) {
        const kk = st.closed ? (k + m) % m : k;
        if (kk < 0 || kk >= segs) continue;
        const d = seg(st, kk);
        if (d) {
          add(d, 1);
          c += 1;
        }
        add(widthOf(st, kk), -twist);
      }
      for (const n of contactsAt.get(st.nodeOffset + j) || []) {
        add(n, FACE_CONTACT_WEIGHT);
        c += FACE_CONTACT_WEIGHT;
      }
      const [xx, yy, zz, xy, xz, yz] = M;
      let b = widthOf(st, j);
      for (let it = 0; it < 3; it++) {
        const v = [
          c * b[0] - (xx * b[0] + xy * b[1] + xz * b[2]),
          c * b[1] - (xy * b[0] + yy * b[1] + yz * b[2]),
          c * b[2] - (xz * b[0] + yz * b[1] + zz * b[2]),
        ];
        const len = Math.hypot(v[0], v[1], v[2]);
        if (len < 1e-12) break;
        b = [v[0] / len, v[1] / len, v[2] / len];
      }
      if (params.alignAtCrossings !== false) {
        for (const n of contactsAt.get(st.nodeOffset + j) || []) {
          const dn = b[0] * n[0] + b[1] * n[1] + b[2] * n[2];
          b = [b[0] - dn * n[0], b[1] - dn * n[1], b[2] - dn * n[2]];
        }
      }
      const dd = b[0] * own[0] + b[1] * own[1] + b[2] * own[2];
      b = [b[0] - dd * own[0], b[1] - dd * own[1], b[2] - dd * own[2]];
      const len = Math.hypot(b[0], b[1], b[2]);
      if (len < 1e-12) continue;
      wd.set([b[0] / len, b[1] / len, b[2] / len], 3 * (st.nodeOffset + j));
    }
    if (!st.closed) wd.copyWithin(3 * (st.nodeOffset + m - 1), 3 * (st.nodeOffset + m - 2), 3 * (st.nodeOffset + m - 1));
  }
}

// Closest points between segments p0-p1 and q0-q1 (Ericson, RTCD 5.1.9):
// returns [u, v, dist2] with u, v in [0,1].
function closestSegSeg(p0, p1, q0, q1) {
  const d1 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
  const d2 = [q1[0] - q0[0], q1[1] - q0[1], q1[2] - q0[2]];
  const r = [p0[0] - q0[0], p0[1] - q0[1], p0[2] - q0[2]];
  const a = d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2];
  const e = d2[0] * d2[0] + d2[1] * d2[1] + d2[2] * d2[2];
  const f = d2[0] * r[0] + d2[1] * r[1] + d2[2] * r[2];
  const clamp = (v) => Math.min(1, Math.max(0, v));
  let u, v;
  if (a < 1e-18 && e < 1e-18) {
    u = v = 0;
  } else if (a < 1e-18) {
    u = 0;
    v = clamp(f / e);
  } else {
    const c = d1[0] * r[0] + d1[1] * r[1] + d1[2] * r[2];
    if (e < 1e-18) {
      v = 0;
      u = clamp(-c / a);
    } else {
      const b = d1[0] * d2[0] + d1[1] * d2[1] + d1[2] * d2[2];
      const denom = a * e - b * b;
      u = denom > 1e-18 ? clamp((b * f - c * e) / denom) : 0;
      v = (b * u + f) / e;
      if (v < 0) {
        v = 0;
        u = clamp(-c / a);
      } else if (v > 1) {
        v = 1;
        u = clamp((b - c) / a);
      }
    }
  }
  const P = [p0[0] + u * d1[0], p0[1] + u * d1[1], p0[2] + u * d1[2]];
  const Q = [q0[0] + v * d2[0], q0[1] + v * d2[1], q0[2] + v * d2[2]];
  return [u, v, (P[0] - Q[0]) ** 2 + (P[1] - Q[1]) ** 2 + (P[2] - Q[2]) ** 2];
}

// Re-find each crossing's material coordinates: closest points between the
// two strips, searched over the segments around the current coordinates.
// This is what makes crossings frictionless - strips slide freely and the
// crossing follows.
function retrackCrossings(prep, params) {
  const { state } = prep;
  const x = state.position;
  const node = (k) => [x[3 * k], x[3 * k + 1], x[3 * k + 2]];
  // local segment indices around coordinate s, valid for the strip
  const around = (st, s) => {
    const m = st.nodeCount;
    const segs = st.closed ? m : m - 1;
    const k = Math.floor((s - st.sStart) / st.h);
    const out = [];
    for (let i = k - 1; i <= k + 1; i++) {
      if (st.closed) out.push(((i % m) + m) % m);
      else if (i >= 0 && i < segs) out.push(i);
    }
    if (!out.length) out.push(st.closed ? 0 : Math.min(Math.max(k, 0), segs - 1));
    return out;
  };
  for (const c of prep.crossings) {
    // a fully clipped apex (apexGrip 1) is pinned in material coordinates:
    // re-tracking would accept whatever slip survived the pass as the clip's
    // new position, so it would still creep
    if (c.twoSided && params.apexGrip >= 1) continue;
    const stA = state.strips[c.stripA];
    const stB = state.strips[c.stripB];
    const sA = state.passS[2 * c.index];
    const sB = state.passS[2 * c.index + 1];
    let best = null;
    for (const i of around(stA, sA)) {
      const a0 = stA.nodeOffset + i;
      const a1 = stA.nodeOffset + ((i + 1) % stA.nodeCount);
      for (const j of around(stB, sB)) {
        const b0 = stB.nodeOffset + j;
        const b1 = stB.nodeOffset + ((j + 1) % stB.nodeCount);
        if (a0 === b0 || a0 === b1 || a1 === b0 || a1 === b1) continue; // self-crossing neighbors
        const [u, v, d2] = closestSegSeg(node(a0), node(a1), node(b0), node(b1));
        if (!best || d2 < best.d2) best = { i, j, u, v, d2 };
      }
    }
    if (!best) continue;
    const wrap = (st, s) => (st.closed ? st.sStart + ((((s - st.sStart) % st.length) + st.length) % st.length) : s);
    state.passS[2 * c.index] = wrap(stA, stA.sStart + (best.i + best.u) * stA.h);
    state.passS[2 * c.index + 1] = wrap(stB, stB.sStart + (best.j + best.v) * stB.h);
  }
}

// Advance one frame. `stripParams` supplies thickness (and apexExtension
// for tails, used by adjustTails).
export function stepSolver(model, state, params, stripParams, frameDt = 1 / 60) {
  if (!state.solverCache || state.solverCache.model !== model) state.solverCache = prepare(model, state);
  const prep = state.solverCache;
  const n = Math.max(1, Math.round(params.substeps));
  const dt = frameDt / n;
  const keep = Math.pow(1 - params.damping, 1 / n);
  const x = state.position;
  const prev = state.prev;
  const nodes = x.length / 3;
  if (!prep.lamCurl || prep.lamCurl.length !== 3 * nodes) {
    prep.lamCurl = new Float64Array(3 * nodes);
    prep.lamSwerve = new Float64Array(3 * nodes);
  }
  const iterations = Math.max(1, Math.round(params.iterations ?? 1));
  for (let sub = 0; sub < n; sub++) {
    for (let i = 0; i < x.length; i++) {
      const v = (x[i] - prev[i]) * keep;
      prev[i] = x[i];
      x[i] += v;
    }
    prep.lamCurl.fill(0);
    prep.lamSwerve.fill(0);
    for (let it = 0; it < iterations; it++) {
      solveConstraints(prep, params, stripParams, dt);
      if (!isOff(params, "refit")) updateWidths(prep, params);
    }
    if (!isOff(params, "retrack")) retrackCrossings(prep, params);
  }
  updateDirectors(state);
}

// Grow or trim open strips' tails so each stays between apexExtension and
// 2*apexExtension past its end crossing. Returns true if any node count
// changed (state arrays are replaced in place on the same state object).
export function adjustTails(model, state, stripParams) {
  const ext = stripParams.apexExtension;
  const passCoord = (v, stripId, index) => {
    const c = model.crossings.get(v);
    const k = c.passes.findIndex((p) => p.strip === stripId && p.index === index);
    return state.passS[2 * c.index + k];
  };
  const changes = state.strips.map((st, i) => {
    if (st.closed) return [0, 0];
    const verts = model.strips[i].verts;
    const tail0 = passCoord(verts[0], i, 0) - st.sStart;
    const tail1 = st.sStart + (st.nodeCount - 1) * st.h - passCoord(verts[verts.length - 1], i, verts.length - 1);
    const delta = (tail) => {
      if (tail < ext) return Math.ceil((ext - tail) / st.h);
      if (tail > 2 * ext) return -Math.floor((tail - 1.5 * ext) / st.h);
      return 0;
    };
    return [delta(tail0), delta(tail1)];
  });
  if (changes.every(([a, b]) => a === 0 && b === 0)) return false;

  const total = state.strips.reduce((t, st, i) => t + st.nodeCount + changes[i][0] + changes[i][1], 0);
  const position = new Float32Array(3 * total);
  const prev = new Float32Array(3 * total);
  const invMass = new Float32Array(total).fill(1);
  const director = new Float32Array(3 * total);
  const width = new Float32Array(3 * total);
  const arc = new Float64Array(total);
  let offset = 0;
  state.strips.forEach((st, i) => {
    const [d0, d1] = changes[i];
    const oldOff = st.nodeOffset;
    const m = st.nodeCount;
    const from = Math.max(0, -d0); // first old node kept
    const to = m - Math.max(0, -d1); // one past last old node kept
    const lead = Math.max(0, d0);
    const trail = Math.max(0, d1);
    const nodes = lead + (to - from) + trail;
    // kept nodes
    position.set(state.position.subarray(3 * (oldOff + from), 3 * (oldOff + to)), 3 * (offset + lead));
    prev.set(state.prev.subarray(3 * (oldOff + from), 3 * (oldOff + to)), 3 * (offset + lead));
    width.set(state.width.subarray(3 * (oldOff + from), 3 * (oldOff + to)), 3 * (offset + lead));
    // grown segments continue the end segments' widths
    const firstW = state.width.subarray(3 * (oldOff + from), 3 * (oldOff + from) + 3);
    const lastW = state.width.subarray(3 * (oldOff + to - 2), 3 * (oldOff + to - 1));
    for (let j = 0; j < lead; j++) width.set(firstW, 3 * (offset + j));
    for (let j = 0; j <= trail; j++) width.set(lastW, 3 * (offset + lead + (to - from) - 1 + j));
    // grown nodes: straight on along the end segment, at rest
    const extend = (endK, innerK, count, write) => {
      const e = 3 * (oldOff + endK);
      const q = 3 * (oldOff + innerK);
      const dir = [state.position[e] - state.position[q], state.position[e + 1] - state.position[q + 1], state.position[e + 2] - state.position[q + 2]];
      const len = Math.hypot(dir[0], dir[1], dir[2]) || 1;
      for (let j = 1; j <= count; j++) {
        const p = [0, 1, 2].map((a) => state.position[e + a] + (dir[a] / len) * st.h * j);
        position.set(p, 3 * write(j));
        prev.set(p, 3 * write(j));
      }
    };
    if (lead) extend(0, 1, lead, (j) => offset + lead - j);
    if (trail) extend(m - 1, m - 2, trail, (j) => offset + lead + (to - from) - 1 + j);
    st.sStart = st.sStart + (from - lead) * st.h;
    st.nodeOffset = offset;
    st.nodeCount = nodes;
    st.length = (nodes - 1) * st.h;
    for (let k = 0; k < nodes; k++) arc[offset + k] = st.sStart + k * st.h;
    offset += nodes;
  });
  Object.assign(state, { position, prev, invMass, director, width, arc });
  state.solverCache = null;
  updateDirectors(state);
  return true;
}
