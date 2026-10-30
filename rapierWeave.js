// rapierWeave.js
// Strip mechanics on the Rapier physics engine.
//
// Each strip is a chain of thin rigid plates, one per segment of the strip
// state (stripState.js: nodes at fixed material spacing, one width vector
// per segment). A plate's local frame is the strip frame: X = width, Y =
// along the strip (node j -> j+1), Z = thickness. Joints:
//   - between neighbouring plates of a strip: a generic joint at the shared
//     node that locks all relative translation (inextensible) and rotation
//     around Z (no swerve). Curl (around X) and twist (around Y) are left
//     free, each held by a joint motor with target angle 0: Rapier's motors
//     are how a joint axis gets a spring, so these are springs pulling back
//     toward a straight, flat strip, not drivers.
//   - at every crossing: the two strips are pinned (no sliding). A revolute
//     joint joins a point on each plate's face, offset thickness/2 along Z
//     toward the other strip, so over/under is fixed by construction. Its
//     axis is both plates' Z, so the strips' Z axes stay aligned and only
//     the crossing angle is free.
// There are no colliders (no self-contact yet); plates get their mass from
// setAdditionalMassProperties.
//
// The strip state stays the hand-off point: after every step, node
// positions and width vectors are written back from plate poses, so
// crossingPosition, rendering, saving and edit reconciliation all work
// unchanged. Material coordinates (arc, passS) never change, since crossings
// don't slide. Edits rebuild the world from the reconciled state.

import RAPIER from "@dimforge/rapier3d-compat";
import { updateDirectors } from "./stripState.js";

export const DEFAULT_RAPIER_PARAMS = {
  stepsPerFrame: 1,
  iterations: 8, // Rapier solver iterations per step (more = joints hold better)
  curlStiffness: 1, // 0..1, log-mapped motor stiffness around the plate's X;
  // softer lets strips buckle between crossings instead of the sheet doming
  twistStiffness: 1, // 0..1, log-mapped motor stiffness around the plate's Y
  damping: 0.1, // fraction of velocity removed per step; 0.5 and up settles
  // too slowly to see a pentagon dome within seconds
};

const DT = 1 / 60;
const PLATE_MASS = 1;

let ready = null;
export function initRapier() {
  if (!ready) ready = RAPIER.init();
  return ready;
}

// Slider 0..1 -> motor stiffness 10^2..10^4. Motors are acceleration-based,
// so this is the squared natural frequency, independent of plate mass.
// Measured on a pentagon session: much below 10^3 strips crumple between
// crossings (bends over 100 degrees at 30); above 10^4 the shape barely
// changes and the strip joints start to stretch (1.2% at 10^6).
export function motorStiffnessFor(x) {
  return 10 ** (2 + 2 * Math.min(Math.max(x, 0), 1));
}

// Fraction of velocity removed per step -> Rapier damping coefficient
// (Rapier scales velocity by 1 / (1 + dt * d) each step).
function dampingFor(f) {
  const c = Math.min(Math.max(f, 0), 0.99);
  return c / ((1 - c) * DT);
}

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function normalize(a, fallback) {
  const len = Math.hypot(a[0], a[1], a[2]);
  return len > 1e-12 ? [a[0] / len, a[1] / len, a[2] / len] : fallback;
}

// Rotation (as {x,y,z,w}) whose columns are the orthonormal frame X, Y, Z.
function quatFromFrame(X, Y, Z) {
  const [m00, m10, m20] = X;
  const [m01, m11, m21] = Y;
  const [m02, m12, m22] = Z;
  const tr = m00 + m11 + m22;
  let x, y, z, w;
  if (tr > 0) {
    const s = 2 * Math.sqrt(tr + 1);
    w = s / 4;
    x = (m21 - m12) / s;
    y = (m02 - m20) / s;
    z = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const s = 2 * Math.sqrt(1 + m00 - m11 - m22);
    w = (m21 - m12) / s;
    x = s / 4;
    y = (m01 + m10) / s;
    z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const s = 2 * Math.sqrt(1 + m11 - m00 - m22);
    w = (m02 - m20) / s;
    x = (m01 + m10) / s;
    y = s / 4;
    z = (m12 + m21) / s;
  } else {
    const s = 2 * Math.sqrt(1 + m22 - m00 - m11);
    w = (m10 - m01) / s;
    x = (m02 + m20) / s;
    y = (m12 + m21) / s;
    z = s / 4;
  }
  return { x, y, z, w };
}

// Local vector v rotated by unit quaternion q.
function rotate(q, v) {
  const { x, y, z, w } = q;
  const tx = 2 * (y * v[2] - z * v[1]);
  const ty = 2 * (z * v[0] - x * v[2]);
  const tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
}

// Plate frame for segment j of a strip (nodes j -> j+1), from the state.
function segmentFrame(state, st, j) {
  const m = st.nodeCount;
  const a = 3 * (st.nodeOffset + j);
  const b = 3 * (st.nodeOffset + ((j + 1) % m));
  const pos = state.position;
  const pa = [pos[a], pos[a + 1], pos[a + 2]];
  const pb = [pos[b], pos[b + 1], pos[b + 2]];
  const Y = normalize(sub(pb, pa), [0, 1, 0]);
  const w = [state.width[a], state.width[a + 1], state.width[a + 2]];
  const wd = dot(w, Y);
  const X = normalize([w[0] - wd * Y[0], w[1] - wd * Y[1], w[2] - wd * Y[2]], normalize(cross(Y, [0, 0, 1]), [1, 0, 0]));
  const Z = cross(X, Y);
  return { center: [(pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2, (pa[2] + pb[2]) / 2], X, Y, Z };
}

function configureSprings(weave, handle, params) {
  const raw = weave.world.impulseJoints.raw;
  const curl = motorStiffnessFor(params.curlStiffness);
  const twist = motorStiffnessFor(params.twistStiffness);
  raw.jointConfigureMotorModel(handle, RAPIER.JointAxis.AngX, RAPIER.MotorModel.AccelerationBased);
  raw.jointConfigureMotorModel(handle, RAPIER.JointAxis.AngY, RAPIER.MotorModel.AccelerationBased);
  // critically damped springs
  raw.jointConfigureMotorPosition(handle, RAPIER.JointAxis.AngX, 0, curl, 2 * Math.sqrt(curl));
  raw.jointConfigureMotorPosition(handle, RAPIER.JointAxis.AngY, 0, twist, 2 * Math.sqrt(twist));
}

// Which plate of a strip holds material coordinate s, and where along it.
function platePoint(state, stripId, s) {
  const st = state.strips[stripId];
  const segs = st.closed ? st.nodeCount : st.nodeCount - 1;
  let u = (s - st.sStart) / st.h;
  if (st.closed) u = ((u % segs) + segs) % segs;
  const j = Math.min(Math.max(Math.floor(u), 0), segs - 1);
  return { j, y: (u - j - 0.5) * st.h };
}

// Build a Rapier world for `model` from `state` (a reconciled strip state).
// The returned weave owns the world; call disposeWeave before dropping it.
export function createWeave(model, state, params, stripParams) {
  const world = new RAPIER.World({ x: 0, y: 0, z: 0 });
  world.timestep = DT;
  world.numSolverIterations = params.iterations;
  const weave = { world, model, state, plates: [], springJoints: [] };

  const damp = dampingFor(params.damping);
  const W = stripParams.width;
  const T = stripParams.thickness;

  // plates
  for (const st of state.strips) {
    const segs = st.closed ? st.nodeCount : st.nodeCount - 1;
    const h = st.h;
    const inertia = {
      x: (PLATE_MASS * (h * h + T * T)) / 12,
      y: (PLATE_MASS * (W * W + T * T)) / 12,
      z: (PLATE_MASS * (W * W + h * h)) / 12,
    };
    const bodies = [];
    for (let j = 0; j < segs; j++) {
      const f = segmentFrame(state, st, j);
      const desc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(f.center[0], f.center[1], f.center[2])
        .setRotation(quatFromFrame(f.X, f.Y, f.Z))
        .setAdditionalMassProperties(PLATE_MASS, { x: 0, y: 0, z: 0 }, inertia, { x: 0, y: 0, z: 0, w: 1 })
        .setLinearDamping(damp)
        .setAngularDamping(damp)
        .setCanSleep(false); // sleeping can freeze a mesh that is still slowly relaxing
      bodies.push(world.createRigidBody(desc));
    }
    weave.plates.push(bodies);
  }

  // strip joints: no stretch, no swerve; curl and twist springs
  const M = RAPIER.JointAxesMask;
  const locked = M.LinX | M.LinY | M.LinZ | M.AngZ;
  state.strips.forEach((st, i) => {
    const bodies = weave.plates[i];
    const h = st.h;
    const pairs = st.closed ? bodies.length : bodies.length - 1;
    for (let j = 0; j < pairs; j++) {
      const a = bodies[j];
      const b = bodies[(j + 1) % bodies.length];
      const data = RAPIER.JointData.generic({ x: 0, y: h / 2, z: 0 }, { x: 0, y: -h / 2, z: 0 }, { x: 1, y: 0, z: 0 }, locked);
      const joint = world.createImpulseJoint(data, a, b, true);
      // A closed strip on a non-orientable mesh comes back with its width
      // vector flipped across the seam; let that joint rest half-twisted.
      if (j === bodies.length - 1 && dot(rotate(a.rotation(), [1, 0, 0]), rotate(b.rotation(), [1, 0, 0])) < 0) {
        joint.setFrameX2({ x: 0, y: 1, z: 0, w: 0 });
      }
      configureSprings(weave, joint.handle, params);
      weave.springJoints.push(joint.handle);
    }
  });

  // crossings: pinned, faces touching, Z axes aligned
  for (const c of model.crossings.values()) {
    if (c.passes.length !== 2) continue;
    const [A, B] = c.passes.map((pass, k) => {
      const s = state.passS[2 * c.index + k];
      const { j, y } = platePoint(state, pass.strip, s);
      const body = weave.plates[pass.strip][j];
      const q = body.rotation();
      return { body, y, Y: rotate(q, [0, 1, 0]), Z: rotate(q, [0, 0, 1]) };
    });
    if (A.body === B.body) continue;
    // Direction from pass A toward pass B, by the over/under rule: pass A
    // sits on the side of n = tA x tB given by contactSign.
    let toB = normalize(cross(A.Y, B.Y), null);
    if (toB) {
      const sgn = -state.contactSign[c.index];
      toB = [sgn * toB[0], sgn * toB[1], sgn * toB[2]];
    } else {
      toB = A.Z; // parallel strips: keep whatever side they are on now
    }
    const sideA = dot(toB, A.Z) < 0 ? -1 : 1;
    const sideB = dot(toB, B.Z) < 0 ? 1 : -1;
    const data = RAPIER.JointData.revoluteWithAxes(
      { x: 0, y: A.y, z: (sideA * T) / 2 },
      { x: 0, y: B.y, z: (sideB * T) / 2 },
      { x: 0, y: 0, z: 1 },
      { x: 0, y: 0, z: dot(A.Z, B.Z) < 0 ? -1 : 1 },
    );
    world.createImpulseJoint(data, A.body, B.body, true);
  }

  readBack(weave);
  return weave;
}

// Apply changed tuning to a live weave.
export function setWeaveParams(weave, params) {
  weave.world.numSolverIterations = params.iterations;
  const damp = dampingFor(params.damping);
  for (const bodies of weave.plates) {
    for (const b of bodies) {
      b.setLinearDamping(damp);
      b.setAngularDamping(damp);
    }
  }
  for (const handle of weave.springJoints) configureSprings(weave, handle, params);
}

// Advance one frame and write plate poses back into the strip state.
export function stepWeave(weave, params) {
  for (let k = 0; k < params.stepsPerFrame; k++) weave.world.step();
  readBack(weave);
}

export function disposeWeave(weave) {
  weave.world.free();
}

// Node positions and width vectors from plate poses. A node shared by two
// plates is the midpoint of their joined ends (they agree up to the joint
// error); an open strip's end nodes are its end plates' outer ends.
function readBack(weave) {
  const { state, plates } = weave;
  const pos = state.position;
  state.strips.forEach((st, i) => {
    const bodies = plates[i];
    const half = st.h / 2;
    const ends = bodies.map((b) => {
      const t = b.translation();
      const q = b.rotation();
      const Y = rotate(q, [0, half, 0]);
      return {
        lo: [t.x - Y[0], t.y - Y[1], t.z - Y[2]],
        hi: [t.x + Y[0], t.y + Y[1], t.z + Y[2]],
        X: rotate(q, [1, 0, 0]),
      };
    });
    const m = st.nodeCount;
    const segs = bodies.length;
    for (let k = 0; k < m; k++) {
      const before = st.closed ? ends[(k - 1 + segs) % segs] : ends[k - 1];
      const after = ends[k];
      let p;
      if (before && after) p = [(before.hi[0] + after.lo[0]) / 2, (before.hi[1] + after.lo[1]) / 2, (before.hi[2] + after.lo[2]) / 2];
      else p = before ? before.hi : after.lo;
      pos.set(p, 3 * (st.nodeOffset + k));
      state.width.set((after || before).X, 3 * (st.nodeOffset + k));
    }
  });
  state.prev.set(pos);
  updateDirectors(state);
}
