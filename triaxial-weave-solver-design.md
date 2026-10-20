# Triaxial Weave Simulation: Solver Design Notes

## Goal

Real-time simulation of triaxial weave: many strips woven together with over-under contact points between strips. Strips are open with bare ends most of the time, and closed loops also occur. Strips bend in one plane but not the other, controlled by a stiffness parameter. Strips do not twist. Thickness is desirable but not required.

## Agreed design for this app (October 2026)

This section records the decisions made for the triaxial app. Where it conflicts with the general notes further down, this section wins; superseded bullets below are marked.

### Ownership of positions

| Layer | Owns | Changes when |
|---|---|---|
| **WeaveGraph** (`weaveGraph.js`) | Topology only: faces, boundary, placement records. No positions. | User edits |
| **StripModel** (`strips.js`) | Strips as ordered crossings, open/closed, color, over/under side of each pass | Rebuilt on user edits |
| **StripState** (`strips.js`) | The only positions: node positions, previous positions, inverse masses, one width vector per strip, material coordinates | Every solver substep; reconciled on edits |
| **Crossing table** (in StripState) | For each graph vertex, the material coordinate `s` of each of its two passes | Re-tracked every substep |

A graph vertex has no stored position. `crossingPosition(v)` returns the midpoint of its two passes' points, evaluated at their material coordinates. Nesting-vertex spheres and placement seeding (the chord-ratio construction in `seeding.js`) read positions this way. Interior vertices keep their crossing records so that a deletion that turns one back into a nesting or apex vertex has its position immediately.

### Strip mechanics

- **Inextensible:** segment length equals the node spacing `h`, as a rigid constraint.
- **No in-plane bending (rigid); twist allowed with a stiffness** (revised; originally no twist at all). Each segment has its own unit width vector `b`. At each node, the strip may not curve along that node's width vector: `(x[i+1] − 2x[i] + x[i−1]) · b_i = 0`, where `b_i` averages the node's two segments. After each constraint pass, each segment's `b` is updated by trading off three things: being perpendicular to its own and the neighbouring segments, so the strip bends about its easy axis; staying aligned with its neighbours' `b` (twist stiffness, default 0.8 on a 0–1 slider); and being perpendicular to the contact normal wherever it touches another strip (face contact). The first version used one `B` per strip and no twist, so every strip had to lie in a single plane. Pentagons then made it ripple and curl; see the findings below.
- **Out-of-plane bending:** soft, with adjustable stiffness.

### Crossings

- **Sliding with friction while touching** (revised after step 3; this replaces "frictionless"). Each pass's material coordinate `s` is re-tracked every substep with a local closest-point search, so whatever slip friction allows becomes sliding. At a crossing pressed together, Coulomb friction (`friction`, default 0.5) undoes the tangential slip of the two material points, up to `friction ×` the normal correction. Crossings that aren't pressed together slide freely.
- **Apex crossings are clipped.** A fraction `apexGrip` (default 1, meaning no slip) of the slip is undone every substep, whatever the pressure, as if the two strips were clipped together.
- **Contact normal:** `n = tA × tB`, computed locally at each crossing; no global orientation is assumed. The over/under side of each pass is fixed by the strip model's Tait-sign rule and never changes.
- **Apex crossings:** two-sided contact. The signed separation along `n` equals `side · thickness`, so the two free ends must stay in contact.
- **Interior and nesting crossings:** one-sided contact. The separation must be at least `thickness`, with the sign fixed, so a crossing can't flip even if a step moves farther than the thickness. Strips may lift apart.
- **Face contact:** `B · n = 0` for both strips, but only while the crossing is in contact. This keeps touching strips flat against each other and stops a straight strip from rolling on edge.
- **Hole-inversion guard:** for consecutive crossings `a, b` along a strip, `s_b − s_a ≥ gmin`, with `gmin` about the strip width. Every hole side is such a span, so no hole can collapse through zero or turn inside out.
- **Hole size** is set by the strips and by friction; no spring pulls on it. Span springs toward length 1 were tried and removed: compressing a strip that can't bend within its own plane buckles it, which showed up as rippling and curling in the app.

### Tails

Open strips extend past their end apex. Each tail is kept between `apexExtension` and `2 · apexExtension`: nodes are appended when a crossing slides toward the end, and trimmed when the tail grows too long. A per-strip `sStart` absorbs changes at the start end. Closed strips have no tails.

### Edits (place and delete)

Edits don't operate on strips one case at a time. Instead:

1. Apply the graph edit.
2. Seed positions for the new vertices from `crossingPosition()` of the site chain.
3. Retrace the strip topology from the graph.
4. Reconcile StripState one span at a time, where a span is the material between two consecutive crossings on a strip:
   - If the two crossings were adjacent on an old strip, copy that material between the old coordinates, preserving its arc length.
   - Otherwise, generate the span from the seeded crossing positions with the over/under offset.
   - Tails copy old material where it exists and extrapolate the rest.
5. Set previous positions equal to current positions on edited strips, so the edit adds no impulse.

An untouched strip carries over unchanged.

### StripState layout

```
nodes:     x[3N], xPrev[3N], invMass[N], width[3N] (segment j's width vector at node j's slot)
strips:    nodeOffset, nodeCount, closed, h, sStart
crossings: kind, passA {strip, s}, passB {strip, s}, side, XPBD multipliers
```

Everything is flat and indexed, so the solver can move to a Web Worker or WebAssembly later.

### Implementation order

1. **Done.** Crossing table and `crossingPosition()`, fed by today's derived strips. Spheres and placement seeding use it.
2. **Done** (`stripState.js`). StripState with one `B` per strip, plus span-by-span reconciliation on place and delete. Not yet connected to the app: with no solver to move it, this state would sit still.
3. **Done** (`solver.js`; the app runs on it). The XPBD solver: stretch, planarity, bending, crossing contacts, face contact, span springs and minimum gaps, plus re-tracking and tail grow/trim. Findings are below.
4. Remove `physics.js` and the graph-position code paths.

### Solver findings (step 3)

All measurements are in Node, on meshes grown from the seed hexagon.

- **Damping must be high.** The default is 0.95 of velocity removed per frame, which is close to quasi-static relaxation. XPBD turns position corrections into velocity, so wherever rigid constraints disagree, each correction adds energy. At 0.02 and at 0.3 the solver blew up: crossings flipped, strips stretched more than 100%, and tails grew without limit.
- **Hexagonal meshes settle cleanly.** A 7-polygon mesh reaches stretch and planarity errors of about 0.1%, with spans at 1.0 ± 0.003. The flat seed stays at rest.
- **A pentagon conflicts with rigid planarity, as predicted.** With planarity rigid (stiffness 1), the mesh never settles: 10–15% stretch error spread across a quarter of all segments, and strips keep sliding. Softening planarity to 0.8 or 0.6 doesn't help. With planarity off, the same mesh settles to 0.6% stretch, but stays nearly flat (z ±0.1), because the strips absorb the pentagon by bending within their own plane. Over/under held in every run, with no crossing flipped. A dome needs strips that can twist while still resisting in-plane bending. That means a width vector per segment, not per strip (the Cosserat option), and is a decision still to be made.
- **Revision after first use in the app:** strips rippled and curled. In a simulated interactive session (ten placements, one of them a pentagon), the sharpest bend between neighbouring segments reached about 50°, and strips came within 0.05 of themselves at 26 points. Removing the span springs cut most of this: the mesh's height (z extent) fell from about 2 units to 0.7, with no near self-contacts. With the springs removed and friction added, hexagon-only sessions are clean, with or without friction: bends of about 2°, flat to ±0.15, no near self-contacts. Everything that remains comes from non-hexagons meeting rigid planarity: bends up to about 30°, curling to ±1, worse over time when friction stops the strips from sliding to relieve the stress. With planarity off, the same session stays smooth (4° bends) and flat.
- **Twist (revision 2).** With per-segment width vectors and twist stiffness 0.8, the same pentagon session no longer curls: no near self-contacts, the sharpest bend is about 9° (30° before), stretch 0.2%, and strips stay in the sheet (width vectors tilt out of it by at most 11°). Twist stiffness between 0.4 and 1 makes little difference. Hexagon-only sessions are unchanged.
- **Why there's still no dome.** The pentagon mesh stays flat (z ±0.13) because hole sizes aren't held. Strips slide, so the pentagon shrinks (sides about 0.7, angles still about 108°) and span lengths spread from 0.68 to 1.45. Straight strips can enclose a pentagon in a flat plane if the holes distort. Unit spans would forbid that, because equilateral triangles fix the angles, and the angle deficit would have to become a dome. Friction only acts where strips are pressed together, and they press only lightly, so it barely holds anything. In an experiment that clipped every crossing like an apex, spans stayed near 1 (90% between 0.98 and 1.003) and the mesh began to dome (z ±0.24), at the cost of 17° bends. What should hold hole sizes is still an open decision.
- **Two defects in revision 2, found by measuring curvature in the strip frame** (X = width, Y = length, Z = thickness; strips must only curve around X).
  - **The in-plane rule ("no curving around Z") was only applied between adjacent nodes.** That removes kinks quickly but straightens a gentle sideways curve very slowly: a single strip bent to radius 5 still had 0.21 curvature after 100 frames. So strips curved like roads. It is now applied at node spacings 1, 2 and 4, which straightens the same arc within 10–100 frames. Spacings 8 and 16 destabilized the width updates.
  - **Bending stiffness was isotropic.** It was a chord-length rule that resisted curving around X just as much as around Z, so the paper-like bend was never the cheap one. It now acts only on curvature around X (the second difference along Z = T × X).
  - In the pentagon session, measured against the weave sheet, these cut sideways curvature from p95 0.094 to 0.015. Up-and-down ripples are gone, the sharpest bend fell from 9° to 4.6°, and X rolls out of the sheet by at most 10°.
- **Still no dome, now clearly because of sliding.** With sideways curving prevented, the strips slide until the pentagon is small enough to lie flat: its sides shrink to 0.3–0.5, and spans across the mesh range from 0.32 to 2.3. Friction only acts on crossings pressed together, which happens rarely, so it doesn't prevent this.
- **Crossing grip and Z alignment** (revision 4). The pentagon is placed with sides of about 1, then collapses within a few frames. Each crossing's two strips slide apart along each other, because the rigid in-plane rule straightens the seeded strips while the sheet stays flat. They slip mostly while the crossing has lifted apart, which happens often, since interior contacts are one-sided and nothing presses them together.
  - **Grip** (`crossingGrip`) is static friction that doesn't depend on pressure, active while touching. A fraction-of-slip version acted like viscosity and never held. The static version holds hole sizes (spans 0.96–1.02 at 0.5) but brings back sideways curvature (p95 about 1). It is off by default.
  - **Z axes now align at every crossing** (the user's constraint): both strips' width vectors are made exactly perpendicular to the crossing normal, so both thickness axes lie along it. Previously this was only a soft, touch-only term. With grip 0.5 it cuts sideways curvature from about 1 to 0.34 and the mesh begins to dome (z −0.77..0.5), but 5.5% stretch error and 15° bends remain.
  - **Remaining limit: the solver.** More substeps (30) blew up: 85% stretch, collapse. Lifting the pentagon into a cone first didn't produce a clean dome either. The rigid constraints disagree, and a single Gauss-Seidel pass per substep turns each disagreement into motion. The likely fix is in the solver itself: proper XPBD multipliers accumulated over several iterations per substep, or a stiffer solve for the rigid rules.
- **Curl scaling** (revision 5; terms: curl = bending around X, swerve = bending around Z). The curl slider did control curl, but like swerve before its fix it only compared adjacent nodes. At slider 1.0 a single strip curled to radius 5 was barely straighter after 100 frames, and 0.6 and 1.0 behaved the same. The slider's 1.0 is now truly rigid. Applying curl at spacings 2 and 4 as well, as swerve does, made a lone strip straighten within 10–100 frames. In the weave it was disastrous: over/under requires curl at every crossing, the stiff rule fought the contacts, and at curl 0.6 sideways curvature (p95) went from 0.02 to 2.8 with 17% stretch. So curl stays at adjacent nodes. Gentle curls are therefore still only weakly resisted, and in the weave a stiffer setting gives slightly *more* curl (p95 0.21 at 1.0 against 0.17 at 0.6). That is the same solver limit as above.
- **Iterations with accumulated multipliers** (revision 6). Each substep now runs the constraints `iterations` times, and curl and swerve accumulate their XPBD multipliers across those passes. Results in the pentagon session:
  - With grip 0.5, sideways curvature (p95) falls from 0.18 (1 iteration) to 0.10 (8 iterations × 5 substeps), but stretch stays at 5–7% and time per frame rises from 16 to 58 ms.
  - Curl applied at spacings 2 and 4 no longer wrecks the weave with 4 iterations (sideways 2.8 → 0.16), but leaves 10% stretch and the mesh flat.
  - Default stays 1 iteration, with an Iterations slider.
- **What iterations can't fix.** A dome is a buckled state: the flat pentagon sheet has to bend out of the plane. Nearly every correction the solver makes acts within the sheet (stretch along the strip, swerve along X, grip along the crossing). Only curl and the contacts act out of the plane, and the flat sheet is symmetric, with nothing to choose up from down. So the solver settles on a flat compromise that violates the in-sheet rules, which is the persistent 5–10% stretch. Two directions remain: seed pentagon placements already curved, using the expected intrinsic geometry (regular angles and unit spans force a lift), or use a solver that minimizes a global energy rather than projecting constraints locally.
- **Speed:** about 9 ms per frame for a 16-polygon mesh (1,400 nodes, 87 crossings, 10 substeps), once warmed up. About 20% of the time is garbage collection, mostly from crossing re-tracking. Moving the solver to a worker or WebAssembly remains the scaling path.

## Terminology

**Self-contact** means strips colliding with other strips, and with themselves. In a weave, strips press on each other at every crossing, and a closed loop can fold back and touch its own earlier section. The solver must prevent interpenetration in both cases.

## Recommended approach

Use **XPBD** (extended position-based dynamics). Each strip is a chain of nodes, and each physical effect is a constraint that the solver projects toward satisfaction. This gives stable, fast results suitable for real-time interaction, and it is the same family of methods Kangaroo uses.

## Strip model

- **Centerline:** a chain of nodes with fixed rest segment lengths. Stretch is a distance constraint with high stiffness, so strips are effectively inextensible.
- **Anisotropic bending:** a bending constraint at each interior node. Its stiffness depends on the bending direction, so out-of-plane bending is cheap and in-plane bending is expensive. This matches a flat strip, which bends readily across its flat face and resists bending along its width. Over-under bends in the weave should use the easy axis.
- **No twist:** each node carries a director that stays perpendicular to the tangent and rotates only with bending. No full twist variable is needed, which keeps the state small. *(Superseded: one width vector per strip; see Agreed design.)*
- **Thickness:** treat each strip as a thick line segment with radius equal to half its thickness. Contact then reduces to segment-to-segment distance, which is cheap to compute.

## Contacts

- Detect contacts with a spatial hash to find nearby segments, then compute closest points between segment pairs. *(Deferred: the crossings known from the weave graph are tracked directly; a spatial hash is only needed later for incidental contact.)*
- For each contact within the thickness distance, add a non-penetration constraint that pushes the two segments apart along the contact normal.
- Over-under order comes from the weave topology. Strips cannot pass through each other, so the constraints keep each crossing's order fixed. The initial pattern is set by the user and preserved by the solver.
- Add friction at contacts. Without friction, a loosely woven structure may slide apart. With friction and some tension in the strips, the weave holds its shape. *(Superseded: frictionless; span springs and minimum gaps hold the shape.)*

## Open ends and closed loops

- Open strips are ordinary chains with free endpoints.
- Closed loops are chains whose last node connects back to the first. Bending and contact constraints must wrap around the index, and a closing constraint may be needed to keep the join in place.

## Performance

- Use several substeps per frame, typically 8 to 20 in XPBD. More substeps give stiffer, less jittery contacts, which matters for weaves.
- Implement the solver in Rust or C++ compiled to WebAssembly. Keep state in typed arrays so inner loops stay fast.
- Run the simulation in a Web Worker so rendering does not stall it.
- Use graph coloring or a Jacobi-style update so constraints can be processed in parallel later, with WebGPU as a possible next step.

## Likely issues

- Dense crossings can produce jitter. More substeps, damping, and a tuned contact margin usually help.
- Weave tightness depends on rest lengths and the friction coefficient, so expect tuning.
- Closed loops and crossings near the loop closure are common sources of bugs and should be tested early.

## Suggested development order

1. Prototype a single crossing.
2. Build a small 3×3 weave.
3. Add closed loops.

## Open questions

- Typical number of strips per scene.
- Target platform: desktop browser, phone, or other.
