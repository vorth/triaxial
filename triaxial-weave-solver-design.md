# Triaxial Weave Simulation: Solver Design Notes

## Goal

Real-time simulation of triaxial weave: many strips woven together with over-under contact points between strips. Strips are open with bare ends most of the time, and closed loops also occur. Strips bend in one plane but not the other, controlled by a stiffness parameter. Strips do not twist. Thickness is desirable but not required.

## Terminology

**Self-contact** means strips colliding with other strips, and with themselves. In a weave, strips press on each other at every crossing, and a closed loop can fold back and touch its own earlier section. The solver must prevent interpenetration in both cases.

## Recommended approach

Use **XPBD** (extended position-based dynamics). Each strip is a chain of nodes, and each physical effect is a constraint that the solver projects toward satisfaction. This gives stable, fast results suitable for real-time interaction, and it is the same family of methods Kangaroo uses.

## Strip model

- **Centerline:** a chain of nodes with fixed rest segment lengths. Stretch is a distance constraint with high stiffness, so strips are effectively inextensible.
- **Anisotropic bending:** a bending constraint at each interior node. Its stiffness depends on the bending direction, so out-of-plane bending is cheap and in-plane bending is expensive. This matches a flat strip, which bends readily across its flat face and resists bending along its width. Over-under bends in the weave should use the easy axis.
- **No twist:** each node carries a director that stays perpendicular to the tangent and rotates only with bending. No full twist variable is needed, which keeps the state small.
- **Thickness:** treat each strip as a thick line segment with radius equal to half its thickness. Contact then reduces to segment-to-segment distance, which is cheap to compute.

## Contacts

- Detect contacts with a spatial hash to find nearby segments, then compute closest points between segment pairs.
- For each contact within the thickness distance, add a non-penetration constraint that pushes the two segments apart along the contact normal.
- Over-under order comes from the weave topology. Strips cannot pass through each other, so the constraints keep each crossing's order fixed. The initial pattern is set by the user and preserved by the solver.
- Add friction at contacts. Without friction, a loosely woven structure may slide apart. With friction and some tension in the strips, the weave holds its shape.

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
