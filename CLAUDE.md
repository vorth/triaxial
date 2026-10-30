# triaxial — weave graph designer

A vanilla-JS, no-build-step Three.js app for designing "weave graphs": polygonal
cells joined vertex-to-vertex, with a triangle filling every gap between them.
Open `index.html` directly or serve the directory with any static file server.

Run tests with `npm test` (Node's built-in test runner; `npm install` first
for the one dev dependency, Rapier). Run it after any change to
`weaveGraph.js`, `strips.js`, `stripState.js` or `rapierWeave.js`.

## App shell (design-app)

`index.html` wraps the app in the `<design-app>` web component
(https://github.com/vorth/design-app), loaded by absolute URL from GitHub
Pages (unpinned). It supplies the nav bar, menu, file New/Open/Save
(`.weave.json`), undo/redo and autosave (key `triaxial.weave`); the canvas
and toolbar sit in its default slot, help in `slot="info"`, solver tuning in
`slot="settings"` (tuning stays in its own localStorage, not the document).

Document = `{ version: 1, graph: WeaveGraph.toJSON(), sigma, positions }`,
positions being every graph vertex's `crossingPosition`. Every edit goes
through `applyEdit` in `main.js`: undo/redo restore whole-document snapshots
rather than calling `deleteAt`. A restore carries surviving strips over
(`reconcileStripState`) and seeds only the vertices the current strips lack;
opening a file or a sigma change rebuilds from scratch.

## Mesh topology (weaveGraph.js)

A weave graph is a mesh of alternating "face polygons", polygons with three or more sides,
and "weave triangles".
The whole structure is a 2-manifold, except for a boundary, or several boundaries.
Away from any boundary, every edge adjoins a face polygon and a weave triangle, 
so there are always three face polygons around a weave triangle, 
and N weave triangles around a face polygon with N sides.
These invariants are broken at a boundary, as we'll see below.
Note that face polygons can be triangular themselves,
but such triangles never count as *weave* triangles; the two classes remain distinct.
For the rest of the document, we'll use "polygon" to mean "face polygon", 
and "triangle" to mean "weave triangle".

A *boundary* of the mesh is a cycle of edges, all bounding weave triangles.
Since only weave triangles are exposed, a boundary edge is ready for a new
face polygon to be attached.  The triangles on the boundary are either
pointing out, away from the interior of the mesh, or pointing into the interior.
An outward triangle has two edges on the boundary; an inward triangle has only one.
We call the outward ones *apex* triangles.

As a cycle of edges, a boundary is also a cycle of vertices. 
There are three kinds of vertices on that cycle:
- *type A* join two edges of an apex triangle
- *type B* join one edge of an apex triangle and one edge of an inward triangle
- *type C* join edges of two inward triangles
Type A vertices are interspersed around the boundary, breaking it up
into separate *nesting sites* for new face polygons.  A nesting site has
the pattern ABA, ABBA, ABCBA, ABCCBA, and so on, with zero or more C vertices,
one or more B vertices, and exactly two A vertices.
The application highlights *nesting vertices*, those of type B or C, where
the user can click to nest a new face polygon at that nesting site.
Clicking any nesting vertex within a nesting site is equivalent.

**Placing an n-gon on a site with `k` nesting vertices** (`A-left, s1..sk,
A-right`, so `k+2` existing vertices total) has three cases, keyed off how
`n` compares to `k+2`:
- `n > k+2` (**normal**): generate `n-k-2` fresh mid-vertices via the
  chord-ratio construction (below), fan triangles around the remaining open
  edges.
- `n == k+2` (**joinEdge**): exact fit, no fresh vertices — close the
  polygon through the existing chain directly and bridge `A-left` to
  `A-right` with one new edge + triangle.  This consumes two apex triangles
  and produces one, so the overall count drops by one.
- `n == k+1` (**fuse**): one too many existing vertices for a plain closure
  — merge `A-left` and `A-right` into a single vertex (which becomes a
  new, single-vertex nesting site itself, still open on one side).
  This consumes two apex triangles and produces zero new ones,
  so the overall count drops by two.
- `n < k+1`: fails outright (too many existing vertices for the requested
  polygon).

Two of the cases above reduce the overall count of apex triangles on that boundary.
This corresponds to an edge boundary shrinking, and the mesh closing in on itself.
When a boundary has only one or zero apex triangles and `k` nesting vertices,
the only possible face polygon placement is a single face with `k` edges.

Deletion is the exact structural inverse of whichever case placed a polygon,
gated on the polygon still being a "leaf" (`childCount === 0` — nothing has
been placed on any vertex it created since).
More needs to be specified about deletion, since the code is not working at the moment.

**Chord-ratio construction** (seeding fresh vertex positions, in
`seeding.js`'s `seedPlacementPositions`): given 3 consecutive polygon vertices `A, B,
C`, the next vertex `D` satisfies `D = A + R(n)·(C − B)` — chord `A→D` runs
parallel to chord `B→C`, scaled by `R(n) = sin(3π/n) / sin(π/n)` (closed
form; verified against true regular-n-gon diagonal/edge ratios: R=1 for
squares, golden ratio for pentagons, 2 for hexagons). Sliding the window
`(A,B,C) → (B,C,D)` generates each successive vertex. Since the starting 3
points are real existing mesh vertices (whatever angle the mesh already has
there, not necessarily the new polygon's ideal interior angle), the result
is "affine regular" — correct parallel-chord structure, but sheared by the
real starting angle — not an exact regular polygon. Physics relaxation
finishes the job. Triangles (`n=3`) are the degenerate case: `R=0`, no fresh
vertices, the polygon is exactly `{u, v, w}`.

**Seed hexagon's boundary triangles should be equilateral**, not isosceles —
the apex radius must be the hexagon's apothem (`hexR·cos(30°)`) plus a unit
equilateral triangle's height (`√3/2`), total `√3`. A hardcoded guess
(`hexR + 0.85`) was wrong by ~6.5% and silently stretched two of each
triangle's three edges; this is the kind of "looks plausible, isn't derived"
constant to be suspicious of elsewhere in the seed/layout code.

### Known bug: fuse with two apexes left can create a triangle-triangle edge

If the boundary has exactly two apexes and the nesting site on the far side
of them is a single vertex `s`, a fuse placement merges the two apex
triangles, which both already contain `s` — they end up sharing an edge, and
the boundary collapses to 2 vertices. Not yet fixed; `strips.test.js` has an
`assertWellFormed` guard for this.

### Known bug: placement on a single-apex boundary builds degenerate faces

With exactly one apex left on a boundary, `nestingSiteAt` returns that apex
as both `apexLeft` and `apexRight`, and `placePolygon` then creates a
self-loop edge `(u,u)` and degenerate faces. The spec above says the only
valid placement there is a single `k`-gon closing the boundary; that case is
not implemented. `stripState.test.js`'s fuzz test stops at such sites.

## Woven strips (strips.js)

Strips are read off the weave graph, never stored in it: a strip goes
straight through each degree-4 vertex (opposite neighbors in the
`degree4VertexNeighbors()` ring) and ends only at apexes. Over/under is one
global bit `sigma` plus a local Tait-sign rule per crossing (uses
`firstSectorIsPolygon`), so it never conflicts and needs no orientability.
Three layers: `StripModel` (topology: node chains, crossings as contact
pairs with fixed over/under; rebuilt on graph edits), `StripState`
(`stripState.js`: nodes at fixed material spacing `nodeSpacing` = 0.25, one
width vector per segment, crossing table `passS`), and the render-only
ribbon mesh. StripState is the only home of positions: graph vertices are
located only through `crossingPosition()`, and edits go through
`reconcileStripState` (span-by-span carry-over; every newly generated span
gets exactly `edgeLength` = 1 of material, scaled onto the seeded curve,
because crossings are pinned and a seeded length would be permanent).

### Strip frame terminology (use these terms)

At any point on a strip's centerline: **X** is the width direction, **Z**
the thickness direction, **Y** along the strip. **Curl** is bending around X
(out of the flat strip's plane; the paper-like bend, allowed). **Swerve** is
bending around Z (within the strip's own plane, like a road; never wanted).
**Twist** is rotation around Y (slight twist allowed). Where two strips
cross, their Z axes must align.

## Strip mechanics (rapierWeave.js, Rapier)

The hand-written XPBD solver (`solver.js`) and the older graph-vertex force
model (`physics.js`) were abandoned and deleted in favour of the Rapier
engine (`@dimforge/rapier3d-compat`, pinned at 0.21.0: loaded from unpkg via
the import map in the browser, a dev dependency for Node tests).
`triaxial-weave-solver-design.md` records the XPBD history and why it never
produced a pentagon dome.

- Each StripState segment is a rigid plate (no colliders, so no
  self-contact yet; mass from `setAdditionalMassProperties`). Plate frame =
  strip frame (X width, Y along, Z thickness).
- Neighbouring plates: one generic joint locking translation and rotation
  about Z (no stretch, no swerve). Curl (X) and twist (Y) get joint *motors*
  with target angle 0, which is how Rapier gives a joint axis a spring.
  The JS wrapper in 0.21 returns every joint as a plain generic joint with no
  motor methods, so they are configured through
  `world.impulseJoints.raw.jointConfigureMotorPosition`; recheck this on any
  Rapier upgrade.
- Crossings are pinned (no sliding): a revolute joint between points on the
  two plates' faces, offset thickness/2 along Z toward each other per
  `contactSign`, about both plates' Z axes, so over/under can't flip and the
  Z axes stay aligned.
- Every frame, plate poses are written back into StripState (`readBack`),
  so everything downstream is unchanged. Every edit disposes the world
  and builds a new one from the reconciled state. Strip width and
  thickness reach the physics only on that rebuild.
- Measured defaults (Node, 10 placements including one pentagon, about
  480 nodes, about 2 ms per frame): curl and twist stiffness 1, the top of
  the slider (stiffness 10^4; the slider spans 10^2..10^4 because softer
  strips crumple between crossings and anything stiffer changes little
  but stretches the joints), damping 0.1 (0.5 and above settles
  too slowly to see), 8 solver iterations, sleeping off (it could freeze a
  mesh that is still relaxing). The pentagon mesh goes out of plane
  (z −0.38..0.51) with spans held at 0.985–1.0.

**Do not assume orientability when reasoning about face normals here** —
the user flagged early that the eventual mesh editor should support
non-orientable constructions (e.g. a Klein-bottle-style embedding), so two
adjacent, coplanar polygons' normals cannot be assumed to point the same
way. The crossing joints use only each crossing's own local geometry, and a
closed strip whose width vector comes back flipped across its seam gets a
half-twisted closing joint. Keep any future term orientation-agnostic too.
