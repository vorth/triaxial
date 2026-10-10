# triaxial — weave graph designer

A vanilla-JS, no-build-step Three.js app for designing "weave graphs": polygonal
cells joined vertex-to-vertex, with a triangle filling every gap between them.
Open `index.html` directly or serve the directory with any static file server.

Run tests with `npm test` (Node's built-in test runner, no dependencies) — see
`weaveGraph.test.js`. Run it after any change to `weaveGraph.js` or `physics.js`.

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
`physics.js`'s `seedPlacement`): given 3 consecutive polygon vertices `A, B,
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

## Physics model (physics.js)

One `forces` accumulator per frame; every term below just adds into it
(order between force terms doesn't matter — they're independent until
summed), then one semi-implicit-Euler integration step applies the total,
followed by a separate hard-clamp pass (direct position correction, not a
force) that keeps every edge within ±10% of unit length.

Current force terms (`DEFAULT_PARAMS` in `physics.js`):
- **spring** (`springStiffness`): edges as stiff springs toward unit rest
  length.
- **angle-regularity** (`angleStiffness`): per-face, rotates each vertex
  in-plane toward the ideal regular-polygon interior angle. Operates
  independently per face.
- **planarity** (`planarStiffness`): per-face, pulls each vertex's
  out-of-plane component (relative to that face's own best-fit
  centroid/normal) to zero. Also independent per face — has no opinion
  about matching a neighboring face's plane.
- **dihedral** (`dihedralStiffness`, **defaults to 0** — see below): for
  every edge with 2 adjacent faces, pulls the triangle-side apex toward a
  180° dihedral relative to the polygon on the other side.
- **collinearity** (`collinearStiffness`): the newest and currently most
  load-bearing flatness term — see below.
- Global momentum removal (not a tunable term): after all forces are
  summed, strips net force and net torque (exact 3×3 inertia tensor) across
  the *whole* graph, so the mesh can deform but never drift or spin as a
  rigid body regardless of any per-term imprecision.

### Collinearity term — what it is and why it's the main flatness mechanism now

At every degree-4 vertex `v` (interior OR boundary nesting vertex — see the
edge-vs-face-count correction above), find its 4 neighbors in true cyclic
order via `WeaveGraph.degree4VertexNeighbors()` (reconstructed from each
face's own adjacency-at-v, chained — works for both the closed 4-cycle case
and the open 3-or-2-face chain case at a boundary gap). The two **opposite**
pairs (`n0,n2` and `n1,n3`) should each be collinear through `v`. Verified
correct even at boundary vertices: hand-derived that for the seed hexagon
corner (60° triangle + 120° hexagon + 60° triangle + 120° gap = 360°
exactly), both opposite-pair sums come out to exactly 180°, so enforcing
both simultaneously at a boundary vertex is mathematically sound, not
over-constraining.

The force itself: `v` is projected onto the current line through each pair,
*and* `v` moves too (not just the two neighbors) — see the bug history
below for why that matters. Neighbors are pulled toward the reflect-through-v
target at half strength since `v` is now also self-correcting.

### Bugs found and fixed this session (read before touching these files again)

1. **`hingeVertexOf` cyclic-wrap bug** (dihedral term): used
   `Math.min(ia,ib)`/`Math.max(ia,ib)` to find a face's neighbors of an
   edge, which is wrong whenever the edge wraps around the array boundary
   (e.g. edge `(1,8)` in triangle `[1,2,8]` — indices 0 and 2 are cyclically
   adjacent via wraparound, not via a "forward" 0→2 range). Silently
   returned an empty hinge set for any wrapped edge, meaning dihedral did
   nothing there. Fixed by explicitly checking `(ia+1)%n===ib` vs.
   `(ib+1)%n===ia` to find the true cyclic predecessor/successor.

2. **`degree4VertexNeighbors` (originally `interiorVertexNeighbors`)
   silently dropped every fully-interior (4-face) vertex** — a regression
   introduced while generalizing the walk to also cover boundary vertices.
   The forward-walk loop ran a fixed `faceNeighborPairs.length - 1` steps,
   which for a closed 4-cycle wraps back around and re-appends the starting
   vertex as a spurious 5th entry, failing the `length === 4` sanity check
   and dropping the vertex entirely. This meant **the collinearity term was
   never actually reaching any interior vertex** for a long stretch of this
   investigation, and every "collinear-only still curls" result from that
   period was measuring a term that wasn't doing its main job. Fixed by
   capping the walk at `neighbors.length < 4` instead of a fixed step count.

3. **Collinearity term's original design only ever treated `v` as a fixed,
   correct anchor** — pulling its neighbors toward collinearity through
   `v`'s *current* position, but never correcting `v` itself toward its own
   neighbors. Since every interior vertex is *also* a neighbor in several
   other vertices' constraints, a vertex that drifted slightly off-plane
   for any reason would have its error faithfully propagated outward to its
   neighbors instead of corrected. Proved this precisely with a from-scratch
   synthetic test (plain JS objects, no app code): one isolated constraint
   is perfectly stable; two constraints sharing one vertex reliably
   amplifies a small initial perturbation into significant drift. Fixed by
   also projecting `v` onto the current line through each neighbor pair
   (not just moving the neighbors), at full strength, with the neighbor
   pull reduced to half strength since `v` now self-corrects too.

4. **Seed hexagon's boundary triangles were isosceles, not equilateral** —
   apex radius was a hardcoded guess (`hexR + 0.85` ≈ 1.85) instead of the
   derived value `hexR·cos(30°) + √3/2 = √3 ≈ 1.732`. Fixed; verified all
   three edges of a boundary triangle come out to exactly 1.0 now.

### Open issue: dihedral conflicts with collinearity once a triangle borders 2 polygons

**`dihedralStiffness` now defaults to 0** and is NOT currently reconciled
with collinearity. Root cause, confirmed via isolated-term long-run tests on
real hex+triangle topology (not synthetic grids — the user correctly
pushed back on square/rectangular test cases not representing our actual
60°/120° topology):

- `collinear + spring` alone (or `+angle`, or `+planar`) converges cleanly
  toward flat over a long run (tens of thousands of steps) on a 37-polygon
  all-hexagon mesh grown from the seed.
- Adding `dihedral` back in (even at its old modest default of 0.08) reliably
  destabilizes that same mesh into significant, growing curl.

The mechanism (confirmed earlier in the same investigation, before the
collinearity-vs-dihedral isolation test): once a mesh has more than one
polygon, some triangles legitimately border **two different polygons** on
two of their three edges (this happens naturally — e.g. the seed hexagon's
own flanking triangles, once a neighboring polygon is placed beside them).
The dihedral term computes its "fold to 180°" target **independently per
edge**, so such a triangle's two non-shared vertices each get pulled toward
flatness relative to a *different* polygon, with nothing coordinating the
two targets. An averaging fix (collect all per-vertex pulls, apply the mean
instead of the sum) was tried and did NOT resolve it — the bug is in the
target *direction* being wrong once a triangle has 2 polygon neighbors, not
in over-summed magnitude.

Dihedral's original purpose — pushing boundary triangles to visibly fold/
pop outward from their cell, per the project's original design goal — is
currently unserved by collinearity (which only acts on degree-4 vertices;
a genuinely open boundary edge with no second face can't have a dihedral
angle at all, and collinearity doesn't model "pop outward" the way dihedral
was meant to). If boundary pop-out is wanted again, dihedral needs either:
(a) to be restricted to true single-polygon-neighbor triangles only (not
fire at all on a triangle that already borders 2 polygons), or (b) a
properly coordinated multi-edge target (e.g. treat the triangle as a rigid
body and least-squares-fit a single orientation satisfying all its
polygon-adjacent edges at once) — neither has been implemented or tested
yet.

**Do not assume orientability when reasoning about face normals here** —
the user flagged early that the eventual mesh editor should support
non-orientable constructions (e.g. a Klein-bottle-style embedding), so two
adjacent, coplanar polygons' normals cannot be assumed to point the same
way. Collinearity and planarity as currently written are orientation-
agnostic (they never compare one face's normal sign against another's) —
keep it that way in any future term.
