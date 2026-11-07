# ADR-0141: A two-axis blend space for directional locomotion

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0124](0124-animation-blend-trees.md),
  [ADR-0104](0104-skeletal-animation.md),
  [ADR-0123](0123-skeletal-state-machines.md)

## Context

`AnimationBlendTree` (ADR-0124) is a one-axis blend: two clips and a single
parameter. It covers locomotion that varies with speed alone -- walk into run --
which is the common case for a character that only ever moves forward.

It does not cover a character that strafes. A third-person character, or a
twin-stick shooter, moves on two axes at once: forward/back and left/right. The
clips that cover it -- idle, walk forward, run forward, strafe left, strafe
right, and the four diagonals -- do not lie on a line. A one-axis blend cannot
mix "run forward" with "strafe left" to make "run forward-left", which is exactly
the pose a player expects when they hold two keys at once.

Constraints that shaped the decision:

- **The blend is a triangulation, not a grid of independent lerps.** Blending
  each axis separately (x on one tree, y on another) does not generalise to
  diagonals: it double-counts the clips at the corners and cannot express a
  diagonal clip at all. The general answer is a blend space: each clip is a point
  in the parameter plane, and the pose for a parameter is the weighted mix of the
  points that surround it. The weights are the barycentric coordinates of the
  parameter in the triangle that contains it -- the natural generalisation of the
  one-axis `lerp`.
- **The triangulation is Delaunay, so the grid need not be regular.** A
  four-corner grid triangulates either way; a grid with a centre clip, or an
  irregular scatter of clips, does not. Delaunay triangulation is the standard
  choice -- it maximises the smallest angle, so no sliver triangle makes a weight
  jump -- and it is the same algorithm a blend-space editor draws.
- **A parameter outside the authored grid clamps, it does not extrapolate.** The
  clips are only meaningful over the region the author placed them in; past it
  the blend falls back to the nearest sample at full weight, so the character
  holds the closest clip rather than a mix that can leave the clips' convex hull.
- **The sample count is tiny.** A locomotion grid is a handful of points, so the
  quadratic Bowyer-Watson triangulation is cheap and clear; an incremental
  algorithm would be premature.

## Decision

Add `AnimationBlendSpace2D`, a component holding a list of `BlendPoint`s (a clip
at a `Vec2` position) and a `Vec2 parameter`, and `AnimationBlendSpaceSystem`,
which evaluates it after `SkeletonAnimationSystem` exactly as
`AnimationBlendTreeSystem` does.

- `AnimationBlendSpace2D::weights(out, capacity)` is the whole geometry: it
  Delaunay-triangulates the sample positions, locates the parameter in the
  containing triangle, and returns the barycentric weights of its vertices, up to
  three. A parameter outside every triangle returns the nearest sample at weight
  one. This is a pure function of the samples and the parameter, so a test
  asserts it with no scene.
- The system samples each selected clip at the animator's clock scaled by that
  point's `speed` (so one authored rate still drives the whole space, and a
  per-point speed only matches clips of different lengths), then blends the poses.
  A three-way blend is two nested two-pose blends: blend the first pair by the
  second's share of their combined weight, then blend that toward the third by the
  third's weight. On the linear channels this is the exact barycentric mix; on
  rotations it is a repeated slerp, the standard approximation.
- The additive layer is the same one the blend tree carries: a `additive_clip`,
  `additive_weight`, and its own clock, composed over the base with
  `compose_additive`. A hit reaction or a lean composes over the locomotion grid
  rather than replacing it.
- The component serializes as one compact JSON string on the `space` property and
  is registered for reflection, the same shape the blend tree and the state
  machines use (reflection has no list-of-struct kind, and a clip reference is a
  property, not a sub-value).

## Consequences

Easier: a strafing character gets its directional locomotion from one component
with no per-clip special-casing; a designer places clips in the plane and the
blend follows, including a diagonal clip; the geometry is a pure function that an
editor can preview and a test can pin without a scene or a renderer.

Harder: the component is a second blend component alongside the tree, so a scene
may carry either -- a node that wants both speed and direction uses the space,
which subsumes the tree's one axis. The blend is a repeated slerp rather than a
true barycentric rotation mix, which is exact enough for gameplay and is the
standard trade-off. A parameter outside the authored region holds the nearest
clip rather than extrapolating, which is deliberate but is a behaviour an author
must know.

## Alternatives considered

- **Blend each axis independently.** Rejected: it cannot express a diagonal clip
  and double-counts the corners.
- **A regular grid with a hand-written cell lookup.** Rejected: it forces the
  samples onto a lattice and a cell lookup is more code than the triangulation
  for the same result on a regular grid, while failing on an irregular one.
- **A true barycentric rotation blend (log/exp map).** Rejected for this cut:
  the repeated slerp is the standard approximation and the difference is below
  what gameplay sees; a log/exp blend is a later refinement if a clip ever needs
  it.
- **Extend `AnimationBlendTree` with a second axis.** Rejected: it would make
  every existing one-axis tree carry an axis it does not use, and the two have
  different weight models (an interval versus a triangulation).
