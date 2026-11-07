# ADR-0134: The extended joint set, in both worlds

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0021](0021-physics2d-joints.md),
  [ADR-0043](0043-physics3d-angular-dynamics.md),
  [ADR-0037](0037-physics2d-oriented-boxes-offsets.md),
  [ADR-0009](0009-physics2d-system.md)

## Context

Both solvers grew from one joint each. The 2D world has a distance joint
([ADR-0021](0021-physics2d-joints.md)) and a revolute joint; the 3D world has a
distance joint and, since the `physics-more-joints` row, a revolute joint. That
set builds a rope, a spring, and a pendulum, and nothing else. A crate welded to
a platform, a door on a hinge that only slides, a piston, an elevator, or a
drawbridge cannot be authored, because no joint holds two bodies in a fixed
relative pose or leaves exactly one axis of sliding free.

The two worlds also disagree on the distance joint's reach: it holds a single
length, so it cannot be a rope that goes slack, a leash that stops a body flying
off, or a rod that pushes back when compressed. A joint that only pulls is half a
joint.

Constraints that shaped the decision:

- **The solver is the same shape in both worlds.** A joint is a small set of
  scalar constraint rows solved by sequential impulses, built by a pure function
  from a body pair and a parameter struct, and applied by two shared helpers
  (one for velocity, one for position). New joints must fit that shape rather
  than add a second solving mechanism.
- **Existing behaviour is frozen.** A scene with a distance or revolute joint
  must step bit-for-bit as it did, and a default-constructed component must
  serialize and behave exactly as before.
- **A joint is a component with a stable identity.** It names its two bodies by
  node id, registers with reflection, and round-trips through the serializer, the
  same rule the existing joints follow.
- **One seam, not two copies.** The 2D and 3D solvers are separate code, but a
  joint kind should be one design expressed twice, not two independent ideas, so
  the naming, the parameters, and the tests line up across the two.

## Decision

Extend the joint set in both worlds with a fixed (weld) joint and a prismatic
(slider) joint, and give the distance joint length limits.

### Fixed joint (`FixedJoint2D`, `FixedJoint3D`)

Holds two bodies in a fixed relative pose: their anchors stay coincident and
their relative angle (2D) or orientation (3D) stays at a captured reference.

- In 2D it is the revolute joint's two anchor rows plus one angle-lock row, so
  `build_fixed_rows` calls `build_revolute_rows` and appends the lock. No new
  row maths.
- In 3D it is the revolute joint's three anchor rows plus an orientation lock,
  built the same way.
- `reference_angle` (2D) / the reference orientation (3D) is captured from the
  pair's pose when the joint is authored, so a weld preserves the arrangement it
  was dropped onto rather than snapping the bodies to a common frame. A helper
  (`fixed_reference_angle`) computes the captured value the same way the row
  measures it, so the two cannot drift apart.

### Prismatic joint (`PrismaticJoint2D`, `PrismaticJoint3D`)

Leaves sliding along one axis as the only degree of freedom: it removes the
translation perpendicular to the axis and locks the relative angle, so the bodies
share a frame in which one axis is free.

- One row removes the translation along the perpendicular; the same angle-lock
  row the fixed joint uses removes the rotation. The two kinds share one builder
  for the lock, so the sign convention is written once.
- The axis is expressed in the pair's shared frame, so it follows the bodies as
  they turn instead of naming a fixed world direction that drifts.
- The perpendicular is derived from the axis at build time, so the row's Jacobian
  is a function of the current pose and no state is stored between steps.

### Distance-joint length limits

`min_length` and `max_length` (both zero by default) turn the rod into a limited
one. A value above zero enables that bound; zero leaves that side unbounded, so a
joint with no bounds holds `rest_length` exactly as before.

- When a bound is set the joint treats it as a limit, not a target: the
  separation is compared against the bounds and the joint holds the nearer bound
  when it is out of range.
- Inside the range the row is **slack** -- an all-zero row the shared apply
  helpers already treat as a no-op -- so a rope can go limp instead of pulling
  both ways. A spring is never slack, because its bias is meaningful at every
  length.
- The rule is one small function per dimension, `distance_limit_target`, so the
  builder asks it what length to hold and whether it constrains at all this step.

### Shared across the two worlds

- Each joint is one component with `body_a`/`body_b` node ids, a parameter struct
  decoupled from the component so the maths is testable alone, and a registered
  reflection type so it round-trips and shows in the inspector.
- Each system gathers every joint kind in the same single scene-walk and solves
  them after the contact pass, so a joint correction is the last word on position.
  Body ids are read through accessors on the gathered `Joint` record, so the solve
  and island passes do not branch on the kind.
- Every joint joins the island graph, so a welded or slaved pair sleeps together.

## Consequences

**Easier:**

- A fixed relative pose is authorable: a weld, a drawbridge, a crate pinned to a
  moving platform.
- One axis of free motion is authorable: a slider, a piston, an elevator, a door
  that only slides.
- A distance joint can be a rope (a minimum, pushes back when compressed), a
  leash (a maximum, stops a body flying off), or a slack line between them,
  without a second joint type.
- The row builders compose from what exists: the fixed joint reuses the revolute
  rows and the two new kinds share one angle-lock row, so the sign conventions
  live in one place and the tests pin them.
- Behaviour is asserted directly: pure row tests pin the angle-lock and
  perpendicular signs, and system tests prove a weld holds a body off a pivot
  against gravity and turns one that starts twisted, a prismatic joint removes
  across-axis motion while leaving along-axis motion free, and a leash stops a
  fast body short of its full travel.

**Harder / not yet done:**

- A prismatic joint's translation limits and a linear motor landed later; so did
  a revolute motor. See [ADR-0135](0135-joint-reactions-and-breaking.md) for
  breaking and [ADR-0153](0153-joint-motors-and-limits.md) for the motors and
  prismatic limits.
- A revolute joint has no *angular* limits: it swings freely through a full turn.
  A limited hinge (a door that stops at 90 degrees) needs the same bound machinery
  the prismatic joint now has, applied to the relative angle row. Not yet done.
- The 2D and 3D solvers remain separate code, so a joint kind is written twice.
  The two share the design and the tests, not the implementation; unifying them
  is not worth the indirection while the row shapes differ (2D is a `Vec2`, 3D a
  `Vec3` with an orientation).

## Alternatives considered

- **A generic constraint row with a mask of locked degrees of freedom.** Rejected
  for now: it is a larger change than the joint set needs and would rewrite the
  existing distance and revolute joints, risking the frozen behaviour, for a
  shape the explicit builders already express clearly.
- **Author a fixed joint as a revolute joint plus an angle limit.** Rejected: it
  would make a weld two components and a limit, which is harder to author and to
  serialize than one joint that means what it says.
- **Express the prismatic axis in world space.** Rejected: the axis would drift
  as the bodies turn, so a slider on a rotating platform would not stay on its
  rail. The shared-frame axis is what makes the joint a frame rather than a
  direction.
- **Make the distance joint a spring with a rest length and no limits.**
  Rejected: a rope and a leash are rigid constraints, and approximating them with
  a stiff spring rings and depends on the step size. The limit is exact.
