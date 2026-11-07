# ADR-0130: Continuous collision detection for the 3D solver

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0043](0043-physics3d-angular-dynamics.md),
  [ADR-0009](0009-physics2d-system.md),
  [ADR-0037](0037-physics2d-oriented-boxes-offsets.md),
  [ADR-0031](0031-physics3d-and-backend-seam.md)

## Context

The 3D solver ([ADR-0043](0043-physics3d-angular-dynamics.md)) is a
sequential-impulse solver that resolves contacts the discrete narrow phase finds
between two steps. A contact only exists at a step's endpoints, so a body moving
further than a thin obstacle's thickness in one step is on one side at the start
and the far side at the end, touching neither: it tunnels straight through. A
thrown object at speed, a small projectile, or a large fixed step all reach this,
and the result is a body behind a wall it should have hit.

The 2D solver already closed this
([ADR-0037](0037-physics2d-oriented-boxes-offsets.md)):
`Physics2DSystem::apply_continuous_collisions` sweeps a fast body's motion against
static geometry and clamps it to the first touch. The 3D solver has no equivalent
-- it was left as the 3D refinement row's open half.

Constraints that shaped the decision:

- **The discrete solver is the common path.** Continuous detection is a safety
  net for the fast case, not a replacement. An ordinary step must be unchanged.
- **No contact manifold, no persistent impulse cache.** The 3D solver re-finds
  contacts each step and has no warm-starting, so the continuous pass has to fit
  that model: a position clamp and a velocity correction, no stored state.
- **The swept test must never miss a contact.** A continuous test that reports a
  hit a hair early is acceptable; one that misses a real crossing is not, because
  the whole point is to not tunnel.
- **Shapes are axis-aligned in the solver** (a rotated body is swept as its
  bounds, the same limit the discrete collider has), so the sweep works on
  bounds and inherits that limit honestly rather than implying more.

## Decision

Add a swept test `physics3d::sweep_shape` and a pass
`Physics3DSystem::apply_continuous_collisions` that mirrors the 2D design.

- `sweep_shape(moving_bounds, displacement, target)` is the 3D Minkowski slab
  test: shrinking the moving shape to its centre and growing the target by the
  moving half-extents turns the sweep into a ray cast against an expanded box.
  It returns `Sweep3D{hit, time, normal}`, where `time` is the fraction of the
  displacement at which the shapes first touch and `normal` points back towards
  the mover (the `Contact3D` convention).
- The pass runs **after integration and before the discrete solver**, over each
  awake dynamic body with a collider, using the position it had before the step
  as the sweep's start.
- **Only a fast body is swept.** A body whose motion in one step is no larger
  than its own smallest half-extent is skipped: the discrete pass sees both its
  endpoints, and sweeping a resting body would clamp one that is merely settling
  against the ground. The threshold scales with the body, so it holds in any
  world unit.
- **Only static geometry is swept against.** A moving target is still resolved by
  the discrete pass; sweeping both would need their relative motion, which is a
  later refinement.
- **A contact already present at the start of the sweep (time zero) is left to
  the discrete solver.** Clamping it here would freeze a body sliding along the
  ground it rests on. Only a contact first reached partway through the motion is
  a tunnelling case the pass must stop.
- On the earliest hit the motion is clamped to the touch point and the velocity
  component into the surface is removed with the same restitution rule the
  discrete solver uses, so a body does not accumulate speed through the wall.
- The count is exposed as `continuous_hit_count()`, zero on an ordinary step, so
  a test can prove the pass fired rather than only that the body ended up in the
  right place.

## Consequences

**Easier:**

- A fast body cannot tunnel through a thin static wall: it is stopped at the
  surface instead of appearing on the far side.
- An ordinary step is unchanged: the pass skips every body moving less than its
  own size and skips a sweep that starts in contact, so a resting or sliding body
  is untouched and the discrete solver owns it as before.
- The behaviour is asserted directly: the swept test is checked on each axis
  (entry time and normal), for a miss, and for a start overlap, and a system test
  fires a fast box at a thin wall and proves both that the pass fired and that
  the body rests at the near face; a second proves a slow body is resolved by the
  discrete pass with the continuous pass never firing.

**Harder / not yet done:**

- A fast body swept against *another moving body* still tunnels. The pass only
  sweeps static geometry; relative-motion sweeping is the next refinement.
- A fast-*rotating* body was not swept either -- the same gap 2D had
  (`physics2d-rotational-ccd`), closed for 3D in
  [ADR-0149](0149-physics3d-rotational-ccd.md): a spinning box's corner arc is
  now swept and clamped, while a sphere is never swept for spin.
- A rotated body is swept as its axis-aligned bounds, so a near-corner crossing
  can be reported a hair early. Conservative, never a miss; an oriented sweep is
  later work.
- The pass is O(fast bodies x all bodies) per step. Only fast bodies enter the
  outer loop, and the inner scan is over bodies, so a step with no fast body
  costs one comparison each; a broad-phase-filtered candidate set is the change
  if a profile asks for it (ADR-0017).

## Alternatives considered

- **Sub-stepping the whole step.** Rejected: it changes every body's trajectory,
  is expensive for the common case, and would make the discrete solver's results
  depend on the step count. The continuous pass touches only the bodies that can
  tunnel and leaves every other result bit-for-bit.
- **Speculative contacts (allow a small penetration and resolve it next step).**
  Rejected for now: it needs a contact manifold and a persistent cache the 3D
  solver does not have, and it changes resting behaviour. A position clamp is
  smaller and fits the existing model.
- **Sweep moving bodies against each other too.** Deferred, not rejected: it
  needs each pair's relative motion and a wider pass, which is a larger change
  than this refinement; the discrete pass already resolves a moving pair whose
  relative speed is not extreme.
- **Sweep every dynamic body, not only a fast one.** Rejected: it doubles the
  cost and, worse, clamps a body settling against the ground, because its
  start-of-step contact is a legitimate resting contact the discrete solver must
  own.
