# ADR-0149: Continuous collision detection for a spinning 3D body

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0130](0130-physics3d-continuous-collision.md),
  [ADR-0043](0043-physics3d-angular-dynamics.md),
  [ADR-0037](0037-physics2d-oriented-boxes-offsets.md),
  [ADR-0009](0009-physics2d-system.md)

## Context

The 3D solver's continuous pass ([ADR-0130](0130-physics3d-continuous-collision.md))
sweeps a fast body's *linear* motion against static geometry and clamps it to the
first touch, closing the case where a thrown object crosses a thin wall between
steps. It does not see rotation: a long thin body spinning fast enough to sweep
its corners through a wall between two steps passes through, because its centre
barely moves and neither the discrete narrow phase nor the linear sweep looks at
the arc a corner traces. ADR-0130 recorded this as an open gap, the same one 2D
closed with `physics2d-rotational-ccd`
([ADR-0037](0037-physics2d-oriented-boxes-offsets.md)).

Constraints that shaped the decision:

- **The discrete solver is the common path.** Rotation continuous detection is a
  safety net for the fast-spin case, not a replacement; an ordinary step must be
  unchanged, and a body already resting against the surface it spins on must not
  be frozen by the sweep.
- **No contact manifold, no persistent cache.** The 3D solver re-finds contacts
  each step; the rotational pass has to fit that model -- a position clamp and a
  velocity correction, no stored state.
- **The swept test must never miss a contact.** A test that reports a hit a hair
  early is acceptable; one that misses a real crossing is not.
- **Orientation is a quaternion, not an angle.** A 3D body's turn is a `Quat`
  integrated in place ([ADR-0043](0043-physics3d-angular-dynamics.md)), so the
  arc has to be bounded over a quaternion path, not a scalar angle, and the
  clamp has to write back a quaternion.

## Decision

Extend `Physics3DSystem::apply_continuous_collisions` with a rotational sweep,
mirroring the 2D design in `Physics2DSystem::apply_continuous_collisions`.

- The integration loop records each dynamic body's **previous orientation**
  alongside its previous position, so the pass knows the start of the turn.
- A turn is swept only when its **arc length exceeds the body's own size**: the
  angle between the start and end orientations (`quat_angle_between`, the
  magnitude of the relative rotation, always in `[0, pi]`) times the shape's
  bounding radius (`half_extent.length()`). A turn too small to move a corner by
  more than the body cannot tunnel. **Only boxes are swept for spin**: a sphere's
  shape does not change as it turns, so rotation alone can never tunnel one.
- The sweep cuts the turn into a fixed number of **substeps**; within each, the
  arc the shape sweeps is bounded by sampling the intermediate orientations
  (`arc_bounds`, unioning the oriented bounds) and tested against the target.
  The first substep whose arc reaches the target is the contact.
- **A contact already present at the start of the turn is left to the discrete
  solver** -- the same rule the linear sweep uses -- so a body spinning against
  the surface it rests on is not frozen.
- On the earliest hit the turn is **clamped** (`Quat::slerp` to the hit fraction)
  and the angular velocity is shed; any linear motion is left to the discrete
  solver and the linear sweep.
- The count is still `continuous_hit_count()`, so a test proves the pass fired
  rather than only that the body ended up in the right place.

## Consequences

**Easier:**

- A fast-spinning box cannot sweep a corner through a thin static wall: it is
  clamped to the touch instead of appearing on the far side.
- An ordinary step is unchanged: a turn too small to tunnel is skipped, a sphere
  is never swept for spin, and a turn that starts in contact is left to the
  discrete solver.
- The behaviour is asserted directly: a test spins a long thin bar a half-turn in
  one step past a ledge only its tips' arc reaches and proves the turn is clamped
  short of it (without the sweep it spins on past the ledge), and a second proves
  a spinning sphere never fires the pass.
- The 3D rotational gap ADR-0130 named is closed, so the 2D and 3D continuous
  passes now cover the same motion.

**Harder / not yet done:**

- A fast body swept against *another moving body* still tunnels. The pass only
  sweeps static geometry; relative-motion sweeping is the next refinement.
- A rotated body is still swept as its axis-aligned bounds, so a near-corner
  crossing can be reported a hair early. Conservative, never a miss.
- The substep count is fixed, not adaptive: a very fast spin is bounded coarsely
  rather than solved exactly. The caller's threshold guarantees the arc exceeds
  the body's own size, so a coarse cut still catches a wall the body could
  tunnel, but a finer solve is later work if a profile asks for it.
- The pass is O(fast bodies x all bodies) per step, now with a substep loop
  inside; only a body turning fast enough to tunnel enters it.

## Alternatives considered

- **Sweep the swept volume as a full oriented-box (OBB) cast.** Rejected for now:
  it needs an oriented continuous test and a wider change than this refinement;
  the sampled arc bound is conservative and fits the existing bounds-based pass.
- **Bound the arc from the endpoints alone (no intermediate samples).** Rejected:
  an intermediate orientation can reach past both endpoint boxes, so the bound
  would miss a corner crossing the wall between the two endpoints -- exactly the
  case the sweep exists to catch.
- **Sweep rotation for a sphere too.** Rejected: a sphere's shape is
  orientation-invariant, so rotation alone can never tunnel one; sweeping it
  would only add cost and a spurious clamp.
- **Sub-stepping the whole step.** Rejected for the same reason as the linear
  pass: it changes every body's trajectory and makes results depend on the step
  count, where the continuous pass touches only the bodies that can tunnel.
