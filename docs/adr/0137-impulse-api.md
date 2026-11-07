# ADR-0137: An impulse API on a rigid body

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0136](0136-per-step-force-accumulator.md),
  [ADR-0009](0009-physics2d-system.md),
  [ADR-0043](0043-physics3d-angular-dynamics.md)

## Context

[ADR-0136](0136-per-step-force-accumulator.md) gave a body a per-step force: a
sustained push spread across a step. That is the right tool for a thruster or a
wind zone, but not for the other half of gameplay physics: the instant kick. A
jump, a bat hitting a ball, an explosion, a bullet's knockback are all "change
the velocity now", and forcing them through a force means the caller has to
divide by the step size and hope the step is what it thinks it is. A one-shot
impulse is a distinct, simpler operation: `velocity += impulse * inverse_mass`,
no step size in the arithmetic at all.

There is also no way to apply a force or impulse off-centre. Every push the
engine models acts at the centre of mass, so a bat cannot spin a ball and a
thrown crate cannot tumble from where it was hit. The contact solver already
computes off-centre impulses internally (the lever arm `r x impulse`), but a game
cannot ask for one.

Constraints that shaped the decision:

- **The frozen default.** A body with no impulse applied must step exactly as it
  did; the new methods touch only state the caller changes.
- **One definition across the two worlds.** 2D and 3D share the design
  (ADR-0134), so the impulse API reads the same way in both.
- **No new per-step state.** An impulse is applied immediately, not accumulated
  and integrated later, so it needs no field and no clear step, unlike the force
  accumulator.

## Decision

Add `apply_impulse` and `apply_impulse_at_offset` to `RigidBody2D` and
`RigidBody3D`.

- `apply_impulse(impulse)` adds `impulse * inverse_mass` to the velocity. A body
  with infinite mass ignores it (the inverse mass is zero).
- `apply_impulse_at_offset(impulse, offset)` also adds the angular part:
  `cross(offset, impulse) * inverse_inertia` in 3D, and the scalar
  `offset.x * impulse.y - offset.y * impulse.x` in 2D. `offset` is measured from
  the centre of mass (the node origin) in the body's frame. An impulse through
  the centre leaves the spin unchanged.
- Both wake the body, matching `add_force`: a settled body must not ignore a kick.

The impulse is applied to the current velocity immediately, so a caller applies
it in the frame phase before the step, exactly like a force.

## Consequences

**Easier:**

- A jump, a hit, a knockback, an explosion are one call, with no step size in the
  arithmetic.
- An off-centre hit spins what it strikes, which is what makes a bat, a cue, or a
  glancing bullet read as physical.
- The API matches the force API in both worlds, so a caller learns one shape.

**Harder / not yet done:**

- The offset is in the body's local frame, not a world point. A caller with a
  world contact point must subtract the body's centre and rotate into the local
  frame; a world-point convenience is not provided.
- The angular response uses the isotropic scalar inertia, so a non-cube box spins
  the same about every axis, the existing limit (ADR-0043). A full inertia tensor
  would make an off-centre impulse depend on the axis it was applied about.
- An impulse does not run through the solver, so it cannot be constrained by a
  joint or a contact in the same step. It is applied before the step and the
  solver then resolves it, which is the intended order.

## Alternatives considered

- **Only a force API.** Rejected: a one-shot kick is a different operation, and
  expressing it as a force makes the caller divide by the step and couples the
  result to the step size.
- **Only an impulse API.** Rejected: a sustained push (a thruster held for a
  second) is a force, and forcing it through repeated impulses ties the result to
  the frame rate.
- **A world-point overload that computes the offset itself.** Deferred: it needs
  the body's current centre and orientation, which the component does not carry
  (the node's transform does), so it belongs on the system rather than the
  component. The local-frame offset is the primitive the system would call.
- **An impulse accumulator that the solver applies with the forces.** Rejected:
  an impulse is immediate and stateless; accumulating it into the per-step force
  would divide by the step and reintroduce the coupling an impulse exists to
  avoid.
