# ADR-0136: A per-step force and torque accumulator on a rigid body

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0009](0009-physics2d-system.md),
  [ADR-0043](0043-physics3d-angular-dynamics.md)

## Context

A rigid body in both worlds can be acted on by exactly three authored things:
gravity (a world setting), a constant `torque` field, and the impulses the solver
derives from contacts and joints. There is no way for a game to push a body: no
thruster, no wind, no scripted kick. The `torque` field is the only authored
per-body force-like value, and it is a *constant* applied every step, not
something that accumulates during a frame.

That gap blocks the common cases: a character controller that applies a jump
impulse, a vehicle that pushes itself forward, a fan that lifts what is above it,
a scripted explosion. Each is a force that exists for one step, or for a few,
added by gameplay code rather than authored as a field.

Constraints that shaped the decision:

- **The frozen default.** A body with no force added must step exactly as it did,
  and the new fields must serialize as no-ops so an existing scene is unchanged.
- **One definition across the two worlds.** The 2D and 3D solvers are separate
  code but share the design (ADR-0134); the accumulator should read the same way
  in both.
- **The value is runtime state, not authored state.** A force added this step is
  consumed this step and gone. Saving it would reload a scene with a stale push,
  the same reason `is_sleeping` and `sleep_timer` are not serialized.
- **Adding a force should wake the body.** A body that settled and slept must not
  ignore a push because it happens to be asleep; a force is exactly the "something
  moved it" a wake is for.

## Decision

Add a per-step force accumulator to `RigidBody2D` and `RigidBody3D`.

- `force` (a `Vec2`/`Vec3` in newtons) and `applied_torque` (a scalar/`Vec3` in
  newton-metres) are the accumulators. They are public fields, added to through
  `add_force`/`add_torque`, which accumulate (so several sources compose) and
  call `wake()`.
- `add_force_at_offset(value, offset)` adds a force acting at a point `offset`
  from the centre of mass (in the body's frame, the same convention as
  `apply_impulse_at_offset`). The lever arm contributes `cross(offset, value)`
  (2D: the Z component `offset.x * value.y - offset.y * value.x`) to
  `applied_torque`, so an off-centre push both translates and spins the body. A
  force through the centre (`offset` zero) is exactly `add_force`.
- Integration adds `force * inverse_mass * delta_time` to the velocity and
  `(torque + applied_torque) * inverse_inertia * delta_time` to the angular
  velocity. The constant `torque` field is kept and summed with the per-step one,
  so an author can combine a steady spin with a scripted push without a second
  field for the constant case.
- The system clears both accumulators at the end of `update`, so a force acts for
  exactly the step it was added and must be re-added to persist. `clear_forces()`
  is the operation, called on every body in the scene walk the update already
  does for other per-step bookkeeping.
- Neither field serializes. They are runtime state, like the sleep timer: a
  reloaded scene starts with no force, which is the correct "nothing is pushing
  this yet" state.

## Consequences

**Easier:**

- A game can push a body: a jump, a thruster, wind, an explosion, a conveyor.
- Several sources compose by adding to the same accumulator in one frame.
- A force wakes a settled body, so gameplay never has to remember to wake before
  pushing.
- The constant `torque` still works and now combines with a per-step torque, so
  nothing authored changes.

**Harder / not yet done:**

- The force is applied at the centre of mass, or at an offset from it through
  `add_force_at_offset`. There is still no form taking a *world* point: the
  offset is in the body's frame, so a caller applying a force at a world point
  subtracts the body's world centre first (a one-line transform the caller
  usually already has the data for).
- There is no impulse form *on this accumulator*: a one-shot impulse is a
  separate operation, added later as [ADR-0137](0137-impulse-api.md)
  (`apply_impulse` / `apply_impulse_at_offset`), not a force.
- The accumulator is a plain field, so it is not thread-safe. The solver is
  single-threaded by design, and a script adding a force runs in the update
  phase, so this is fine, but a parallel gameplay system would need care.

## Alternatives considered

- **Apply forces only through the constant `torque` field.** Rejected: a constant
  cannot express "push for one frame", and it is already the wrong shape for a
  linear force (it is angular).
- **Serialize the accumulator.** Rejected: a force is consumed in the step it is
  added, so a saved value would be a stale push on reload. The sleep state is the
  precedent for keeping per-step runtime state out of the document.
- **A separate `ForceAccumulator` component.** Rejected: it would be a second
  component to add to every body that wants a force, where two fields on the body
  itself cost nothing and clear with it.
- **An impulse API instead of a force API.** Rejected as the primary form: a
  force reads the same at any step size and composes with the existing
  mass/inertia integration, while an impulse would be a second, step-size-aware
  path. An impulse helper can be layered on the force later.
