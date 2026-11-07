# ADR-0139: A per-body gravity scale

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0009](0009-physics2d-system.md),
  [ADR-0043](0043-physics3d-angular-dynamics.md)

## Context

A body's relation to gravity is currently a boolean: `affected_by_gravity`. That
covers "falls" and "does not fall", but not the middle ground a game keeps
reaching for: a balloon that rises (negative gravity), a moon that pulls a third
as hard, a paper plane that falls slowly, a heavy character that falls a little
faster than the rest. Each of those is a *scale* on world gravity, not a switch,
and with a boolean the only ways to fake them are to set the body's own velocity
each step or to put it in a separate world with different gravity. Neither is
what a per-body property should require.

Constraints that shaped the decision:

- **The frozen default.** A body that never sets the field must step exactly as
  it did, so the default is 1.0 and the multiply is a no-op at 1.0.
- **One definition across the two worlds.** 2D and 3D share the body shape
  (ADR-0134), so the field belongs on both, with the same meaning.
- **Authored, so it serializes.** Unlike the force accumulator (ADR-0136), which
  is per-step runtime state, gravity scale is a property of the body an author
  sets and a saved scene must reload, so it goes through the reflection registry
  and the serializer like mass or damping.
- **It composes with the boolean.** `affected_by_gravity = false` must still mean
  "no gravity at all", independent of the scale.

## Decision

Add `gravity_scale` (default 1.0) to `RigidBody2D` and `RigidBody3D`.

- The integration adds `gravity * (gravity_scale * delta_time)` instead of
  `gravity * delta_time`, inside the existing `affected_by_gravity` check. The
  boolean still gates the whole term, so a body with gravity off ignores the
  scale; a scale of 0.0 is a composable way to say the same thing when the
  boolean is left true.
- A negative scale is allowed and makes the body fall upward, which is the
  balloon case and the reason the field is not clamped.
- The field is registered with the reflection registry and serializes with the
  other authored body properties, in both worlds.

## Consequences

**Easier:**

- A balloon, a moon, a paper plane, and a heavy character are one number each.
- A gameplay system can animate the scale (a gravity flip, a low-gravity zone)
  by writing the field, with no second world or per-step velocity hack.
- The value round-trips through the serializer, so a scene that sets it reloads
  with it.

**Harder / not yet done:**

- Gravity is still global to the system, so a per-region gravity needs the system
  gravity or the per-body scale written by a gameplay system; there is no gravity
  field volume.
- The scale multiplies the world gravity only. It does not scale a per-body force
  (ADR-0136) or an impulse (ADR-0137), which are separate inputs and not gravity.

## Alternatives considered

- **Reuse `affected_by_gravity` as a float.** Rejected: it would break every
  existing scene and every call site that reads the boolean, and the two are not
  the same idea ("is gravity applied at all" versus "how much").
- **A separate gravity vector per body.** Rejected for now: a full vector would
  let a body fall sideways, which is a stronger feature than the scale and brings
  its own questions (does it rotate with the body?). The scale covers the common
  case and a vector can be layered on later without moving this rule.
- **A gravity-region volume in the scene.** Rejected as the first step: it is a
  scene-level feature with a query cost every step, where the per-body scale is a
  field the solver already has in hand. A region would set the scale, not replace
  it.
