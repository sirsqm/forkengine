# ADR-0140: A contact event carries the point and normal of the touch

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0009](0009-physics2d-system.md),
  [ADR-0037](0037-physics2d-oriented-boxes-offsets.md)

## Context

The contact-event stream (Begin and End, keyed by node pair) exists so gameplay
can react to a touch. It reported only *which* two nodes touched, deliberately:
the doc said a listener that needs geometry "can run a query". In practice the
first thing every listener wants is *where* the touch happened -- to spawn a dust
puff at the point of landing, a spark at the point of impact, or to attach a decal
-- and the query it was told to run has to re-run the narrow phase for the pair,
which is both duplicated work and a second place the geometry can be computed
differently.

The narrow phase already computes the normal and the penetration. It does not
compute a point, because the solver never needed one: the impulse is applied to
the bodies' velocities, not at a point. So the work to report a point is to
compute one representative point in each of the narrow-phase tests.

Constraints that shaped the decision:

- **One representative point, not a manifold.** A true contact manifold is a set
  of points (a box resting flat has two). Modelling that is a real change to the
  solver, which is out of scope; a single point that sits on the contact plane is
  what a particle burst or a decal needs, and the field is named `point`, not
  `manifold`, to keep that honest.
- **The pair's stable order is a key, not a direction.** The event's `a`/`b` are
  ordered by raw id bytes so the pair diffs across frames. The narrow phase's
  normal runs from the first *gathered* body to the second, which is unrelated to
  that order, so the normal must be flipped when the pair is ordered the other
  way or the event's normal would point the wrong way half the time.
- **The diff must not see the geometry.** The pair is sorted and compared by its
  two ids. If the point or normal took part in `operator<` or `operator==`, two
  frames of the same touching pair could compare unequal and emit a spurious End
  and Begin. The fields are carried alongside the key, ignored by both operators.
- **The frozen default.** Nothing about which pairs touch, when, or in what order
  may change; only the new fields are added.

## Decision

`Contact2D` and `Contact3D` gain a `point` (world space), and the 2D
`ContactEvent` gains `point` and `normal`.

- Each narrow-phase test computes the point:
  - Box versus box (axis-aligned and SAT): the centre of the overlapping span
    along the separating axis, on the line through the first box's centre.
  - Circle versus circle: halfway between the two surfaces along the normal,
    starting from the first circle's surface.
  - Circle versus box: the same rule from the circle's surface, which lands on
    the box face the circle is nearest.
- `BodyPair` carries the point and normal alongside the ids; the id operators are
  unchanged, so the sort and diff see the same key as before.
- `append_event` copies them onto the event. The normal is flipped to run from
  `pair.first` to `pair.second`, matching the event's endpoint order.
- The 3D change adds `point` to `Contact3D`; wiring it onto the 3D event is left
  for the 3D pass (the 3D solver's contact code is a separate path).

## Consequences

**Easier:**

- A listener can spawn an effect where the hit happened without re-running the
  narrow phase, and without a second implementation that could disagree.
- The normal lets the effect be oriented (a splash along the surface, a spark
  off it) with no extra query.
- The point is available on both a Begin and an End (the last recorded touch),
  which is what an "I left the ground" listener wants for a departure effect.

**Harder / not yet done:**

- The point is a single representative point, so a flat box-on-box rest reports
  one point where a real manifold has two. A solver that applies torque from the
  contact needs the manifold; this point is for effects, not for solving.
- The 3D event does not yet carry the point even though `Contact3D` now computes
  it; the 3D `ContactEvent` is unchanged in this cut.
- The point is the first iteration's contact, not a per-iteration average. A
  solver that jitters the contact over the iterations reports the initial one,
  which is the stable choice for an effect (the point should not wander).

## Alternatives considered

- **Keep the event as a signal and require a query.** Rejected: every listener
  wants the point, the narrow phase already has almost all of it, and re-running
  it in the listener is duplicated work that can drift from the solver's version.
- **A full manifold (a list of points) on the event.** Rejected for this cut: it
  is a solver change (the impulse must be distributed across the points), not a
  reporting change, and it would change the solved result, which the frozen
  default forbids. A single point can grow into a manifold later.
- **Put the geometry only on the `BodyPair` and let a listener read it there.**
  Rejected: `BodyPair` is private state used for the diff; the event is the
  public surface, and a listener already reads the event, not the pair.
