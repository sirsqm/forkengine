# ADR-0151: The 3D contact event carries the point and normal of the touch

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0140](0140-contact-event-geometry.md),
  [ADR-0043](0043-physics3d-angular-dynamics.md),
  [ADR-0009](0009-physics2d-system.md)

## Context

[ADR-0140](0140-contact-event-geometry.md) gave a contact event the point and
normal of the touch so a listener can spawn an effect where the hit happened,
and wired that through the 2D system. It added `point` to `Contact3D` too, but
left the 3D event unchanged: the 3D solver's contact path is separate code, and
the cut ended before it was wired. The result is an asymmetry -- a 2D game can
place an impact effect from the event, a 3D game cannot -- and a field on
`Contact3D` that nothing reads.

Constraints that shaped the decision:

- **The frozen default.** Nothing about which pairs touch, when, or in what order
  may change; only the new fields are added to the event, and the pair key that
  drives the Begin/End diff is untouched.
- **One representative point, not a manifold.** The same limit 2D accepted: the
  point is for effects, not for solving, and the field stays named `point`.
- **The pair order is a key, not a direction.** The event's `a`/`b` are ordered
  by raw id bytes so the pair diffs across frames; the narrow phase's normal runs
  from the first *gathered* body, so it must be flipped when the pair is ordered
  the other way, exactly as 2D does.
- **A box-box point must sit where the boxes are, not between their centres.** The
  first 2D cut took the point on A's centre line and put a small box landing on a
  wide floor at the floor's centre; ADR-0140 fixed that by using the overlap
  region. The 3D SAT path must not repeat the mistake.

## Decision

`Contact3D` gains a `point` (world space), each 3D narrow-phase test computes it,
the 3D `BodyPair` carries the point and normal, and the 3D `ContactEvent` gains
`point` and `normal` -- the same shape as 2D.

- Each narrow-phase test computes the point:
  - Box versus box (axis-aligned): the centre of the region the two boxes
    overlap, which is a point on the contact plane and independent of which box
    is A.
  - Sphere versus sphere: halfway between the two surfaces along the normal,
    starting from the first sphere's surface.
  - Sphere versus box: on the segment between the sphere's surface and the box's
    nearest surface, half way through the overlap.
  - Oriented box versus oriented box (the SAT test): the centroid of the two
    boxes' overlapping face region, found by clipping one face polygon against
    the other box's four side planes. The centre-midpoint shortcut is not used
    because it collapses the tangential position of a small box on a wide floor
    onto the floor's centre, the same failure ADR-0140 corrected for 2D. The
    sphere-versus-oriented-box path reuses the axis-aligned point and rotates it
    back to world space with the normal.
- `BodyPair` carries the point and normal alongside the ids; the id operators are
  unchanged, so the sort and diff see the same key as before.
- `append_event` copies them onto the event. The normal is flipped to run from
  `pair.first` to `pair.second`, matching the event's endpoint order; the point
  is direction-free.
- The 3D trigger path records the pair before the trigger check, so a trigger
  reports the geometry exactly as it reports Begin and End.

## Consequences

**Easier:**

- A 3D listener can spawn an effect where the hit happened and orient it by the
  normal, without re-running the narrow phase, symmetric with 2D.
- `Contact3D::point`, added by ADR-0140 but unread, now has a consumer.

**Harder / not yet done:**

- The point is a single representative point, so a flat box-on-box rest reports
  one point where a real manifold has two, the same limit 2D has.
- The point is the first iteration's contact, not a per-iteration average, for
  the same reason as 2D: an effect point should not wander.
- The box-box point is the overlap centroid, not a solver-grade manifold; a
  solver that applies torque from the contact still needs more than this.

## Alternatives considered

- **Leave the 3D event a bare signal and let a listener query.** Rejected for the
  same reason as ADR-0140: every listener wants the point, the narrow phase
  already has almost all of it, and a query is duplicated work that can drift.
- **Reuse the 2D centre-midpoint formula for the oriented boxes.** Rejected: it
  mis-places the point for the common case of a small box on a large floor, the
  exact bug ADR-0140 fixed, and a SAT contact is common in 3D.
- **A full manifold on the event.** Rejected for this cut for the same reason as
  ADR-0140: it is a solver change, not a reporting change.
