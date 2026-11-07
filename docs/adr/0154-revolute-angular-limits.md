# ADR-0154: Revolute joint angular limits

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0134](0134-extended-joint-set.md),
  [ADR-0153](0153-joint-motors-and-limits.md),
  [ADR-0021](0021-physics2d-joints.md)

## Context

The joint set grew a prismatic (slider) joint with translation limits
([ADR-0134](0134-extended-joint-set.md)) and joint motors
([ADR-0153](0153-joint-motors-and-limits.md)). A revolute (pin) joint still
swung through a full turn with no stops, so a door could not stop at 90 degrees,
a limb could not stop at its extreme, and a flipper had no travel. This was the
one refinement gap ADR-0153 named as still open: "a revolute joint still has no
angular limits (a hinge that stops at 90 degrees); only the prismatic joint has
translation limits."

The prismatic joint's translation limits already solved the same problem in a
linear coordinate: two one-sided rows, each active only when the pair is past its
bound, each clamped so it pushes the pair back inside and never pulls it out.
Angular limits are that machinery applied to the free angular coordinate, so the
design is a port rather than a new mechanism.

## Decision

A revolute joint gains three fields, mirroring the prismatic joint's limits:

- `enable_limits` (off by default) turns the free swing into a limited one.
- `lower_angle` and `upper_angle` (radians) bound the relative angle `b - a`,
  measured as B's rotation minus A's, the same coordinate the pin leaves free.

The system builds the pin's two rows as before and then builds two limit rows on
top of them, so a limited hinge solves four rows and a plain pin still solves
two. `build_revolute_limit_rows` writes both rows in place:

- Each row is purely angular (`angular_a = -1`, `angular_b = +1`): the free
  coordinate is the relative angle, so the row measures its rate directly.
- Each row is one-sided. The row for a bound is built only when the relative
  angle is at or past it, and its impulse is clamped to one sign, so a limit
  pushes the pair back inside the range and never pulls it out. Inside the range
  both rows are all-zero slack rows the apply functions already skip, so the
  joint swings freely.
- The sign of each clamp follows the row's velocity convention, exactly as the
  prismatic limits' does: past the upper bound only a negative impulse (which
  shrinks the angle) is allowed, past the lower bound the opposite.

The fields are component fields with reflection, so they round-trip through the
serializer and show in the inspector, and a default-constructed joint behaves
exactly as before.

## Consequences

**Easier:**

- A limited hinge is authorable: a door that stops at 90 degrees, a jointed limb
  with an extreme, a flipper with travel, a drawbridge that cannot pass its rest.
- A limited joint swings freely inside its range: the slack rows exert nothing,
  so a limit the pair is not touching does not damp its motion.
- The limit's reaction counts toward the joint's reported load and its break
  threshold, because the limit rows are solved and read in the same pass as the
  pin rows ([ADR-0135](0135-joint-reactions-and-breaking.md)).
- The default is unchanged, so every existing scene steps as before.

**Harder / not yet done:**

- The limits compare against the raw relative angle, so authoring a range wider
  than one turn, or one that wraps past ±pi, is the caller's responsibility. A
  wrapping-aware comparison is a separate refinement; the usual swing of less
  than a full turn is unaffected.
- The 2D joint has angular limits; the 3D revolute joint does not yet. The same
  one-sided-row machinery applies, but a 3D hinge also needs an authored
  reference axis and a signed swing, so it is a larger change than a port.
- The limits are hard stops. A soft limit (a spring near the bound) is not
  modelled; a spring motor target ([ADR-0153](0153-joint-motors-and-limits.md))
  is the nearer building block for it.

## Alternatives considered

- **Fold the limits into `build_revolute_rows`, growing the row count.** Rejected
  for the reason ADR-0153 kept the motor separate: the pin's row count is a
  public constant the tests name, and a joint with limits off should build and
  solve exactly the two rows it always did. Building the limit rows on top keeps
  both true.
- **Compare a wrapped relative angle so any authored range works.** Deferred: the
  unwrapped comparison is the smaller change and covers the ordinary case; a
  wrapped comparison can be layered on without changing the row design.
- **A single two-sided limit row.** Rejected: a two-sided row cannot tell which
  way to push without state, whereas two one-sided rows each know their bound and
  clamp to one sign, which is what lets a body at a stop leave it again.
- **A separate `RevoluteLimitJoint2D` component.** Rejected: the limit is a
  property of the hinge, not a second joint on the same pair, and a separate
  component would need its own anchors and identity.
