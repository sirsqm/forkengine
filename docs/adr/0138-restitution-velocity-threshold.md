# ADR-0138: A restitution velocity threshold

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0009](0009-physics2d-system.md),
  [ADR-0043](0043-physics3d-angular-dynamics.md)

## Context

A contact's restitution is applied as `-(1 + e) * v_n`, where `v_n` is the
approach speed along the normal. That is correct for a real impact, but it is
wrong for a body at rest. Gravity is re-applied every step, so a resting body
approaches the floor it sits on at `gravity * dt` (about 0.16 m/s at 60 Hz)
before the solver cancels it. With restitution 1.0 the solver reflects that
approach, so the body is thrown back up a hair every step and lands again: an
infinite low-amplitude buzz that never settles, and one that the sleeping system
only papers over (it sleeps a body whose velocity happens to read as small, then
wakes it on the next contact).

The standard fix, which Box2D and Bullet both use, is to ignore restitution below
a small approach speed. A real impact arrives far faster than `gravity * dt`, so
suppressing restitution under a threshold removes the buzz without touching a
genuine bounce.

Constraints that shaped the decision:

- **The frozen default for a real impact.** A fast collision must bounce exactly
  as it did; only the barely-approaching case changes.
- **One rule across the two worlds.** 2D and 3D share the contact solver's shape
  (ADR-0134), so the threshold belongs in both.
- **A tunable, not a constant.** The right value depends on gravity and the step
  size, so it is a system field an author can set, not a hardcoded number.

## Decision

Add `restitution_threshold` (default 1.0 world units per second) to
`Physics2DSystem` and `Physics3DSystem`, and suppress restitution when the
approach speed is below it.

- The pair's restitution is `max(a, b)` only when `-v_n > restitution_threshold`,
  and zero otherwise. `-v_n` is the approach speed (`v_n` is negative when the
  pair is closing), so the test reads as "a fast enough impact".
- The threshold is a public field on the system, tunable per world, defaulting to
  1.0: above `gravity * dt` for any sane gravity and step, and well below a
  meaningful impact.
- The rule is applied in the same place in both solvers, in the normal-impulse
  computation, so the two stay in step.

## Consequences

**Easier:**

- A bouncy body comes to rest on a floor instead of buzzing forever, so a bouncy
  ball or a stack of crates settles as a player expects.
- The sleeping system no longer has to hide the buzz: a settled body is genuinely
  at rest, so it sleeps for the right reason.
- The threshold is visible and tunable, so a game that wants micro-bounces can
  lower it, and one that wants maximum stability can raise it.

**Harder / not yet done:**

- The threshold is a single global value per world. A material that wants a
  different threshold than the default has no per-collider override; the friction
  and restitution combine rules are per-collider, but the threshold is not.
- The threshold interacts with the step size: a much larger step makes
  `gravity * dt` larger, and a fixed threshold could then be too small. The
  default is chosen for a 60 Hz step; a caller with a very different step should
  tune it.

## Alternatives considered

- **Leave restitution exact and rely on sleeping.** Rejected: the buzz is real
  while the body is awake (a bouncy body on a moving platform never sleeps), and
  sleeping is an optimisation, not a physical fix.
- **Zero restitution for all resting contacts.** Rejected: that is what the
  threshold does, but stated as a speed rather than a rest flag, so it also
  covers the barely-moving case (a slow roll onto a bouncy surface) without a
  separate "is resting" test that would need its own definition.
- **A per-collider threshold.** Deferred: the combine rule would need a second
  pairwise operation, and the global default covers the case the bug was about.
  The field is on the system precisely so a per-collider value can be layered on
  later without moving the rule.
