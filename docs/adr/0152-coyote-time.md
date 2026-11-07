# ADR-0152: Coyote time in the character controller

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0078](0078-engine-shipped-gameplay-behaviours.md),
  [ADR-0075](0075-action-seam.md),
  [ADR-0031](0031-physics3d-and-backend-seam.md)

## Context

`CharacterController3D` ([ADR-0078](0078-engine-shipped-gameplay-behaviours.md))
is the third-person character the engine ships, and it is the gameplay a browser
build has to run. Its jump was strict: a jump fires only on a frame the ground
probe reports the floor underfoot. On a flat floor that is fine; at the edge of a
platform it is not. A player runs off a ledge and presses jump one or two frames
late -- a few tens of milliseconds, well inside human reaction slop -- and the
jump is dropped, so the character walks off into the gap instead of leaping it.
The failure is invisible in code review and obvious in play, and it is the single
most common "why did my platformer feel bad" complaint.

The fix is a *grace window*: after the character leaves the ground, keep a jump
possible for a short time. The name for it is coyote time.

Constraints that shaped the decision:

- **The default must not change an existing scene's jump.** A character on the
  ground must jump exactly as before, and a character that has been airborne for a
  while must still not jump. Only the few frames just after leaving the ground
  change, and only to *allow* a jump that was previously dropped.
- **The window is a design knob, not a constant.** The right value depends on the
  game's gravity and jump arc, so it is a reflected, serialized property like the
  other controller fields, not a hardcoded number.
- **A press stays a press.** The controller already ignores a held jump so a held
  key is not continuous thrust; the window must not reopen that door by letting a
  held jump re-fire while the character is still falling.
- **The window is consumed on use.** A jump fired from the window must not leave
  the window open for a second jump in the air.

## Decision

Add a `coyote_time` property (seconds, default `0.1`) to `CharacterController3D`,
and track `air_time_` -- the seconds since the character was last grounded.

- Each update, `air_time_` is reset to zero when grounded and advanced by the
  frame's delta otherwise.
- A jump is allowed when the character is grounded **or** `air_time_ <=
  coyote_time`, and the body is not already rising. The `vertical_velocity_ <= 0`
  guard is what keeps a held jump from re-firing inside the window.
- A jump fired from the window sets `air_time_` past the window, so it is
  consumed: a second press while airborne is ignored exactly as before.
- `coyote_time` is registered with the ordinary scalar property builder, so the
  inspector shows it and the serializer round-trips it. `0` disables the window,
  restoring the strict behaviour.

## Consequences

**Easier:**

- A jump pressed a moment after running off a ledge still fires, which is what
  makes the shipped character feel fair rather than precise.
- The knob is per-character and live: a game can raise it for a floaty platformer
  and set it to zero for a puzzle that wants exact timing.

**Harder / not yet done:**

- The window is measured in time, not in distance. At a very high fall speed the
  character drops a large distance during the window, so a jump can fire from
  well below the ledge's lip. That is the standard behaviour and is bounded by
  the short window, but a distance-bounded window is a later refinement if a game
  asks for it.
- There is no jump *buffer* (a press made just *before* landing is remembered and
  fired on the landing frame), the mirror of coyote time. It is a separate
  feature and is not added here.
- The window does not distinguish a deliberate step off from a knockback: a
  character shoved off a ledge by a collision gets the same grace. That is
  harmless for a jump and is the usual trade.

## Alternatives considered

- **Probe the ground a little further down so a character just past the lip still
  reads as grounded.** Rejected: it makes the character report grounded while
  visibly airborne, which changes `grounded()` for every reader (a follow camera,
  a test) and can leave the character "grounded" hovering over a gap.
- **Widen the jump to any airborne frame.** Rejected: that is flight, not a jump;
  the character could climb forever by holding jump.
- **A fixed constant instead of a property.** Rejected: the value is a design
  knob that depends on the game, and every other controller field is a property.
- **Remember the last grounded position and allow a jump within a distance.**
  Deferred: time is the familiar formulation and the simpler one; distance is a
  refinement, noted above.
