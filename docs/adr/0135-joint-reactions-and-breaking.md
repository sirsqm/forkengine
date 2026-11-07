# ADR-0135: Joint reactions and breaking

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0134](0134-extended-joint-set.md),
  [ADR-0021](0021-physics2d-joints.md),
  [ADR-0043](0043-physics3d-angular-dynamics.md)

## Context

Every joint in both worlds constrains a body pair forever. Once a joint is
authored it is permanent: a rope cannot fray, a weld cannot give way, and a
bridge cannot collapse under a load it was never meant to carry. A game wants the
opposite -- a structure that holds until it is hit hard enough, then comes apart,
and the load it carried is the thing that decides.

The solver already computes that load every step, but throws it away. A row's
impulse is the impulse the constraint applied to cancel the relative velocity
along it; the positional correction is the impulse that removed the leftover
error. Both are the joint's reaction, and both are discarded the moment they are
applied. A break threshold needs exactly those numbers, and a debug overlay wants
them too.

Constraints that shaped the decision:

- **The frozen default.** A joint with no break threshold must behave exactly as
  it did, and a default-constructed component must serialize and step
  bit-for-bit as before.
- **One reaction, defined once.** The 2D and 3D solvers are separate code but a
  joint kind is one design expressed twice (ADR-0134). The reaction must be read
  the same way in both, so a test can assert the same number in either world.
- **The load is a force, not a step-dependent impulse.** A threshold authored in
  newtons must mean the same at 60 Hz and 120 Hz, so the reaction is converted
  from per-step impulse to force with `1 / delta_time` before it is compared.
- **A joint is a component with a stable identity.** The threshold and the
  snapped flag are component fields, registered with reflection and round-tripped
  by the serializer, the same rule every joint already follows.

## Decision

Give every joint a reaction and a break threshold.

### Reaction

`row_reactions` in each solver turns a solved row set into forces: for each row
whose effective mass is positive, the impulse accumulated in it is divided by the
step and accumulated. It reports the net force on the second body, the net
torque, and the largest single-row force magnitude. The peak magnitude is taken
from the impulse itself, not the projected vector, so it does not depend on which
end of the joint is the movable one -- a static-to-dynamic joint reports the same
load as a dynamic-to-dynamic one.

The peak is the number a break threshold compares against, because a joint gives
way where it is loaded hardest, not on the average of its rows. The per-joint
reaction is stored on the gathered `Joint` record and exposed through
`joint_reaction(body_a, body_b)` on both systems, so a test or a debug overlay
reads the force a specific joint carried.

### Breaking

Each joint component gains `break_force`, `break_torque` (both zero by default,
meaning "never break"), and `broken`. After a joint is solved, if
`peak_force > break_force` or `|reaction_torque| > break_torque`, the joint sets
its own `broken` flag. A broken joint is skipped from the next step on, so it
applies its final correction the step it snaps and constrains nothing after.

- Zero on both thresholds is the frozen default: a joint that does not opt in
  never breaks, and its step is unchanged.
- Clearing `broken` re-arms the joint, so a repair is a field write and not a new
  component.
- A broken joint is not counted in `joint_count`, matching how a joint that
  cannot be solved is already excluded, so the count means "joints that acted
  this step".

## Consequences

**Easier:**

- A structure can be authored to fail: a rope that frays, a weld that gives way,
  a bridge that drops its load when a span is overloaded.
- The reaction is inspectable: a test asserts the exact load a joint carries
  (a 2 kg bob on a pin reports ~19.6 N), and a debug overlay can draw it without
  re-deriving anything.
- The threshold is frame-rate independent, because it is compared against a force
  and not against the per-step impulse.
- Breaking is one field on one component, so it composes with every joint kind in
  both worlds and needs no new solver mechanism.

**Harder / not yet done:**

- The break test is a single-step test: a joint snaps the first step its peak
  exceeds the threshold. A joint that should accumulate fatigue over many steps
  would need a running integral, which is not modelled.
- The positional correction is not included in the reported reaction. The
  velocity impulse is the standard joint reaction and the one a break threshold
  wants; folding the transient placement correction in would make a freshly
  placed joint report a large one-off force and snap for no reason.
- A broken joint leaves a diagnostic-free hole: nothing reports *that* it broke
  this step beyond the `broken` flag. An event stream is later work, mirroring
  how a skipped joint currently records a diagnostic.
- The 2D and 3D reactions are separate functions. They share the design and the
  tests, not the implementation, for the same reason the row shapes differ.

## Alternatives considered

- **Compare the accumulated impulse directly against the threshold.** Rejected:
  the threshold would then be an impulse and would depend on the step size, so
  the same joint would break at 60 Hz and survive at 120 Hz.
- **Break on the average row force rather than the peak.** Rejected: a joint
  fails where it is loaded hardest. A weld that is being torn at one anchor is
  not saved by its other rows being slack.
- **Include the positional correction in the reported reaction.** Rejected for
  the break test: a joint dropped onto a misaligned pair carries a large one-off
  correction on its first step, and would snap on placement. The velocity
  impulse is the load the joint sustains.
- **A separate breakable-joint component.** Rejected: it would be a second thing
  to author and to keep in step with the joint kind, where three fields on the
  joint itself are enough and serialize with it.
- **Report the reaction through the event stream.** Rejected for now: the
  reaction is a per-step scalar a test reads directly, and an event per joint per
  step would be a large, mostly-ignored stream. The `joint_reaction` accessor
  answers the question without one.
