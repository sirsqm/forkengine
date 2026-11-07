# ADR-0144: Animation events on the clip timeline

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0104](0104-skeletal-animation.md),
  [ADR-0123](0123-skeletal-state-machines.md),
  [ADR-0124](0124-animation-blend-trees.md)

## Context

Animation reaches gameplay in two ways: through the pose (a hitbox reads a hand
joint's transform) and through *time*. The second is missing. A walk cycle needs a
footstep sound at the instant the foot lands; an attack needs a window where the
hitbox is live and another where it is not; a landing needs a dust burst on
impact. None of those is a pose -- they are events on the clip's timeline, and the
frame the clock passes one is the frame gameplay reacts to.

Today a game has no way to know when playback crossed a point in a clip. It can
poll the animator's clock and re-derive the crossing itself, but every game would
re-derive it, get the wrap and the half-open window subtly different, and fire a
footstep twice or not at all.

Constraints that shaped the decision:

- **The engine reports, gameplay decides.** An event is a *name*, and what a name
  means (play a sound, open a hitbox, spawn particles) is the game's business. The
  engine must not know that "footstep" means audio.
- **A marker fires exactly once.** As the clock sweeps past a marker, the marker
  is reported on the single frame that crossed it -- not on every frame the clock
  sits beyond it, and not zero times.
- **A loop must not drop a marker at the seam.** A clip that loops has markers
  near both ends; a frame that wraps past the end and back to the start passed
  both, in the order it passed them, and both must be reported.
- **The pose is unaffected.** A clip with no events must behave exactly as it did
  before, and an event must not change a single joint.

## Decision

Add `AnimationEvent` (a name and a time in the clip's own seconds) to
`AnimationClip`, and report crossings from `SkeletonAnimationSystem`.

- The clip keeps its events sorted by time (`set_events`, `add_event`), so a
  crossing can be reported in timeline order without re-sorting each frame.
- `collect_clip_events(clip, previous_time, current_time, out)` is a free
  function that appends the events in the half-open window
  `(previous_time, current_time]`, in order. Both arguments are *wrapped*
  positions, so the function compares positions rather than the raw clock. When
  `current_time` is behind `previous_time` the clock wrapped, and the window is
  the tail `(previous_time, duration]` followed by the head `[0, current_time]`,
  reported in the order playback passed them.
- `SkeletonAnimationSystem` records each animator's wrapped position from the
  previous frame and, each frame, collects the markers the primary clip crossed
  into the new position, publishing them through `events()`. Only the primary clip
  contributes, so an outgoing cross-fade clip does not fire a marker twice.

## Consequences

Easier: a game reacts to a footstep, a hitbox window, or a landing by name, on the
frame it happens, without re-deriving the crossing rule; the wrap case is handled
once, in the engine, rather than in every game; a clip with no events is
byte-for-byte the clip it was.

Harder: the system carries one extra `f32` per animator and one shared event
buffer, and the crossing scan is linear over a clip's events (a handful in
practice). The events live on the clip *resource*, which is shared by every
instance of a character, so a game that wants per-instance markers must key them
by name rather than mutate the clip. The report is forward-playback only: a clip
driven backwards (a negative speed, or `PingPong`'s reflected half) sees a wrap
where there is none, so a caller playing backwards collects its own events from
the raw clock; that limit is documented at the function.

## Alternatives considered

- **Events as a component list rather than on the clip.** Rejected: the markers
  are a property of the animation (they sit at times in the clip), and putting them
  on the component would let the clip and its markers disagree, and would not
  travel with the clip to its other users.
- **Report a marker on every frame the clock is past it.** Rejected: that is not
  an event, it is a state, and a footstep would fire sixty times a second.
- **Report the raw clock and let the game compare.** Rejected: that is the
  re-derivation this ADR exists to remove, and the wrap case is easy to get wrong.
- **A callback per event.** Rejected: a callback needs a target and a lifetime,
  and the systems model is a per-frame report a game system reads, exactly as it
  reads an action rather than a key.
