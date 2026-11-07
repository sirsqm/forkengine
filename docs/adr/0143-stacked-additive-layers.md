# ADR-0143: Stacked additive layers

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0124](0124-animation-blend-trees.md),
  [ADR-0141](0141-two-axis-blend-space.md),
  [ADR-0142](0142-per-layer-joint-mask.md)

## Context

`AnimationBlendTree` (ADR-0124) and `AnimationBlendSpace2D` (ADR-0141) each
carry one additive layer: a single `additive_clip`, `additive_weight`, clock,
and mask, composed over the base pose. That covers the common case -- one
reaction, one lean, one breath over a locomotion blend -- and it does not cover
the next one. A character that leans *into* a turn *and* breathes *and* takes a
hit wants three additive effects at once, each with its own clip, its own weight,
its own clock (a one-shot reaction at its own rate over a looping idle), and its
own joint set (the lean on the spine, the breath on the chest, the hit on the
upper body).

Folding those into the single layer's fields is not possible: they have different
weights and clocks. A fixed set of named layers (`additive_2`, `additive_3`) is
not better -- how many a character needs is a property of the character, not of
the engine -- and it would multiply the serialized surface for no gain.

Constraints that shaped the decision:

- **Compose in order, each over the last.** An additive layer is a delta over a
  pose, so a stack has a natural order and each layer must see the pose the one
  before it produced. The order is the authoring order.
- **The existing single layer stays.** It is the common case and it is already
  serialized in shipped scenes; a refinement that forced every scene to migrate to
  a list would be a breaking change for no benefit.
- **Each layer is self-contained.** Clip, weight, clock, speed, and mask travel
  together, so adding a layer is adding one entry and removing one is removing
  one, with nothing to keep in sync elsewhere.

## Decision

Add `AdditiveLayer`, a value holding a clip (and its reference key), a weight, a
playback clock and rate, and a `JointMask`, and give both blend components an
ordered `additive_layers` vector applied after the single additive layer.

- The systems apply the stack in order after the base blend and the single
  additive layer. Each active layer advances its own clock while the animator is
  playing, samples its clip, resolves its mask against the skeleton, and composes
  its delta over the running pose with `compose_additive_masked`.
- A layer with a null clip or a weight of zero is skipped, so an unfilled entry
  costs nothing and a layer can be toggled off by weight alone.
- The stack serializes as an `additive_layers` array of `{clip, weight, time,
  speed, mask}` objects inside the component's existing JSON property, and
  `resolve_clips` resolves each layer's clip by its key the same way it resolves
  the base samples and the single additive clip.

## Consequences

Easier: a character can carry several additive effects at once, each scoped and
paced independently, and they compose in a readable order; a new effect is one
entry; the single-layer path is untouched, so no shipped scene changes.

Harder: the component's JSON now has two ways to express an additive effect (the
single layer and the stack), which is a small redundancy kept deliberately for
backward compatibility. The stack is applied after the single layer rather than
merged with it, so an author who fills both gets both -- documented here and in
the field comments, and the natural reading of "a layer and a stack of further
layers". A long stack is a per-frame cost proportional to the number of active
layers, each a clip sample; the counts here are small, and a layer can be skipped
by weight.

## Alternatives considered

- **Replace the single layer with a one-element stack.** Rejected: a breaking
  change to every shipped scene and to the serialized format for a purely
  additive feature.
- **A fixed set of numbered layers.** Rejected: the count is a property of the
  character, and numbered fields multiply the serialized surface.
- **Compose the stack in parallel from the base rather than in order.** Rejected:
  an additive layer is a delta over a pose, so "over which pose" must be defined;
  in-order composition is the definition that makes a lean and a breath interact
  the way an animator expects, and parallel composition would need an explicit
  base to be meaningful.
