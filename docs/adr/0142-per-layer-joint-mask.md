# ADR-0142: A per-layer joint mask

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0124](0124-animation-blend-trees.md),
  [ADR-0141](0141-two-axis-blend-space.md),
  [ADR-0104](0104-skeletal-animation.md)

## Context

The additive layer of `AnimationBlendTree` (ADR-0124) and of
`AnimationBlendSpace2D` (ADR-0141) composes a delta over the base pose for every
joint the delta's clip mentions. That is right when the layer is a whole-body
effect -- a stagger, a land -- and wrong when it is not. A hit reaction authored
as a delta on the upper body swings the arms; composed over a run cycle it also
swings the legs, because the clip's tracks happen to reach them, or it fights the
legs' base motion where they overlap. A breathing idle should move the chest and
not the feet. There was no way to say "this layer owns these joints".

Constraints that shaped the decision:

- **The mask is per layer, not per clip.** The same clip may be applied masked on
  one layer and unmasked on another; the restriction belongs to the layer that
  composes it, so it lives on the component, beside `additive_weight`.
- **Names, resolved once.** A human authors joint names, not indices, and a name
  is what survives a serialization round trip. Resolving names to indices is done
  against the skeleton once, before the per-frame compose, so the per-joint test
  in the hot loop is an index lookup rather than a string search.
- **The default is "no restriction".** An empty mask restricts nothing, and a
  mask that has not been resolved against a skeleton also restricts nothing,
  rather than silently dropping the whole layer. A forgotten resolve must fail
  toward the old behaviour, not toward a no-op that looks like a bug elsewhere.

## Decision

Add `JointMask`, a small value type holding a list of joint names and a resolved,
sorted list of joint indices.

- `resolve(skeleton)` turns the names into indices, dropping a name the skeleton
  does not have and returning how many resolved, so a mask authored against a
  different rig restricts what it can. `contains(index)` answers the per-joint
  test: false for a joint the resolved mask does not name, true for every joint
  when the mask is empty or unresolved.
- `compose_additive_masked(base, delta, alpha, mask)` is `compose_additive` with
  a per-joint guard; an empty mask forwards to the unmasked function, so the two
  agree exactly.
- `AnimationBlendTree` and `AnimationBlendSpace2D` each gain an `additive_mask`
  field, serialized as a `joints` name array inside their existing JSON property,
  and their systems resolve the mask against the animator's skeleton before the
  additive compose.

## Consequences

Easier: an additive layer can be scoped to the joints it is about -- upper body,
a single limb, the spine -- so a hit reaction no longer fights the legs and a
breathing idle no longer moves the feet; the same clip can be used masked and
unmasked; the mask is authored, serialized, and resolved the same way on both
blend components.

Harder: the mask is a second thing to author per additive layer, and a mask
authored against a rig that renames a joint silently loses that joint from the
layer (reported by `resolve`'s count, so a test or an editor can see it). The
resolution runs on every frame the layer is active rather than being cached, which
is a name search per active masked layer; the sample counts here are tiny, so this
is not worth a cache in this cut.

## Alternatives considered

- **A bitmask over joint indices.** Rejected: it cannot be authored by hand or
  survive a rig change, and a rig's joint count is not known when the mask is
  written.
- **Store resolved indices only.** Rejected: the mask must serialize and reopen,
  and an index is meaningless across a rig change.
- **A mask on the clip.** Rejected: the same clip is legitimately masked on one
  layer and not another; the restriction belongs to the layer.
- **Resolve lazily inside `contains`.** Rejected: `contains` is in the per-frame
  loop and takes no skeleton; resolving once before the compose keeps the loop a
  binary search.
