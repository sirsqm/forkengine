# ADR-0146: A blend tree feeding a blend tree

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0124](0124-animation-blend-trees.md),
  [ADR-0141](0141-two-axis-blend-space.md),
  [ADR-0142](0142-per-layer-joint-mask.md),
  [ADR-0144](0144-animation-events.md)

## Context

`AnimationBlendTree` is a one-axis blend: two entries, one parameter, a weight
from the parameter's position between the entries' thresholds. That covers a
locomotion blend of walk and run by speed. It does not cover the case the
shipping queue names as the remaining animation work: **a tree feeding a tree**.

The case is real and common. A character who walks, runs, and *aims* has two
blends that do not share a parameter: the legs vary with speed, the upper body
varies with aim pitch. Neither is a single clip, and one parameter cannot drive
both. Today the only answer is to enumerate the combinations as clips, which is
the authoring the blend tree exists to avoid.

The one-axis tree already blends two *poses*. A pose is what a tree produces. So
the smallest change that answers the case is to let an entry be a tree rather than
only a clip, and let the parent blend the sub-tree's pose exactly as it blends a
clip's. Nothing about the parent's blend changes; only what an entry *is*.

Constraints that shaped the decision:

- **No new evaluation model.** The parent already samples each entry into a pose
  and blends the two. A sub-tree produces a pose through the same path, so the
  change is one level of recursion, not a second algorithm.
- **No allocation in the frame step.** The engine treats a steady-state frame as
  allocation-free. A recursive evaluation that allocated a pose per level would
  break that, so the poses come from a caller-owned pool.
- **A cycle is an authoring mistake, not a crash.** A tree can reference another
  tree; nothing stops an author (or a corrupted file) from making that a cycle.
  Evaluation must terminate regardless.
- **Authored data, not a resource.** A sub-tree is part of the tree that owns it,
  not a separately addressable asset, so it serializes inline rather than as a
  reference key.

## Decision

`BlendSample` gains `subtree`, a `shared_ptr<AnimationBlendTree>`. When it is set,
the entry is evaluated as a tree instead of a clip: the sub-tree's own samples,
thresholds, and additive layers apply, and its `drive_animator` is ignored, because
a sub-tree always produces a pose for its parent to blend.

The base blend and the timeline-event collection move out of the system and into
one pure function, `evaluate_blend_tree`, which:

- resets the output pose to the skeleton's rest pose (so a joint no entry animates
  holds its authored position, as a single clip already does),
- samples each of the two entries, recursing when an entry is a sub-tree,
- blends them by the parent's weight, and
- collects the timeline markers of the *leaf* entries it played, at each entry's
  own rate (ADR-0144), so a sub-tree's clips report their events to the same
  stream the parent's do.

Two guards make it safe. A pose pool is sized to the whole allowed depth **before**
recursion, because a reference into the pool taken at one level must stay valid
while a deeper level recurses -- growing it mid-recursion would dangle that
reference (this was a real crash while building the change, not a hypothetical).
And a depth cap (`kMaxBlendTreeDepth`, 8) makes evaluation stop and treat the
entry as the rest pose rather than recursing without end.

`resolve_clips` recurses into a sub-tree, so a nested tree's clips load through the
same key resolution. `to_json` writes a sub-tree inline as an object; `from_json`
rebuilds it, so the whole tree is one document.

## Consequences

Easier: a character with independent upper- and lower-body blends is one
component, not an enumeration of clips. The parent's blend, weights, and additive
layers are unchanged, so an existing tree that uses only clips behaves exactly as
before. Events reach gameplay from every clip in the tree, however it nests.

Harder: the depth cap means a tree nested deeper than 8 silently falls back to the
rest pose at the deepest level; that is a documented limit, not a checked error,
and a scene that trips it would see a still character rather than a message. The
evaluation is recursive, so a very deep tree costs stack proportional to its
depth (bounded by the cap). A sub-tree is shared state (`shared_ptr`) but is
treated as immutable by evaluation, so a scene that mutates a sub-tree's parameter
between frames changes what every parent that references it blends -- which is the
intended sharing, but means a sub-tree is not a per-instance value.

## Alternatives considered

- **A general N-ary tree with a per-entry weight.** Rejected for now: a one-axis
  two-entry blend is the whole of what the queue names, and a general tree adds an
  authoring surface (an explicit weight per child) that the two-axis blend space
  already serves for the "many clips at once" case. Nesting two one-axis trees
  covers the upper/lower-body case without it.
- **A resource reference for the sub-tree.** Rejected: a sub-tree is authored data
  on the tree that owns it, not a separately mounted asset; an inline document
  keeps the whole tree in one place and one load.
- **Growing the pose pool on demand.** Rejected: a mid-recursion `resize` invalidates
  the references the caller already holds, which crashes. Sizing it up front for
  the whole allowed depth is simpler and safe.
- **Detecting a cycle by identity.** Rejected in favour of a depth cap: identity
  tracking needs a set threaded through the recursion for a case that is always a
  mistake, and the cap bounds the work whatever the shape of the cycle.
