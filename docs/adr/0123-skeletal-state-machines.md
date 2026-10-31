# ADR-0123: Skeletal animation state machines

- **Status:** Accepted
- **Date:** 2026-10
- **Related:** [ADR-0059](0059-animation-state-machines.md),
  [ADR-0104](0104-skeletal-animation.md),
  [ADR-0002](0002-scene-graph-and-systems.md),
  [ADR-0044](0044-scripting-native-modules.md)

## Context

[ADR-0059](0059-animation-state-machines.md) added a state machine for the
*sprite* animator: an `AnimationStateMachine` component holding named states
(each a cell range of a sheet) and transitions gated on a finish and on
triggers, advanced by `AnimationStateMachineSystem`. [ADR-0104](0104-skeletal-animation.md)
then added skeletal animation — a `Skeleton`, an `AnimationClip`, an `Animator`,
and `SkeletonAnimationSystem` — and recorded that "blend trees, additive layers,
and skeletal state machines build on this shape and remain." The roadmap's
animation row carries the same gap, and `docs/SHIPPING_QUEUE.md` lists
`skeletal-state-machine` as the next Phase 5 item.

So the engine can play one skeletal clip, and can cross-fade two, but a game
cannot say "idle plays until the player moves, then run plays; an attack
interrupts either and returns to idle when it finishes" for a skinned character.
It can say it for a sprite and not for a rig. The question this record answers is
what shape the skeletal machine takes and how it drives the animator.

The forces:

- **It is the same problem as the sprite machine.** Named states, transitions
  gated on a finish and on triggers, an "any state" interrupt edge. A designer
  who authored one should recognize the other. A second, different model for the
  same concept is a cost the engine should not pay.
- **A skeletal state names a clip, not a cell range.** The sprite machine names
  an absolute first cell and a frame count into the animator's sheet; a rig plays
  a whole `AnimationClip` with its own duration and loop mode. The state's data
  differs even though its *shape* does not.
- **A clip is a resource with a key.** A scene must persist which clip a state
  plays across a save and load, the way `Animator` persists its own clip. The
  sprite machine needs no reference because a cell is a number; the skeletal one
  does.
- **The machine drives the animator's clip, and the animation system samples
  it.** In the sprite pair the machine writes the animator's `current_frame`
  *after* `SpriteAnimationSystem` has stepped, so the machine's cell is the last
  writer and the rendered UVs are the machine's. A skeletal clip is different:
  the machine *chooses* the clip and the animation system *samples* it, so the
  ordering is the opposite.
- **Reflection is a closed, scalar model** ([ADR-0005](0005-reflection-and-serialization.md),
  [ADR-0006](0006-component-reflection.md)) with no list-of-struct kind, exactly
  as [ADR-0059](0059-animation-state-machines.md) already found.

## Decision

Add a `SkeletalStateMachine` component and a `SkeletalStateMachineSystem`, in the
`anim` module beside the skeletal types, mirroring the sprite machine's split
between authored state and per-frame behaviour.

### The component is the graph; the system is the step

`SkeletalStateMachine` holds a list of `SkeletalState` (a name, a clip, the loop
mode) and a list of `SkeletalTransition` (from, to, a require-finished gate, and
trigger conditions), an initial state, a `playing` flag, and a `drive_animator`
flag. `SkeletalStateMachineSystem` gathers the machine-bearing nodes into a flat
list and advances each once per frame, the same shape every system follows
([ADR-0002](0002-scene-graph-and-systems.md)). The component is testable with no
scene and no animator, because it owns its own clock.

### A state names a clip, by resource key

Where the sprite state names `first_frame`/`frame_count`, the skeletal state
holds an `AnimationClip` and its reference key. The key is the same
`clip->path()` form `Animator` stores, so one reference convention serves both
components. The graph serializes the key; `resolve_clips(manager)` loads the key
back through the `ResourceManager` and is idempotent, skipping a state that
already holds a clip and retrying a key that does not load yet — because an asset
may be mounted after the scene, and a hot reload must be able to fill a state in.
This is why the machine resolves its clips through the manager rather than
through a per-state resource property: a resource reference is a `Property` on
the *component*, and a list of states is not a component property.

### The machine owns the finish clock; the animator owns playback

The machine advances its own `elapsed_` by the frame delta and compares it to the
active state's clip duration to decide `finished_`. A looping state never
finishes; a state with no clip is finished at once, so a "rest" state hands off
without a timer. When a transition enters a state, the machine raises a
one-shot *clip request*; the system consumes it and, when `drive_animator` is
set, calls `animator->play(clip, loop)` on the node's animator, which restarts
the animator's clock so a state entry is a clean cut. The request is
one-shot precisely so the bind happens once per entry rather than every frame,
and so a machine that never changes state never touches the animator.

### Ordering is before the animation system

Add `SkeletalStateMachineSystem` *before* `SkeletonAnimationSystem`. The machine
chooses the clip and the animation system samples it in the same frame, so a
state change is visible in the pose that frame rather than the next. This is the
mirror of the sprite pair, where the machine runs *after* the sprite animation
system because it writes the animator's cell directly.

### The graph serializes as one compact JSON string

As in [ADR-0059](0059-animation-state-machines.md), rather than widen
`reflect::Value` with a list-of-struct kind for one component, the machine exposes
one `String` property (`graph`) whose getter calls `to_json()` and whose setter
calls `from_json()`. The clip key travels inside that JSON. The property carries
the `Multiline` hint, so a scene file is hand-editable and the inspector shows a
text box until a dedicated graph editor exists.

## Consequences

- A skinned character can be driven by an authored state machine: idle/run/attack
  with finish-gated and trigger-gated edges, the same authoring model the sprite
  machine already uses. The new tests cover entry, a trigger transition, a
  finish-gated return, a looping state's clock wrap, a triggerless self-edge
  being ignored, pause, the JSON round-trip, driving a real animator, the
  `drive_animator = false` path, the skipped-no-animator path, and the
  reflection registration.
- The sprite machine is untouched. The two components are separate types with
  separate systems, because a sprite state and a skeletal state carry different
  data even though their graphs have the same shape.
- Blend trees and additive layers remain open, as
  [ADR-0104](0104-skeletal-animation.md) recorded; this record closes only the
  state-machine half. A blend tree would sit between the machine and the animator,
  choosing a blend from continuous parameters the way the machine chooses a clip
  from discrete states.
- The clip request is a one-shot flag rather than a per-frame comparison, so a
  machine that does not change state costs one flag check and no animator write.
- A state with no clip holds the animator's pose rather than clearing it. A game
  that wants a state to play nothing authors a state with no clip and drives it
  deliberately; clearing the animator's clip would drop the pose, which is the
  same reasoning `Animator` uses for pausing.

## Alternatives considered

- **Reuse `AnimationStateMachine` with a clip instead of a cell range.** Rejected:
  the two states carry different data (a cell range versus a resource reference
  and a loop mode), and folding both into one struct would put a union of two
  unrelated shapes on every state and make the reflection property carry both.
  Two components with the same graph shape are clearer than one with two modes.
- **Have the system bind the clip every frame.** Rejected: `play()` restarts the
  animator's clock, so rebinding each frame would freeze the clip on its first
  pose. The one-shot request is what makes a state entry a cut and a steady state
  free.
- **A per-state resource property on the component.** Rejected: the reflection
  model has no list-of-struct kind, so there is no place to hang a per-state
  resource reference. The clip key inside the graph JSON is the same decision
  [ADR-0059](0059-animation-state-machines.md) made for the sprite graph.
- **Drive the animator in the animation system instead.** Rejected: the machine
  would have to publish its chosen clip somewhere the animation system reads, and
  that is a second channel for the same information. One system owning the bind
  keeps the ordering explicit and the data flow one-directional.
- **Resolve clips in `from_json`.** Rejected: `from_json` has no `ResourceManager`,
  and a scene load must not depend on one being present. Resolving in the frame
  step, when the context carries a manager, keeps a document load pure and lets a
  clip appear later.
