# ADR-0124: Animation blend trees and additive layers

- **Status:** Accepted
- **Date:** 2026-10
- **Related:** [ADR-0104](0104-skeletal-animation.md),
  [ADR-0123](0123-skeletal-state-machines.md),
  [ADR-0059](0059-animation-state-machines.md),
  [ADR-0002](0002-scene-graph-and-systems.md)

## Context

[ADR-0104](0104-skeletal-animation.md) landed the skeletal runtime, the glTF
import, and the skinned draw, and recorded that "blend trees, additive layers,
and skeletal state machines build on this shape and remain."
[ADR-0123](0123-skeletal-state-machines.md) closed the state-machine half.
`docs/SHIPPING_QUEUE.md` lists `animation-blend-tree` as the next Phase 5 item,
and the roadmap carries the same two gaps.

The engine can now play one clip, cross-fade to a second, and switch between
named states. Two things a character needs are still missing.

**A continuous blend.** Locomotion is not a state change; it is a *mix*. A
character at a speed between a walk and a run should play a blend of both, and
the mix should follow a gameplay parameter (speed, aim pitch, health) rather than
snap at a threshold. The animator's cross-fade is a *transition* — it runs once,
over a fixed duration, from one clip to another — and cannot express "hold a
mixture of two clips for as long as the parameter says so".

**An additive layer.** A hit reaction, a breathing idle, or a lean should compose
*over* whatever the base is playing rather than replace it. Playing the reaction
as a state stops the locomotion; the reaction is a *delta* on top of it.

The forces:

- **A blend is a weight, not a transition.** The animator's cross-fade owns a
  clock and a remaining time; a blend owns nothing but a weight the parameter
  maps to. Reusing the cross-fade for locomotion would fight it: a continuous
  blend never "finishes", so the fade would never retire.
- **Two clips is the whole of a one-axis blend, but the mapping is not a lerp.**
  Blending walk and run at the speed midpoint is wrong when the two clips have
  different natural speeds; the author wants "walk until 2 m/s, then blend to run
  by 4 m/s". The tree therefore carries a threshold pair, not a bare 0-to-1
  weight, so the weight is the parameter's position between two authored values.
- **An additive clip is a delta from the bind pose.** Its value for a joint is an
  *offset*, so a joint its tracks do not mention must contribute nothing (the
  identity), not the rest value. And the rotation offset is a rotation: it
  composes onto the base quaternion, it is not added to an Euler triple.
- **The pose is a plain value and the palette is derived.** The blend and the
  additive composition are pure functions over `SkeletonPose`; the palette is
  recomputed from the result. That is what keeps the whole thing testable with no
  scene, no device, and no GPU — the property [ADR-0104](0104-skeletal-animation.md)
  established for the runtime.
- **Reflection is a closed, scalar model** with no list-of-struct kind, exactly
  as [ADR-0059](0059-animation-state-machines.md) and
  [ADR-0123](0123-skeletal-state-machines.md) already found.

## Decision

Add an `AnimationBlendTree` component and an `AnimationBlendTreeSystem`, in the
`anim` module beside the other skeletal types, and two free pose functions for
the composition.

### A component is the tree; a system is the step

`AnimationBlendTree` holds up to two `BlendSample`s (a clip, a reference key, a
threshold pair, and a playback-rate multiplier), the continuous `parameter`, a
`use_parameter` flag with a directly settable `weight`, and the additive layer
(`additive_clip`, `additive_weight`, its own clock). `AnimationBlendTreeSystem`
gathers the trees into a flat list and evaluates each once per frame, the shape
every system follows ([ADR-0002](0002-scene-graph-and-systems.md)). The weight
mapping (`blend_weight`, `set_parameter`) is a method on the component and is
testable with no scene.

### The weight is the parameter's position between two thresholds

Each sample carries `threshold` (where it is fully weighted) and
`other_threshold` (where the other is), so the weight is the parameter's clamped
position between the two and a parameter past either end pins the nearer clip.
Storing the pair on each sample, rather than one interval on the tree, is what
lets the two entries be listed in either order. A degenerate interval is a hard
cut. `use_parameter` false reads `weight` verbatim, so a caller that already
computed a blend — a script, a curve, the editor — drives the tree directly.

### The animator owns the clock; a per-sample speed matches clip lengths

The tree has no clock for the base blend. It samples each clip at the animator's
`time` scaled by that sample's `speed`, so one authored rate still starts, stops,
and scrubs the whole tree, and a per-sample speed only matches clips of different
lengths (a short run cycle beside a long walk). The additive layer keeps its own
clock, because a one-shot reaction plays at its own rate over a locomotion blend
that loops.

### The additive clip is sampled over identity and composed

`sample_additive_pose` resets its output to the *identity* pose, not the rest
pose, and samples the clip over that: a joint the clip does not animate then
samples as the identity, which is the no-op for the composition. `compose_additive`
adds the delta's translation and scale and composes the rotation as
`base.rotation * slerp(identity, delta.rotation, alpha)`. Slerping from the
identity is how the weight scales a rotation without leaving the unit sphere; a
componentwise scale would not be a rotation.

### The system runs after the animation system and recomputes the palette

Add `AnimationBlendTreeSystem` *after* `SkeletonAnimationSystem`. The animation
system advances the animator's clock and computes the base palette; the blend
system then replaces the pose with the tree's blend (when `drive_animator` is
set) and composes the additive layer over it, and recomputes the palette so what
the renderer reads is the blended pose. A node driven by a blend tree normally
leaves the animator's `clip` null, so the animation system's own sample is the
rest pose and this system's write is the whole answer. A tree that wrote nothing
leaves the animation system's palette in place.

### The tree serializes as one compact JSON string

As in [ADR-0059](0059-animation-state-machines.md) and
[ADR-0123](0123-skeletal-state-machines.md), the tree exposes one `String`
property (`tree`) whose getter calls `to_json()` and whose setter calls
`from_json()`. The clip keys travel inside that JSON and `resolve_clips` loads
them through the `ResourceManager`, idempotently, because a document load has no
manager and an asset may be mounted after the scene.

## Consequences

- A character can be driven by a continuous locomotion blend with an additive
  reaction on top: the queue item's acceptance criteria, proven by tests that
  check the palette at the extremes and the midpoint. The tests also cover the
  weight mapping, a degenerate interval, `use_parameter` off, the identity
  sampling of an unanimated joint, the JSON round-trip, the reflection
  registration, the skip path, and `drive_animator` off.
- The animator, the state machine, and the sprite machine are untouched. The
  blend tree is a separate component with a separate system; a scene can use any
  subset.
- A one-axis (two-clip) blend is the whole of this cut. A two-axis blend (a
  locomotion grid of idle/walk/run strafe), a blend of more than two clips, and a
  hierarchy of trees (a tree feeding a tree) are additive on this shape and
  remain; the pure pose functions compose, so a blend-of-blends is a caller
  combining `blend_poses` results rather than a new mechanism.
- The additive layer composes over the base blend only. Additive-on-additive and
  a per-layer mask (apply a delta to some joints but not others) remain.
- The system recomputes the palette once per tree per frame, which is a second
  `world * inverse_bind` walk for a blended character. That is the cost of the
  blend being authoritative; a character with no blend tree pays nothing, because
  the system has no tree to visit.
- A tree with an animator but no skeleton is skipped rather than producing an
  empty palette, matching `SkeletonAnimationSystem`'s rule, so a scene between
  "component added" and "asset assigned" degrades instead of crashing.

## Alternatives considered

- **Reuse the animator's cross-fade for locomotion.** Rejected: a cross-fade is a
  timed transition that retires; a continuous blend is a held weight that never
  retires, and a parameter that moves back and forth would restart the fade each
  frame. The two are different in kind, so the blend is its own component.
- **Store one interval on the tree instead of a threshold pair per sample.**
  Rejected: it forces the entries into a fixed order and makes the mapping depend
  on which entry is "first". The pair per sample makes the order irrelevant and
  the weight a symmetric function of the two thresholds.
- **A bare 0-to-1 weight with no thresholds.** Rejected: it puts the
  parameter-to-weight mapping on every caller, so each game repeats the same
  clamp-and-normalise and they disagree at the edges. The threshold pair is the
  mapping, authored once.
- **Add the additive rotation as Euler angles.** Rejected: a rotation delta is a
  rotation, and adding angles takes a path the author never intended and sweeps
  the long way across the wrap — the same reason `JointPose` stores a quaternion.
- **Sample the additive clip over the rest pose.** Rejected: a joint the delta
  does not animate would then contribute the rest value, so composing it would
  double the rest transform. Identity is the additive no-op, and the clip is a
  delta from the bind pose.
- **Compose the additive layer inside `SkeletonAnimationSystem`.** Rejected: it
  would put two unrelated behaviours in one system and force the animation system
  to know about blend trees. Two systems with an explicit order keeps the data
  flow one-directional, the same split the state machine uses.
