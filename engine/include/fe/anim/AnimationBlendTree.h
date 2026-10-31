// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/anim/AnimationClip.h"
#include "fe/core/Types.h"
#include "fe/scene/Component.h"

#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace fe {
class ResourceManager;
}

namespace fe::anim {

class Animator;
class Skeleton;

// A two-clip blend tree over the skeletal `Animator`, plus an additive layer.
//
// The `Animator` plays one clip and can cross-fade to a second; that covers a
// state change ("idle to walk") but not a *continuous* blend. Locomotion is the
// case that needs one: a character at a speed between a walk and a run should
// play a mix of both, not snap from one to the other. That is a blend tree, and
// this component is a one-axis cut of it: two clips, a parameter, and a mapping
// from the parameter to the blend weight.
//
// The mapping is the part that makes a blend tree more than a `lerp`. A naive
// blend of walk and run at the speed midpoint is wrong when the two clips have
// different natural speeds: the author wants "walk until 2 m/s, then blend to
// run by 4 m/s". The tree therefore carries a threshold pair per clip, and the
// weight is the parameter's position between them, clamped. `set_parameter` is
// how gameplay drives it; a caller that already knows the weight can set
// `weight` directly.
//
// The additive layer is the second half. A hit reaction, a breathing idle, or a
// lean should compose *over* whatever the base is playing rather than replace
// it. An additive clip is authored as a *delta* from the rest pose, and
// composing it means adding its per-joint delta to the base pose -- with the
// rotation delta applied as a quaternion, not an Euler add. This component holds
// an additive clip and a weight, and the system applies it after the base blend.
//
// Like the state machine, the tree is authored data on a component and the step
// is a system: `AnimationBlendTreeSystem` samples the tree into the node's
// animator pose after `SkeletonAnimationSystem` has sampled the base clip, so
// the blend composes on top of the base rather than replacing it. See ADR-0124.
//
// Serialization: the clip references travel as their resource keys, and the
// whole tree serializes as one compact JSON string on the `tree` property, the
// same shape the state machines use and for the same reason (reflection has no
// list-of-struct kind, and a resource reference is a property on the component,
// not on a sub-value).

// One clip of the blend tree, with the parameter range over which it is active.
//
// `threshold` is the parameter value at which this clip is fully weighted;
// `other_threshold` is the value of the *other* clip, so the pair describes the
// interval over which the blend happens. Storing both on each entry (rather than
// one interval on the tree) is what lets the entries be listed in any order and
// the weights be derived from whichever clip the parameter is nearer.
struct BlendSample {
    std::string name; // for the editor and for a test; not required to be unique
    std::shared_ptr<AnimationClip> clip;
    std::string clip_key;       // the reference key the clip is loaded from and saved as
    f32 threshold = 0.0f;       // parameter value at which this clip has full weight
    f32 other_threshold = 1.0f; // parameter value at which the other clip has full weight
    f32 speed = 1.0f;           // playback-rate multiplier, so a short clip can match a long one
};

// A two-clip blend tree with an optional additive layer.
class AnimationBlendTree final : public Component {
public:
    static constexpr const char* kTypeName = "AnimationBlendTree";

    [[nodiscard]] const char* type_name() const noexcept override { return kTypeName; }

    // The two clips. Fewer than two is legal: a tree with one clip is that clip,
    // and an empty tree is a no-op the system skips. A third entry is ignored,
    // because a one-axis blend has two ends.
    std::vector<BlendSample> samples;

    // The continuous parameter gameplay drives (speed, health, aim pitch). The
    // system turns it into a weight with `blend_weight()`.
    f32 parameter = 0.0f;

    // When true, `parameter` is used and the weight is derived from the two
    // thresholds. When false, `weight` is used verbatim, so a caller that already
    // computed a blend (a script, a curve) can drive the tree directly.
    bool use_parameter = true;

    // The blend weight in [0, 1]: 0 is the first clip, 1 is the second. Written
    // by `set_parameter` when `use_parameter` is set; read directly otherwise.
    f32 weight = 0.0f;

    // When true the base clip(s) are sampled and blended and written onto the
    // animator. When false the tree still evaluates (so an editor or a test can
    // read `weight`) but writes nothing, which is what a scene that only wants
    // the additive layer sets on its base tree.
    bool drive_animator = true;

    // --- The additive layer ------------------------------------------------

    // The clip composed *over* the base pose. Null is the common case: no
    // additive layer. An additive clip is authored as a delta from the rest pose,
    // so a joint its tracks do not mention contributes nothing rather than
    // holding its rest value.
    std::shared_ptr<AnimationClip> additive_clip;
    std::string additive_clip_key;

    // How much of the additive delta to apply, in [0, 1]. Zero is off; a hit
    // reaction ramps it in and back out.
    f32 additive_weight = 0.0f;

    // The additive clip's own playback clock, in seconds. Kept separate from the
    // base clock so a one-shot reaction plays at its own rate over a locomotion
    // blend that is looping.
    f32 additive_time = 0.0f;
    f32 additive_speed = 1.0f;

    // --- Queried state -----------------------------------------------------

    // The blend weight derived from `parameter` and the samples' thresholds,
    // clamped to [0, 1]. With fewer than two samples the result is 0. With
    // `use_parameter` false this is the stored `weight`, so a caller reads one
    // value either way.
    [[nodiscard]] f32 blend_weight() const noexcept;

    // Sets `parameter` and, when `use_parameter` is set, recomputes `weight`
    // from it. The call a gameplay system makes each frame.
    void set_parameter(f32 value) noexcept;

    // --- Resource resolution ----------------------------------------------

    // Loads the sample clips and the additive clip through `manager` from their
    // keys, returning how many were resolved this call. Idempotent: a clip
    // already held is skipped, and a key that does not load is left in place and
    // retried, because an asset may be mounted after the scene. Called by the
    // system when the frame carries a manager.
    usize resolve_clips(ResourceManager& manager);

    // --- Persistence -------------------------------------------------------

    [[nodiscard]] std::string to_json() const;
    bool from_json(std::string_view text);
};

// --- Pure pose functions ---------------------------------------------------
//
// These are free functions over poses rather than members of the component,
// because they are the values a test asserts on with no scene and no animator:
// "sample this clip over the rest pose" and "add this delta over this base" are
// the whole behaviour. They take the skeleton so a delta can be resolved to the
// rig it composes onto, and they write into caller-owned poses so the frame step
// allocates nothing.

// Samples `clip` over `skeleton`'s rest pose into `out`, wrapped for the clip's
// own duration and loop mode. A null clip (or one with no duration) leaves `out`
// at rest, so a one-clip tree is that clip and a missing binding degrades to the
// rest pose rather than to garbage. The building block the blend tree composes.
void sample_clip_pose(const Skeleton& skeleton, const AnimationClip* clip, f32 time, LoopMode mode, SkeletonPose& out);

// Samples an additive `clip` into `out` as a *delta* from the identity pose.
//
// An additive clip is authored as an offset from the bind pose, so a joint its
// tracks do not mention must sample as the identity (a no-op), not as the rest
// pose. That is why this resets `out` to identity rather than to rest, and why
// it is a separate function from `sample_clip_pose`.
void sample_additive_pose(const Skeleton& skeleton, const AnimationClip& clip, f32 time, LoopMode mode,
                          SkeletonPose& out);

// Composes the additive `delta` pose over `base` in place.
//
// Translation and scale blend toward the delta; rotation is applied as
// `base.rotation * slerp(identity, delta.rotation, alpha)` -- the delta in the
// joint's local frame -- not as an Euler sum, because a rotation delta is a
// rotation and adding angles sweeps the wrong path. `alpha` scales the whole
// delta, so a reaction ramps in and back out. `base` is unchanged when `alpha`
// is zero or the two poses differ in length past the shorter.
void compose_additive(SkeletonPose& base, const SkeletonPose& delta, f32 alpha);

} // namespace fe::anim
