// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/anim/AnimationClip.h"
#include "fe/anim/JointMask.h"
#include "fe/core/Types.h"

#include <memory>
#include <string>

namespace fe::anim {

// One additive layer over a base pose.
//
// The single `additive_clip`/`additive_weight`/`additive_mask` fields on the
// blend components cover the common case: one reaction or one lean over a
// locomotion base. A character that leans *and* breathes *and* takes a hit wants
// several, each with its own clip, weight, clock, and joint set -- and they
// compose in order, so the second layer sees the pose the first produced. That is
// what this value is: one entry of a stack.
//
// The layers are a list rather than a fixed set because how many a character
// needs is a property of the character, not of the engine. The order is the
// order they are listed, which is the order a designer reads them in.
struct AdditiveLayer {
    // The clip composed over the running pose. Null is skipped, so an unfilled
    // entry costs nothing.
    std::shared_ptr<AnimationClip> clip;
    std::string clip_key; // the reference key the clip is loaded from and saved as

    // How much of the delta to apply, in [0, 1]. Zero is off.
    f32 weight = 0.0f;

    // The layer's own playback clock, in seconds, and its rate. Separate from the
    // base clock so a one-shot reaction plays at its own rate over a looping
    // locomotion blend.
    f32 time = 0.0f;
    f32 speed = 1.0f;

    // The joints this layer may write. Empty means every joint.
    JointMask mask;
};

} // namespace fe::anim
