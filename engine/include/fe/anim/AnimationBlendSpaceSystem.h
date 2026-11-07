// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/anim/AnimationBlendSpace2D.h"
#include "fe/anim/AnimationClip.h"
#include "fe/anim/Skeleton.h"
#include "fe/systems/System.h"

#include <string_view>
#include <vector>

namespace fe::anim {

class Animator;

// Evaluates every `AnimationBlendSpace2D` in the scene once per frame and writes
// the resulting pose onto the node's `Animator`.
//
// Ordering: add this system *after* `SkeletonAnimationSystem`, exactly like
// `AnimationBlendTreeSystem`. The animation system advances the animator's clock,
// samples the animator's own clip, and computes the palette; this system then
// replaces the pose with the space's blend (when the space drives the animator)
// and composes the additive layer over it, and recomputes the palette so the
// renderer reads the blended pose. A node driven by a blend space normally leaves
// the animator's `clip` null, so the animation system's own sampling is the rest
// pose and this system's write is the whole answer.
//
// The clock is the animator's, as in the one-axis tree: each sample is taken at
// the animator's `time` scaled by that sample's `speed`, so one authored rate
// still starts, stops, and scrubs the whole space. The additive layer keeps its
// own clock (`additive_time`).
class AnimationBlendSpaceSystem final : public System {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "AnimationBlendSpaceSystem"; }

    void update(const FrameContext& context, Scene& scene) override;

    // Spaces evaluated during the most recent update.
    [[nodiscard]] usize last_advanced_count() const noexcept { return advanced_; }

    // Spaces skipped for want of an animator or a skeleton.
    [[nodiscard]] usize last_skipped_count() const noexcept { return skipped_; }

    // Clips resolved through the frame's resource manager during the most recent
    // update. Non-zero only on the frames after a load.
    [[nodiscard]] usize last_resolved_count() const noexcept { return resolved_; }

    // The timeline events of the contributing clips that playback crossed during
    // the most recent update, in the order they were passed. A marker two samples
    // share (a matched pair of directional clips) is reported once. Empty when no
    // sample carried events. Rebuilt every update.
    [[nodiscard]] const std::vector<AnimationEvent>& events() const noexcept { return events_; }

private:
    std::vector<AnimationBlendSpace2D*> spaces_;

    // Reused across frames so a steady-state scene allocates nothing. Three for
    // the base blend (a triangle is the widest case) and one for the additive
    // delta.
    SkeletonPose pose_a_;
    SkeletonPose pose_b_;
    SkeletonPose pose_c_;
    SkeletonPose additive_scratch_;

    std::vector<AnimationEvent> events_;
    usize advanced_ = 0;
    usize skipped_ = 0;
    usize resolved_ = 0;
};

} // namespace fe::anim
