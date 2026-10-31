// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/anim/AnimationBlendTree.h"
#include "fe/anim/Skeleton.h"
#include "fe/systems/System.h"

#include <string_view>
#include <vector>

namespace fe::anim {

class Animator;

// Evaluates every `AnimationBlendTree` in the scene once per frame and writes
// the resulting pose onto the node's `Animator`.
//
// Ordering: add this system *after* `SkeletonAnimationSystem`. The animation
// system advances the animator's clock, samples the animator's own clip, and
// computes the palette; this system then replaces the pose with the tree's blend
// (when the tree drives the animator) and composes the additive layer over it,
// and recomputes the palette so what the renderer reads is the blended pose. A
// node driven by a blend tree normally leaves the animator's `clip` null, so the
// animation system's own sampling is the rest pose and this system's write is
// the whole answer.
//
// The clock is the animator's. The tree has no clock of its own for the base
// blend: it samples each clip at the animator's `time` scaled by that sample's
// `speed`, so one authored rate (on the animator) still starts, stops, and
// scrubs the whole tree, and a per-sample speed only matches clips of different
// lengths. The additive layer keeps its own clock (`additive_time`), because a
// one-shot reaction plays at its own rate over a locomotion blend that loops.
//
// A tree with no animator, or with an animator that has no skeleton, is skipped
// rather than treated as an error: a scene between "component added" and "asset
// assigned" looks exactly like that.
class AnimationBlendTreeSystem final : public System {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "AnimationBlendTreeSystem"; }

    void update(const FrameContext& context, Scene& scene) override;

    // Trees evaluated during the most recent update.
    [[nodiscard]] usize last_advanced_count() const noexcept { return advanced_; }

    // Trees skipped for want of an animator or a skeleton.
    [[nodiscard]] usize last_skipped_count() const noexcept { return skipped_; }

    // Clips resolved through the frame's resource manager during the most recent
    // update. Non-zero only on the frames after a load.
    [[nodiscard]] usize last_resolved_count() const noexcept { return resolved_; }

private:
    std::vector<AnimationBlendTree*> trees_;

    // Reused across frames so a steady-state scene allocates nothing. One for
    // each side of the base blend and one for the additive delta.
    SkeletonPose pose_a_;
    SkeletonPose pose_b_;
    SkeletonPose additive_scratch_;

    usize advanced_ = 0;
    usize skipped_ = 0;
    usize resolved_ = 0;
};

} // namespace fe::anim
