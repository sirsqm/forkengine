// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/anim/SkeletalStateMachine.h"
#include "fe/systems/System.h"

#include <string_view>
#include <vector>

namespace fe::anim {

class Animator;

// Advances every `SkeletalStateMachine` in the scene once per frame and, when a
// machine asks to drive playback, binds the active state's clip onto the node's
// `Animator`.
//
// This is the behaviour half of the machine, in the same split `Animator`
// (state) and `SkeletonAnimationSystem` (behaviour) already use: the component is
// authored and serialized, the system owns the traversal and the step. It gathers
// the machine-bearing nodes into a flat list once and walks that, exactly as the
// execution model (ADR-0002) prescribes.
//
// Ordering: add this system *before* `SkeletonAnimationSystem`. The machine
// chooses which clip the animator plays, and the animation system samples that
// clip in the same frame, so a state change is visible in the pose this frame
// rather than the next. This is the mirror of the sprite pair, where
// `AnimationStateMachineSystem` runs *after* `SpriteAnimationSystem` because it
// writes the animator's cell directly; here the machine writes the animator's
// *clip* and the animation system is the one that samples it.
//
// A machine with `drive_animator` false advances and transitions but leaves the
// animator alone, which is how a scene drives a machine purely for its triggers.
class SkeletalStateMachineSystem final : public System {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "SkeletalStateMachineSystem"; }

    void update(const FrameContext& context, Scene& scene) override;

    // Machines advanced during the most recent update. Exposed for tests and the
    // editor's debug view; not a gameplay-facing value.
    [[nodiscard]] usize last_advanced_count() const noexcept { return advanced_; }

    // Clips resolved through the frame's resource manager during the most recent
    // update. Non-zero only on the frames after a load, because a resolved state
    // is skipped on later frames.
    [[nodiscard]] usize last_resolved_count() const noexcept { return resolved_; }

private:
    std::vector<SkeletalStateMachine*> machines_;
    usize advanced_ = 0;
    usize resolved_ = 0;
};

} // namespace fe::anim
