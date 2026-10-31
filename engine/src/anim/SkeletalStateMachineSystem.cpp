// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/anim/SkeletalStateMachineSystem.h"

#include "fe/anim/Animator.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"

namespace fe::anim {

void SkeletalStateMachineSystem::update(const FrameContext& context, Scene& scene) {
    advanced_ = 0;
    resolved_ = 0;
    machines_.clear();

    // One traversal to gather the machines, exactly as the other animation
    // systems do. The scene's node count is what a frame pays for this, not the
    // machine count.
    scene.for_each_node([&](Node& node) {
        if (SkeletalStateMachine* machine = node.get_component<SkeletalStateMachine>(); machine != nullptr) {
            machines_.push_back(machine);
        }
    });

    // Resolve any clip the frame's manager can load before advancing, so a state
    // that references a clip by key has it on the frame it first becomes active.
    if (context.resources != nullptr) {
        for (SkeletalStateMachine* machine : machines_) {
            resolved_ += machine->resolve_clips(*context.resources);
        }
    }

    for (SkeletalStateMachine* machine : machines_) {
        machine->advance(context.delta_time);
        ++advanced_;

        if (!machine->drive_animator) {
            // The request is still consumed so a later flip to driving does not
            // fire a stale bind; the machine is authoritative about its state,
            // the animator is not.
            (void)machine->take_clip_request();
            continue;
        }
        if (!machine->take_clip_request()) {
            continue;
        }

        auto* animator = machine->node() != nullptr ? machine->node()->get_component<Animator>() : nullptr;
        if (animator == nullptr) {
            continue;
        }
        // The animator's own clock is restarted by play(), so a state entry is a
        // clean cut rather than a continuation of whatever was playing. A state
        // with no clip leaves the animator's clip as it is, which is the "hold
        // the pose" state; a game that wants to clear it can author an empty
        // state and drive it deliberately.
        if (machine->active_clip() != nullptr) {
            animator->play(machine->active_clip(), machine->active_loop());
        }
    }
}

} // namespace fe::anim
