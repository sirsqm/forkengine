// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/anim/AnimationBlendTreeSystem.h"

#include "fe/anim/Animator.h"
#include "fe/anim/Skeleton.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"

#include <algorithm>

namespace fe::anim {
namespace {

// Samples one blend sample at the animator's clock scaled by the sample's speed.
// The animator owns the rate; a per-sample speed only matches clips of different
// lengths, so a short run cycle can play alongside a long walk.
void sample_sample(const Skeleton& skeleton, const BlendSample& sample, f32 base_time, LoopMode mode,
                   SkeletonPose& out) {
    sample_clip_pose(skeleton, sample.clip.get(), base_time * sample.speed, mode, out);
}

} // namespace

void AnimationBlendTreeSystem::update(const FrameContext& context, Scene& scene) {
    advanced_ = 0;
    skipped_ = 0;
    resolved_ = 0;
    trees_.clear();

    scene.for_each_node([&](Node& node) {
        if (AnimationBlendTree* tree = node.get_component<AnimationBlendTree>(); tree != nullptr) {
            trees_.push_back(tree);
        }
    });

    if (context.resources != nullptr) {
        for (AnimationBlendTree* tree : trees_) {
            resolved_ += tree->resolve_clips(*context.resources);
        }
    }

    for (AnimationBlendTree* tree : trees_) {
        Node* owner = tree->node();
        Animator* animator = owner != nullptr ? owner->get_component<Animator>() : nullptr;
        if (animator == nullptr) {
            ++skipped_;
            continue;
        }
        const Skeleton* skeleton = animator->skeleton_resource();
        if (skeleton == nullptr || skeleton->empty()) {
            ++skipped_;
            continue;
        }

        const LoopMode mode = animator->loop;
        const f32 base_time = animator->time;
        bool wrote_pose = false;

        // --- 1. The base blend ------------------------------------------------
        // A tree with two clips blends them; a tree with one is that clip; an
        // empty tree leaves the animator's own pose (the animation system's rest
        // sample, or its own clip) untouched.
        if (tree->drive_animator) {
            SkeletonPose& base = animator->mutable_pose();
            if (tree->samples.size() >= 2) {
                const f32 weight = tree->blend_weight();
                sample_sample(*skeleton, tree->samples[0], base_time, mode, pose_a_);
                sample_sample(*skeleton, tree->samples[1], base_time, mode, pose_b_);
                blend_poses(pose_a_, pose_b_, weight, base);
                wrote_pose = true;
            } else if (tree->samples.size() == 1) {
                sample_sample(*skeleton, tree->samples[0], base_time, mode, base);
                wrote_pose = true;
            }
        }

        // --- 2. The additive layer -------------------------------------------
        if (tree->additive_clip != nullptr && tree->additive_weight > 0.0f) {
            if (animator->playing) {
                tree->additive_time += context.delta_time * tree->additive_speed;
            }
            sample_additive_pose(*skeleton, *tree->additive_clip, tree->additive_time, mode, additive_scratch_);
            compose_additive(animator->mutable_pose(), additive_scratch_, tree->additive_weight);
            wrote_pose = true;
        }

        // --- 3. Palette -------------------------------------------------------
        // Recompute from the blended pose, so the renderer and a CPU skin read
        // the blend rather than the base clip the animation system sampled. Only
        // when this system actually wrote the pose: a tree that drove nothing
        // leaves the animation system's palette in place.
        if (wrote_pose) {
            animator->mutable_pose().skinning_matrices(*skeleton, animator->mutable_palette());
            animator->mark_palette_valid();
        }
        ++advanced_;
    }
}

} // namespace fe::anim
