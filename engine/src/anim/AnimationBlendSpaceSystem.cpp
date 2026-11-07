// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/anim/AnimationBlendSpaceSystem.h"

#include "fe/anim/AnimationBlendTree.h" // sample_clip_pose, sample_additive_pose, compose_additive
#include "fe/anim/Animator.h"
#include "fe/anim/Skeleton.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"

#include <algorithm>

namespace fe::anim {
namespace {

// Samples one blend point at the animator's clock scaled by the point's speed.
void sample_point(const Skeleton& skeleton, const BlendPoint& point, f32 base_time, LoopMode mode, SkeletonPose& out) {
    sample_clip_pose(skeleton, point.clip.get(), base_time * point.speed, mode, out);
}

} // namespace

void AnimationBlendSpaceSystem::update(const FrameContext& context, Scene& scene) {
    advanced_ = 0;
    skipped_ = 0;
    resolved_ = 0;
    spaces_.clear();

    scene.for_each_node([&](Node& node) {
        if (AnimationBlendSpace2D* space = node.get_component<AnimationBlendSpace2D>(); space != nullptr) {
            spaces_.push_back(space);
        }
    });

    if (context.resources != nullptr) {
        for (AnimationBlendSpace2D* space : spaces_) {
            resolved_ += space->resolve_clips(*context.resources);
        }
    }

    BlendWeight selected[3];
    for (AnimationBlendSpace2D* space : spaces_) {
        Node* owner = space->node();
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
        // The samples and weights that surround the parameter, at most three. A
        // weighted blend of three poses is done as two nested two-pose blends:
        // blend the first pair by the second's share of their combined weight,
        // then blend that toward the third by the third's weight. The result is
        // the exact barycentric mix on the linear channels and a repeated slerp
        // on rotations, which is the standard approximation.
        if (space->drive_animator) {
            const usize count = space->weights(selected, 3);
            if (count >= 1) {
                SkeletonPose& base = animator->mutable_pose();
                sample_point(*skeleton, space->samples[selected[0].sample], base_time, mode, pose_a_);
                if (count == 1) {
                    base = pose_a_;
                } else if (count == 2) {
                    sample_point(*skeleton, space->samples[selected[1].sample], base_time, mode, pose_b_);
                    blend_poses(pose_a_, pose_b_, selected[1].weight, base);
                } else {
                    sample_point(*skeleton, space->samples[selected[1].sample], base_time, mode, pose_b_);
                    sample_point(*skeleton, space->samples[selected[2].sample], base_time, mode, pose_c_);
                    const f32 pair = selected[0].weight + selected[1].weight;
                    const f32 first_share = pair > 1e-6f ? selected[1].weight / pair : 0.0f;
                    blend_poses(pose_a_, pose_b_, first_share, pose_a_);
                    blend_poses(pose_a_, pose_c_, selected[2].weight, base);
                }
                wrote_pose = true;
            }
        }

        // --- 2. The additive layer -------------------------------------------
        if (space->additive_clip != nullptr && space->additive_weight > 0.0f) {
            if (animator->playing) {
                space->additive_time += context.delta_time * space->additive_speed;
            }
            sample_additive_pose(*skeleton, *space->additive_clip, space->additive_time, mode, additive_scratch_);
            if (!space->additive_mask.empty()) {
                space->additive_mask.resolve(*skeleton);
            }
            compose_additive_masked(animator->mutable_pose(), additive_scratch_, space->additive_weight,
                                    space->additive_mask);
            wrote_pose = true;
        }

        // --- 2b. The additive stack ------------------------------------------
        for (AdditiveLayer& layer : space->additive_layers) {
            if (layer.clip == nullptr || layer.weight <= 0.0f) {
                continue;
            }
            if (animator->playing) {
                layer.time += context.delta_time * layer.speed;
            }
            sample_additive_pose(*skeleton, *layer.clip, layer.time, mode, additive_scratch_);
            if (!layer.mask.empty()) {
                layer.mask.resolve(*skeleton);
            }
            compose_additive_masked(animator->mutable_pose(), additive_scratch_, layer.weight, layer.mask);
            wrote_pose = true;
        }

        // --- 3. Palette -------------------------------------------------------
        if (wrote_pose) {
            animator->mutable_pose().skinning_matrices(*skeleton, animator->mutable_palette());
            animator->mark_palette_valid();
        }
        ++advanced_;
    }
}

} // namespace fe::anim
