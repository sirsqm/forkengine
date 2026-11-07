// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/anim/AnimationBlendTreeSystem.h"

#include "fe/anim/Animator.h"
#include "fe/anim/Skeleton.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"

#include <algorithm>

namespace fe::anim {
void AnimationBlendTreeSystem::update(const FrameContext& context, Scene& scene) {
    advanced_ = 0;
    skipped_ = 0;
    resolved_ = 0;
    trees_.clear();
    events_.clear();

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
        // The clock the previous frame sampled at, so the markers the samples
        // crossed between the two can be reported. Captured before it is updated.
        const f32 previous_base_time = tree->previous_base_time;
        tree->previous_base_time = base_time;
        bool wrote_pose = false;

        // --- 1. The base blend ------------------------------------------------
        // A tree with two entries blends them; a tree with one is that entry; an
        // empty tree leaves the animator's own pose (the animation system's rest
        // sample, or its own clip) untouched. An entry that is itself a tree is
        // evaluated recursively, so a tree can feed a tree.
        //
        // Timeline events are collected on the same walk: a blend has no single
        // clip, so a marker fires when the clock crossed it in any entry the tree
        // played, at that entry's own rate, and a marker two entries share (a
        // matched pair of locomotion clips) is reported once.
        if (tree->drive_animator && !tree->samples.empty()) {
            evaluate_blend_tree(*tree, *skeleton, base_time, previous_base_time, mode, tree_scratch_, 0,
                                animator->mutable_pose(), &events_);
            wrote_pose = true;
        }

        // --- 2. The additive layer -------------------------------------------
        if (tree->additive_clip != nullptr && tree->additive_weight > 0.0f) {
            if (animator->playing) {
                tree->additive_time += context.delta_time * tree->additive_speed;
            }
            sample_additive_pose(*skeleton, *tree->additive_clip, tree->additive_time, mode, additive_scratch_);
            // A masked layer resolves against the skeleton first, so the per-joint
            // test in the compose is an index lookup rather than a name search.
            if (!tree->additive_mask.empty()) {
                tree->additive_mask.resolve(*skeleton);
            }
            compose_additive_masked(animator->mutable_pose(), additive_scratch_, tree->additive_weight,
                                    tree->additive_mask);
            wrote_pose = true;
        }

        // --- 2b. The additive stack ------------------------------------------
        // Further layers compose in order, each over the pose the previous one
        // produced, so a lean and a breath and a hit all stack. A layer with no
        // clip or no weight is skipped.
        for (AdditiveLayer& layer : tree->additive_layers) {
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
