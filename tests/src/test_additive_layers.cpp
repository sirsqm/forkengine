// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// Tests for stacked additive layers (docs/SHIPPING_QUEUE.md, the backlog's
// "wider shape" of the blend tree: additive-on-additive).
//
// Several additive layers compose in order, each over the pose the previous one
// produced, so a lean and a breath and a hit all stack. A layer with no clip or
// no weight is skipped, and a layer's mask scopes it to the joints it owns.

#include <doctest/doctest.h>

#include "fe/anim/AnimationBlendTree.h"
#include "fe/anim/AnimationBlendTreeSystem.h"
#include "fe/anim/AnimationClip.h"
#include "fe/anim/Animator.h"
#include "fe/anim/Skeleton.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/systems/FrameContext.h"

#include <cmath>
#include <memory>
#include <ostream>
#include <string>
#include <string_view>
#include <vector>

using namespace fe;
using namespace fe::anim;

namespace {

constexpr f32 kTolerance = 1e-4f;

[[nodiscard]] bool nearly_equal(f32 a, f32 b, f32 tolerance = kTolerance) {
    return std::fabs(a - b) <= tolerance;
}

[[nodiscard]] bool nearly_equal(const Vec3& a, const Vec3& b, f32 tolerance = kTolerance) {
    return nearly_equal(a.x, b.x, tolerance) && nearly_equal(a.y, b.y, tolerance) && nearly_equal(a.z, b.z, tolerance);
}

[[nodiscard]] std::shared_ptr<Skeleton> make_chain_skeleton() {
    std::vector<Skeleton::Joint> joints(3);
    joints[0].name = "root";
    joints[0].parent = Skeleton::kNoParent;
    joints[1].name = "middle";
    joints[1].parent = 0;
    joints[1].rest.position = Vec3{0.0f, 1.0f, 0.0f};
    joints[2].name = "tip";
    joints[2].parent = 1;
    joints[2].rest.position = Vec3{0.0f, 1.0f, 0.0f};

    auto skeleton = std::make_shared<Skeleton>("skeleton:chain", std::move(joints));
    skeleton->compute_inverse_bind_matrices();
    return skeleton;
}

// A clip holding the root at a constant translation, so a composed result
// depends only on the weights and the order.
[[nodiscard]] std::shared_ptr<AnimationClip> make_root_clip(const std::shared_ptr<Skeleton>& skeleton, std::string name,
                                                            const Vec3& translation) {
    std::vector<JointTrack> tracks(1);
    tracks[0].joint_name = "root";

    VectorTrack translation_track;
    translation_track.add_key(VectorKey{0.0f, translation, {}, {}});
    translation_track.add_key(VectorKey{1.0f, translation, {}, {}});
    tracks[0].translation = translation_track;

    auto clip = std::make_shared<AnimationClip>("clip:" + name, std::move(name), std::move(tracks));
    clip->bind(*skeleton);
    return clip;
}

[[nodiscard]] FrameContext frame(f32 delta) {
    FrameContext context;
    context.delta_time = delta;
    context.elapsed_time = static_cast<f64>(delta);
    return context;
}

// A tree with two clips (a base) and the given additive layer stack, wired onto
// a fresh scene node. Returns the node so the caller can read the animator.
Node* make_tree_node(Scene& scene, const std::shared_ptr<Skeleton>& skeleton,
                     const std::vector<AdditiveLayer>& layers) {
    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    animator->skeleton = skeleton;
    animator->playing = false;

    auto* tree = node->add_component<AnimationBlendTree>();
    tree->samples.resize(2);
    tree->samples[0].clip = make_root_clip(skeleton, "walk", Vec3{0.0f, 0.0f, 0.0f});
    tree->samples[0].threshold = 0.0f;
    tree->samples[0].other_threshold = 1.0f;
    tree->samples[1].clip = make_root_clip(skeleton, "run", Vec3{2.0f, 0.0f, 0.0f});
    tree->samples[1].threshold = 1.0f;
    tree->samples[1].other_threshold = 0.0f;
    tree->set_parameter(0.0f); // base root at 0
    tree->additive_layers = layers;
    return node;
}

} // namespace

TEST_CASE("two additive layers stack in order") {
    auto skeleton = make_chain_skeleton();
    Scene scene;

    AdditiveLayer lean;
    lean.clip = make_root_clip(skeleton, "lean", Vec3{0.0f, 1.0f, 0.0f});
    lean.weight = 1.0f;
    AdditiveLayer breath;
    breath.clip = make_root_clip(skeleton, "breath", Vec3{0.0f, 0.5f, 0.0f});
    breath.weight = 1.0f;

    Node* node = make_tree_node(scene, skeleton, {lean, breath});
    auto* animator = node->get_component<Animator>();

    AnimationBlendTreeSystem system;
    system.update(frame(0.1f), scene);

    // The base root is 0; the lean adds 1 and the breath adds 0.5 on top, so the
    // two compose by addition.
    CHECK(nearly_equal(animator->pose().at(0).translation, Vec3{0.0f, 1.5f, 0.0f}));
}

TEST_CASE("a layer with no clip or no weight is skipped") {
    auto skeleton = make_chain_skeleton();
    Scene scene;

    AdditiveLayer live;
    live.clip = make_root_clip(skeleton, "lean", Vec3{0.0f, 1.0f, 0.0f});
    live.weight = 1.0f;
    AdditiveLayer empty_clip; // no clip
    empty_clip.weight = 1.0f;
    AdditiveLayer zero_weight;
    zero_weight.clip = make_root_clip(skeleton, "breath", Vec3{0.0f, 5.0f, 0.0f});
    zero_weight.weight = 0.0f;

    Node* node = make_tree_node(scene, skeleton, {live, empty_clip, zero_weight});
    auto* animator = node->get_component<Animator>();

    AnimationBlendTreeSystem system;
    system.update(frame(0.1f), scene);

    // Only the live layer contributes.
    CHECK(nearly_equal(animator->pose().at(0).translation, Vec3{0.0f, 1.0f, 0.0f}));
}

TEST_CASE("a layer's mask scopes it to the joints it owns") {
    auto skeleton = make_chain_skeleton();
    Scene scene;

    // The clip moves both the root and the tip; the mask owns only the tip.
    std::vector<JointTrack> tracks(2);
    tracks[0].joint_name = "root";
    VectorTrack root_track;
    root_track.add_key(VectorKey{0.0f, Vec3{0.0f, 3.0f, 0.0f}, {}, {}});
    root_track.add_key(VectorKey{1.0f, Vec3{0.0f, 3.0f, 0.0f}, {}, {}});
    tracks[0].translation = root_track;
    tracks[1].joint_name = "tip";
    VectorTrack tip_track;
    tip_track.add_key(VectorKey{0.0f, Vec3{0.0f, 4.0f, 0.0f}, {}, {}});
    tip_track.add_key(VectorKey{1.0f, Vec3{0.0f, 4.0f, 0.0f}, {}, {}});
    tracks[1].translation = tip_track;
    auto both = std::make_shared<AnimationClip>("clip:both", "both", std::move(tracks));
    both->bind(*skeleton);

    AdditiveLayer masked;
    masked.clip = both;
    masked.weight = 1.0f;
    masked.mask.add("tip");

    Node* node = make_tree_node(scene, skeleton, {masked});
    auto* animator = node->get_component<Animator>();

    AnimationBlendTreeSystem system;
    system.update(frame(0.1f), scene);

    // The root is outside the mask, so it keeps the base value.
    CHECK(nearly_equal(animator->pose().at(0).translation, Vec3{0.0f, 0.0f, 0.0f}));
    // The tip is in the mask, so the offset applies to its rest translation.
    CHECK(nearly_equal(animator->pose().at(2).translation, Vec3{0.0f, 1.0f + 4.0f, 0.0f}));
}

TEST_CASE("the single additive layer and the stack both apply") {
    auto skeleton = make_chain_skeleton();
    Scene scene;

    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    animator->skeleton = skeleton;
    animator->playing = false;

    auto* tree = node->add_component<AnimationBlendTree>();
    tree->samples.resize(2);
    tree->samples[0].clip = make_root_clip(skeleton, "walk", Vec3{0.0f, 0.0f, 0.0f});
    tree->samples[0].threshold = 0.0f;
    tree->samples[0].other_threshold = 1.0f;
    tree->samples[1].clip = make_root_clip(skeleton, "run", Vec3{0.0f, 0.0f, 0.0f});
    tree->samples[1].threshold = 1.0f;
    tree->samples[1].other_threshold = 0.0f;
    tree->set_parameter(0.0f);

    // The single additive layer adds 1; the stack adds a further 0.25.
    tree->additive_clip = make_root_clip(skeleton, "hit", Vec3{0.0f, 1.0f, 0.0f});
    tree->additive_weight = 1.0f;
    AdditiveLayer extra;
    extra.clip = make_root_clip(skeleton, "lean", Vec3{0.0f, 1.0f, 0.0f});
    extra.weight = 0.25f;
    tree->additive_layers = {extra};

    AnimationBlendTreeSystem system;
    system.update(frame(0.1f), scene);

    CHECK(nearly_equal(animator->pose().at(0).translation, Vec3{0.0f, 1.25f, 0.0f}));
}

TEST_CASE("a layer stack round-trips through the tree's JSON") {
    auto skeleton = make_chain_skeleton();
    AnimationBlendTree tree;
    tree.samples.resize(2);
    tree.samples[0].threshold = 0.0f;
    tree.samples[0].other_threshold = 1.0f;
    tree.samples[1].threshold = 1.0f;
    tree.samples[1].other_threshold = 0.0f;

    AdditiveLayer layer;
    layer.clip_key = "clip:lean";
    layer.weight = 0.4f;
    layer.time = 0.2f;
    layer.speed = 1.5f;
    layer.mask.add("spine");
    tree.additive_layers = {layer};

    const std::string text = tree.to_json();
    AnimationBlendTree loaded;
    REQUIRE(loaded.from_json(text));
    REQUIRE(loaded.additive_layers.size() == 1);
    CHECK(loaded.additive_layers[0].clip_key == "clip:lean");
    CHECK(nearly_equal(loaded.additive_layers[0].weight, 0.4f));
    CHECK(nearly_equal(loaded.additive_layers[0].time, 0.2f));
    CHECK(nearly_equal(loaded.additive_layers[0].speed, 1.5f));
    REQUIRE(loaded.additive_layers[0].mask.size() == 1);
    CHECK(loaded.additive_layers[0].mask.names()[0] == "spine");
}
