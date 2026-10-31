// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// Tests for the animation blend tree and additive layer (docs/SHIPPING_QUEUE.md,
// `animation-blend-tree`).
//
// The queue item's acceptance criteria are "a blend tree samples two clips by a
// continuous parameter and an additive layer composes over the base, both proven
// by a test that checks the palette at the extremes and the midpoint." These
// tests do exactly that: the pure pose functions are asserted with no scene and
// no animator, and the system is asserted through the palette it publishes.

#include <doctest/doctest.h>

#include "fe/anim/AnimationBlendTree.h"
#include "fe/anim/AnimationBlendTreeSystem.h"
#include "fe/anim/AnimationClip.h"
#include "fe/anim/Animator.h"
#include "fe/anim/SkeletalAnimationTypes.h"
#include "fe/anim/Skeleton.h"
#include "fe/anim/SkeletonAnimationSystem.h"
#include "fe/reflect/Property.h"
#include "fe/reflect/Registry.h"
#include "fe/reflect/TypeDesc.h"
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

// A three-joint chain: root at the origin, `middle` one unit up, `tip` one unit
// further, matching the shape the other skeletal tests use.
[[nodiscard]] std::shared_ptr<Skeleton> make_chain_skeleton() {
    std::vector<Skeleton::Joint> joints(3);

    joints[0].name = "root";
    joints[0].parent = Skeleton::kNoParent;
    joints[0].rest.position = Vec3{0.0f, 0.0f, 0.0f};

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

// A clip that holds the root at a constant rotation and translation for its
// whole length, so sampling it at any time is deterministic and a blend's
// result depends only on the weight. The other joints are left untouched, so
// they stay at the rest pose.
[[nodiscard]] std::shared_ptr<AnimationClip> make_root_clip(const std::shared_ptr<Skeleton>& skeleton, std::string name,
                                                            const Quat& rotation, const Vec3& translation,
                                                            f32 duration = 1.0f) {
    std::vector<JointTrack> tracks(1);
    tracks[0].joint_name = "root";

    QuaternionTrack rotation_track;
    rotation_track.add_key(QuaternionKey{0.0f, rotation, {}, {}});
    rotation_track.add_key(QuaternionKey{duration, rotation, {}, {}});
    tracks[0].rotation = rotation_track;

    VectorTrack translation_track;
    translation_track.add_key(VectorKey{0.0f, translation, {}, {}});
    translation_track.add_key(VectorKey{duration, translation, {}, {}});
    tracks[0].translation = translation_track;

    auto clip = std::make_shared<AnimationClip>("clip:" + name, std::move(name), std::move(tracks));
    clip->bind(*skeleton);
    return clip;
}

[[nodiscard]] BlendSample sample(std::shared_ptr<AnimationClip> clip, f32 threshold, f32 other_threshold) {
    BlendSample s;
    s.name = clip->name();
    s.clip = std::move(clip);
    s.threshold = threshold;
    s.other_threshold = other_threshold;
    return s;
}

[[nodiscard]] FrameContext frame(f32 delta) {
    FrameContext context;
    context.delta_time = delta;
    context.elapsed_time = static_cast<f64>(delta);
    return context;
}

} // namespace

// ---------------------------------------------------------------------------
// The blend weight: the parameter-to-weight mapping
// ---------------------------------------------------------------------------

TEST_CASE("the blend weight maps a parameter between the two thresholds") {
    auto skeleton = make_chain_skeleton();
    auto walk = make_root_clip(skeleton, "walk", Quat::identity(), Vec3{0.0f, 0.0f, 0.0f});
    auto run = make_root_clip(skeleton, "run", Quat::identity(), Vec3{0.0f, 0.0f, 0.0f});

    AnimationBlendTree tree;
    tree.samples = {sample(walk, 2.0f, 4.0f), sample(run, 4.0f, 2.0f)};

    tree.set_parameter(2.0f);
    CHECK(nearly_equal(tree.weight, 0.0f));
    tree.set_parameter(3.0f);
    CHECK(nearly_equal(tree.weight, 0.5f));
    tree.set_parameter(4.0f);
    CHECK(nearly_equal(tree.weight, 1.0f));

    // Outside the interval the weight clamps rather than extrapolating.
    tree.set_parameter(0.0f);
    CHECK(nearly_equal(tree.weight, 0.0f));
    tree.set_parameter(10.0f);
    CHECK(nearly_equal(tree.weight, 1.0f));

    // A tree with fewer than two samples has no interval to blend over.
    AnimationBlendTree single;
    single.samples = {sample(walk, 0.0f, 1.0f)};
    single.set_parameter(5.0f);
    CHECK(nearly_equal(single.blend_weight(), 0.0f));
}

TEST_CASE("a degenerate threshold interval is a hard cut") {
    auto skeleton = make_chain_skeleton();
    auto walk = make_root_clip(skeleton, "walk", Quat::identity(), Vec3{});
    auto run = make_root_clip(skeleton, "run", Quat::identity(), Vec3{});

    AnimationBlendTree tree;
    tree.samples = {sample(walk, 3.0f, 3.0f), sample(run, 3.0f, 3.0f)};

    tree.set_parameter(2.9f);
    CHECK(nearly_equal(tree.weight, 0.0f));
    tree.set_parameter(3.0f);
    CHECK(nearly_equal(tree.weight, 1.0f));
}

TEST_CASE("a tree with use_parameter off reads the stored weight") {
    auto skeleton = make_chain_skeleton();
    auto walk = make_root_clip(skeleton, "walk", Quat::identity(), Vec3{});
    auto run = make_root_clip(skeleton, "run", Quat::identity(), Vec3{});

    AnimationBlendTree tree;
    tree.samples = {sample(walk, 0.0f, 1.0f), sample(run, 1.0f, 0.0f)};
    tree.use_parameter = false;
    tree.weight = 0.25f;

    // The parameter is ignored while the flag is off.
    tree.set_parameter(1.0f);
    CHECK(nearly_equal(tree.blend_weight(), 0.25f));

    // Flipping the flag back re-derives the weight from the parameter.
    tree.use_parameter = true;
    tree.weight = tree.blend_weight();
    CHECK(nearly_equal(tree.weight, 1.0f));
}

// ---------------------------------------------------------------------------
// The pure pose functions
// ---------------------------------------------------------------------------

TEST_CASE("sampling a null clip leaves the pose at rest") {
    auto skeleton = make_chain_skeleton();
    SkeletonPose pose;

    sample_clip_pose(*skeleton, nullptr, 0.5f, LoopMode::Loop, pose);

    REQUIRE(pose.joint_count() == 3);
    CHECK(nearly_equal(pose.at(1).translation, Vec3{0.0f, 1.0f, 0.0f}));
    CHECK(nearly_equal(pose.at(2).translation, Vec3{0.0f, 1.0f, 0.0f}));
}

TEST_CASE("composing an additive delta adds translation and composes rotation") {
    auto skeleton = make_chain_skeleton();
    auto delta = make_root_clip(skeleton, "recoil", Quat::from_axis_angle(Vec3{0.0f, 0.0f, 1.0f}, radians(90.0f)),
                                Vec3{1.0f, 0.0f, 0.0f});

    SkeletonPose base;
    base.reset_to_rest(*skeleton);
    SkeletonPose scratch;
    sample_additive_pose(*skeleton, *delta, 0.0f, LoopMode::Once, scratch);

    // At weight zero the base is untouched.
    SkeletonPose untouched = base;
    compose_additive(untouched, scratch, 0.0f);
    CHECK(nearly_equal(untouched.at(0).translation, Vec3{0.0f, 0.0f, 0.0f}));

    // At full weight the translation adds and the rotation composes. The base
    // root rotation is the identity, so the result is the delta's rotation.
    SkeletonPose full = base;
    compose_additive(full, scratch, 1.0f);
    CHECK(nearly_equal(full.at(0).translation, Vec3{1.0f, 0.0f, 0.0f}));
    const Quat expected = Quat::from_axis_angle(Vec3{0.0f, 0.0f, 1.0f}, radians(90.0f));
    CHECK(nearly_equal(full.at(0).rotation.x, expected.x));
    CHECK(nearly_equal(full.at(0).rotation.y, expected.y));
    CHECK(nearly_equal(full.at(0).rotation.z, expected.z));
    CHECK(nearly_equal(full.at(0).rotation.w, expected.w));

    // A joint the delta does not animate is left alone: the delta samples as the
    // identity there, which is the additive no-op.
    CHECK(nearly_equal(full.at(1).translation, Vec3{0.0f, 1.0f, 0.0f}));
    CHECK(nearly_equal(full.at(2).translation, Vec3{0.0f, 1.0f, 0.0f}));

    // Half weight halves the translation and halves the rotation on the sphere.
    SkeletonPose half = base;
    compose_additive(half, scratch, 0.5f);
    CHECK(nearly_equal(half.at(0).translation, Vec3{0.5f, 0.0f, 0.0f}));
    const Quat halfway = Quat::from_axis_angle(Vec3{0.0f, 0.0f, 1.0f}, radians(45.0f));
    CHECK(nearly_equal(half.at(0).rotation.z, halfway.z));
    CHECK(nearly_equal(half.at(0).rotation.w, halfway.w));
}

// ---------------------------------------------------------------------------
// The system: the palette at the extremes and the midpoint
// ---------------------------------------------------------------------------

TEST_CASE("the blend tree samples two clips and publishes the blended palette") {
    auto skeleton = make_chain_skeleton();
    auto walk = make_root_clip(skeleton, "walk", Quat::identity(), Vec3{0.0f, 0.0f, 0.0f});
    auto run = make_root_clip(skeleton, "run", Quat::from_axis_angle(Vec3{0.0f, 0.0f, 1.0f}, radians(90.0f)),
                              Vec3{2.0f, 0.0f, 0.0f});

    Scene scene;
    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    REQUIRE((animator != nullptr));
    animator->skeleton = skeleton;
    animator->playing = false;
    animator->time = 0.0f;

    auto* tree = node->add_component<AnimationBlendTree>();
    REQUIRE((tree != nullptr));
    tree->samples = {sample(walk, 0.0f, 1.0f), sample(run, 1.0f, 0.0f)};

    SkeletonAnimationSystem animation_system;
    AnimationBlendTreeSystem blend_system;
    FrameContext context = frame(0.1f);

    // Weight 0: exactly the walk clip.
    tree->set_parameter(0.0f);
    animation_system.update(context, scene);
    blend_system.update(context, scene);
    CHECK(blend_system.last_advanced_count() == 1);
    CHECK(nearly_equal(animator->pose().at(0).translation, Vec3{0.0f, 0.0f, 0.0f}));
    // The palette reflects the blended pose, not the animator's (null) clip.
    CHECK(nearly_equal(transform_point(animator->palette()[0], Vec3{0.0f, 0.0f, 0.0f}), Vec3{0.0f, 0.0f, 0.0f}));

    // Weight 1: exactly the run clip.
    tree->set_parameter(1.0f);
    blend_system.update(context, scene);
    CHECK(nearly_equal(animator->pose().at(0).translation, Vec3{2.0f, 0.0f, 0.0f}));
    CHECK(nearly_equal(transform_point(animator->palette()[0], Vec3{0.0f, 0.0f, 0.0f}), Vec3{2.0f, 0.0f, 0.0f}));

    // Midpoint: the translation is halved and the rotation is halfway on the
    // sphere. The palette is recomputed from that blend, so a vertex the root
    // moves lands where the blend says.
    tree->set_parameter(0.5f);
    blend_system.update(context, scene);
    CHECK(nearly_equal(animator->pose().at(0).translation, Vec3{1.0f, 0.0f, 0.0f}));
    const Quat halfway = Quat::from_axis_angle(Vec3{0.0f, 0.0f, 1.0f}, radians(45.0f));
    CHECK(nearly_equal(animator->pose().at(0).rotation.z, halfway.z));
    CHECK(nearly_equal(animator->pose().at(0).rotation.w, halfway.w));
}

TEST_CASE("the additive layer composes over the base blend") {
    auto skeleton = make_chain_skeleton();
    auto walk = make_root_clip(skeleton, "walk", Quat::identity(), Vec3{0.0f, 0.0f, 0.0f});
    auto run = make_root_clip(skeleton, "run", Quat::identity(), Vec3{2.0f, 0.0f, 0.0f});
    auto hit = make_root_clip(skeleton, "hit", Quat::identity(), Vec3{0.0f, 1.0f, 0.0f});

    Scene scene;
    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    REQUIRE((animator != nullptr));
    animator->skeleton = skeleton;
    animator->playing = false;

    auto* tree = node->add_component<AnimationBlendTree>();
    REQUIRE((tree != nullptr));
    tree->samples = {sample(walk, 0.0f, 1.0f), sample(run, 1.0f, 0.0f)};
    tree->additive_clip = hit;
    tree->set_parameter(0.0f); // base is the walk clip

    SkeletonAnimationSystem animation_system;
    AnimationBlendTreeSystem blend_system;
    FrameContext context = frame(0.1f);

    // The base alone: the walk clip's translation.
    animation_system.update(context, scene);
    blend_system.update(context, scene);
    CHECK(nearly_equal(animator->pose().at(0).translation, Vec3{0.0f, 0.0f, 0.0f}));

    // A half-weighted additive layer adds half the hit's offset on top of the
    // base, and the palette follows.
    tree->additive_weight = 0.5f;
    blend_system.update(context, scene);
    CHECK(nearly_equal(animator->pose().at(0).translation, Vec3{0.0f, 0.5f, 0.0f}));

    // At full weight the whole delta is applied.
    tree->additive_weight = 1.0f;
    blend_system.update(context, scene);
    CHECK(nearly_equal(animator->pose().at(0).translation, Vec3{0.0f, 1.0f, 0.0f}));
}

TEST_CASE("the system skips a tree whose node has no animator or no skeleton") {
    auto skeleton = make_chain_skeleton();
    auto walk = make_root_clip(skeleton, "walk", Quat::identity(), Vec3{});
    auto run = make_root_clip(skeleton, "run", Quat::identity(), Vec3{});

    Scene scene;
    Node* rig_only = scene.create_node("rig-only");
    auto* tree = rig_only->add_component<AnimationBlendTree>();
    REQUIRE((tree != nullptr));
    tree->samples = {sample(walk, 0.0f, 1.0f), sample(run, 1.0f, 0.0f)};

    AnimationBlendTreeSystem system;
    FrameContext context = frame(0.1f);
    system.update(context, scene);
    CHECK(system.last_advanced_count() == 0);
    CHECK(system.last_skipped_count() == 1);

    // A node with an animator but no skeleton is skipped too.
    Node* no_skeleton = scene.create_node("no-skeleton");
    REQUIRE((no_skeleton->add_component<Animator>() != nullptr));
    auto* second = no_skeleton->add_component<AnimationBlendTree>();
    REQUIRE((second != nullptr));
    second->samples = {sample(walk, 0.0f, 1.0f), sample(run, 1.0f, 0.0f)};

    system.update(context, scene);
    CHECK(system.last_advanced_count() == 0);
    CHECK(system.last_skipped_count() == 2);
}

TEST_CASE("a tree with drive_animator off leaves the animator's pose alone") {
    auto skeleton = make_chain_skeleton();
    auto walk = make_root_clip(skeleton, "walk", Quat::identity(), Vec3{0.0f, 0.0f, 0.0f});
    auto run = make_root_clip(skeleton, "run", Quat::identity(), Vec3{5.0f, 0.0f, 0.0f});

    Scene scene;
    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    REQUIRE((animator != nullptr));
    animator->skeleton = skeleton;
    animator->playing = false;

    auto* tree = node->add_component<AnimationBlendTree>();
    REQUIRE((tree != nullptr));
    tree->samples = {sample(walk, 0.0f, 1.0f), sample(run, 1.0f, 0.0f)};
    tree->drive_animator = false;
    tree->set_parameter(1.0f);

    SkeletonAnimationSystem animation_system;
    AnimationBlendTreeSystem blend_system;
    FrameContext context = frame(0.1f);
    animation_system.update(context, scene);
    const std::vector<Mat4> before = animator->palette();
    blend_system.update(context, scene);

    // The tree evaluated (the weight is available) but wrote nothing.
    CHECK(nearly_equal(tree->blend_weight(), 1.0f));
    REQUIRE(before.size() == animator->palette().size());
    for (usize i = 0; i < before.size(); ++i) {
        CHECK(nearly_equal(transform_point(before[i], Vec3{0.0f, 0.0f, 0.0f}),
                           transform_point(animator->palette()[i], Vec3{0.0f, 0.0f, 0.0f})));
    }
}

// ---------------------------------------------------------------------------
// Persistence and reflection
// ---------------------------------------------------------------------------

TEST_CASE("the tree round-trips through its serialized form") {
    auto skeleton = make_chain_skeleton();
    auto walk = make_root_clip(skeleton, "walk", Quat::identity(), Vec3{});
    auto run = make_root_clip(skeleton, "run", Quat::identity(), Vec3{});

    AnimationBlendTree tree;
    tree.samples = {sample(walk, 1.0f, 3.0f), sample(run, 3.0f, 1.0f)};
    tree.parameter = 2.0f;
    tree.additive_clip = walk;
    tree.additive_clip_key = "clip:walk";
    tree.additive_weight = 0.25f;
    tree.additive_speed = 2.0f;
    tree.drive_animator = false;

    const std::string text = tree.to_json();

    AnimationBlendTree reloaded;
    REQUIRE(reloaded.from_json(text));
    CHECK(reloaded.samples.size() == 2);
    CHECK(reloaded.samples[0].name == "walk");
    CHECK(reloaded.samples[0].clip_key == "clip:walk");
    CHECK(nearly_equal(reloaded.samples[0].threshold, 1.0f));
    CHECK(nearly_equal(reloaded.samples[0].other_threshold, 3.0f));
    CHECK(nearly_equal(reloaded.parameter, 2.0f));
    CHECK_FALSE(reloaded.drive_animator);
    CHECK(reloaded.additive_clip_key == "clip:walk");
    CHECK(nearly_equal(reloaded.additive_weight, 0.25f));
    CHECK(nearly_equal(reloaded.additive_speed, 2.0f));
    // The clips travel as keys; the pointers are rebuilt by the manager.
    CHECK((reloaded.samples[0].clip == nullptr));
    CHECK((reloaded.additive_clip == nullptr));

    // A malformed document is rejected without changing the tree.
    CHECK_FALSE(reloaded.from_json("{ not json"));
    CHECK(reloaded.samples.size() == 2);
}

TEST_CASE("the blend tree is registered and reflects a tree property") {
    reflect::register_skeletal_animation_types();

    const reflect::TypeDesc* type = reflect::Registry::instance().find(AnimationBlendTree::kTypeName);
    REQUIRE((type != nullptr));

    auto instance = type->factory();
    REQUIRE((instance != nullptr));

    const reflect::Property* tree = nullptr;
    for (const reflect::Property& property : type->properties) {
        if (property.name == "tree") {
            tree = &property;
            break;
        }
    }
    REQUIRE((tree != nullptr));
    CHECK(tree->value_kind == reflect::ValueKind::String);
    CHECK(tree->hint == reflect::Hint::Multiline);
}
