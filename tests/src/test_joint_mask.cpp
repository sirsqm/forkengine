// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// Tests for the per-layer joint mask (docs/SHIPPING_QUEUE.md, the backlog's
// "wider shape" of the blend tree: a per-layer joint mask).
//
// A masked additive layer writes only the joints it names; every other joint
// keeps whatever the base pose held. The mask's own resolution and the masked
// compose are asserted directly, and the system is asserted through the pose it
// publishes.

#include <doctest/doctest.h>

#include "fe/anim/AnimationBlendTree.h"
#include "fe/anim/AnimationBlendTreeSystem.h"
#include "fe/anim/AnimationClip.h"
#include "fe/anim/Animator.h"
#include "fe/anim/JointMask.h"
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

// A clip that moves the named joint by a constant offset for its whole length.
[[nodiscard]] std::shared_ptr<AnimationClip> make_joint_clip(const std::shared_ptr<Skeleton>& skeleton,
                                                             std::string name, std::string joint, const Vec3& offset) {
    std::vector<JointTrack> tracks(1);
    tracks[0].joint_name = std::move(joint);

    VectorTrack translation_track;
    translation_track.add_key(VectorKey{0.0f, offset, {}, {}});
    translation_track.add_key(VectorKey{1.0f, offset, {}, {}});
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

} // namespace

// ---------------------------------------------------------------------------
// The mask: name resolution and membership
// ---------------------------------------------------------------------------

TEST_CASE("an empty mask restricts nothing") {
    auto skeleton = make_chain_skeleton();
    JointMask mask;
    CHECK(mask.empty());
    // Even unresolved, an empty mask contains every joint.
    CHECK(mask.contains(0));
    CHECK(mask.contains(2));
    CHECK(mask.resolve(*skeleton) == 0);
    CHECK(mask.contains(0));
}

TEST_CASE("a mask resolves names to joint indices") {
    auto skeleton = make_chain_skeleton();
    JointMask mask;
    mask.add("tip");
    mask.add("middle");
    CHECK(mask.size() == 2);
    CHECK(mask.resolve(*skeleton) == 2);
    CHECK(mask.is_resolved());
    CHECK(mask.contains(1)); // middle
    CHECK(mask.contains(2)); // tip
    CHECK_FALSE(mask.contains(0));
}

TEST_CASE("a mask drops a name the skeleton does not have") {
    auto skeleton = make_chain_skeleton();
    JointMask mask;
    mask.add("tip");
    mask.add("does_not_exist");
    CHECK(mask.resolve(*skeleton) == 1);
    CHECK(mask.contains(2));
    CHECK_FALSE(mask.contains(0));
}

TEST_CASE("an unresolved non-empty mask is the unmasked default, not a no-op") {
    JointMask mask;
    mask.add("tip");
    // Not resolved yet: the mask must not silently drop the whole layer.
    CHECK(mask.contains(0));
    CHECK(mask.contains(2));
}

TEST_CASE("a mask round-trips through its JSON") {
    JointMask mask;
    mask.add("middle");
    mask.add("tip");
    const std::string text = mask.to_json();

    JointMask loaded;
    REQUIRE(loaded.from_json(text));
    REQUIRE(loaded.size() == 2);
    CHECK(loaded.names()[0] == "middle");
    CHECK(loaded.names()[1] == "tip");
}

// ---------------------------------------------------------------------------
// The masked compose
// ---------------------------------------------------------------------------

TEST_CASE("a masked compose writes only the joints in the mask") {
    auto skeleton = make_chain_skeleton();
    SkeletonPose base;
    base.resize(skeleton->joint_count());
    for (JointPose& joint : base.joints()) {
        joint = JointPose::identity();
    }

    SkeletonPose delta;
    delta.resize(skeleton->joint_count());
    for (JointPose& joint : delta.joints()) {
        joint = JointPose::identity();
    }
    delta.at(1).translation = Vec3{0.0f, 5.0f, 0.0f}; // middle
    delta.at(2).translation = Vec3{0.0f, 7.0f, 0.0f}; // tip

    JointMask mask;
    mask.add("middle");
    mask.resolve(*skeleton);

    compose_additive_masked(base, delta, 1.0f, mask);
    CHECK(nearly_equal(base.at(1).translation, Vec3{0.0f, 5.0f, 0.0f}));
    CHECK(nearly_equal(base.at(2).translation, Vec3{0.0f, 0.0f, 0.0f})); // untouched
}

TEST_CASE("a masked compose with an empty mask matches the unmasked one") {
    auto skeleton = make_chain_skeleton();
    SkeletonPose masked_base;
    SkeletonPose plain_base;
    masked_base.resize(skeleton->joint_count());
    plain_base.resize(skeleton->joint_count());
    for (usize i = 0; i < skeleton->joint_count(); ++i) {
        masked_base.at(i) = JointPose::identity();
        plain_base.at(i) = JointPose::identity();
    }

    SkeletonPose delta;
    delta.resize(skeleton->joint_count());
    for (JointPose& joint : delta.joints()) {
        joint = JointPose::identity();
    }
    delta.at(2).translation = Vec3{1.0f, 2.0f, 3.0f};

    JointMask empty;
    compose_additive_masked(masked_base, delta, 0.5f, empty);
    compose_additive(plain_base, delta, 0.5f);
    for (usize i = 0; i < skeleton->joint_count(); ++i) {
        CHECK(nearly_equal(masked_base.at(i).translation, plain_base.at(i).translation));
    }
}

// ---------------------------------------------------------------------------
// The system: the mask reaches the animator's pose
// ---------------------------------------------------------------------------

TEST_CASE("a masked additive layer leaves the joints it does not name at the base") {
    auto skeleton = make_chain_skeleton();
    auto walk = make_joint_clip(skeleton, "walk", "root", Vec3{0.0f, 0.0f, 0.0f});
    auto run = make_joint_clip(skeleton, "run", "root", Vec3{2.0f, 0.0f, 0.0f});
    // The additive clip moves both the root and the tip.
    auto hit = std::make_shared<AnimationClip>("clip:hit", "hit", [] {
        std::vector<JointTrack> tracks(2);
        tracks[0].joint_name = "root";
        VectorTrack root_track;
        root_track.add_key(VectorKey{0.0f, Vec3{0.0f, 1.0f, 0.0f}, {}, {}});
        root_track.add_key(VectorKey{1.0f, Vec3{0.0f, 1.0f, 0.0f}, {}, {}});
        tracks[0].translation = root_track;
        tracks[1].joint_name = "tip";
        VectorTrack tip_track;
        tip_track.add_key(VectorKey{0.0f, Vec3{0.0f, 4.0f, 0.0f}, {}, {}});
        tip_track.add_key(VectorKey{1.0f, Vec3{0.0f, 4.0f, 0.0f}, {}, {}});
        tracks[1].translation = tip_track;
        return tracks;
    }());
    hit->bind(*skeleton);

    Scene scene;
    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    REQUIRE((animator != nullptr));
    animator->skeleton = skeleton;
    animator->playing = false;

    auto* tree = node->add_component<AnimationBlendTree>();
    REQUIRE((tree != nullptr));
    tree->samples.resize(2);
    tree->samples[0].clip = walk;
    tree->samples[0].threshold = 0.0f;
    tree->samples[0].other_threshold = 1.0f;
    tree->samples[1].clip = run;
    tree->samples[1].threshold = 1.0f;
    tree->samples[1].other_threshold = 0.0f;
    tree->additive_clip = hit;
    tree->additive_weight = 1.0f;
    tree->set_parameter(0.0f); // base is walk: root at the origin
    // Only the root may be written by the additive layer.
    tree->additive_mask.add("root");

    AnimationBlendTreeSystem system;
    system.update(frame(0.1f), scene);

    // The base root is at 0 and the additive root adds 1.
    CHECK(nearly_equal(animator->pose().at(0).translation, Vec3{0.0f, 1.0f, 0.0f}));
    // The tip is not in the mask, so the additive tip offset of 4 is not applied
    // and the joint holds its rest translation.
    CHECK(nearly_equal(animator->pose().at(2).translation, Vec3{0.0f, 1.0f, 0.0f}));
}
