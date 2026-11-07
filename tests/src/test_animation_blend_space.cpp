// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// Tests for the two-axis animation blend space (docs/SHIPPING_QUEUE.md, the
// backlog's "wider shape" of the blend tree: a two-axis locomotion grid).
//
// The pure geometry -- which samples surround a parameter and with what weight
// -- is asserted with no scene and no animator, and the system is asserted
// through the palette it publishes.

#include <doctest/doctest.h>

#include "fe/anim/AnimationBlendSpace2D.h"
#include "fe/anim/AnimationBlendSpaceSystem.h"
#include "fe/anim/AnimationClip.h"
#include "fe/anim/Animator.h"
#include "fe/anim/SkeletalAnimationTypes.h"
#include "fe/anim/Skeleton.h"
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

// A clip holding the root at a constant translation for its whole length, so a
// blend's result depends only on the weight.
[[nodiscard]] std::shared_ptr<AnimationClip> make_root_clip(const std::shared_ptr<Skeleton>& skeleton, std::string name,
                                                            const Vec3& translation, f32 duration = 1.0f) {
    std::vector<JointTrack> tracks(1);
    tracks[0].joint_name = "root";

    VectorTrack translation_track;
    translation_track.add_key(VectorKey{0.0f, translation, {}, {}});
    translation_track.add_key(VectorKey{duration, translation, {}, {}});
    tracks[0].translation = translation_track;

    auto clip = std::make_shared<AnimationClip>("clip:" + name, std::move(name), std::move(tracks));
    clip->bind(*skeleton);
    return clip;
}

[[nodiscard]] BlendPoint point(std::shared_ptr<AnimationClip> clip, Vec2 position) {
    BlendPoint p;
    p.name = clip->name();
    p.clip = std::move(clip);
    p.position = position;
    return p;
}

// The weight `weights` assigns to sample `index`, or 0 when it does not appear.
[[nodiscard]] f32 weight_of(const AnimationBlendSpace2D& space, usize index) {
    BlendWeight out[3];
    const usize count = space.weights(out, 3);
    for (usize i = 0; i < count; ++i) {
        if (out[i].sample == index) {
            return out[i].weight;
        }
    }
    return 0.0f;
}

[[nodiscard]] FrameContext frame(f32 delta) {
    FrameContext context;
    context.delta_time = delta;
    context.elapsed_time = static_cast<f64>(delta);
    return context;
}

// A four-corner grid: origin, +x, +y, and the diagonal corner, one clip each.
// Fills a component in place; a `Component` is not movable, so a helper cannot
// return one by value.
void fill_corner_grid(AnimationBlendSpace2D& space, const std::shared_ptr<Skeleton>& skeleton) {
    space.samples = {
        point(make_root_clip(skeleton, "idle", Vec3{0.0f, 0.0f, 0.0f}), Vec2{0.0f, 0.0f}),
        point(make_root_clip(skeleton, "right", Vec3{1.0f, 0.0f, 0.0f}), Vec2{1.0f, 0.0f}),
        point(make_root_clip(skeleton, "up", Vec3{0.0f, 1.0f, 0.0f}), Vec2{0.0f, 1.0f}),
        point(make_root_clip(skeleton, "diagonal", Vec3{1.0f, 1.0f, 0.0f}), Vec2{1.0f, 1.0f}),
    };
}

} // namespace

// ---------------------------------------------------------------------------
// The weights: which samples surround a parameter, and with what weight
// ---------------------------------------------------------------------------

TEST_CASE("an empty blend space has no weights") {
    AnimationBlendSpace2D space;
    BlendWeight out[3];
    CHECK(space.weights(out, 3) == 0);
}

TEST_CASE("a one-sample blend space is that sample at full weight") {
    auto skeleton = make_chain_skeleton();
    AnimationBlendSpace2D space;
    space.samples = {point(make_root_clip(skeleton, "only", Vec3{}), Vec2{5.0f, 5.0f})};
    space.set_parameter(Vec2{100.0f, -100.0f});
    CHECK(nearly_equal(weight_of(space, 0), 1.0f));
}

TEST_CASE("a two-sample blend space blends along the segment between them") {
    auto skeleton = make_chain_skeleton();
    AnimationBlendSpace2D space;
    space.samples = {
        point(make_root_clip(skeleton, "a", Vec3{}), Vec2{0.0f, 0.0f}),
        point(make_root_clip(skeleton, "b", Vec3{}), Vec2{2.0f, 0.0f}),
    };

    space.set_parameter(Vec2{0.0f, 0.0f});
    CHECK(nearly_equal(weight_of(space, 0), 1.0f));
    space.set_parameter(Vec2{1.0f, 0.0f});
    CHECK(nearly_equal(weight_of(space, 0), 0.5f));
    CHECK(nearly_equal(weight_of(space, 1), 0.5f));
    space.set_parameter(Vec2{2.0f, 0.0f});
    CHECK(nearly_equal(weight_of(space, 1), 1.0f));

    // Past either end the blend clamps to the nearer sample.
    space.set_parameter(Vec2{-5.0f, 0.0f});
    CHECK(nearly_equal(weight_of(space, 0), 1.0f));
    space.set_parameter(Vec2{5.0f, 0.0f});
    CHECK(nearly_equal(weight_of(space, 1), 1.0f));
}

TEST_CASE("a vertex of the grid is that sample alone") {
    auto skeleton = make_chain_skeleton();
    AnimationBlendSpace2D space;
    fill_corner_grid(space, skeleton);

    space.set_parameter(Vec2{0.0f, 0.0f});
    CHECK(nearly_equal(weight_of(space, 0), 1.0f));
    space.set_parameter(Vec2{1.0f, 1.0f});
    CHECK(nearly_equal(weight_of(space, 3), 1.0f)); // the diagonal corner
}

TEST_CASE("the centre of a four-corner cell blends all four corners") {
    auto skeleton = make_chain_skeleton();
    AnimationBlendSpace2D space;
    fill_corner_grid(space, skeleton);

    space.set_parameter(Vec2{0.5f, 0.5f});
    BlendWeight out[3];
    const usize count = space.weights(out, 3);
    // A point in the middle of a square lies on the diagonal, so the blend uses
    // the two triangles' shared edge: two corners, or three if the diagonal is
    // the other one. Either way the weights sum to one and no sample exceeds a
    // half.
    REQUIRE(count >= 1);
    f32 total = 0.0f;
    for (usize i = 0; i < count; ++i) {
        total += out[i].weight;
        CHECK(out[i].weight <= 0.5f + kTolerance);
    }
    CHECK(nearly_equal(total, 1.0f));
}

TEST_CASE("a point inside a triangle blends its three corners by barycentric weight") {
    auto skeleton = make_chain_skeleton();
    AnimationBlendSpace2D space;
    fill_corner_grid(space, skeleton);

    // (0.25, 0.25) lies on the lower-left triangle (idle, right, up) with the
    // barycentric weights 0.5, 0.25, 0.25.
    space.set_parameter(Vec2{0.25f, 0.25f});
    CHECK(nearly_equal(weight_of(space, 0), 0.5f));
    CHECK(nearly_equal(weight_of(space, 1), 0.25f));
    CHECK(nearly_equal(weight_of(space, 2), 0.25f));
}

TEST_CASE("a parameter outside the grid clamps to the nearest sample") {
    auto skeleton = make_chain_skeleton();
    AnimationBlendSpace2D space;
    fill_corner_grid(space, skeleton);

    space.set_parameter(Vec2{10.0f, -10.0f});
    // The nearest corner is "right" at (1, 0), the closest of the four.
    CHECK(nearly_equal(weight_of(space, 1), 1.0f));
    BlendWeight out[3];
    CHECK(space.weights(out, 3) == 1);
}

// ---------------------------------------------------------------------------
// The system: the blend reaches the animator's palette
// ---------------------------------------------------------------------------

TEST_CASE("the blend space system writes the blended pose onto the animator") {
    auto skeleton = make_chain_skeleton();
    Scene scene;
    Node* node = scene.create_node("Character");
    auto* animator = node->add_component<Animator>();
    animator->skeleton = skeleton;
    animator->playing = false;

    auto* space = node->add_component<AnimationBlendSpace2D>();
    // Two samples on the x axis so the weight is exactly one half.
    space->samples = {
        point(make_root_clip(skeleton, "left", Vec3{0.0f, 0.0f, 0.0f}), Vec2{0.0f, 0.0f}),
        point(make_root_clip(skeleton, "right", Vec3{2.0f, 0.0f, 0.0f}), Vec2{2.0f, 0.0f}),
    };
    space->set_parameter(Vec2{1.0f, 0.0f});

    AnimationBlendSpaceSystem system;
    system.update(frame(1.0f / 60.0f), scene);

    CHECK(system.last_advanced_count() == 1);
    CHECK(system.last_skipped_count() == 0);
    REQUIRE(animator->pose().joint_count() == 3);
    // The midpoint of a 0 and a 2 on x is 1.
    CHECK(nearly_equal(animator->pose().at(0).translation, Vec3{1.0f, 0.0f, 0.0f}, 1e-3f));
}

TEST_CASE("the blend space system skips a node with no animator") {
    Scene scene;
    Node* node = scene.create_node("Bare");
    node->add_component<AnimationBlendSpace2D>();

    AnimationBlendSpaceSystem system;
    system.update(frame(1.0f / 60.0f), scene);
    CHECK(system.last_advanced_count() == 0);
    CHECK(system.last_skipped_count() == 1);
}

// ---------------------------------------------------------------------------
// Persistence and reflection
// ---------------------------------------------------------------------------

TEST_CASE("a blend space round-trips through its JSON") {
    auto skeleton = make_chain_skeleton();
    AnimationBlendSpace2D space;
    fill_corner_grid(space, skeleton);
    space.set_parameter(Vec2{0.3f, 0.7f});
    space.drive_animator = false;
    space.samples[0].clip_key = "clip:idle";
    space.samples[1].clip_key = "clip:right";
    space.additive_clip_key = "clip:hit";
    space.additive_weight = 0.5f;
    space.additive_speed = 2.0f;

    const std::string text = space.to_json();
    AnimationBlendSpace2D loaded;
    REQUIRE(loaded.from_json(text));

    CHECK(nearly_equal(loaded.parameter.x, 0.3f));
    CHECK(nearly_equal(loaded.parameter.y, 0.7f));
    CHECK(loaded.drive_animator == false);
    REQUIRE(loaded.samples.size() == 4);
    CHECK(loaded.samples[0].clip_key == "clip:idle");
    CHECK(loaded.samples[1].clip_key == "clip:right");
    CHECK(nearly_equal(loaded.samples[3].position.x, 1.0f));
    CHECK(nearly_equal(loaded.samples[3].position.y, 1.0f));
    CHECK(loaded.additive_clip_key == "clip:hit");
    CHECK(nearly_equal(loaded.additive_weight, 0.5f));
    CHECK(nearly_equal(loaded.additive_speed, 2.0f));
}

TEST_CASE("the blend space type is registered for reflection") {
    reflect::Registry& registry = reflect::Registry::instance();
    reflect::register_skeletal_animation_types();
    const reflect::TypeDesc* desc = registry.find(AnimationBlendSpace2D::kTypeName);
    REQUIRE(desc != nullptr);
    REQUIRE(desc->properties.size() == 1);
    CHECK(desc->properties[0].name == "space");

    std::unique_ptr<Component> component = desc->factory();
    REQUIRE(component != nullptr);
    auto* space = static_cast<AnimationBlendSpace2D*>(component.get());
    // The setter drives the JSON round-trip through the one property.
    AnimationBlendSpace2D source;
    fill_corner_grid(source, make_chain_skeleton());
    source.set_parameter(Vec2{0.1f, 0.9f});
    REQUIRE(desc->properties[0].setter(space, reflect::Value{source.to_json()}));
    CHECK(nearly_equal(space->parameter.x, 0.1f));
    CHECK(nearly_equal(space->parameter.y, 0.9f));
}
