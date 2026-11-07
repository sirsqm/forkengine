// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// Tests for timeline events on the clips a blend contributes.
//
// `SkeletonAnimationSystem` reports the markers of an animator's primary clip;
// this covers the blend tree and blend space, where several clips contribute at
// once and a marker fires when the clock crossed it in any of them. The pure
// function is tested with no scene; the systems are tested through the events
// they publish.

#include <doctest/doctest.h>

#include "fe/anim/AnimationBlendSpace2D.h"
#include "fe/anim/AnimationBlendSpaceSystem.h"
#include "fe/anim/AnimationBlendTree.h"
#include "fe/anim/AnimationBlendTreeSystem.h"
#include "fe/anim/AnimationClip.h"
#include "fe/anim/Animator.h"
#include "fe/anim/Skeleton.h"
#include "fe/anim/SkeletonAnimationSystem.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/systems/FrameContext.h"

#include <memory>
#include <ostream>
#include <string>
#include <vector>

using namespace fe;
using namespace fe::anim;

namespace {

[[nodiscard]] std::shared_ptr<Skeleton> make_skeleton() {
    std::vector<Skeleton::Joint> joints(1);
    joints[0].name = "root";
    joints[0].parent = Skeleton::kNoParent;

    auto skeleton = std::make_shared<Skeleton>("skeleton:one", std::move(joints));
    skeleton->compute_inverse_bind_matrices();
    return skeleton;
}

[[nodiscard]] std::shared_ptr<AnimationClip> make_clip(const std::shared_ptr<Skeleton>& skeleton, const char* name,
                                                       f32 duration, const std::vector<AnimationEvent>& events) {
    std::vector<JointTrack> tracks(1);
    tracks[0].joint_name = "root";
    VectorTrack translation;
    translation.add_key(VectorKey{0.0f, Vec3{}, {}, {}});
    translation.add_key(VectorKey{duration, Vec3{}, {}, {}});
    tracks[0].translation = translation;

    auto clip = std::make_shared<AnimationClip>(std::string("clip:") + name, name, std::move(tracks));
    clip->bind(*skeleton);
    clip->set_events(events);
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
// The pure function
// ---------------------------------------------------------------------------

TEST_CASE("a blend reports the marker of every contributing clip") {
    auto skeleton = make_skeleton();
    auto walk = make_clip(skeleton, "walk", 1.0f, {{"step", 0.5f}});
    auto run = make_clip(skeleton, "run", 1.0f, {{"step", 0.5f}});

    BlendClipTiming timings[2];
    timings[0] = BlendClipTiming{walk.get(), 0.0f, 0.6f, LoopMode::Loop, 0.5f};
    timings[1] = BlendClipTiming{run.get(), 0.0f, 0.6f, LoopMode::Loop, 0.5f};

    std::vector<AnimationEvent> out;
    // Both clips carry "step" at the same time, so it is reported once, not twice.
    CHECK(collect_blend_events(timings, 2, out) == 1);
    REQUIRE(out.size() == 1);
    CHECK(out[0].name == "step");
}

TEST_CASE("a clip at zero weight contributes no events") {
    auto skeleton = make_skeleton();
    auto walk = make_clip(skeleton, "walk", 1.0f, {{"step", 0.5f}});
    auto run = make_clip(skeleton, "run", 1.0f, {{"other", 0.5f}});

    BlendClipTiming timings[2];
    timings[0] = BlendClipTiming{walk.get(), 0.0f, 0.6f, LoopMode::Loop, 1.0f};
    timings[1] = BlendClipTiming{run.get(), 0.0f, 0.6f, LoopMode::Loop, 0.0f};

    std::vector<AnimationEvent> out;
    CHECK(collect_blend_events(timings, 2, out) == 1);
    REQUIRE(out.size() == 1);
    CHECK(out[0].name == "step");
}

TEST_CASE("two clips with different markers both report") {
    auto skeleton = make_skeleton();
    auto walk = make_clip(skeleton, "walk", 1.0f, {{"foot_left", 0.3f}});
    auto run = make_clip(skeleton, "run", 1.0f, {{"foot_right", 0.6f}});

    BlendClipTiming timings[2];
    timings[0] = BlendClipTiming{walk.get(), 0.0f, 0.9f, LoopMode::Loop, 0.5f};
    timings[1] = BlendClipTiming{run.get(), 0.0f, 0.9f, LoopMode::Loop, 0.5f};

    std::vector<AnimationEvent> out;
    CHECK(collect_blend_events(timings, 2, out) == 2);
    REQUIRE(out.size() == 2);
    CHECK(out[0].name == "foot_left");
    CHECK(out[1].name == "foot_right");
}

// ---------------------------------------------------------------------------
// The blend tree system
// ---------------------------------------------------------------------------

TEST_CASE("the blend tree system reports a marker the base blend crossed") {
    auto skeleton = make_skeleton();
    auto walk = make_clip(skeleton, "walk", 1.0f, {{"step", 0.5f}});

    Scene scene;
    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    animator->skeleton = skeleton;
    auto* tree = node->add_component<AnimationBlendTree>();
    tree->samples.resize(1);
    tree->samples[0].clip = walk;
    tree->samples[0].speed = 1.0f;

    // The animator's clock is advanced by the animation system, so run it first,
    // exactly as the schedule does.
    SkeletonAnimationSystem animation;
    AnimationBlendTreeSystem system;

    animation.update(frame(0.4f), scene);
    system.update(frame(0.4f), scene);
    CHECK(system.events().empty());

    animation.update(frame(0.4f), scene);
    system.update(frame(0.4f), scene);
    REQUIRE(system.events().size() == 1);
    CHECK(system.events()[0].name == "step");
}

TEST_CASE("the blend tree system does not double-report a marker two clips share") {
    auto skeleton = make_skeleton();
    auto walk = make_clip(skeleton, "walk", 1.0f, {{"step", 0.5f}});
    auto run = make_clip(skeleton, "run", 1.0f, {{"step", 0.5f}});

    Scene scene;
    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    animator->skeleton = skeleton;
    auto* tree = node->add_component<AnimationBlendTree>();
    tree->samples.resize(2);
    tree->samples[0].clip = walk;
    tree->samples[1].clip = run;
    tree->use_parameter = false;
    tree->weight = 0.5f;

    SkeletonAnimationSystem animation;
    AnimationBlendTreeSystem system;
    animation.update(frame(0.4f), scene);
    system.update(frame(0.4f), scene);
    animation.update(frame(0.4f), scene);
    system.update(frame(0.4f), scene);

    // Both clips carry "step" at the same time, so a blended walk/run pair sounds
    // one footstep.
    REQUIRE(system.events().size() == 1);
    CHECK(system.events()[0].name == "step");
}

// ---------------------------------------------------------------------------
// The blend space system
// ---------------------------------------------------------------------------

TEST_CASE("the blend space system reports a marker of the sample it played") {
    auto skeleton = make_skeleton();
    auto forward = make_clip(skeleton, "forward", 1.0f, {{"step", 0.5f}});

    Scene scene;
    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    animator->skeleton = skeleton;
    auto* space = node->add_component<AnimationBlendSpace2D>();
    space->samples.resize(1);
    space->samples[0].clip = forward;
    space->samples[0].position = Vec2{0.0f, 0.0f};

    SkeletonAnimationSystem animation;
    AnimationBlendSpaceSystem system;

    animation.update(frame(0.4f), scene);
    system.update(frame(0.4f), scene);
    CHECK(system.events().empty());

    animation.update(frame(0.4f), scene);
    system.update(frame(0.4f), scene);
    REQUIRE(system.events().size() == 1);
    CHECK(system.events()[0].name == "step");
}
