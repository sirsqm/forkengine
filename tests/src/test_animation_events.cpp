// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// Tests for clip timeline events (animation notifies).
//
// A clip carries named markers at times; the animation system reports the ones
// playback crossed this frame, in the order it passed them, so gameplay can
// react to a footstep or a live hitbox window. The crossing rule is a pure
// function of two wrapped positions, tested with no scene; the system is tested
// through the events it publishes.

#include <doctest/doctest.h>

#include "fe/anim/AnimationClip.h"
#include "fe/anim/Animator.h"
#include "fe/anim/Skeleton.h"
#include "fe/anim/SkeletonAnimationSystem.h"
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

[[nodiscard]] std::shared_ptr<Skeleton> make_skeleton() {
    std::vector<Skeleton::Joint> joints(1);
    joints[0].name = "root";
    joints[0].parent = Skeleton::kNoParent;

    auto skeleton = std::make_shared<Skeleton>("skeleton:one", std::move(joints));
    skeleton->compute_inverse_bind_matrices();
    return skeleton;
}

// A clip of `duration` seconds holding the root still, with the given markers.
[[nodiscard]] std::shared_ptr<AnimationClip> make_clip(const std::shared_ptr<Skeleton>& skeleton, f32 duration,
                                                       const std::vector<AnimationEvent>& events) {
    std::vector<JointTrack> tracks(1);
    tracks[0].joint_name = "root";
    VectorTrack translation;
    translation.add_key(VectorKey{0.0f, Vec3{}, {}, {}});
    translation.add_key(VectorKey{duration, Vec3{}, {}, {}});
    tracks[0].translation = translation;

    auto clip = std::make_shared<AnimationClip>("clip:test", "test", std::move(tracks));
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
// The crossing rule
// ---------------------------------------------------------------------------

TEST_CASE("a marker in the window is reported") {
    auto skeleton = make_skeleton();
    auto clip = make_clip(skeleton, 1.0f, {{"step", 0.5f}});
    std::vector<AnimationEvent> out;
    CHECK(collect_clip_events(*clip, 0.0f, 0.6f, out) == 1);
    REQUIRE(out.size() == 1);
    CHECK(out[0].name == "step");
    CHECK(out[0].time == doctest::Approx(0.5f));
}

TEST_CASE("a marker exactly at the current position fires, one at the previous does not") {
    auto skeleton = make_skeleton();
    auto clip = make_clip(skeleton, 1.0f, {{"a", 0.5f}, {"b", 0.7f}});
    std::vector<AnimationEvent> out;
    // (0.5, 0.7]: "b" at the right edge fires, "a" at the left edge does not.
    CHECK(collect_clip_events(*clip, 0.5f, 0.7f, out) == 1);
    REQUIRE(out.size() == 1);
    CHECK(out[0].name == "b");
}

TEST_CASE("a marker past the window is not reported") {
    auto skeleton = make_skeleton();
    auto clip = make_clip(skeleton, 1.0f, {{"late", 0.9f}});
    std::vector<AnimationEvent> out;
    CHECK(collect_clip_events(*clip, 0.0f, 0.5f, out) == 0);
    CHECK(out.empty());
}

TEST_CASE("a wrap reports the tail before the head") {
    auto skeleton = make_skeleton();
    // Duration 1.0; a marker near the end and one near the start.
    auto clip = make_clip(skeleton, 1.0f, {{"start", 0.05f}, {"end", 0.95f}});
    std::vector<AnimationEvent> out;
    // Playback wrapped from 0.9 back to 0.1: it passed "end" (in (0.9, 1.0]) then
    // "start" (in [0, 0.1]).
    CHECK(collect_clip_events(*clip, 0.9f, 0.1f, out) == 2);
    REQUIRE(out.size() == 2);
    CHECK(out[0].name == "end");
    CHECK(out[1].name == "start");
}

TEST_CASE("a clip with no events reports nothing") {
    auto skeleton = make_skeleton();
    auto clip = make_clip(skeleton, 1.0f, {});
    std::vector<AnimationEvent> out;
    CHECK(collect_clip_events(*clip, 0.0f, 1.0f, out) == 0);
}

TEST_CASE("events are kept sorted by time however they are added") {
    auto skeleton = make_skeleton();
    auto clip = make_clip(skeleton, 1.0f, {});
    clip->add_event("third", 0.8f);
    clip->add_event("first", 0.2f);
    clip->add_event("second", 0.5f);
    REQUIRE(clip->events().size() == 3);
    CHECK(clip->events()[0].name == "first");
    CHECK(clip->events()[1].name == "second");
    CHECK(clip->events()[2].name == "third");
}

// ---------------------------------------------------------------------------
// The system reports the events
// ---------------------------------------------------------------------------

TEST_CASE("the animation system reports the marker a frame crosses") {
    auto skeleton = make_skeleton();
    auto clip = make_clip(skeleton, 1.0f, {{"step", 0.5f}});

    Scene scene;
    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    animator->skeleton = skeleton;
    animator->clip = clip;
    animator->loop = LoopMode::Loop;
    animator->time = 0.0f;

    SkeletonAnimationSystem system;

    // First frame: 0 -> 0.4, no marker yet.
    system.update(frame(0.4f), scene);
    CHECK(system.events().empty());

    // Second frame: 0.4 -> 0.8, crosses 0.5.
    system.update(frame(0.4f), scene);
    REQUIRE(system.events().size() == 1);
    CHECK(system.events()[0].name == "step");
}

TEST_CASE("the animation system reports a marker once as the clock passes it") {
    auto skeleton = make_skeleton();
    auto clip = make_clip(skeleton, 1.0f, {{"step", 0.5f}});

    Scene scene;
    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    animator->skeleton = skeleton;
    animator->clip = clip;
    animator->loop = LoopMode::Loop;

    SkeletonAnimationSystem system;
    usize total = 0;
    for (int i = 0; i < 10; ++i) {
        system.update(frame(0.1f), scene);
        total += system.events().size();
    }
    // Ten tenths of a second over a one-second clip: the marker at 0.5 fires
    // exactly once, not once per frame while the clock sits past it.
    CHECK(total == 1);
}

TEST_CASE("the animation system reports a marker on the frame a loop wraps") {
    auto skeleton = make_skeleton();
    // A marker just before the end and one just after the start.
    auto clip = make_clip(skeleton, 1.0f, {{"tail", 0.95f}, {"head", 0.05f}});

    Scene scene;
    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    animator->skeleton = skeleton;
    animator->clip = clip;
    animator->loop = LoopMode::Loop;

    SkeletonAnimationSystem system;
    // 0 -> 0.9: "head" (0.05) is crossed, "tail" (0.95) is not.
    system.update(frame(0.9f), scene);
    REQUIRE(system.events().size() == 1);
    CHECK(system.events()[0].name == "head");
    // 0.9 -> 1.1, wrapping to 0.1: the tail of the clip is passed first, then the
    // head after the wrap, so "tail" is reported before the second "head".
    system.update(frame(0.2f), scene);
    REQUIRE(system.events().size() == 2);
    CHECK(system.events()[0].name == "tail");
    CHECK(system.events()[1].name == "head");
}
