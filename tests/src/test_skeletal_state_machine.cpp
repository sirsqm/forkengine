// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// Tests for the skeletal animation state machine (docs/SHIPPING_QUEUE.md,
// `skeletal-state-machine`).
//
// The machine is the skeletal counterpart of AnimationStateMachine (ADR-0059):
// named states each naming a clip, transitions gated on a finish and on
// triggers. These tests cover the queue item's acceptance criteria -- entry, a
// trigger transition, and a finish-gated return -- and the wiring onto an
// Animator, so a state change changes what plays.

#include <doctest/doctest.h>

#include "fe/anim/AnimationClip.h"
#include "fe/anim/Animator.h"
#include "fe/anim/SkeletalAnimationTypes.h"
#include "fe/anim/SkeletalStateMachine.h"
#include "fe/anim/SkeletalStateMachineSystem.h"
#include "fe/anim/Skeleton.h"
#include "fe/anim/SkeletonAnimationSystem.h"
#include "fe/reflect/Property.h"
#include "fe/reflect/Registry.h"
#include "fe/reflect/TypeDesc.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/serialization/SceneSerializer.h"
#include "fe/systems/FrameContext.h"

#include <memory>
#include <ostream>
#include <string>
#include <string_view>
#include <vector>

using namespace fe;
using namespace fe::anim;

namespace {

// A three-joint chain, matching the shape the skeletal tests use: root at the
// origin, `middle` one unit up, `tip` one unit further.
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

// A clip of `duration` seconds that rotates the root about Z, bound to the chain.
[[nodiscard]] std::shared_ptr<AnimationClip> make_clip(const std::shared_ptr<Skeleton>& skeleton, std::string name,
                                                       f32 duration) {
    std::vector<JointTrack> tracks(1);
    tracks[0].joint_name = "root";
    QuaternionTrack rotation;
    rotation.add_key(QuaternionKey{0.0f, Quat::identity(), {}, {}});
    rotation.add_key(QuaternionKey{duration, Quat::from_axis_angle(Vec3{0.0f, 0.0f, 1.0f}, radians(90.0f)), {}, {}});
    tracks[0].rotation = rotation;
    auto clip = std::make_shared<AnimationClip>("clip:" + name, std::move(name), std::move(tracks));
    clip->bind(*skeleton);
    return clip;
}

[[nodiscard]] SkeletalState state(std::string name, std::shared_ptr<AnimationClip> clip,
                                  LoopMode loop = LoopMode::Loop) {
    SkeletalState s;
    s.name = std::move(name);
    s.clip = std::move(clip);
    s.loop = loop;
    return s;
}

[[nodiscard]] SkeletalTransition transition(std::string from, std::string to, bool require_finished = false,
                                            std::vector<std::string> conditions = {}) {
    SkeletalTransition edge;
    edge.from = std::move(from);
    edge.to = std::move(to);
    edge.require_finished = require_finished;
    edge.conditions = std::move(conditions);
    return edge;
}

[[nodiscard]] FrameContext frame(f32 delta) {
    FrameContext context;
    context.delta_time = delta;
    context.elapsed_time = static_cast<f64>(delta);
    return context;
}

} // namespace

TEST_CASE("a skeletal state machine finds and enters states by name") {
    auto skeleton = make_chain_skeleton();
    auto idle = make_clip(skeleton, "idle", 1.0f);
    auto run = make_clip(skeleton, "run", 1.0f);

    SkeletalStateMachine machine;
    machine.states = {state("idle", idle), state("run", run)};

    CHECK(machine.find_state("idle") == 0);
    CHECK(machine.find_state("run") == 1);
    CHECK(machine.find_state("missing") == -1);
    CHECK(machine.active_state() == -1);
    CHECK(machine.active_state_name().empty());
    CHECK((machine.active_clip() == nullptr));

    CHECK(machine.play("run"));
    CHECK(machine.active_state() == 1);
    CHECK(machine.active_state_name() == "run");
    CHECK((machine.active_clip() == run));
    CHECK(machine.active_loop() == LoopMode::Loop);

    // Playing an unknown state changes nothing and reports the miss.
    CHECK_FALSE(machine.play("missing"));
    CHECK(machine.active_state() == 1);
}

TEST_CASE("advance enters the initial state, then the first state as a fallback") {
    auto skeleton = make_chain_skeleton();
    auto idle = make_clip(skeleton, "idle", 1.0f);
    auto run = make_clip(skeleton, "run", 1.0f);

    SkeletalStateMachine explicit_initial;
    explicit_initial.states = {state("idle", idle), state("run", run)};
    explicit_initial.initial_state = "run";
    explicit_initial.advance(0.1f);
    CHECK(explicit_initial.active_state_name() == "run");

    SkeletalStateMachine default_initial;
    default_initial.states = {state("idle", idle), state("run", run)};
    default_initial.advance(0.1f);
    CHECK(default_initial.active_state_name() == "idle");

    // An empty graph is a well-defined no-op rather than a crash.
    SkeletalStateMachine empty;
    empty.advance(0.1f);
    CHECK(empty.active_state() == -1);
    CHECK_FALSE(empty.state_finished());
}

TEST_CASE("a looping state advances without ever finishing and wraps its clock") {
    auto skeleton = make_chain_skeleton();
    auto idle = make_clip(skeleton, "idle", 1.0f);

    SkeletalStateMachine machine;
    machine.states = {state("idle", idle, LoopMode::Loop)};
    machine.initial_state = "idle";

    machine.advance(0.4f);
    CHECK(machine.elapsed() == doctest::Approx(0.4f));
    CHECK_FALSE(machine.state_finished());

    // Past the clip's end the clock wraps rather than reporting finished.
    machine.advance(0.8f);
    CHECK(machine.elapsed() == doctest::Approx(0.2f));
    CHECK_FALSE(machine.state_finished());
}

TEST_CASE("a one-shot state finishes on its clip duration") {
    auto skeleton = make_chain_skeleton();
    auto attack = make_clip(skeleton, "attack", 0.5f);

    SkeletalStateMachine machine;
    machine.states = {state("attack", attack, LoopMode::Once)};
    machine.initial_state = "attack";

    machine.advance(0.25f);
    CHECK_FALSE(machine.state_finished());

    machine.advance(0.25f);
    CHECK(machine.state_finished());
    // The clock clamps at the duration rather than running past it.
    CHECK(machine.elapsed() == doctest::Approx(0.5f));
}

TEST_CASE("a trigger transition fires and is consumed once") {
    auto skeleton = make_chain_skeleton();
    auto idle = make_clip(skeleton, "idle", 1.0f);
    auto attack = make_clip(skeleton, "attack", 1.0f);

    SkeletalStateMachine machine;
    machine.states = {state("idle", idle), state("attack", attack)};
    machine.transitions = {transition("idle", "attack", false, {"Attack"})};
    machine.initial_state = "idle";
    machine.advance(0.1f);
    CHECK(machine.active_state_name() == "idle");

    // A frame with the trigger unset does not transition.
    machine.advance(0.1f);
    CHECK(machine.active_state_name() == "idle");

    machine.set_trigger("Attack");
    machine.advance(0.1f);
    CHECK(machine.active_state_name() == "attack");
    // The trigger was consumed by the transition that used it.
    CHECK_FALSE(machine.has_trigger("Attack"));
}

TEST_CASE("a finish-gated transition returns to the previous state") {
    auto skeleton = make_chain_skeleton();
    auto idle = make_clip(skeleton, "idle", 1.0f);
    auto attack = make_clip(skeleton, "attack", 0.4f);

    SkeletalStateMachine machine;
    machine.states = {state("idle", idle), state("attack", attack, LoopMode::Once)};
    // The attack interrupts from any state, and returns to idle when it finishes.
    machine.transitions = {transition("", "attack", false, {"Attack"}), transition("attack", "idle", true)};
    machine.initial_state = "idle";

    machine.advance(0.1f);
    CHECK(machine.active_state_name() == "idle");

    machine.set_trigger("Attack");
    machine.advance(0.1f);
    CHECK(machine.active_state_name() == "attack");
    CHECK_FALSE(machine.state_finished());

    // The attack clip is 0.4s; the second step carries it past the end, and the
    // finish-gated edge back to idle fires on the same frame.
    machine.advance(0.4f);
    CHECK(machine.state_finished() == false);
    CHECK(machine.active_state_name() == "idle");
}

TEST_CASE("a triggerless self-edge never fires, so a state cannot re-enter itself forever") {
    auto skeleton = make_chain_skeleton();
    auto idle = make_clip(skeleton, "idle", 1.0f);

    SkeletalStateMachine machine;
    machine.states = {state("idle", idle)};
    machine.transitions = {transition("idle", "idle")};
    machine.initial_state = "idle";

    for (int i = 0; i < 5; ++i) {
        machine.advance(0.1f);
        CHECK(machine.active_state_name() == "idle");
    }
}

TEST_CASE("a paused machine holds its state and clock") {
    auto skeleton = make_chain_skeleton();
    auto idle = make_clip(skeleton, "idle", 1.0f);

    SkeletalStateMachine machine;
    machine.states = {state("idle", idle)};
    machine.initial_state = "idle";
    machine.advance(0.2f);
    const f32 held = machine.elapsed();

    machine.playing = false;
    machine.advance(0.5f);
    CHECK(machine.elapsed() == doctest::Approx(held));
    CHECK(machine.active_state_name() == "idle");
}

TEST_CASE("the graph round-trips through its serialized form") {
    auto skeleton = make_chain_skeleton();
    auto idle = make_clip(skeleton, "idle", 1.0f);
    auto attack = make_clip(skeleton, "attack", 0.4f);

    SkeletalStateMachine machine;
    machine.states = {state("idle", idle), state("attack", attack, LoopMode::Once)};
    machine.transitions = {transition("", "attack", false, {"Attack"}), transition("attack", "idle", true)};
    machine.initial_state = "idle";
    machine.drive_animator = false;

    const std::string text = machine.to_json();

    SkeletalStateMachine reloaded;
    REQUIRE(reloaded.from_json(text));
    CHECK(reloaded.states.size() == 2);
    CHECK(reloaded.transitions.size() == 2);
    CHECK(reloaded.initial_state == "idle");
    CHECK_FALSE(reloaded.drive_animator);
    CHECK(reloaded.find_state("attack") == 1);
    CHECK(reloaded.states[1].loop == LoopMode::Once);
    // The clip travels as its key; the resolved pointer is rebuilt by the
    // manager, not the document.
    CHECK(reloaded.states[1].clip_key == "clip:attack");
    CHECK((reloaded.states[1].clip == nullptr));

    // A malformed document is rejected without changing the machine.
    CHECK_FALSE(reloaded.from_json("{ not json"));
    CHECK(reloaded.states.size() == 2);
}

TEST_CASE("the system drives the node's animator from the active state") {
    auto skeleton = make_chain_skeleton();
    auto idle = make_clip(skeleton, "idle", 1.0f);
    auto run = make_clip(skeleton, "run", 1.0f);

    Scene scene;
    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    REQUIRE((animator != nullptr));
    animator->skeleton = skeleton;

    auto* machine = node->add_component<SkeletalStateMachine>();
    REQUIRE((machine != nullptr));
    machine->states = {state("idle", idle), state("run", run)};
    machine->transitions = {transition("idle", "run", false, {"Run"})};
    machine->initial_state = "idle";

    SkeletalStateMachineSystem machine_system;
    SkeletonAnimationSystem animation_system;
    FrameContext context = frame(0.1f);

    machine_system.update(context, scene);
    CHECK(machine_system.last_advanced_count() == 1);
    // Entering the initial state bound its clip onto the animator.
    CHECK((animator->clip == idle));
    CHECK(animator->loop == LoopMode::Loop);

    // The animation system samples the clip the machine chose this frame.
    animation_system.update(context, scene);
    CHECK(animator->has_palette());

    // A trigger moves to the other state and rebinds the animator.
    machine->set_trigger("Run");
    machine_system.update(context, scene);
    CHECK((animator->clip == run));
}

TEST_CASE("a machine with drive_animator off advances but leaves the animator alone") {
    auto skeleton = make_chain_skeleton();
    auto idle = make_clip(skeleton, "idle", 1.0f);
    auto run = make_clip(skeleton, "run", 1.0f);

    Scene scene;
    Node* node = scene.create_node("character");
    auto* animator = node->add_component<Animator>();
    REQUIRE((animator != nullptr));
    animator->skeleton = skeleton;
    animator->play(idle);

    auto* machine = node->add_component<SkeletalStateMachine>();
    REQUIRE((machine != nullptr));
    machine->states = {state("idle", idle), state("run", run)};
    machine->initial_state = "run";
    machine->drive_animator = false;

    SkeletalStateMachineSystem system;
    FrameContext context = frame(0.1f);
    system.update(context, scene);

    CHECK(system.last_advanced_count() == 1);
    CHECK(machine->active_state_name() == "run");
    // The animator keeps the clip it had; the machine did not rebind it.
    CHECK((animator->clip == idle));
}

TEST_CASE("the state machine is registered and reflects a graph property") {
    reflect::register_skeletal_animation_types();

    const reflect::TypeDesc* type = reflect::Registry::instance().find(SkeletalStateMachine::kTypeName);
    REQUIRE((type != nullptr));

    auto instance = type->factory();
    REQUIRE((instance != nullptr));

    const reflect::Property* graph = nullptr;
    for (const reflect::Property& property : type->properties) {
        if (property.name == "graph") {
            graph = &property;
            break;
        }
    }
    REQUIRE((graph != nullptr));
    CHECK(graph->value_kind == reflect::ValueKind::String);
    CHECK(graph->hint == reflect::Hint::Multiline);
}

TEST_CASE("the system skips a machine whose node has no animator") {
    auto skeleton = make_chain_skeleton();
    auto idle = make_clip(skeleton, "idle", 1.0f);

    Scene scene;
    Node* node = scene.create_node("rig-only");
    auto* machine = node->add_component<SkeletalStateMachine>();
    REQUIRE((machine != nullptr));
    machine->states = {state("idle", idle)};
    machine->initial_state = "idle";

    SkeletalStateMachineSystem system;
    FrameContext context = frame(0.1f);
    // No animator on the node: the machine still advances and reports it, and
    // the system does not crash trying to bind onto a missing component.
    system.update(context, scene);
    CHECK(system.last_advanced_count() == 1);
    CHECK(machine->active_state_name() == "idle");
}
