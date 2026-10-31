// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/anim/AnimationClip.h"
#include "fe/core/Types.h"
#include "fe/scene/Component.h"

#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace fe {
class ResourceManager;
}

namespace fe::anim {

class Animator;

// A small animation state machine for a node's skeletal `Animator`.
//
// This is the skeletal counterpart of `AnimationStateMachine` (ADR-0059), which
// does the same job for a `SpriteAnimator`. The sprite machine plays a *cell* of
// a sheet; a character made of a skinned mesh plays a whole `AnimationClip`, so
// the state names a clip rather than a frame range. Everything else -- named
// states, transitions gated on a finish and on triggers, an initial state, and
// the split where the component is the authored graph and a system is the
// per-frame step -- is the same shape, deliberately, so a designer who has
// authored one has authored both.
//
// Why not a Script (ADR-0044)? The same reason the sprite machine is not one: a
// state machine is a *structured* behaviour an editor can list and draw, and a
// designer can author without writing code. Keeping it a component is what makes
// it serializable and inspectable like every other piece of authored data.
//
// A state names its clip by resource reference. At runtime the state holds the
// resolved `AnimationClip`; the graph serializes the clip's *key* (the same
// `clip->path()` form `Animator` stores) and `resolve_clips` loads it back
// through the `ResourceManager`, which is how every other resource reference in
// the engine survives a save/load cycle. A state whose clip is null is legal: it
// holds the animator's pose, which is what a "rest" or "dead" state wants.
//
// The machine owns its own clock, exactly as the sprite machine does, so a state
// machine is fully testable with no scene and no animator, and so a state with
// `drive_animator` off still advances and transitions. The clock advances with
// the frame's delta at the animator's default rate; the finish gate compares it
// to the active state's clip duration. See ADR-0123.

// One state of a skeletal animation state machine.
struct SkeletalState {
    // The state's name, the identity a transition's `from`/`to` names it by.
    std::string name;

    // The clip the state plays. Null is legal and means "hold the current pose".
    std::shared_ptr<AnimationClip> clip;

    // The clip's reference key as it is serialized and resolved. Kept beside the
    // resolved pointer for the same reason `Animator` keeps one: a save writes
    // the key, a load resolves it, and the two never drift because the key is
    // written from the pointer when one is bound.
    std::string clip_key;

    // How the clip plays. A `Once` state is the one that can report finished, so
    // it is the one a `require_finished` transition waits on; a looping state
    // never finishes and is left by a trigger instead.
    LoopMode loop = LoopMode::Loop;
};

// A directed transition between two states.
//
// `from` and `to` name states. An empty `from` is the "any state" source: the
// transition is considered from every state, which is how an interrupt ("hit
// cancels anything") is authored without repeating the edge on each state.
//
// Conditions are combined with AND. A transition with no conditions fires as
// soon as its source state is active and it is not `require_finished` (or the
// source has finished), which is a plain unconditional edge.
struct SkeletalTransition {
    std::string from; // empty means "any state"
    std::string to;
    bool require_finished = false;       // fire only once the source state's clip finished
    std::vector<std::string> conditions; // triggers that must all be set; combined with AND
};

// The state machine. See the file comment for the model.
class SkeletalStateMachine final : public Component {
public:
    static constexpr const char* kTypeName = "SkeletalStateMachine";

    [[nodiscard]] const char* type_name() const noexcept override { return kTypeName; }

    // The authored graph.
    std::vector<SkeletalState> states;
    std::vector<SkeletalTransition> transitions;

    // The state playback starts in, by name. Empty means "the first state".
    std::string initial_state;

    // When true the machine does not advance. The active state and its elapsed
    // time are kept, so pausing and resuming is exact.
    bool playing = true;

    // When true the machine drives the node's `Animator`: when a transition
    // enters a state it binds that state's clip and loop mode onto the animator,
    // which is how the machine changes what plays. When false the machine still
    // advances and transitions, but the animator keeps its own clip.
    bool drive_animator = true;

    // --- Authored / queried graph -----------------------------------------

    // The index of the named state, or -1.
    [[nodiscard]] i32 find_state(std::string_view name) const noexcept;

    // The index of the machine's active state, or -1 when the graph is empty.
    [[nodiscard]] i32 active_state() const noexcept { return active_; }

    // The name of the active state, or "" when the graph is empty.
    [[nodiscard]] std::string_view active_state_name() const noexcept;

    // The clip the active state plays, or null when the graph is empty or the
    // state holds its pose. What the system binds onto the animator.
    [[nodiscard]] const std::shared_ptr<AnimationClip>& active_clip() const noexcept;

    // The active state's loop mode, or `LoopMode::Loop` when the graph is empty.
    [[nodiscard]] LoopMode active_loop() const noexcept;

    // Seconds accumulated in the active state. Exposed so a test can assert the
    // clock rather than infer it from a transition.
    [[nodiscard]] f32 elapsed() const noexcept { return elapsed_; }

    // True once the active state's clip has played out. A looping state never
    // reports finished; a state with no clip is finished at once, because there
    // is nothing to wait for.
    [[nodiscard]] bool state_finished() const noexcept { return finished_; }

    // --- Playback ----------------------------------------------------------

    // Enters the named state, restarting its clip and clock. Returns false (and
    // changes nothing) when no state has that name. A transition is evaluated
    // before this, so the usual caller is the system, not gameplay.
    bool play(std::string_view state_name);

    // Sets a trigger true. Triggers are edge events consumed by the transition
    // that fires on them, so a trigger fires at most one transition per frame.
    void set_trigger(std::string_view name);

    // Clears a trigger without firing anything.
    void clear_trigger(std::string_view name);

    [[nodiscard]] bool has_trigger(std::string_view name) const noexcept;

    // Advances the machine by `delta_time` seconds and applies the first
    // transition whose conditions hold. Idempotent when the graph is empty or
    // the machine is paused. Called by SkeletalStateMachineSystem once per frame;
    // exposed for tests.
    void advance(f32 delta_time);

    // Returns true once after the machine has entered a state that the animator
    // has not been told about yet -- the initial state on the first advance, and
    // every transition after. The system calls this to know when to drive the
    // animator, so a machine that changes state only when it should. Consuming it
    // is what makes the bind happen exactly once per entry rather than every
    // frame.
    bool take_clip_request() noexcept;

    // --- Resource resolution ----------------------------------------------

    // Loads every state's clip through `manager` from its `clip_key`, and
    // returns how many were resolved this call. A state that already holds a clip
    // is skipped, so a second call is cheap and idempotent. A key that does not
    // load is left in place and retried on the next call, because an asset may
    // appear later (a hot reload, a pack mounted after the scene). Called by the
    // system when the frame carries a manager.
    usize resolve_clips(ResourceManager& manager);

    // --- Persistence -------------------------------------------------------

    // Serializes the graph to the compact string the reflection String property
    // stores, and parses it back. The format is the engine's own and is stable
    // across the component's lifetime; a malformed string leaves the machine
    // empty and returns false rather than throwing.
    [[nodiscard]] std::string to_json() const;
    bool from_json(std::string_view text);

private:
    // Enters state `index` (already validated), restarts its clock, and raises
    // the clip request.
    void enter(i32 index);

    // The first transition that fires from the active state, or -1. Does not
    // consume triggers; the caller does, once it has chosen the edge.
    [[nodiscard]] i32 next_transition() const;

    i32 active_ = -1;
    f32 elapsed_ = 0.0f; // seconds accumulated in the active state
    bool finished_ = false;
    bool clip_request_ = false;
    std::vector<std::string> triggers_;
};

} // namespace fe::anim
