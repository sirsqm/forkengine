// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/anim/SkeletalStateMachine.h"

#include "fe/resources/ResourceManager.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>

namespace fe::anim {

i32 SkeletalStateMachine::find_state(std::string_view name) const noexcept {
    for (usize i = 0; i < states.size(); ++i) {
        if (states[i].name == name) {
            return static_cast<i32>(i);
        }
    }
    return -1;
}

std::string_view SkeletalStateMachine::active_state_name() const noexcept {
    if (active_ < 0 || static_cast<usize>(active_) >= states.size()) {
        return {};
    }
    return states[static_cast<usize>(active_)].name;
}

const std::shared_ptr<AnimationClip>& SkeletalStateMachine::active_clip() const noexcept {
    static const std::shared_ptr<AnimationClip> kNone;
    if (active_ < 0 || static_cast<usize>(active_) >= states.size()) {
        return kNone;
    }
    return states[static_cast<usize>(active_)].clip;
}

LoopMode SkeletalStateMachine::active_loop() const noexcept {
    if (active_ < 0 || static_cast<usize>(active_) >= states.size()) {
        return LoopMode::Loop;
    }
    return states[static_cast<usize>(active_)].loop;
}

bool SkeletalStateMachine::play(std::string_view state_name) {
    const i32 index = find_state(state_name);
    if (index < 0) {
        return false;
    }
    enter(index);
    return true;
}

void SkeletalStateMachine::set_trigger(std::string_view name) {
    for (const std::string& trigger : triggers_) {
        if (trigger == name) {
            return;
        }
    }
    triggers_.emplace_back(name);
}

void SkeletalStateMachine::clear_trigger(std::string_view name) {
    triggers_.erase(std::remove(triggers_.begin(), triggers_.end(), name), triggers_.end());
}

bool SkeletalStateMachine::has_trigger(std::string_view name) const noexcept {
    return std::find(triggers_.begin(), triggers_.end(), name) != triggers_.end();
}

void SkeletalStateMachine::enter(i32 index) {
    active_ = index;
    elapsed_ = 0.0f;
    finished_ = false;
    // A fresh state is a fresh clip for the animator to play, so the system is
    // told to drive it exactly once on the frame this entry is seen.
    clip_request_ = true;
}

bool SkeletalStateMachine::take_clip_request() noexcept {
    const bool requested = clip_request_;
    clip_request_ = false;
    return requested;
}

i32 SkeletalStateMachine::next_transition() const {
    if (active_ < 0 || static_cast<usize>(active_) >= states.size()) {
        return -1;
    }
    const std::string& current = states[static_cast<usize>(active_)].name;

    for (usize i = 0; i < transitions.size(); ++i) {
        const SkeletalTransition& transition = transitions[i];

        // A transition is eligible when its source is the active state, or when
        // it is an "any state" edge (empty `from`).
        const bool source_matches = transition.from.empty() || transition.from == current;
        if (!source_matches) {
            continue;
        }
        // A self-edge with no trigger would fire forever; require a trigger to
        // have been set for a transition back to the same state.
        if (transition.to == current && transition.conditions.empty()) {
            continue;
        }
        if (transition.require_finished && !finished_) {
            continue;
        }
        const bool conditions_hold = std::all_of(transition.conditions.begin(), transition.conditions.end(),
                                                 [this](const std::string& name) { return has_trigger(name); });
        if (conditions_hold) {
            return static_cast<i32>(i);
        }
    }
    return -1;
}

void SkeletalStateMachine::advance(f32 delta_time) {
    if (states.empty()) {
        active_ = -1;
        finished_ = false;
        return;
    }

    // Enter the initial state on the first advance (or after a graph edit that
    // left no active state), so a machine works with no explicit play() call.
    if (active_ < 0 || static_cast<usize>(active_) >= states.size()) {
        const i32 initial = find_state(initial_state);
        enter(initial >= 0 ? initial : 0);
    }

    if (!playing) {
        return;
    }

    const SkeletalState& state = states[static_cast<usize>(active_)];

    // The clock is the clip's own playback position. A state with no clip has
    // nothing to time, so it reports finished at once and the machine relies on
    // triggers to leave it.
    elapsed_ += delta_time;

    if (state.clip == nullptr) {
        finished_ = true;
    } else {
        const f32 duration = state.clip->duration();
        if (duration <= 0.0f) {
            // A zero-length clip is a single pose: it is "finished" immediately,
            // which lets a one-shot rest state hand off without a timer.
            finished_ = true;
        } else if (state.loop == LoopMode::Loop) {
            // A looping state never finishes; it is left by a trigger.
            finished_ = false;
            if (elapsed_ >= duration) {
                elapsed_ = std::fmod(elapsed_, duration);
            }
        } else {
            finished_ = elapsed_ >= duration;
            if (finished_) {
                elapsed_ = duration;
            }
        }
    }

    // Apply the first transition that fires. A transition's conditions are
    // consumed when it fires, so a trigger drives at most one edge.
    const i32 chosen = next_transition();
    if (chosen >= 0) {
        const SkeletalTransition& transition = transitions[static_cast<usize>(chosen)];
        for (const std::string& condition : transition.conditions) {
            clear_trigger(condition);
        }
        const i32 target = find_state(transition.to);
        if (target >= 0) {
            enter(target);
        }
    }
}

usize SkeletalStateMachine::resolve_clips(ResourceManager& manager) {
    usize resolved = 0;
    for (SkeletalState& state : states) {
        if (state.clip != nullptr || state.clip_key.empty()) {
            continue;
        }
        auto loaded = manager.load(state.clip_key);
        if (!loaded) {
            // Left in place and retried: the asset may not be mounted yet.
            continue;
        }
        auto clip = std::dynamic_pointer_cast<AnimationClip>(loaded.value());
        if (clip == nullptr) {
            continue;
        }
        state.clip = std::move(clip);
        ++resolved;
    }
    return resolved;
}

std::string SkeletalStateMachine::to_json() const {
    nlohmann::json document;
    document["initial_state"] = initial_state;
    document["playing"] = playing;
    document["drive_animator"] = drive_animator;

    nlohmann::json states_json = nlohmann::json::array();
    for (const SkeletalState& state : states) {
        // The clip travels as its reference key. A state whose clip was bound
        // directly rather than loaded writes the key the caller set; a state
        // resolved through the manager writes the key it was resolved from.
        std::string key = state.clip_key;
        if (key.empty() && state.clip != nullptr) {
            key = state.clip->path();
        }
        states_json.push_back(nlohmann::json{
            {"name", state.name},
            {"clip", key},
            {"loop", static_cast<i32>(state.loop)},
        });
    }
    document["states"] = std::move(states_json);

    nlohmann::json transitions_json = nlohmann::json::array();
    for (const SkeletalTransition& transition : transitions) {
        transitions_json.push_back(nlohmann::json{
            {"from", transition.from},
            {"to", transition.to},
            {"require_finished", transition.require_finished},
            {"conditions", transition.conditions},
        });
    }
    document["transitions"] = std::move(transitions_json);

    return document.dump();
}

bool SkeletalStateMachine::from_json(std::string_view text) {
    nlohmann::json document;
    try {
        document = nlohmann::json::parse(text);
    } catch (const nlohmann::json::exception&) {
        return false;
    }
    if (!document.is_object()) {
        return false;
    }

    // Parse into temporaries so a malformed document leaves the machine
    // unchanged rather than half-applied.
    std::vector<SkeletalState> new_states;
    if (document.contains("states") && document.at("states").is_array()) {
        for (const auto& entry : document.at("states")) {
            if (!entry.is_object()) {
                continue;
            }
            SkeletalState state;
            state.name = entry.value("name", std::string{});
            state.clip_key = entry.value("clip", std::string{});
            const i32 loop = entry.value("loop", static_cast<i32>(LoopMode::Loop));
            if (loop <= static_cast<i32>(LoopMode::Once)) {
                state.loop = LoopMode::Once;
            } else if (loop >= static_cast<i32>(LoopMode::PingPong)) {
                state.loop = LoopMode::PingPong;
            } else {
                state.loop = LoopMode::Loop;
            }
            if (!state.name.empty()) {
                new_states.push_back(std::move(state));
            }
        }
    }

    std::vector<SkeletalTransition> new_transitions;
    if (document.contains("transitions") && document.at("transitions").is_array()) {
        for (const auto& entry : document.at("transitions")) {
            if (!entry.is_object()) {
                continue;
            }
            SkeletalTransition transition;
            transition.from = entry.value("from", std::string{});
            transition.to = entry.value("to", std::string{});
            transition.require_finished = entry.value("require_finished", false);
            if (entry.contains("conditions") && entry.at("conditions").is_array()) {
                transition.conditions = entry.at("conditions").get<std::vector<std::string>>();
            }
            if (!transition.to.empty()) {
                new_transitions.push_back(std::move(transition));
            }
        }
    }

    states = std::move(new_states);
    transitions = std::move(new_transitions);
    initial_state = document.value("initial_state", std::string{});
    playing = document.value("playing", true);
    drive_animator = document.value("drive_animator", true);

    // The graph changed, so any live playback state is stale; re-enter on the
    // next advance.
    active_ = -1;
    elapsed_ = 0.0f;
    finished_ = false;
    clip_request_ = false;
    triggers_.clear();
    return true;
}

} // namespace fe::anim
