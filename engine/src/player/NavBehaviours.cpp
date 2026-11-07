// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/player/NavBehaviours.h"

#include "fe/core/Math.h"
#include "fe/nav/NavAgent.h"
#include "fe/reflect/PropertyBuilders.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/systems/FrameContext.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace fe::player {
namespace {

// Splits a comma-separated list into trimmed names, dropping empty entries. A
// route authored as "A, B, ,C" yields {"A", "B", "C"}, so a trailing comma or a
// stray space is not a broken route.
[[nodiscard]] std::vector<std::string> split_names(const std::string& text) {
    std::vector<std::string> names;
    std::size_t start = 0;
    while (start <= text.size()) {
        const std::size_t comma = text.find(',', start);
        const std::size_t end = comma == std::string::npos ? text.size() : comma;
        std::size_t first = start;
        std::size_t last = end;
        while (first < last && std::isspace(static_cast<unsigned char>(text[first])) != 0) {
            ++first;
        }
        while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1])) != 0) {
            --last;
        }
        if (last > first) {
            names.push_back(text.substr(first, last - first));
        }
        if (comma == std::string::npos) {
            break;
        }
        start = comma + 1;
    }
    return names;
}

// Sets an agent's goal to a node, clearing the static position so the two cannot
// disagree. The nav system resolves a goal node each frame, so a moving target is
// followed without the behaviour holding a stale position.
void set_goal_node(nav::NavAgent3D& agent, const Uuid& id) {
    agent.goal_node = id;
    agent.has_goal = true;
}

// Sets an agent's goal to a world position, clearing the node so the static goal
// is the one the system reads.
void set_goal_position(nav::NavAgent3D& agent, const Vec3& position) {
    agent.goal_node = Uuid{};
    agent.goal = position;
    agent.has_goal = true;
}

} // namespace

// --- PatrolRoute3D ---------------------------------------------------------

Error PatrolRoute3D::attach(Node& node) {
    // The route is resolved lazily on the first update, so a scene that binds the
    // behaviour before its waypoint nodes are loaded still connects.
    (void)node;
    resolved_ = false;
    current_ = 0;
    wait_remaining_ = 0.0f;
    waiting_ = false;
    return Error::none();
}

void PatrolRoute3D::resolve_route(const Node& owner) {
    const Scene* scene = owner.scene();
    names_ = split_names(waypoints);
    ids_.clear();
    ids_.reserve(names_.size());
    for (const std::string& name : names_) {
        const Node* waypoint = scene != nullptr ? scene->find_node_by_name(name) : nullptr;
        ids_.push_back(waypoint != nullptr ? waypoint->id() : Uuid{});
    }
    // Drop waypoints that did not resolve, so a partially-built scene patrols the
    // posts that exist rather than stalling on a missing one.
    ids_.erase(std::remove_if(ids_.begin(), ids_.end(), [](const Uuid& id) { return id.is_nil(); }), ids_.end());
    resolved_ = true;
    if (current_ >= ids_.size()) {
        current_ = 0;
    }
}

void PatrolRoute3D::advance() {
    if (ids_.empty()) {
        return;
    }
    if (current_ + 1 < ids_.size()) {
        ++current_;
    } else if (loop) {
        current_ = 0;
    }
    waiting_ = false;
    wait_remaining_ = 0.0f;
}

Error PatrolRoute3D::update(const FrameContext& context) {
    Node* owner = node();
    if (owner == nullptr) {
        return Error::none();
    }
    if (!resolved_) {
        resolve_route(*owner);
    }
    if (ids_.empty()) {
        return Error::none();
    }
    auto* agent = owner->get_component<nav::NavAgent3D>();
    if (agent == nullptr) {
        return Error::none();
    }

    Scene* scene = owner->scene();
    Node* waypoint = scene != nullptr ? scene->find_node(ids_[current_]) : nullptr;
    if (waypoint == nullptr) {
        // The current post is gone (a scene edit); step to the next one.
        advance();
        return Error::none();
    }

    if (waiting_) {
        wait_remaining_ -= context.delta_time;
        if (wait_remaining_ <= 0.0f) {
            advance();
        }
    } else {
        // Arrive when the agent's node is within the agent's own arrive radius of
        // the post, so the patrol and the pathfinder agree on what "there" means.
        const Vec3 here = owner->transform().position;
        const Vec3 there = waypoint->transform().position;
        if ((there - here).length() <= std::max(agent->arrive_radius, 0.01f)) {
            if (wait_time > 0.0f) {
                waiting_ = true;
                wait_remaining_ = wait_time;
            } else {
                advance();
            }
        }
    }

    // Point the agent at the current post after the step above, so the goal and
    // the current index never disagree. Setting it every frame also lets a chase
    // behaviour on the same node override it and the patrol take back over the
    // frame after.
    set_goal_node(*agent, ids_[current_]);
    return Error::none();
}

// --- ChaseTarget3D ---------------------------------------------------------

Error ChaseTarget3D::attach(Node& node) {
    (void)node;
    resolved_ = false;
    engaged_ = false;
    return Error::none();
}

Error ChaseTarget3D::update(const FrameContext& context) {
    (void)context;
    Node* owner = node();
    if (owner == nullptr || target.empty()) {
        return Error::none();
    }
    Scene* scene = owner->scene();
    if (scene == nullptr) {
        return Error::none();
    }

    Node* followed = resolved_ ? scene->find_node(target_id_) : nullptr;
    if (followed == nullptr) {
        followed = scene->find_node_by_name(target);
        resolved_ = followed != nullptr;
        target_id_ = followed != nullptr ? followed->id() : Uuid{};
    }
    if (followed == nullptr) {
        engaged_ = false;
        return Error::none();
    }

    auto* agent = owner->get_component<nav::NavAgent3D>();
    if (agent == nullptr) {
        return Error::none();
    }

    const Vec3 here = owner->transform().position;
    const Vec3 there = followed->transform().position;
    const f32 distance = (there - here).length();

    // Hysteresis: start inside `sight_radius`, keep going until `lose_radius`. A
    // target hovering between the two keeps the current state rather than
    // flickering the goal on and off.
    if (engaged_) {
        if (distance > lose_radius) {
            engaged_ = false;
        }
    } else if (distance <= sight_radius) {
        engaged_ = true;
    }

    if (!engaged_) {
        // Leave whatever a patrol behaviour set; do not clobber it.
        return Error::none();
    }

    if (flee) {
        // Aim for a point `flee_distance` from the target, along the line from the
        // target to this node, so the agent runs directly away. A zero-length
        // direction (the agent exactly on the target) falls back to +X so the
        // goal is defined rather than NaN.
        Vec3 away = here - there;
        const f32 length = away.length();
        away = length > 1e-4f ? away / length : Vec3{1.0f, 0.0f, 0.0f};
        set_goal_position(*agent, there + away * flee_distance);
    } else {
        set_goal_node(*agent, followed->id());
    }
    return Error::none();
}

// --- Registration ----------------------------------------------------------

void register_nav_behaviours(ScriptRegistry& registry) {
    {
        PatrolRoute3D defaults;
        ScriptDesc desc;
        desc.name = "PatrolRoute3D";
        desc.factory = [] { return std::unique_ptr<Script>(new PatrolRoute3D()); };
        desc.properties.push_back(reflect::make_scalar<PatrolRoute3D, std::string>(
            "waypoints", &PatrolRoute3D::waypoints, reflect::ValueKind::String, &defaults));
        desc.properties.push_back(reflect::make_scalar<PatrolRoute3D, f32>("wait_time", &PatrolRoute3D::wait_time,
                                                                           reflect::ValueKind::Float, &defaults));
        desc.properties.push_back(reflect::make_scalar<PatrolRoute3D, bool>("loop", &PatrolRoute3D::loop,
                                                                            reflect::ValueKind::Bool, &defaults));
        registry.register_desc(std::move(desc));
    }
    {
        ChaseTarget3D defaults;
        ScriptDesc desc;
        desc.name = "ChaseTarget3D";
        desc.factory = [] { return std::unique_ptr<Script>(new ChaseTarget3D()); };
        desc.properties.push_back(reflect::make_scalar<ChaseTarget3D, std::string>(
            "target", &ChaseTarget3D::target, reflect::ValueKind::String, &defaults));
        desc.properties.push_back(reflect::make_scalar<ChaseTarget3D, f32>("sight_radius", &ChaseTarget3D::sight_radius,
                                                                           reflect::ValueKind::Float, &defaults));
        desc.properties.push_back(reflect::make_scalar<ChaseTarget3D, f32>("lose_radius", &ChaseTarget3D::lose_radius,
                                                                           reflect::ValueKind::Float, &defaults));
        desc.properties.push_back(reflect::make_scalar<ChaseTarget3D, bool>("flee", &ChaseTarget3D::flee,
                                                                            reflect::ValueKind::Bool, &defaults));
        desc.properties.push_back(reflect::make_scalar<ChaseTarget3D, f32>(
            "flee_distance", &ChaseTarget3D::flee_distance, reflect::ValueKind::Float, &defaults));
        registry.register_desc(std::move(desc));
    }
}

} // namespace fe::player
