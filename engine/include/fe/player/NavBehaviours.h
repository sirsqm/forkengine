// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/script/Script.h"
#include "fe/script/ScriptRegistry.h"

#include <string>
#include <string_view>
#include <vector>

namespace fe {

// The AI behaviours that drive a navigation agent.
//
// These are the "AI" half of ADR-0156: `nav` finds the route, and these decide
// *where* to go. They are engine-shipped Scripts (like the behaviours in
// BuiltinBehaviours.h) rather than gameplay-module code, for the reason ADR-0078
// gives: a browser has no dynamic loader, so a behaviour a web game needs must be
// compiled into the engine. They read and write a `NavAgent2D`/`NavAgent3D` on
// their own node and never touch a key, so the same code runs on every target.
//
// The behaviours are deliberately small and composable. A guard that patrols and
// then chases the player is `PatrolRoute3D` plus `ChaseTarget3D` on one node, in
// that order: the patrol sets a goal while the target is out of sight, the chase
// overrides it when the target comes close. Neither knows about the other; both
// only set the agent's goal.
namespace player {

// Walks its node's navigation agent around a route of named waypoint nodes.
//
// The route is named, not referenced: `waypoints` is a comma-separated list of
// node names ("GuardPost1, GuardPost2, GuardPost3"). That keeps the route in the
// scene file, editable in the inspector as one string, and independent of node
// order — the same choice FollowCamera3D makes for its target. A name that does
// not resolve is skipped, so a partially-built scene still patrols what it has.
//
// The behaviour waits `wait_time` seconds at each waypoint before moving on, so a
// guard pauses at its post. When `loop` is false the route is walked once and the
// agent stops at the last waypoint.
class PatrolRoute3D final : public Script {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "PatrolRoute3D"; }

    [[nodiscard]] Error attach(Node& node) override;
    [[nodiscard]] Error update(const FrameContext& context) override;

    // The waypoint node names, comma-separated. Spaces around a name are ignored,
    // so "A, B" and "A,B" are the same route.
    std::string waypoints = "Waypoint1,Waypoint2";
    // Seconds to pause at each waypoint before moving to the next.
    f32 wait_time = 1.0f;
    // Whether to start over after the last waypoint.
    bool loop = true;

    // The index of the waypoint being walked to. Read by a test.
    [[nodiscard]] usize current_waypoint() const noexcept { return current_; }

private:
    void resolve_route(const Node& owner);
    void advance();

    std::vector<std::string> names_;
    std::vector<Uuid> ids_;
    bool resolved_ = false;
    usize current_ = 0;
    f32 wait_remaining_ = 0.0f;
    bool waiting_ = false;
};

// Sends its node's navigation agent after (or away from) a named target node.
//
// The target is named for the same reason FollowCamera3D's is. The behaviour
// resolves the name once and keeps the id, re-resolving if the node goes away, so
// a scene reload reconnects rather than holding a stale pointer.
//
// The sight and lose radii are separate so the behaviour does not flicker: a
// target inside `sight_radius` is chased, and the chase continues until the
// target is beyond `lose_radius`. `flee` inverts the goal, so the agent runs to a
// point `flee_distance` away from the target instead of to it — a fleeing
// creature, or a "keep away" drone. When the target is not in play the goal is
// cleared, so a patrol behaviour sharing the node is free to set one.
class ChaseTarget3D final : public Script {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "ChaseTarget3D"; }

    [[nodiscard]] Error attach(Node& node) override;
    [[nodiscard]] Error update(const FrameContext& context) override;

    // The node to chase (or flee), by name.
    std::string target = "Player";
    // How close (world units) the target must be to start the chase.
    f32 sight_radius = 12.0f;
    // How far the target must get before the chase is abandoned. Kept larger than
    // `sight_radius` so a target hovering at the edge does not start and stop the
    // chase every frame.
    f32 lose_radius = 16.0f;
    // Whether to run away from the target instead of towards it.
    bool flee = false;
    // How far from the target a fleeing agent aims, in world units.
    f32 flee_distance = 15.0f;

    // Whether the target was in play at the end of the last update. Read by a
    // test and by a game that wants to react (play a sound, switch an animation).
    [[nodiscard]] bool engaged() const noexcept { return engaged_; }

private:
    Uuid target_id_{};
    bool resolved_ = false;
    bool engaged_ = false;
};

// Registers the AI behaviours into `registry`. Called by
// `register_builtin_behaviours`, so a game that registers the built-ins gets
// these too.
void register_nav_behaviours(ScriptRegistry& registry);

} // namespace player
} // namespace fe
