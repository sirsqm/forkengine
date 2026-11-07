// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Math.h"
#include "fe/core/Types.h"
#include "fe/scene/Component.h"

#include <string>
#include <vector>

namespace fe::nav {

// A 2D navigation agent: a node that walks the level's nav grid towards a goal.
//
// The agent is a component, and the system that advances it (`NavigationSystem2D`)
// owns the grid (ADR-0156). An agent names the goal in one of two ways, so both
// an authored scene and a script can drive it:
//
//   * `goal_node`, a node id, resolved through the scene each frame, so a moving
//     target (the player) is followed without the agent holding a stale pointer;
//   * `goal`, a world position, used when `goal_node` is nil.
//
// A goal node wins when it is set and resolves. The agent re-plans only when the
// goal cell changes, so a chasing agent does not run A* every frame.
//
// The agent moves its own node directly (a kinematic mover, the same shape
// `CharacterController3D` uses), publishing `desired_velocity` each frame. A game
// that wants physics reads the published velocity and drives a rigid body
// instead; a game that does not gets the movement for free. The steering weights
// are reflected fields, so an agent round-trips through the serializer and shows
// in the inspector.
class NavAgent2D final : public Component {
public:
    static constexpr const char* kTypeName = "NavAgent2D";

    [[nodiscard]] const char* type_name() const noexcept override { return kTypeName; }

    // The grid's host node: the node carrying the `Tilemap` the grid was built
    // from. When nil the system uses the scene's first tilemap, so a one-level
    // scene needs no binding. A scene with several tilemaps binds each agent to
    // the right one by id.
    Uuid grid_node{};

    // The node to walk towards, or nil to use `goal`.
    Uuid goal_node{};

    // A world-space goal, used when `goal_node` is nil.
    Vec2 goal{};

    // Whether the agent has a goal at all. Off by default, so an agent placed in
    // a scene does nothing until a goal is set (by a designer or a script).
    bool has_goal = false;

    // Whether the agent moves its own node. On by default; turn it off to have
    // the system publish `desired_velocity` only, so a script or a physics body
    // performs the move.
    bool auto_move = true;

    // Walk speed, in world units per second.
    f32 speed = 3.0f;

    // How sharply the agent turns towards its desired velocity, in units per
    // second squared. A larger value turns faster; zero snaps immediately.
    f32 acceleration = 20.0f;

    // How close (world units) the agent must be to a waypoint to advance to the
    // next one.
    f32 waypoint_radius = 0.25f;

    // How close (world units) to the final goal counts as arrived; the agent
    // stops and clears its path.
    f32 arrive_radius = 0.25f;

    // Steering weights. `separation` keeps agents from stacking; `avoidance`
    // steers around walls ahead. Both default to a small positive value so a
    // crowd behaves without tuning; zero disables the behaviour.
    f32 separation_weight = 0.5f;
    f32 avoidance_weight = 0.5f;
    f32 separation_radius = 1.0f;
    f32 avoidance_radius = 1.0f;

    // --- Runtime state (not serialized) ------------------------------------
    // The planned path, as world-space waypoints. Owned by the agent so a
    // consumer (a debug draw, a test) can read it, but rebuilt by the system.
    // `next_waypoint` is the index the agent is walking towards.
    //
    // These are plain public fields the system writes. They are deliberately not
    // properties, so a saved scene does not reload mid-path.
    std::vector<Vec2> waypoints;
    usize next_waypoint = 0;
    bool arrived = false;

    // The velocity the system computed this frame. Read by a physics-driven
    // caller; ignored by the default mover.
    Vec2 desired_velocity{};
};

// The 3D counterpart of NavAgent2D. The goal and grid are 3D, the agent walks the
// voxel grid, and its node moves in three dimensions (a flying or swimming agent)
// or is projected to the ground by the grid's `require_ground` build.
class NavAgent3D final : public Component {
public:
    static constexpr const char* kTypeName = "NavAgent3D";

    [[nodiscard]] const char* type_name() const noexcept override { return kTypeName; }

    Uuid goal_node{};
    Vec3 goal{};
    bool has_goal = false;
    bool auto_move = true;

    f32 speed = 4.0f;
    f32 acceleration = 20.0f;
    f32 waypoint_radius = 0.5f;
    f32 arrive_radius = 0.5f;

    f32 separation_weight = 0.5f;
    f32 avoidance_weight = 0.5f;
    f32 separation_radius = 2.0f;
    f32 avoidance_radius = 2.0f;

    std::vector<Vec3> waypoints;
    usize next_waypoint = 0;
    bool arrived = false;

    Vec3 desired_velocity{};
};

} // namespace fe::nav
