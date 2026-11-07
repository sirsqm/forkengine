// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/nav/NavigationSystem2D.h"

#include "fe/nav/NavAgent.h"
#include "fe/nav/Path2D.h"
#include "fe/nav/Steering.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/scene/Tilemap.h"

#include <algorithm>
#include <cmath>

namespace fe::nav {
namespace {

[[nodiscard]] Vec2 node_position_2d(const Node& node) noexcept {
    const Vec3 world = transform_point(node.world_matrix(), Vec3{0.0f, 0.0f, 0.0f});
    return Vec2{world.x, world.y};
}

} // namespace

const NavGrid2D* NavigationSystem2D::grid_for(const Uuid& tilemap_node) const noexcept {
    const auto it = grids_.find(tilemap_node);
    return it == grids_.end() ? nullptr : &it->second;
}

void NavigationSystem2D::rebuild_grids(Scene& scene) {
    grids_.clear();
    default_grid_node_ = Uuid{};
    scene.for_each_node([&](Node& node) {
        const auto* map = node.get_component<Tilemap>();
        if (map == nullptr || map->cell_count() == 0) {
            return;
        }
        if (default_grid_node_.is_nil()) {
            default_grid_node_ = node.id();
        }
        grids_.emplace(node.id(), build_nav_grid_2d(scene, node, *map, cell_size_, agent_radius_));
    });
    built_ = true;
    rebuild_requested_ = false;
}

void NavigationSystem2D::update(const FrameContext& context, Scene& scene) {
    agent_count_ = 0;

    if (!built_ || rebuild_requested_) {
        rebuild_grids(scene);
    }

    // Gather the agents and resolve each one's grid and goal. The flat list is
    // rebuilt each frame so a node that left the scene drops out.
    agents_.clear();
    scene.for_each_node([&](Node& node) {
        auto* agent = node.get_component<NavAgent2D>();
        if (agent == nullptr) {
            return;
        }
        const Uuid grid_id = agent->grid_node.is_nil() ? default_grid_node_ : agent->grid_node;
        const auto it = grids_.find(grid_id);
        if (it == grids_.end()) {
            return;
        }
        AgentRef ref;
        ref.node = &node;
        ref.agent = agent;
        ref.grid = &it->second;
        agents_.push_back(ref);
    });

    if (agents_.empty()) {
        return;
    }

    // Positions of every agent this frame, so separation has a neighbourhood to
    // read. Collected once rather than per agent.
    neighbour_positions_.clear();
    neighbour_positions_.reserve(agents_.size());
    for (const AgentRef& ref : agents_) {
        neighbour_positions_.push_back(node_position_2d(*ref.node));
    }

    for (usize i = 0; i < agents_.size(); ++i) {
        AgentRef& ref = agents_[i];
        NavAgent2D& agent = *ref.agent;
        const NavGrid2D& grid = *ref.grid;
        if (grid.empty()) {
            continue;
        }
        ++agent_count_;

        const Vec2 position = neighbour_positions_[i];

        // Resolve the goal: a goal node wins over a static position, and a
        // missing goal node falls back to `goal`.
        Vec2 goal = agent.goal;
        bool has_goal = agent.has_goal;
        if (!agent.goal_node.is_nil()) {
            if (const Node* target = scene.find_node(agent.goal_node); target != nullptr) {
                goal = node_position_2d(*target);
                has_goal = true;
            }
        }
        if (!has_goal) {
            agent.waypoints.clear();
            agent.next_waypoint = 0;
            agent.arrived = false;
            agent.desired_velocity = Vec2{};
            continue;
        }

        // Re-plan only when the goal moved to a different cell, so a chasing
        // agent does not run A* every frame.
        i32 goal_cell_x = 0;
        i32 goal_cell_y = 0;
        grid.world_to_cell(goal, goal_cell_x, goal_cell_y);
        const bool goal_cell_changed =
            !ref.has_planned_cell || goal_cell_x != ref.goal_cell_x || goal_cell_y != ref.goal_cell_y;

        if (goal_cell_changed || agent.waypoints.empty()) {
            const Error path_error = find_path_2d(grid, position, goal, scratch_path_);
            if (path_error.ok()) {
                smooth_path_2d(grid, scratch_path_);
                agent.waypoints = scratch_path_;
                agent.next_waypoint = agent.waypoints.size() > 1 ? 1 : 0;
                agent.arrived = false;
            }
            ref.goal_cell_x = goal_cell_x;
            ref.goal_cell_y = goal_cell_y;
            ref.has_planned_cell = true;
        }

        // Advance past waypoints already reached.
        while (agent.next_waypoint < agent.waypoints.size() &&
               (agent.waypoints[agent.next_waypoint] - position).length() <= agent.waypoint_radius) {
            ++agent.next_waypoint;
        }

        if (agent.waypoints.empty() || agent.next_waypoint >= agent.waypoints.size()) {
            agent.arrived = (goal - position).length() <= std::max(agent.arrive_radius, agent.waypoint_radius);
            // Fall back to a direct arrive when the path is exhausted but the
            // goal is still a little further (a smoothed path's last point can
            // sit a waypoint radius short).
            Vec2 desired =
                agent.arrived ? Vec2{} : arrive_2d(position, goal, agent.speed, agent.speed, agent.arrive_radius);
            agent.desired_velocity = desired;
            if (agent.auto_move && desired.length() > 0.0f) {
                ref.node->transform().position +=
                    Vec3{desired.x * context.delta_time, desired.y * context.delta_time, 0.0f};
            }
            continue;
        }

        // Steer towards the current waypoint, with separation and avoidance.
        const Vec2 waypoint = agent.waypoints[agent.next_waypoint];
        Vec2 desired = arrive_2d(position, waypoint, agent.speed, agent.speed, agent.waypoint_radius);

        if (agent.separation_weight > 0.0f) {
            // Every other agent, as an offset from this one.
            std::vector<Vec2> neighbours;
            neighbours.reserve(agents_.size());
            for (usize j = 0; j < agents_.size(); ++j) {
                if (j == i) {
                    continue;
                }
                neighbours.push_back(neighbour_positions_[j] - position);
            }
            desired +=
                separation_2d(Vec2{}, neighbours, agent.separation_radius, agent.speed) * agent.separation_weight;
        }

        if (agent.avoidance_weight > 0.0f) {
            // The blocked cells just ahead of the intended heading, as offsets.
            obstacle_points_.clear();
            const Vec2 forward = desired.normalized();
            const i32 probe = std::max(1, static_cast<i32>(std::ceil(agent.avoidance_radius / grid.cell_size())));
            i32 cx = 0;
            i32 cy = 0;
            grid.world_to_cell(position, cx, cy);
            for (i32 dy = -probe; dy <= probe; ++dy) {
                for (i32 dx = -probe; dx <= probe; ++dx) {
                    const i32 tx = cx + dx;
                    const i32 ty = cy + dy;
                    if (grid.walkable(tx, ty)) {
                        continue;
                    }
                    obstacle_points_.push_back(grid.cell_center(tx, ty) - position);
                }
            }
            desired +=
                avoidance_2d(forward, obstacle_points_, agent.avoidance_radius, agent.speed) * agent.avoidance_weight;
        }

        // Clamp to the agent's speed so the summed behaviours cannot exceed it.
        const f32 speed = desired.length();
        if (speed > agent.speed && speed > 1e-6f) {
            desired = desired / speed * agent.speed;
        }

        // Ease the published velocity towards the desired one, so a direction
        // change is a turn rather than a snap.
        if (agent.acceleration > 0.0f) {
            const Vec2 delta = desired - agent.desired_velocity;
            const f32 max_delta = agent.acceleration * context.delta_time;
            agent.desired_velocity = delta.length() > max_delta && delta.length() > 1e-6f
                                         ? agent.desired_velocity + delta.normalized() * max_delta
                                         : desired;
        } else {
            agent.desired_velocity = desired;
        }

        if (agent.auto_move) {
            ref.node->transform().position += Vec3{agent.desired_velocity.x * context.delta_time,
                                                   agent.desired_velocity.y * context.delta_time, 0.0f};
        }
    }
}

} // namespace fe::nav
