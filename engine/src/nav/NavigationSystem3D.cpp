// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/nav/NavigationSystem3D.h"

#include "fe/nav/NavAgent.h"
#include "fe/nav/Path3D.h"
#include "fe/nav/Steering.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"

#include <algorithm>
#include <cmath>

namespace fe::nav {
namespace {

[[nodiscard]] Vec3 node_position_3d(const Node& node) noexcept {
    return transform_point(node.world_matrix(), Vec3{0.0f, 0.0f, 0.0f});
}

} // namespace

void NavigationSystem3D::rebuild_grid(Scene& scene) {
    grid_ = build_nav_grid_3d(scene, cell_size_, require_ground_);
    built_ = true;
    rebuild_requested_ = false;
}

void NavigationSystem3D::update(const FrameContext& context, Scene& scene) {
    agent_count_ = 0;

    if (!built_ || rebuild_requested_) {
        rebuild_grid(scene);
    }

    agents_.clear();
    scene.for_each_node([&](Node& node) {
        auto* agent = node.get_component<NavAgent3D>();
        if (agent == nullptr) {
            return;
        }
        AgentRef ref;
        ref.node = &node;
        ref.agent = agent;
        agents_.push_back(ref);
    });

    if (agents_.empty() || grid_.empty()) {
        return;
    }

    neighbour_positions_.clear();
    neighbour_positions_.reserve(agents_.size());
    for (const AgentRef& ref : agents_) {
        neighbour_positions_.push_back(node_position_3d(*ref.node));
    }

    for (usize i = 0; i < agents_.size(); ++i) {
        AgentRef& ref = agents_[i];
        NavAgent3D& agent = *ref.agent;
        ++agent_count_;

        const Vec3 position = neighbour_positions_[i];

        Vec3 goal = agent.goal;
        bool has_goal = agent.has_goal;
        if (!agent.goal_node.is_nil()) {
            if (const Node* target = scene.find_node(agent.goal_node); target != nullptr) {
                goal = node_position_3d(*target);
                has_goal = true;
            }
        }
        if (!has_goal) {
            agent.waypoints.clear();
            agent.next_waypoint = 0;
            agent.arrived = false;
            agent.desired_velocity = Vec3{};
            continue;
        }

        i32 goal_cell_x = 0;
        i32 goal_cell_y = 0;
        i32 goal_cell_z = 0;
        grid_.world_to_cell(goal, goal_cell_x, goal_cell_y, goal_cell_z);
        const bool goal_cell_changed = !ref.has_planned_cell || goal_cell_x != ref.goal_cell_x ||
                                       goal_cell_y != ref.goal_cell_y || goal_cell_z != ref.goal_cell_z;

        if (goal_cell_changed || agent.waypoints.empty()) {
            const Error path_error = find_path_3d(grid_, position, goal, scratch_path_);
            if (path_error.ok()) {
                smooth_path_3d(grid_, scratch_path_);
                agent.waypoints = scratch_path_;
                agent.next_waypoint = agent.waypoints.size() > 1 ? 1 : 0;
                agent.arrived = false;
            }
            ref.goal_cell_x = goal_cell_x;
            ref.goal_cell_y = goal_cell_y;
            ref.goal_cell_z = goal_cell_z;
            ref.has_planned_cell = true;
        }

        while (agent.next_waypoint < agent.waypoints.size() &&
               (agent.waypoints[agent.next_waypoint] - position).length() <= agent.waypoint_radius) {
            ++agent.next_waypoint;
        }

        if (agent.waypoints.empty() || agent.next_waypoint >= agent.waypoints.size()) {
            agent.arrived = (goal - position).length() <= std::max(agent.arrive_radius, agent.waypoint_radius);
            const Vec3 desired =
                agent.arrived ? Vec3{} : arrive_3d(position, goal, agent.speed, agent.speed, agent.arrive_radius);
            agent.desired_velocity = desired;
            if (agent.auto_move && desired.length() > 0.0f) {
                ref.node->transform().position += desired * context.delta_time;
            }
            continue;
        }

        const Vec3 waypoint = agent.waypoints[agent.next_waypoint];
        Vec3 desired = arrive_3d(position, waypoint, agent.speed, agent.speed, agent.waypoint_radius);

        if (agent.separation_weight > 0.0f) {
            std::vector<Vec3> neighbours;
            neighbours.reserve(agents_.size());
            for (usize j = 0; j < agents_.size(); ++j) {
                if (j == i) {
                    continue;
                }
                neighbours.push_back(neighbour_positions_[j] - position);
            }
            desired +=
                separation_3d(Vec3{}, neighbours, agent.separation_radius, agent.speed) * agent.separation_weight;
        }

        if (agent.avoidance_weight > 0.0f) {
            obstacle_points_.clear();
            const Vec3 forward = desired.normalized();
            const i32 probe = std::max(1, static_cast<i32>(std::ceil(agent.avoidance_radius / grid_.cell_size())));
            i32 cx = 0;
            i32 cy = 0;
            i32 cz = 0;
            grid_.world_to_cell(position, cx, cy, cz);
            for (i32 dz = -probe; dz <= probe; ++dz) {
                for (i32 dy = -probe; dy <= probe; ++dy) {
                    for (i32 dx = -probe; dx <= probe; ++dx) {
                        const i32 tx = cx + dx;
                        const i32 ty = cy + dy;
                        const i32 tz = cz + dz;
                        if (grid_.walkable(tx, ty, tz)) {
                            continue;
                        }
                        obstacle_points_.push_back(grid_.cell_center(tx, ty, tz) - position);
                    }
                }
            }
            desired +=
                avoidance_3d(forward, obstacle_points_, agent.avoidance_radius, agent.speed) * agent.avoidance_weight;
        }

        const f32 speed = desired.length();
        if (speed > agent.speed && speed > 1e-6f) {
            desired = desired / speed * agent.speed;
        }

        if (agent.acceleration > 0.0f) {
            const Vec3 delta = desired - agent.desired_velocity;
            const f32 max_delta = agent.acceleration * context.delta_time;
            agent.desired_velocity = delta.length() > max_delta && delta.length() > 1e-6f
                                         ? agent.desired_velocity + delta.normalized() * max_delta
                                         : desired;
        } else {
            agent.desired_velocity = desired;
        }

        if (agent.auto_move) {
            ref.node->transform().position += agent.desired_velocity * context.delta_time;
        }
    }
}

} // namespace fe::nav
