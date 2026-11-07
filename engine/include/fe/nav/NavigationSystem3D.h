// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/nav/NavGrid3D.h"
#include "fe/systems/System.h"

#include <string_view>
#include <vector>

namespace fe {
class Node;
namespace nav {

class NavAgent3D;

// Advances every `NavAgent3D` in the scene along the level's 3D nav grid.
//
// The 3D counterpart of `NavigationSystem2D`. It builds one voxel grid from the
// scene's static `Collider3D` shapes (ADR-0156) and advances every agent along
// it. Unlike the 2D system there is no per-tilemap binding: a 3D level is one
// collider set, so there is one grid. `require_ground` on the build decides
// whether agents walk the floor or fly through free space.
//
// Ordering: add this system after the physics system and before the renderer.
class NavigationSystem3D final : public System {
public:
    NavigationSystem3D() = default;

    [[nodiscard]] std::string_view name() const noexcept override { return "NavigationSystem3D"; }

    void update(const FrameContext& context, Scene& scene) override;

    // Rebuilds the grid on the next update. Call this when the level's colliders
    // change. The first update builds regardless.
    void request_rebuild() noexcept { rebuild_requested_ = true; }

    [[nodiscard]] const NavGrid3D& grid() const noexcept { return grid_; }
    [[nodiscard]] usize agent_count() const noexcept { return agent_count_; }

    // Voxel size, in world units. Zero (the default) uses one world unit. Read on
    // the next build.
    void set_cell_size(f32 cell_size) noexcept { cell_size_ = cell_size; }

    // Whether a walkable voxel must have ground below it. Off by default, so a
    // flying agent paths through free space. Read on the next build.
    void set_require_ground(bool require_ground) noexcept { require_ground_ = require_ground; }

private:
    void rebuild_grid(Scene& scene);

    NavGrid3D grid_;
    bool rebuild_requested_ = true;
    bool built_ = false;
    f32 cell_size_ = 0.0f;
    bool require_ground_ = false;
    usize agent_count_ = 0;

    struct AgentRef {
        Node* node = nullptr;
        NavAgent3D* agent = nullptr;
        i32 goal_cell_x = 0;
        i32 goal_cell_y = 0;
        i32 goal_cell_z = 0;
        bool has_planned_cell = false;
    };
    std::vector<AgentRef> agents_;
    std::vector<Vec3> neighbour_positions_;
    std::vector<Vec3> obstacle_points_;
    std::vector<Vec3> scratch_path_;
};

} // namespace nav
} // namespace fe
