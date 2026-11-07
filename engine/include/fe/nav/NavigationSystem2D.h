// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Uuid.h"
#include "fe/nav/NavGrid2D.h"
#include "fe/systems/System.h"

#include <string_view>
#include <unordered_map>
#include <vector>

namespace fe {
class Node;
namespace nav {

class NavAgent2D;

// Advances every `NavAgent2D` in the scene along its level's nav grid.
//
// The system owns the grids: it builds one per `Tilemap` in the scene (derived
// from the tiles' solidity and the scene's static colliders, ADR-0156) and
// rebuilds them when asked. A grid is a function of the level, so owning it here
// rather than on a component keeps it out of the save file and lets every agent
// share one build.
//
// Each frame the system: resolves each agent's goal, re-plans its path only when
// the goal cell changed, steers along the path (with separation from the other
// agents and avoidance of nearby walls), and moves the agent's node when its
// `auto_move` is set. It is an ordinary `System`, so it runs wherever a schedule
// runs — the example, play-in-editor, and a headless test — with no editor or
// RHI dependency.
//
// Ordering: add this system after the physics system if an agent is meant to
// react to a body that moved this frame, and before the renderer regardless (it
// moves nodes, and the renderer reads them).
class NavigationSystem2D final : public System {
public:
    NavigationSystem2D() = default;

    [[nodiscard]] std::string_view name() const noexcept override { return "NavigationSystem2D"; }

    void update(const FrameContext& context, Scene& scene) override;

    // Asks for the grids to be rebuilt on the next update. Call this when the
    // level changes (a tilemap edited, an obstacle added). The first update
    // builds the grids regardless, so a caller that never edits the level never
    // calls this.
    void request_rebuild() noexcept { rebuild_requested_ = true; }

    // The grid built from `tilemap_node`, or nullptr when none exists (the scene
    // has no tilemap, or the id is not one). A debug draw and a test read it.
    [[nodiscard]] const NavGrid2D* grid_for(const Uuid& tilemap_node) const noexcept;

    // The id of the scene's first tilemap node, which is the grid a one-level
    // scene uses. Nil when the scene has no tilemap. Cached from the last build.
    [[nodiscard]] const Uuid& default_grid_node() const noexcept { return default_grid_node_; }

    // The number of agents the last update advanced.
    [[nodiscard]] usize agent_count() const noexcept { return agent_count_; }

    // How finely a grid subdivides a tile. Zero (the default) means one cell per
    // tile. Read on the next build, so it can be set before the first update.
    void set_cell_size(f32 cell_size) noexcept { cell_size_ = cell_size; }

    // The clearance, in world units, kept between an agent's route and a wall.
    // Read on the next build.
    void set_agent_radius(f32 radius) noexcept { agent_radius_ = radius; }

private:
    void rebuild_grids(Scene& scene);

    std::unordered_map<Uuid, NavGrid2D> grids_;
    Uuid default_grid_node_{};
    bool rebuild_requested_ = true;
    bool built_ = false;
    f32 cell_size_ = 0.0f;
    f32 agent_radius_ = 0.0f;
    usize agent_count_ = 0;

    // Per-frame scratch, kept between frames so a steady scene does not allocate.
    struct AgentRef {
        Node* node = nullptr;
        NavAgent2D* agent = nullptr;
        const NavGrid2D* grid = nullptr;
        i32 goal_cell_x = 0;
        i32 goal_cell_y = 0;
        bool has_planned_cell = false;
    };
    std::vector<AgentRef> agents_;
    std::vector<Vec2> neighbour_positions_;
    std::vector<Vec2> obstacle_points_;
    std::vector<Vec2> scratch_path_;
};

} // namespace nav
} // namespace fe
