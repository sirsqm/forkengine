// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Math.h"
#include "fe/core/Types.h"

#include <vector>

namespace fe {

class Scene;
class Node;
class Tilemap;

namespace nav {

// A uniform grid of walkable cells derived from a 2D level.
//
// The grid is not authored; it is *built* from what the game already has: a
// `Tilemap`'s solidity rule (the same one the physics collision build reads,
// ADR-0062) and, optionally, the scene's `Collider2D` obstacles. That is what
// makes the ground an agent walks on and the ground a body collides with one
// fact rather than two that can drift apart (ADR-0156).
//
// Coordinates. Cell (0, 0) is the top-left of the level, +X right and +Y down,
// matching `Tilemap`. A cell's world centre is `origin + (x + 0.5, y + 0.5) *
// cell_size`; `origin` is the level's world-space top-left corner, captured at
// build time from the tilemap node's world matrix so a moved or scaled map is
// walked where it is drawn.
//
// The grid owns a flat walkable mask (`width * height` bytes) rather than a
// vector of vectors, so a query is one index and a scan is contiguous.
class NavGrid2D {
public:
    NavGrid2D() = default;

    [[nodiscard]] u32 width() const noexcept { return width_; }
    [[nodiscard]] u32 height() const noexcept { return height_; }
    [[nodiscard]] f32 cell_size() const noexcept { return cell_size_; }
    [[nodiscard]] const Vec2& origin() const noexcept { return origin_; }

    // True when the grid holds no cells, which is the state of a default
    // instance and of a build that found no level. A system treats an empty grid
    // as "nothing to navigate" rather than querying it.
    [[nodiscard]] bool empty() const noexcept { return width_ == 0 || height_ == 0; }

    [[nodiscard]] usize cell_count() const noexcept { return static_cast<usize>(width_) * height_; }

    // Reserves a grid of `width` x `height` cells, all blocked, at `cell_size`
    // with its top-left corner at `origin`. Rebuilding a grid of the same size
    // reuses the buffer.
    void resize(u32 width, u32 height, f32 cell_size, const Vec2& origin);

    [[nodiscard]] bool in_bounds(i32 x, i32 y) const noexcept {
        return x >= 0 && y >= 0 && static_cast<u32>(x) < width_ && static_cast<u32>(y) < height_;
    }

    // True when the cell may be walked. Out of bounds is not walkable, so a
    // search that runs off the edge stops rather than wrapping.
    [[nodiscard]] bool walkable(i32 x, i32 y) const noexcept { return in_bounds(x, y) && cells_[index(x, y)] != 0; }

    // Marks a cell walkable (true) or blocked (false). Out-of-bounds writes are
    // ignored, so a tool that walks a rectangle past the edge cannot corrupt the
    // grid.
    void set_walkable(i32 x, i32 y, bool walk) noexcept {
        if (in_bounds(x, y)) {
            cells_[index(x, y)] = walk ? 1u : 0u;
        }
    }

    // The world-space centre of a cell. `origin` is the level's top-left corner
    // and +Y goes *down* the grid while world +Y goes up, so the row offset is
    // subtracted. Defined for any coordinate (it is a pure function of the
    // layout), so a caller can convert a goal position to a cell even when that
    // cell turns out to be blocked.
    [[nodiscard]] Vec2 cell_center(i32 x, i32 y) const noexcept {
        return Vec2{origin_.x + (static_cast<f32>(x) + 0.5f) * cell_size_,
                    origin_.y - (static_cast<f32>(y) + 0.5f) * cell_size_};
    }

    // The cell containing a world position, clamped into the grid, so a goal
    // just outside the level resolves to its nearest edge cell rather than to an
    // out-of-bounds coordinate.
    void world_to_cell(const Vec2& world, i32& out_x, i32& out_y) const noexcept;

    // The nearest walkable cell to `(x, y)`, searching outward in rings, so a
    // goal that lands on a wall or a prop still yields a reachable target. Fills
    // `out_x`/`out_y` and returns true when one was found within `max_radius`
    // cells; returns false (leaving the outputs untouched) when the whole
    // searched region is blocked.
    [[nodiscard]] bool nearest_walkable(i32 x, i32 y, i32& out_x, i32& out_y, i32 max_radius = 8) const noexcept;

private:
    [[nodiscard]] usize index(i32 x, i32 y) const noexcept {
        return static_cast<usize>(y) * width_ + static_cast<usize>(x);
    }

    u32 width_ = 0;
    u32 height_ = 0;
    f32 cell_size_ = 1.0f;
    Vec2 origin_{};
    std::vector<u8> cells_;
};

// Builds a navigation grid from a tilemap.
//
// A cell is walkable when the tile under it is not solid. When `agent_radius` is
// positive the grid is *eroded*: a cell within `ceil(agent_radius / cell_size)`
// cells of a solid tile is also blocked, so an agent whose body has a radius does
// not clip a wall it is routed along. A radius of 0 leaves the grid exactly the
// walkable tiles.
//
// `cell_size` <= 0 uses the tilemap's own `tile_size`, so the grid is one cell
// per tile and no resolution decision is needed; a smaller value gives finer
// routing within a tile.
//
// Returns an empty grid when the map is empty.
[[nodiscard]] NavGrid2D build_nav_grid_2d(const Scene& scene, const Node& map_node, const Tilemap& map,
                                          f32 cell_size = 0.0f, f32 agent_radius = 0.0f);

// As above, resolving the first tilemap by walking the scene. `node_out`, when
// given, receives the tilemap's host node, which is the node an agent names to
// bind to this grid. Returns an empty grid when the scene has no tilemap.
[[nodiscard]] NavGrid2D build_nav_grid_2d(const Scene& scene, f32 cell_size = 0.0f, f32 agent_radius = 0.0f,
                                          const Node** node_out = nullptr);

// True when a straight line from `a` to `b` (world space) crosses no blocked
// cell. Used to smooth a path: the smoothed waypoints are exactly the raw ones
// that can still see each other. A supercover traversal visits every cell the
// segment touches, so a segment that grazes a wall corner is correctly blocked
// rather than slipping through the gap between two cells.
[[nodiscard]] bool line_of_sight_2d(const NavGrid2D& grid, const Vec2& a, const Vec2& b) noexcept;

} // namespace nav
} // namespace fe
