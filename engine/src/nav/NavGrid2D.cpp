// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/nav/NavGrid2D.h"

#include "fe/physics2d/Collider2D.h"
#include "fe/physics2d/Geometry.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/scene/Tilemap.h"

#include <algorithm>
#include <cmath>

namespace fe::nav {
namespace {

// Marks every cell a world-space box blocks as unwalkable. The box is the
// collider's world shape (already carrying the node's offset and Z rotation via
// `world_center`/`oriented_box`), so an obstacle blocks exactly where it is
// drawn. A collider with no solid extent is skipped.
void block_box(NavGrid2D& grid, const physics2d::Obb2D& box) {
    if (box.half_extent.x <= 0.0f || box.half_extent.y <= 0.0f) {
        return;
    }
    // The world-space AABB of the oriented box, from its four rotated corners.
    f32 min_x = box.center.x;
    f32 max_x = box.center.x;
    f32 min_y = box.center.y;
    f32 max_y = box.center.y;
    for (i32 corner = 0; corner < 4; ++corner) {
        const f32 sx = (corner & 1) != 0 ? 1.0f : -1.0f;
        const f32 sy = (corner & 2) != 0 ? 1.0f : -1.0f;
        const Vec2 local{sx * box.half_extent.x, sy * box.half_extent.y};
        const Vec2 world = box.center + box.axis_x() * local.x + box.axis_y() * local.y;
        min_x = std::min(min_x, world.x);
        max_x = std::max(max_x, world.x);
        min_y = std::min(min_y, world.y);
        max_y = std::max(max_y, world.y);
    }

    // Convert the corners to cell indices, then mark the covered range. A cell
    // is blocked when its centre lies inside the box's AABB, which is a
    // conservative superset for a rotated box and exact for an axis-aligned one.
    const f32 size = grid.cell_size();
    const Vec2 origin = grid.origin();
    const auto cell_of = [&](f32 x, f32 y, i32& cx, i32& cy) {
        cx = static_cast<i32>(std::floor((x - origin.x) / size));
        cy = static_cast<i32>(std::floor((origin.y - y) / size));
    };

    i32 x0 = 0;
    i32 y0 = 0;
    i32 x1 = 0;
    i32 y1 = 0;
    cell_of(min_x, min_y, x0, y0);
    cell_of(max_x, max_y, x1, y1);
    // The Y axis runs the other way in cell space, so the two corners can be in
    // either order; sort so the loop below always runs forwards.
    const i32 x_lo = std::min(x0, x1);
    const i32 x_hi = std::max(x0, x1);
    const i32 y_lo = std::min(y0, y1);
    const i32 y_hi = std::max(y0, y1);
    for (i32 y = y_lo; y <= y_hi; ++y) {
        for (i32 x = x_lo; x <= x_hi; ++x) {
            if (!grid.in_bounds(x, y)) {
                continue;
            }
            const Vec2 center = grid.cell_center(x, y);
            if (center.x < min_x || center.x > max_x || center.y < min_y || center.y > max_y) {
                continue;
            }
            grid.set_walkable(x, y, false);
        }
    }
}

// Blocks every cell within `radius_cells` of any blocked cell, so an agent with
// a body radius is routed a clearance away from a wall. A zero radius is a
// no-op. Implemented as a dilation: collect the blocked set, then stamp a disc
// of clearance around each, so the erosion does not cascade off cells it just
// blocked in the same pass.
void erode(NavGrid2D& grid, i32 radius_cells) {
    if (radius_cells <= 0) {
        return;
    }
    const u32 width = grid.width();
    const u32 height = grid.height();
    std::vector<u8> blocked(static_cast<usize>(width) * height, 0u);
    for (u32 y = 0; y < height; ++y) {
        for (u32 x = 0; x < width; ++x) {
            if (!grid.walkable(static_cast<i32>(x), static_cast<i32>(y))) {
                blocked[static_cast<usize>(y) * width + x] = 1u;
            }
        }
    }
    const i32 r = radius_cells;
    for (u32 y = 0; y < height; ++y) {
        for (u32 x = 0; x < width; ++x) {
            if (blocked[static_cast<usize>(y) * width + x] == 0u) {
                continue;
            }
            const i32 cx = static_cast<i32>(x);
            const i32 cy = static_cast<i32>(y);
            for (i32 dy = -r; dy <= r; ++dy) {
                for (i32 dx = -r; dx <= r; ++dx) {
                    if (dx * dx + dy * dy > r * r) {
                        continue;
                    }
                    grid.set_walkable(cx + dx, cy + dy, false);
                }
            }
        }
    }
}

} // namespace

void NavGrid2D::resize(u32 width, u32 height, f32 cell_size, const Vec2& origin) {
    width_ = width;
    height_ = height;
    cell_size_ = cell_size > 0.0f ? cell_size : 1.0f;
    origin_ = origin;
    cells_.assign(static_cast<usize>(width_) * height_, 0u);
}

void NavGrid2D::world_to_cell(const Vec2& world, i32& out_x, i32& out_y) const noexcept {
    if (empty() || cell_size_ <= 0.0f) {
        out_x = 0;
        out_y = 0;
        return;
    }
    const i32 x = static_cast<i32>(std::floor((world.x - origin_.x) / cell_size_));
    const i32 y = static_cast<i32>(std::floor((origin_.y - world.y) / cell_size_));
    out_x = std::clamp(x, 0, static_cast<i32>(width_) - 1);
    out_y = std::clamp(y, 0, static_cast<i32>(height_) - 1);
}

bool NavGrid2D::nearest_walkable(i32 x, i32 y, i32& out_x, i32& out_y, i32 max_radius) const noexcept {
    if (empty()) {
        return false;
    }
    // The requested cell itself, when it is walkable, is the answer: a goal on
    // open floor must not be nudged.
    if (walkable(x, y)) {
        out_x = x;
        out_y = y;
        return true;
    }
    // Ring search by Chebyshev radius, so every cell within `r` is considered
    // before any cell at `r + 1`; the first walkable cell found is a nearest.
    const i32 max_r = std::max(max_radius, 0);
    for (i32 r = 1; r <= max_r; ++r) {
        for (i32 dy = -r; dy <= r; ++dy) {
            for (i32 dx = -r; dx <= r; ++dx) {
                // Only the ring at exactly `r`, so inner cells are not revisited.
                if (std::max(std::abs(dx), std::abs(dy)) != r) {
                    continue;
                }
                const i32 cx = x + dx;
                const i32 cy = y + dy;
                if (walkable(cx, cy)) {
                    out_x = cx;
                    out_y = cy;
                    return true;
                }
            }
        }
    }
    return false;
}

NavGrid2D build_nav_grid_2d(const Scene& scene, const Node& map_node, const Tilemap& map, f32 cell_size,
                            f32 agent_radius) {
    NavGrid2D grid;
    if (map.cell_count() == 0 || map.tile_size.x <= 0.0f || map.tile_size.y <= 0.0f) {
        return grid;
    }

    const f32 size = cell_size > 0.0f ? cell_size : map.tile_size.x;
    // One cell per tile keeps the 1:1 mapping the tilemap already has; a finer
    // cell_size subdivides each tile, so the grid dimensions scale accordingly.
    const u32 width =
        static_cast<u32>(std::max(1.0f, std::round(static_cast<f32>(map.width) * map.tile_size.x / size)));
    const u32 height =
        static_cast<u32>(std::max(1.0f, std::round(static_cast<f32>(map.height) * map.tile_size.y / size)));

    // The level's world-space top-left corner. The tilemap's node origin *is* the
    // top-left (ADR-0062), so its world position is the grid origin.
    const Mat4 world = map_node.world_matrix();
    const Vec3 origin_world = transform_point(world, Vec3{0.0f, 0.0f, 0.0f});
    grid.resize(width, height, size, Vec2{origin_world.x, origin_world.y});

    // Walkable when the tile under the cell is not solid. A cell's centre maps
    // back to a tile by dividing by the tile size (the same convention the
    // collision build uses), so a subdivided cell inherits its tile's solidity.
    for (u32 cy = 0; cy < height; ++cy) {
        for (u32 cx = 0; cx < width; ++cx) {
            const Vec2 center = grid.cell_center(static_cast<i32>(cx), static_cast<i32>(cy));
            // Local tile-space coordinates: the map spans x in [0, w*ts.x] and
            // y in [-h*ts.y, 0] in the node's local frame.
            const Vec3 local = transform_point(inverse(world), Vec3{center.x, center.y, 0.0f});
            const f32 tile_fx = local.x / map.tile_size.x;
            const f32 tile_fy = -local.y / map.tile_size.y;
            const i32 tile_x = static_cast<i32>(std::floor(tile_fx));
            const i32 tile_y = static_cast<i32>(std::floor(tile_fy));
            const bool solid = tile_x >= 0 && tile_y >= 0 && static_cast<u32>(tile_x) < map.width &&
                               static_cast<u32>(tile_y) < map.height &&
                               map.is_solid(static_cast<u32>(tile_x), static_cast<u32>(tile_y));
            grid.set_walkable(static_cast<i32>(cx), static_cast<i32>(cy), !solid);
        }
    }

    // Static colliders (props the level placed as nodes, not as tiles) block
    // their cells too, so an agent routes around a crate the physics world also
    // sees. A trigger collider blocks nothing.
    scene.for_each_node([&](Node& node) {
        const auto* collider = node.get_component<physics2d::Collider2D>();
        if (collider == nullptr || collider->is_trigger) {
            return;
        }
        const f32 rotation = node.transform().rotation.z;
        const Vec2 center =
            collider->world_center(Vec2{node.transform().position.x, node.transform().position.y}, rotation);
        block_box(grid, collider->oriented_box(center, rotation));
    });

    erode(grid, static_cast<i32>(std::ceil(agent_radius / size)));
    return grid;
}

NavGrid2D build_nav_grid_2d(const Scene& scene, f32 cell_size, f32 agent_radius, const Node** node_out) {
    const Node* map_node = nullptr;
    const Tilemap* map = nullptr;
    scene.for_each_node([&](Node& node) {
        if (map != nullptr) {
            return;
        }
        const auto* candidate = node.get_component<Tilemap>();
        if (candidate != nullptr) {
            map = candidate;
            map_node = &node;
        }
    });
    if (node_out != nullptr) {
        *node_out = map_node;
    }
    if (map == nullptr || map_node == nullptr) {
        return NavGrid2D{};
    }
    return build_nav_grid_2d(scene, *map_node, *map, cell_size, agent_radius);
}

bool line_of_sight_2d(const NavGrid2D& grid, const Vec2& a, const Vec2& b) noexcept {
    if (grid.empty()) {
        return false;
    }
    // Work in *cell space*: a fractional coordinate where +x and +y both advance
    // with the cell index, so the DDA below is the ordinary form. The Y flip
    // between grid rows (down) and world Y (up) is undone here rather than in
    // every comparison.
    const f32 size = grid.cell_size();
    const Vec2 origin = grid.origin();
    const Vec2 p0{(a.x - origin.x) / size, (origin.y - a.y) / size};
    const Vec2 p1{(b.x - origin.x) / size, (origin.y - b.y) / size};

    i32 x = static_cast<i32>(std::floor(p0.x));
    i32 y = static_cast<i32>(std::floor(p0.y));
    const i32 x1 = std::clamp(static_cast<i32>(std::floor(p1.x)), 0, static_cast<i32>(grid.width()) - 1);
    const i32 y1 = std::clamp(static_cast<i32>(std::floor(p1.y)), 0, static_cast<i32>(grid.height()) - 1);

    const Vec2 dir = p1 - p0;
    const f32 length = dir.length();
    if (length <= 1e-6f) {
        return grid.walkable(x, y);
    }
    const Vec2 step_dir = dir / length;

    const i32 step_x = step_dir.x > 0.0f ? 1 : (step_dir.x < 0.0f ? -1 : 0);
    const i32 step_y = step_dir.y > 0.0f ? 1 : (step_dir.y < 0.0f ? -1 : 0);

    const auto next_boundary = [&](i32 cell, i32 step, f32 start, f32 direction) -> f32 {
        if (step == 0 || direction == 0.0f) {
            return 3.4e38f; // never crosses on this axis
        }
        const f32 boundary = static_cast<f32>(cell) + (step > 0 ? 1.0f : 0.0f);
        return (boundary - start) / direction;
    };

    f32 t_max_x = next_boundary(x, step_x, p0.x, step_dir.x);
    f32 t_max_y = next_boundary(y, step_y, p0.y, step_dir.y);
    const f32 t_delta_x = step_x != 0 ? 1.0f / std::abs(step_dir.x) : 3.4e38f;
    const f32 t_delta_y = step_y != 0 ? 1.0f / std::abs(step_dir.y) : 3.4e38f;

    const i32 max_steps = static_cast<i32>(grid.width() + grid.height()) * 2 + 4;
    for (i32 step = 0; step < max_steps; ++step) {
        if (!grid.walkable(x, y)) {
            return false;
        }
        if (x == x1 && y == y1) {
            return true;
        }
        if (t_max_x < t_max_y) {
            x += step_x;
            t_max_x += t_delta_x;
        } else {
            y += step_y;
            t_max_y += t_delta_y;
        }
        // Ran off the grid: the segment left the level, which is not a clear line
        // to a target inside it.
        if (!grid.in_bounds(x, y)) {
            return false;
        }
    }
    return grid.walkable(x1, y1);
}

} // namespace fe::nav
