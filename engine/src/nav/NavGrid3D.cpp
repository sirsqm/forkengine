// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/nav/NavGrid3D.h"

#include "fe/physics3d/Collider3D.h"
#include "fe/physics3d/Geometry3D.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"

#include <algorithm>
#include <cmath>

namespace fe::nav {
namespace {

// The world AABB a collider occupies, from its node's world position. A rotated
// box blocks its axis-aligned bounds, which is a conservative superset of its
// true footprint: a nav grid may refuse a cell a rotated box only grazes, which
// is safe (an agent walks around) rather than permissive (an agent walks through).
[[nodiscard]] physics3d::Aabb3D collider_bounds(const Node& node, const physics3d::Collider3D& collider) {
    const Vec3 world = transform_point(node.world_matrix(), Vec3{0.0f, 0.0f, 0.0f});
    return collider.bounds(world);
}

} // namespace

void NavGrid3D::resize(u32 width, u32 height, u32 depth, f32 cell_size, const Vec3& origin) {
    width_ = width;
    height_ = height;
    depth_ = depth;
    cell_size_ = cell_size > 0.0f ? cell_size : 1.0f;
    origin_ = origin;
    cells_.assign(static_cast<usize>(width_) * height_ * depth_, 0u);
}

void NavGrid3D::world_to_cell(const Vec3& world, i32& out_x, i32& out_y, i32& out_z) const noexcept {
    if (empty() || cell_size_ <= 0.0f) {
        out_x = 0;
        out_y = 0;
        out_z = 0;
        return;
    }
    const i32 x = static_cast<i32>(std::floor((world.x - origin_.x) / cell_size_));
    const i32 y = static_cast<i32>(std::floor((world.y - origin_.y) / cell_size_));
    const i32 z = static_cast<i32>(std::floor((world.z - origin_.z) / cell_size_));
    out_x = std::clamp(x, 0, static_cast<i32>(width_) - 1);
    out_y = std::clamp(y, 0, static_cast<i32>(height_) - 1);
    out_z = std::clamp(z, 0, static_cast<i32>(depth_) - 1);
}

bool NavGrid3D::nearest_walkable(i32 x, i32 y, i32 z, i32& out_x, i32& out_y, i32& out_z,
                                 i32 max_radius) const noexcept {
    if (empty()) {
        return false;
    }
    if (walkable(x, y, z)) {
        out_x = x;
        out_y = y;
        out_z = z;
        return true;
    }
    const i32 max_r = std::max(max_radius, 0);
    for (i32 r = 1; r <= max_r; ++r) {
        for (i32 dz = -r; dz <= r; ++dz) {
            for (i32 dy = -r; dy <= r; ++dy) {
                for (i32 dx = -r; dx <= r; ++dx) {
                    if (std::max(std::max(std::abs(dx), std::abs(dy)), std::abs(dz)) != r) {
                        continue;
                    }
                    if (walkable(x + dx, y + dy, z + dz)) {
                        out_x = x + dx;
                        out_y = y + dy;
                        out_z = z + dz;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

NavGrid3D build_nav_grid_3d(const Scene& scene, f32 cell_size, bool require_ground, i32 ground_cells) {
    NavGrid3D grid;

    // Gather the collider bounds once, so the build is one scene walk and the
    // bounds are not recomputed per voxel.
    std::vector<physics3d::Aabb3D> obstacles;
    scene.for_each_node([&](Node& node) {
        const auto* collider = node.get_component<physics3d::Collider3D>();
        if (collider == nullptr || collider->is_trigger) {
            return;
        }
        obstacles.push_back(collider_bounds(node, *collider));
    });
    if (obstacles.empty()) {
        return grid;
    }

    const f32 size = cell_size > 0.0f ? cell_size : 1.0f;

    // The bounds of everything, expanded by one cell so a route along the outer
    // wall has a walkable cell to stand in.
    Vec3 min = obstacles.front().center - obstacles.front().half_extent;
    Vec3 max = obstacles.front().center + obstacles.front().half_extent;
    for (const auto& box : obstacles) {
        min.x = std::min(min.x, box.center.x - box.half_extent.x);
        min.y = std::min(min.y, box.center.y - box.half_extent.y);
        min.z = std::min(min.z, box.center.z - box.half_extent.z);
        max.x = std::max(max.x, box.center.x + box.half_extent.x);
        max.y = std::max(max.y, box.center.y + box.half_extent.y);
        max.z = std::max(max.z, box.center.z + box.half_extent.z);
    }
    const Vec3 origin = min - Vec3{size, size, size};
    const Vec3 span = (max + Vec3{size, size, size}) - origin;
    const u32 width = static_cast<u32>(std::max(1.0f, std::ceil(span.x / size)));
    const u32 height = static_cast<u32>(std::max(1.0f, std::ceil(span.y / size)));
    const u32 depth = static_cast<u32>(std::max(1.0f, std::ceil(span.z / size)));
    grid.resize(width, height, depth, size, origin);

    // A voxel is walkable when its centre is not inside any obstacle.
    for (u32 z = 0; z < depth; ++z) {
        for (u32 y = 0; y < height; ++y) {
            for (u32 x = 0; x < width; ++x) {
                const Vec3 center = grid.cell_center(static_cast<i32>(x), static_cast<i32>(y), static_cast<i32>(z));
                bool blocked = false;
                for (const auto& box : obstacles) {
                    if (std::abs(center.x - box.center.x) <= box.half_extent.x &&
                        std::abs(center.y - box.center.y) <= box.half_extent.y &&
                        std::abs(center.z - box.center.z) <= box.half_extent.z) {
                        blocked = true;
                        break;
                    }
                }
                grid.set_walkable(static_cast<i32>(x), static_cast<i32>(y), static_cast<i32>(z), !blocked);
            }
        }
    }

    // A ground-based agent needs solid ground below a walkable voxel; a flying
    // agent does not. The check is a downward scan of `ground_cells` cells.
    if (require_ground && ground_cells > 0) {
        for (u32 z = 0; z < depth; ++z) {
            for (u32 y = 0; y < height; ++y) {
                for (u32 x = 0; x < width; ++x) {
                    const i32 cx = static_cast<i32>(x);
                    const i32 cy = static_cast<i32>(y);
                    const i32 cz = static_cast<i32>(z);
                    if (!grid.walkable(cx, cy, cz)) {
                        continue;
                    }
                    bool has_ground = false;
                    for (i32 below = 1; below <= ground_cells; ++below) {
                        if (grid.in_bounds(cx, cy - below, cz) && !grid.walkable(cx, cy - below, cz)) {
                            has_ground = true;
                            break;
                        }
                    }
                    if (!has_ground) {
                        grid.set_walkable(cx, cy, cz, false);
                    }
                }
            }
        }
    }

    return grid;
}

bool line_of_sight_3d(const NavGrid3D& grid, const Vec3& a, const Vec3& b) noexcept {
    if (grid.empty()) {
        return false;
    }
    i32 x = 0;
    i32 y = 0;
    i32 z = 0;
    i32 x1 = 0;
    i32 y1 = 0;
    i32 z1 = 0;
    grid.world_to_cell(a, x, y, z);
    grid.world_to_cell(b, x1, y1, z1);

    const Vec3 dir = b - a;
    const f32 length = dir.length();
    if (length <= 1e-6f) {
        return grid.walkable(x, y, z);
    }
    const Vec3 step_dir = dir / length;
    const f32 size = grid.cell_size();

    const i32 step_x = step_dir.x > 0.0f ? 1 : (step_dir.x < 0.0f ? -1 : 0);
    const i32 step_y = step_dir.y > 0.0f ? 1 : (step_dir.y < 0.0f ? -1 : 0);
    const i32 step_z = step_dir.z > 0.0f ? 1 : (step_dir.z < 0.0f ? -1 : 0);

    const auto next_boundary = [&](i32 cell, i32 step, f32 origin, f32 direction) -> f32 {
        if (step == 0 || direction == 0.0f) {
            return 3.4e38f;
        }
        const f32 boundary = origin + (static_cast<f32>(cell) + (step > 0 ? 1.0f : 0.0f)) * size;
        return (boundary - origin) / direction;
    };

    f32 t_max_x = next_boundary(x, step_x, a.x, step_dir.x);
    f32 t_max_y = next_boundary(y, step_y, a.y, step_dir.y);
    f32 t_max_z = next_boundary(z, step_z, a.z, step_dir.z);
    const f32 t_delta_x = step_x != 0 ? size / std::abs(step_dir.x) : 3.4e38f;
    const f32 t_delta_y = step_y != 0 ? size / std::abs(step_dir.y) : 3.4e38f;
    const f32 t_delta_z = step_z != 0 ? size / std::abs(step_dir.z) : 3.4e38f;

    const i32 max_steps = static_cast<i32>(grid.width() + grid.height() + grid.depth()) * 2 + 4;
    for (i32 step = 0; step < max_steps; ++step) {
        if (!grid.walkable(x, y, z)) {
            return false;
        }
        if (x == x1 && y == y1 && z == z1) {
            return true;
        }
        if (t_max_x <= t_max_y && t_max_x <= t_max_z) {
            x += step_x;
            t_max_x += t_delta_x;
        } else if (t_max_y <= t_max_z) {
            y += step_y;
            t_max_y += t_delta_y;
        } else {
            z += step_z;
            t_max_z += t_delta_z;
        }
        if (!grid.in_bounds(x, y, z)) {
            return false;
        }
    }
    return grid.walkable(x1, y1, z1);
}

} // namespace fe::nav
