// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Math.h"
#include "fe/core/Types.h"

#include <vector>

namespace fe {

class Scene;

namespace nav {

// A uniform voxel grid of walkable cells derived from a 3D level.
//
// The 3D counterpart of `NavGrid2D`. A 3D level is a set of static `Collider3D`
// shapes on the scene graph (ADR-0031), not a grid, so the grid is *built* from
// those shapes: a voxel is walkable when it intersects no collider box or
// sphere. That is the honest first cut ADR-0156 describes; a polygon navmesh is
// the refinement if the grid's memory ever matters.
//
// Coordinates. Cell (0, 0, 0) is the grid's minimum corner; +X, +Y, and +Z are
// the world axes. A cell's world centre is `origin + (x + 0.5, y + 0.5, z + 0.5)
// * cell_size`. `origin` is captured at build time as the minimum corner of the
// scene's collider bounds expanded by one cell, so the whole level fits.
//
// The grid owns a flat walkable mask, so a query is one index and a scan is
// contiguous.
class NavGrid3D {
public:
    NavGrid3D() = default;

    [[nodiscard]] u32 width() const noexcept { return width_; }
    [[nodiscard]] u32 height() const noexcept { return height_; }
    [[nodiscard]] u32 depth() const noexcept { return depth_; }
    [[nodiscard]] f32 cell_size() const noexcept { return cell_size_; }
    [[nodiscard]] const Vec3& origin() const noexcept { return origin_; }

    [[nodiscard]] bool empty() const noexcept { return width_ == 0 || height_ == 0 || depth_ == 0; }

    [[nodiscard]] usize cell_count() const noexcept { return static_cast<usize>(width_) * height_ * depth_; }

    // Reserves a grid, all blocked, at `cell_size` with its minimum corner at
    // `origin`.
    void resize(u32 width, u32 height, u32 depth, f32 cell_size, const Vec3& origin);

    [[nodiscard]] bool in_bounds(i32 x, i32 y, i32 z) const noexcept {
        return x >= 0 && y >= 0 && z >= 0 && static_cast<u32>(x) < width_ && static_cast<u32>(y) < height_ &&
               static_cast<u32>(z) < depth_;
    }

    [[nodiscard]] bool walkable(i32 x, i32 y, i32 z) const noexcept {
        return in_bounds(x, y, z) && cells_[index(x, y, z)] != 0u;
    }

    void set_walkable(i32 x, i32 y, i32 z, bool walk) noexcept {
        if (in_bounds(x, y, z)) {
            cells_[index(x, y, z)] = walk ? 1u : 0u;
        }
    }

    [[nodiscard]] Vec3 cell_center(i32 x, i32 y, i32 z) const noexcept {
        return Vec3{origin_.x + (static_cast<f32>(x) + 0.5f) * cell_size_,
                    origin_.y + (static_cast<f32>(y) + 0.5f) * cell_size_,
                    origin_.z + (static_cast<f32>(z) + 0.5f) * cell_size_};
    }

    // The cell containing a world position, clamped into the grid.
    void world_to_cell(const Vec3& world, i32& out_x, i32& out_y, i32& out_z) const noexcept;

    // The nearest walkable cell to `(x, y, z)`, searching outward in cube rings.
    // Returns false (leaving the outputs untouched) when the searched region is
    // all blocked.
    [[nodiscard]] bool nearest_walkable(i32 x, i32 y, i32 z, i32& out_x, i32& out_y, i32& out_z,
                                        i32 max_radius = 8) const noexcept;

private:
    [[nodiscard]] usize index(i32 x, i32 y, i32 z) const noexcept {
        return (static_cast<usize>(z) * height_ + static_cast<usize>(y)) * width_ + static_cast<usize>(x);
    }

    u32 width_ = 0;
    u32 height_ = 0;
    u32 depth_ = 0;
    f32 cell_size_ = 1.0f;
    Vec3 origin_{};
    std::vector<u8> cells_;
};

// Builds a navigation grid from the scene's static 3D colliders.
//
// `cell_size` <= 0 uses a default of one world unit. A voxel is walkable when it
// intersects no collider. When `require_ground` is set, a walkable voxel must
// also have a solid voxel within `ground_cells` below it, so a ground-based agent
// is not routed through empty air; leave it off for a flying agent, which paths
// through free space.
//
// The grid's bounds are the collider set's bounds expanded by one cell on each
// side, so a route along the outer wall has a cell to stand in. Returns an empty
// grid when the scene holds no colliders.
[[nodiscard]] NavGrid3D build_nav_grid_3d(const Scene& scene, f32 cell_size = 0.0f, bool require_ground = false,
                                          i32 ground_cells = 1);

// True when the straight segment from `a` to `b` crosses no blocked voxel. A 3D
// DDA traversal visits every voxel the segment enters, so a segment that grazes
// a corner is correctly blocked.
[[nodiscard]] bool line_of_sight_3d(const NavGrid3D& grid, const Vec3& a, const Vec3& b) noexcept;

} // namespace nav
} // namespace fe
