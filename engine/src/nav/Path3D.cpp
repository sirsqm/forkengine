// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/nav/Path3D.h"

#include "fe/nav/NavGrid3D.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace fe::nav {
namespace {

[[nodiscard]] usize voxel_index(const NavGrid3D& grid, i32 x, i32 y, i32 z) noexcept {
    return (static_cast<usize>(z) * grid.height() + static_cast<usize>(y)) * grid.width() + static_cast<usize>(x);
}

struct Node {
    f32 f = 0.0f;
    i32 x = 0;
    i32 y = 0;
    i32 z = 0;
    bool operator>(const Node& other) const noexcept { return f > other.f; }
};

} // namespace

Error find_path_3d(const NavGrid3D& grid, const Vec3& start, const Vec3& goal, std::vector<Vec3>& out_path,
                   i32 snap_radius) {
    out_path.clear();
    if (grid.empty()) {
        return Error("navigation: the 3D grid is empty");
    }

    i32 sx = 0;
    i32 sy = 0;
    i32 sz = 0;
    i32 gx = 0;
    i32 gy = 0;
    i32 gz = 0;
    grid.world_to_cell(start, sx, sy, sz);
    grid.world_to_cell(goal, gx, gy, gz);
    if (!grid.nearest_walkable(sx, sy, sz, sx, sy, sz, snap_radius)) {
        return Error("navigation: the start has no walkable voxel nearby");
    }
    if (!grid.nearest_walkable(gx, gy, gz, gx, gy, gz, snap_radius)) {
        return Error("navigation: the goal has no walkable voxel nearby");
    }
    if (sx == gx && sy == gy && sz == gz) {
        out_path.push_back(grid.cell_center(sx, sy, sz));
        return Error::none();
    }

    const usize count = grid.cell_count();
    constexpr f32 kUnvisited = 3.4e38f;
    std::vector<f32> g_score(count, kUnvisited);
    std::vector<i32> came_from(count, -1);
    std::vector<u8> closed(count, 0u);

    const auto heuristic = [&](i32 x, i32 y, i32 z) {
        const f32 dx = static_cast<f32>(x - gx);
        const f32 dy = static_cast<f32>(y - gy);
        const f32 dz = static_cast<f32>(z - gz);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    g_score[voxel_index(grid, sx, sy, sz)] = 0.0f;
    open.push(Node{heuristic(sx, sy, sz), sx, sy, sz});

    bool found = false;
    while (!open.empty()) {
        const Node current = open.top();
        open.pop();
        const usize current_index = voxel_index(grid, current.x, current.y, current.z);
        if (closed[current_index] != 0u) {
            continue;
        }
        closed[current_index] = 1u;
        if (current.x == gx && current.y == gy && current.z == gz) {
            found = true;
            break;
        }

        for (i32 dz = -1; dz <= 1; ++dz) {
            for (i32 dy = -1; dy <= 1; ++dy) {
                for (i32 dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const i32 nx = current.x + dx;
                    const i32 ny = current.y + dy;
                    const i32 nz = current.z + dz;
                    if (!grid.walkable(nx, ny, nz)) {
                        continue;
                    }
                    const usize neighbour_index = voxel_index(grid, nx, ny, nz);
                    if (closed[neighbour_index] != 0u) {
                        continue;
                    }
                    const f32 step_cost = std::sqrt(static_cast<f32>(dx * dx + dy * dy + dz * dz));
                    const f32 tentative = g_score[current_index] + step_cost;
                    if (tentative < g_score[neighbour_index]) {
                        g_score[neighbour_index] = tentative;
                        came_from[neighbour_index] = static_cast<i32>(current_index);
                        open.push(Node{tentative + heuristic(nx, ny, nz), nx, ny, nz});
                    }
                }
            }
        }
    }

    if (!found) {
        return Error("navigation: no 3D route between the endpoints");
    }

    i32 cx = gx;
    i32 cy = gy;
    i32 cz = gz;
    while (cx >= 0 && cy >= 0 && cz >= 0) {
        out_path.push_back(grid.cell_center(cx, cy, cz));
        const i32 parent = came_from[voxel_index(grid, cx, cy, cz)];
        if (parent < 0) {
            break;
        }
        cx = parent % static_cast<i32>(grid.width());
        cy = (parent / static_cast<i32>(grid.width())) % static_cast<i32>(grid.height());
        cz = parent / static_cast<i32>(grid.width() * grid.height());
    }
    std::reverse(out_path.begin(), out_path.end());
    return Error::none();
}

void smooth_path_3d(const NavGrid3D& grid, std::vector<Vec3>& path) {
    if (path.size() < 3) {
        return;
    }
    std::vector<Vec3> smoothed;
    smoothed.reserve(path.size());
    smoothed.push_back(path.front());

    usize anchor = 0;
    while (anchor + 1 < path.size()) {
        usize best = anchor + 1;
        for (usize candidate = anchor + 2; candidate < path.size(); ++candidate) {
            if (line_of_sight_3d(grid, path[anchor], path[candidate])) {
                best = candidate;
            } else {
                break;
            }
        }
        smoothed.push_back(path[best]);
        anchor = best;
    }
    path = std::move(smoothed);
}

} // namespace fe::nav
