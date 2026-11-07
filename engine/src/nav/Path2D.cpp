// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/nav/Path2D.h"

#include "fe/nav/NavGrid2D.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace fe::nav {
namespace {

// The cost of a straight step and a diagonal step. The diagonal is the Euclidean
// length of a unit diagonal, so the heuristic is admissible against these.
constexpr f32 kStraightCost = 1.0f;
constexpr f32 kDiagonalCost = 1.41421356f;

[[nodiscard]] usize cell_index(const NavGrid2D& grid, i32 x, i32 y) noexcept {
    return static_cast<usize>(y) * grid.width() + static_cast<usize>(x);
}

// A priority-queue entry, ordered so the smallest `f` is popped first.
struct Node {
    f32 f = 0.0f;
    i32 x = 0;
    i32 y = 0;
    bool operator>(const Node& other) const noexcept { return f > other.f; }
};

} // namespace

Error find_path_2d(const NavGrid2D& grid, const Vec2& start, const Vec2& goal, std::vector<Vec2>& out_path,
                   i32 snap_radius) {
    out_path.clear();
    if (grid.empty()) {
        return Error("navigation: the grid is empty");
    }

    i32 sx = 0;
    i32 sy = 0;
    i32 gx = 0;
    i32 gy = 0;
    grid.world_to_cell(start, sx, sy);
    grid.world_to_cell(goal, gx, gy);

    // Snap each endpoint to the nearest walkable cell, so a start or a goal that
    // lands on a wall or a prop still routes rather than failing outright.
    if (!grid.nearest_walkable(sx, sy, sx, sy, snap_radius)) {
        return Error("navigation: the start has no walkable cell nearby");
    }
    if (!grid.nearest_walkable(gx, gy, gx, gy, snap_radius)) {
        return Error("navigation: the goal has no walkable cell nearby");
    }
    if (sx == gx && sy == gy) {
        out_path.push_back(grid.cell_center(sx, sy));
        return Error::none();
    }

    const usize count = grid.cell_count();
    constexpr f32 kUnvisited = 3.4e38f;
    std::vector<f32> g_score(count, kUnvisited);
    std::vector<i32> came_from(count, -1);
    std::vector<u8> closed(count, 0u);

    const auto heuristic = [&](i32 x, i32 y) {
        const f32 dx = static_cast<f32>(x - gx);
        const f32 dy = static_cast<f32>(y - gy);
        return std::sqrt(dx * dx + dy * dy);
    };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    const usize start_index = cell_index(grid, sx, sy);
    g_score[start_index] = 0.0f;
    open.push(Node{heuristic(sx, sy), sx, sy});

    // The 8 neighbours. A diagonal is allowed only when both orthogonal
    // neighbours that share it are walkable, so the path never cuts a corner
    // between two diagonal walls.
    static constexpr i32 kOffsets[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    bool found = false;
    while (!open.empty()) {
        const Node current = open.top();
        open.pop();
        const usize current_index = cell_index(grid, current.x, current.y);
        if (closed[current_index] != 0u) {
            continue;
        }
        closed[current_index] = 1u;
        if (current.x == gx && current.y == gy) {
            found = true;
            break;
        }

        for (const auto& offset : kOffsets) {
            const i32 nx = current.x + offset[0];
            const i32 ny = current.y + offset[1];
            if (!grid.walkable(nx, ny)) {
                continue;
            }
            const bool diagonal = offset[0] != 0 && offset[1] != 0;
            if (diagonal && (!grid.walkable(current.x + offset[0], current.y) ||
                             !grid.walkable(current.x, current.y + offset[1]))) {
                continue; // would cut a corner
            }
            const usize neighbour_index = cell_index(grid, nx, ny);
            if (closed[neighbour_index] != 0u) {
                continue;
            }
            const f32 tentative = g_score[current_index] + (diagonal ? kDiagonalCost : kStraightCost);
            if (tentative < g_score[neighbour_index]) {
                g_score[neighbour_index] = tentative;
                came_from[neighbour_index] = static_cast<i32>(current_index);
                open.push(Node{tentative + heuristic(nx, ny), nx, ny});
            }
        }
    }

    if (!found) {
        return Error("navigation: no route between the endpoints");
    }

    // Walk the parent chain back to the start, then reverse into cell centres.
    i32 cx = gx;
    i32 cy = gy;
    while (cx >= 0 && cy >= 0) {
        out_path.push_back(grid.cell_center(cx, cy));
        const i32 parent = came_from[cell_index(grid, cx, cy)];
        if (parent < 0) {
            break;
        }
        cx = parent % static_cast<i32>(grid.width());
        cy = parent / static_cast<i32>(grid.width());
    }
    std::reverse(out_path.begin(), out_path.end());
    return Error::none();
}

void smooth_path_2d(const NavGrid2D& grid, std::vector<Vec2>& path) {
    if (path.size() < 3) {
        return;
    }
    std::vector<Vec2> smoothed;
    smoothed.reserve(path.size());
    smoothed.push_back(path.front());

    usize anchor = 0;
    while (anchor + 1 < path.size()) {
        // The farthest waypoint still reachable in a straight line from the
        // anchor; everything between is dropped, which is what removes the
        // stair-step.
        usize best = anchor + 1;
        for (usize candidate = anchor + 2; candidate < path.size(); ++candidate) {
            if (line_of_sight_2d(grid, path[anchor], path[candidate])) {
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
