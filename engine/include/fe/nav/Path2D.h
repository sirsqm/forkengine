// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Error.h"
#include "fe/core/Math.h"
#include "fe/core/Types.h"

#include <vector>

namespace fe::nav {

class NavGrid2D;

// Finds a walkable route through a `NavGrid2D` with A*.
//
// The search is 8-connected with a Euclidean heuristic and *no corner cutting*:
// a diagonal step is allowed only when both orthogonal neighbours that share the
// diagonal are walkable, so an agent cannot slip between two diagonal walls. The
// result is the sequence of walkable cell *centres* from the start cell to the
// goal cell, inclusive.
//
// `start`/`goal` are world positions. Each is snapped to its nearest walkable
// cell within `snap_radius`, so a goal just inside a prop or a start a hair off
// the floor still yields a route rather than an error. The path is cleared
// before it is filled, so a caller can reuse one buffer across agents.
//
// Returns an error (with a message) when the grid is empty, when neither endpoint
// has a walkable cell within `snap_radius`, or when no route exists.
[[nodiscard]] Error find_path_2d(const NavGrid2D& grid, const Vec2& start, const Vec2& goal,
                                 std::vector<Vec2>& out_path, i32 snap_radius = 8);

// Smooths a grid path in place by line of sight: each waypoint is replaced by the
// farthest later waypoint that can still be seen in a straight line, so the
// result hugs corners and the agent walks diagonals instead of stair-steps. The
// first and last waypoints are always kept. A path of fewer than three points is
// returned unchanged.
void smooth_path_2d(const NavGrid2D& grid, std::vector<Vec2>& path);

} // namespace fe::nav
