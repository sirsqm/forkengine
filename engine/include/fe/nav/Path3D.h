// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Error.h"
#include "fe/core/Math.h"

#include <vector>

namespace fe::nav {

class NavGrid3D;

// Finds a walkable route through a `NavGrid3D` with A*.
//
// The search is 26-connected with a Euclidean heuristic. Diagonal steps between
// voxels that share only an edge or a corner are permitted, which is the honest
// choice for a 3D voxel grid: an agent flying or swimming moves in any of the 26
// directions, and forbidding the face-diagonal cuts would quantise its motion to
// the three axes. A ground agent is kept honest by the grid's `require_ground`
// build flag instead.
//
// `start`/`goal` are world positions, each snapped to its nearest walkable voxel
// within `snap_radius`. The path is the sequence of voxel centres from start to
// goal, inclusive, and is cleared before it is filled.
[[nodiscard]] Error find_path_3d(const NavGrid3D& grid, const Vec3& start, const Vec3& goal,
                                 std::vector<Vec3>& out_path, i32 snap_radius = 8);

// Smooths a voxel path in place by line of sight, keeping the farthest waypoint
// each step can still see. The first and last waypoints are always kept. A path
// of fewer than three points is returned unchanged.
void smooth_path_3d(const NavGrid3D& grid, std::vector<Vec3>& path);

} // namespace fe::nav
