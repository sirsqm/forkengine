// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Math.h"
#include "fe/core/Types.h"

#include <vector>

namespace fe::nav {

// Steering behaviours: pure functions that turn a goal, a neighbourhood, and a
// current velocity into a *desired* velocity contribution.
//
// The split from pathfinding is deliberate (ADR-0156). A path says where to go;
// steering says how to get there smoothly, avoiding the other agents doing the
// same. Each function is a pure function of its inputs, so it is tested without
// a scene and composed freely: a caller sums the contributions it wants, by
// weight, and clamps the result to a maximum speed.
//
// The 2D and 3D sets are the same behaviours in their own dimensions. They are
// separate free functions rather than a template because the vector types and
// the neighbour list differ, and a shared template would obscure more than it
// saves at this size.

// --- 2D --------------------------------------------------------------------

// A velocity straight at the goal, of length `max_speed`. The simplest "go
// there" behaviour: it ignores the current velocity, so it turns instantly.
[[nodiscard]] Vec2 seek_2d(const Vec2& position, const Vec2& target, f32 max_speed) noexcept;

// The mirror of seek: a velocity straight away from the target. Used for a
// fleeing agent.
[[nodiscard]] Vec2 flee_2d(const Vec2& position, const Vec2& target, f32 max_speed) noexcept;

// A velocity towards the goal that slows down inside `slow_radius`, so an agent
// eases into its target instead of overshooting and oscillating around it. At
// or beyond `slow_radius` this is `seek`; inside it the speed scales with the
// distance. Within `arrive_radius` the agent is considered arrived and the
// result is zero.
[[nodiscard]] Vec2 arrive_2d(const Vec2& position, const Vec2& target, f32 max_speed, f32 slow_radius,
                             f32 arrive_radius = 0.0f) noexcept;

// A persistent wander heading: `wander_angle` is advanced by a bounded random
// turn each call and the result is a unit vector along it, scaled by
// `max_speed`. The angle is passed by reference so the caller keeps the state
// between frames — wander is a heading that drifts, not per-frame noise.
[[nodiscard]] Vec2 wander_2d(f32& wander_angle, f32& rng_state, f32 max_speed, f32 turn_rate) noexcept;

// A push away from nearby agents, so a crowd spreads out instead of collapsing
// onto one point. Each neighbour within `radius` contributes a repulsion that
// grows as it gets closer; the result is summed and scaled to `max_speed`. An
// empty list yields zero.
[[nodiscard]] Vec2 separation_2d(const Vec2& position, const std::vector<Vec2>& neighbours, f32 radius,
                                 f32 max_speed) noexcept;

// A push away from a set of obstacle points (typically the blocked cells ahead of
// the agent), so it steers around a wall its path runs alongside. `look_ahead` is
// how far ahead of `forward` a point must be to count. An empty list yields zero.
[[nodiscard]] Vec2 avoidance_2d(const Vec2& forward, const std::vector<Vec2>& obstacles, f32 radius,
                                f32 max_speed) noexcept;

// --- 3D --------------------------------------------------------------------

[[nodiscard]] Vec3 seek_3d(const Vec3& position, const Vec3& target, f32 max_speed) noexcept;
[[nodiscard]] Vec3 flee_3d(const Vec3& position, const Vec3& target, f32 max_speed) noexcept;
[[nodiscard]] Vec3 arrive_3d(const Vec3& position, const Vec3& target, f32 max_speed, f32 slow_radius,
                             f32 arrive_radius = 0.0f) noexcept;
[[nodiscard]] Vec3 wander_3d(f32& yaw, f32& pitch, f32& rng_state, f32 max_speed, f32 turn_rate) noexcept;
[[nodiscard]] Vec3 separation_3d(const Vec3& position, const std::vector<Vec3>& neighbours, f32 radius,
                                 f32 max_speed) noexcept;
[[nodiscard]] Vec3 avoidance_3d(const Vec3& forward, const std::vector<Vec3>& obstacles, f32 radius,
                                f32 max_speed) noexcept;

} // namespace fe::nav
