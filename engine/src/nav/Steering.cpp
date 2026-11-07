// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/nav/Steering.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace fe::nav {
namespace {

// A small, deterministic pseudo-random number generator (a 32-bit xorshift). The
// engine has no global RNG and does not want one: a steering behaviour that
// reached for `std::rand` would make a replay non-deterministic, which breaks the
// operation-model pillar (ADR-0047) for anything that records an agent's motion.
// The caller owns the state word, so two agents wander differently and a test
// that seeds it gets the same walk every run.
[[nodiscard]] f32 next_random(f32& state) noexcept {
    // Reinterpret the float state word as a u32 bit pattern and mix it.
    u32 bits = 0;
    std::memcpy(&bits, &state, sizeof(bits));
    if (bits == 0u) {
        bits = 0x9e3779b9u;
    }
    bits ^= bits << 13u;
    bits ^= bits >> 17u;
    bits ^= bits << 5u;
    std::memcpy(&state, &bits, sizeof(state));
    // Map to [-1, 1].
    return static_cast<f32>(bits) / 2147483648.0f - 1.0f;
}

// Clamps a vector to `max_speed`, leaving shorter vectors untouched. A non-
// positive max speed yields a zero vector.
template <typename V>
[[nodiscard]] V clamp_speed(const V& v, f32 max_speed) noexcept {
    const f32 length = v.length();
    if (max_speed <= 0.0f) {
        return V{};
    }
    if (length <= max_speed || length <= 1e-6f) {
        return v;
    }
    return v / length * max_speed;
}

} // namespace

Vec2 seek_2d(const Vec2& position, const Vec2& target, f32 max_speed) noexcept {
    const Vec2 to_target = target - position;
    const f32 length = to_target.length();
    if (length <= 1e-6f) {
        return Vec2{};
    }
    return to_target / length * max_speed;
}

Vec2 flee_2d(const Vec2& position, const Vec2& target, f32 max_speed) noexcept {
    return seek_2d(position, target, max_speed) * -1.0f;
}

Vec2 arrive_2d(const Vec2& position, const Vec2& target, f32 max_speed, f32 slow_radius, f32 arrive_radius) noexcept {
    const Vec2 to_target = target - position;
    const f32 distance = to_target.length();
    if (distance <= arrive_radius || distance <= 1e-6f) {
        return Vec2{};
    }
    const f32 speed = distance < slow_radius ? max_speed * (distance / slow_radius) : max_speed;
    return to_target / distance * speed;
}

Vec2 wander_2d(f32& wander_angle, f32& rng_state, f32 max_speed, f32 turn_rate) noexcept {
    wander_angle += next_random(rng_state) * turn_rate;
    return Vec2{std::cos(wander_angle), std::sin(wander_angle)} * max_speed;
}

Vec2 separation_2d(const Vec2& position, const std::vector<Vec2>& neighbours, f32 radius, f32 max_speed) noexcept {
    if (radius <= 0.0f) {
        return Vec2{};
    }
    Vec2 push{};
    for (const Vec2& neighbour : neighbours) {
        const Vec2 away = position - neighbour;
        const f32 distance = away.length();
        if (distance <= 1e-6f || distance > radius) {
            continue;
        }
        // Weight by how deep into the radius the neighbour is, so a distant one
        // barely nudges and a close one pushes hard.
        push += away / distance * ((radius - distance) / radius);
    }
    return clamp_speed(push, max_speed);
}

Vec2 avoidance_2d(const Vec2& forward, const std::vector<Vec2>& obstacles, f32 radius, f32 max_speed) noexcept {
    if (radius <= 0.0f) {
        return Vec2{};
    }
    Vec2 push{};
    for (const Vec2& obstacle : obstacles) {
        const f32 distance = obstacle.length();
        if (distance <= 1e-6f || distance > radius) {
            continue;
        }
        // Only steer away from an obstacle that lies ahead (its direction has a
        // positive component along the intended heading), so the agent does not
        // dodge something it has already passed.
        const Vec2 direction = obstacle / distance;
        if (forward.dot(direction) <= 0.0f) {
            continue;
        }
        push += direction * -1.0f * ((radius - distance) / radius);
    }
    return clamp_speed(push, max_speed);
}

Vec3 seek_3d(const Vec3& position, const Vec3& target, f32 max_speed) noexcept {
    const Vec3 to_target = target - position;
    const f32 length = to_target.length();
    if (length <= 1e-6f) {
        return Vec3{};
    }
    return to_target / length * max_speed;
}

Vec3 flee_3d(const Vec3& position, const Vec3& target, f32 max_speed) noexcept {
    return seek_3d(position, target, max_speed) * -1.0f;
}

Vec3 arrive_3d(const Vec3& position, const Vec3& target, f32 max_speed, f32 slow_radius, f32 arrive_radius) noexcept {
    const Vec3 to_target = target - position;
    const f32 distance = to_target.length();
    if (distance <= arrive_radius || distance <= 1e-6f) {
        return Vec3{};
    }
    const f32 speed = distance < slow_radius ? max_speed * (distance / slow_radius) : max_speed;
    return to_target / distance * speed;
}

Vec3 wander_3d(f32& yaw, f32& pitch, f32& rng_state, f32 max_speed, f32 turn_rate) noexcept {
    yaw += next_random(rng_state) * turn_rate;
    pitch += next_random(rng_state) * turn_rate;
    // Keep the pitch within a right angle of the horizon so a wanderer never
    // inverts.
    pitch = std::clamp(pitch, -1.5f, 1.5f);
    const f32 cos_pitch = std::cos(pitch);
    return Vec3{std::cos(yaw) * cos_pitch, std::sin(pitch), std::sin(yaw) * cos_pitch} * max_speed;
}

Vec3 separation_3d(const Vec3& position, const std::vector<Vec3>& neighbours, f32 radius, f32 max_speed) noexcept {
    if (radius <= 0.0f) {
        return Vec3{};
    }
    Vec3 push{};
    for (const Vec3& neighbour : neighbours) {
        const Vec3 away = position - neighbour;
        const f32 distance = away.length();
        if (distance <= 1e-6f || distance > radius) {
            continue;
        }
        push += away / distance * ((radius - distance) / radius);
    }
    return clamp_speed(push, max_speed);
}

Vec3 avoidance_3d(const Vec3& forward, const std::vector<Vec3>& obstacles, f32 radius, f32 max_speed) noexcept {
    if (radius <= 0.0f) {
        return Vec3{};
    }
    Vec3 push{};
    for (const Vec3& obstacle : obstacles) {
        const f32 distance = obstacle.length();
        if (distance <= 1e-6f || distance > radius) {
            continue;
        }
        const Vec3 direction = obstacle / distance;
        if (forward.dot(direction) <= 0.0f) {
            continue;
        }
        push += direction * -1.0f * ((radius - distance) / radius);
    }
    return clamp_speed(push, max_speed);
}

} // namespace fe::nav
