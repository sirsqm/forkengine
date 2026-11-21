// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/vfx/ParticleSystem.h"

#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/vfx/ParticleEmitter3D.h"

#include <algorithm>
#include <cmath>

namespace fe {
namespace {

// xorshift32. Small, fast, and identical on every platform, which is what the
// determinism test relies on; it is not a cryptographic generator and does not
// need to be.
[[nodiscard]] u32 next_random(u32& state) noexcept {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

// A uniform value in [0, 1). The top 24 bits are enough for a float's mantissa,
// and taking them from the high end avoids the weak low bits of a xorshift.
[[nodiscard]] f32 random_unit(u32& state) noexcept {
    return static_cast<f32>(next_random(state) >> 8) * (1.0f / 16777216.0f);
}

// Any unit vector perpendicular to `axis`. The reference is chosen from the axis
// component that is smallest in magnitude, so the cross product is never near
// zero and the basis stays stable for every aim.
[[nodiscard]] Vec3 perpendicular(const Vec3& axis) noexcept {
    const Vec3 reference = std::abs(axis.y) < 0.9f ? Vec3{0.0f, 1.0f, 0.0f} : Vec3{1.0f, 0.0f, 0.0f};
    return axis.cross(reference).normalized();
}

// A direction inside a cone of `spread_radians` around `axis`, uniformly
// distributed over the cone's solid angle. Sampling cos(theta) uniformly is what
// makes the distribution uniform rather than clustered at the axis.
[[nodiscard]] Vec3 random_direction(u32& state, const Vec3& axis, f32 spread_radians) noexcept {
    if (spread_radians <= 0.0f) {
        return axis;
    }
    const Vec3 right = perpendicular(axis);
    const Vec3 up = axis.cross(right).normalized();
    const f32 cos_max = std::cos(std::min(spread_radians, 3.14159265f));
    const f32 cos_theta = cos_max + (1.0f - cos_max) * random_unit(state);
    const f32 sin_theta = std::sqrt(std::max(0.0f, 1.0f - cos_theta * cos_theta));
    const f32 phi = 6.28318531f * random_unit(state);
    return (axis * cos_theta + (right * std::cos(phi) + up * std::sin(phi)) * sin_theta).normalized();
}

void spawn_particle(ParticleEmitter3D& emitter, const Mat4& world) {
    if (emitter.particles.size() >= emitter.max_particles) {
        return;
    }

    const Vec3 origin = transform_point(world, Vec3{});
    const Vec3 local_axis = emitter.direction.length_squared() > 0.0f ? emitter.direction : Vec3{0.0f, 1.0f, 0.0f};
    Vec3 axis = transform_direction(world, local_axis);
    // A node scaled to zero would collapse the axis; fall back to world up rather
    // than dividing by zero and emitting NaNs into the scene.
    axis = axis.length_squared() > 0.0f ? axis.normalized() : Vec3{0.0f, 1.0f, 0.0f};

    const Vec3 direction = random_direction(emitter.rng_state, axis, radians(emitter.spread_degrees));
    const f32 speed =
        std::max(0.0f, emitter.start_speed + emitter.speed_spread * (2.0f * random_unit(emitter.rng_state) - 1.0f));
    const f32 lifetime =
        emitter.lifetime_min + (emitter.lifetime_max - emitter.lifetime_min) * random_unit(emitter.rng_state);

    Particle particle;
    particle.position = origin;
    particle.velocity = direction * speed;
    particle.age = 0.0f;
    particle.lifetime = std::max(lifetime, 1e-3f);
    particle.start_size = emitter.start_size;
    particle.end_size = emitter.end_size;
    particle.start_color = emitter.start_color;
    particle.end_color = emitter.end_color;
    emitter.particles.push_back(particle);
}

void step_emitter(ParticleEmitter3D& emitter, const Mat4& world, f32 dt) {
    if (!emitter.started) {
        emitter.started = true;
        emitter.rng_state = emitter.seed != 0 ? emitter.seed : 1u;
        for (u32 i = 0; i < emitter.burst; ++i) {
            spawn_particle(emitter, world);
        }
    }

    // Integrate first, then emit: a particle born this frame starts at its
    // origin, which is what a burst should look like on the frame it happens.
    for (Particle& particle : emitter.particles) {
        particle.velocity += emitter.gravity * dt;
        particle.position += particle.velocity * dt;
        particle.age += dt;
    }
    std::erase_if(emitter.particles, [](const Particle& particle) { return particle.age >= particle.lifetime; });

    if (!emitter.emitting || emitter.emission_rate <= 0.0f || dt <= 0.0f) {
        return;
    }
    emitter.emit_accumulator += emitter.emission_rate * dt;
    while (emitter.emit_accumulator >= 1.0f) {
        emitter.emit_accumulator -= 1.0f;
        if (emitter.particles.size() >= emitter.max_particles) {
            continue; // Full: drop the emission rather than recycle a live particle.
        }
        spawn_particle(emitter, world);
    }
}

} // namespace

void ParticleSystem::update(const FrameContext& context, Scene& scene) {
    last_particle_count_ = 0;
    const f32 dt = std::clamp(context.delta_time, 0.0f, kMaxStepSeconds);

    scene.for_each_node([&](Node& node) {
        auto* emitter = node.get_component<ParticleEmitter3D>();
        if (emitter == nullptr) {
            return;
        }
        step_emitter(*emitter, node.world_matrix(), dt);
        last_particle_count_ += emitter->particles.size();
    });
}

} // namespace fe
