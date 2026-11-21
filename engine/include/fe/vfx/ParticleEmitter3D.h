// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Math.h"
#include "fe/core/Types.h"
#include "fe/scene/Component.h"

#include <vector>

namespace fe {

// One live particle.
//
// This is runtime state, not authored data: `ParticleSystem` advances it and the
// renderer's extraction step reads it to emit a billboard. It is deliberately
// not reflected, so a scene saved from the editor stores the emitter's
// parameters and not the particles that happened to be alive at the save.
//
// The start/end fields are copied onto the particle when it is born rather than
// read from the emitter at draw time. That is what lets a designer change the
// emitter while a burst is in flight without every existing particle changing
// size and colour retroactively -- the same reason a sound keeps the parameters
// it started with.
struct Particle {
    Vec3 position{};
    Vec3 velocity{};
    f32 age = 0.0f;
    f32 lifetime = 1.0f;
    f32 start_size = 0.1f;
    f32 end_size = 0.0f;
    Vec4 start_color{1.0f, 1.0f, 1.0f, 1.0f};
    Vec4 end_color{1.0f, 1.0f, 1.0f, 0.0f};
};

// Emits short-lived billboards from its node.
//
// The emitter is the authoring half and `ParticleSystem` is the execution half:
// the component carries the parameters a designer edits, the system advances a
// flat particle array on it, and the 3D renderer draws that array through its
// transparent pass. The split mirrors `Animator` (whose palette the animation
// system fills and the renderer reads) rather than putting simulation in the
// renderer, so particles are deterministic and testable with no GPU.
//
// Emission is a rate while `emitting` is set plus a one-shot `burst` when the
// system first sees the emitter. A particle is born at the node's world position
// and travels along the node's local `direction` (so rotating the node aims the
// emitter), within a cone of `spread_degrees`. Size and colour interpolate from
// the start pair to the end pair over the particle's life, and `gravity` is a
// constant acceleration in world units per second squared.
//
// The default is a small upward fountain that fades out: useful enough to drop on
// a node and see something, and cheap enough that an emitter with a low rate
// costs a handful of billboards.
class ParticleEmitter3D final : public Component {
public:
    static constexpr const char* kTypeName = "ParticleEmitter3D";

    [[nodiscard]] const char* type_name() const noexcept override { return kTypeName; }

    // --- Authored ----------------------------------------------------------

    // Whether the emitter is emitting. Turning it off stops new particles; the
    // live ones keep moving and fade out, so a burst does not vanish mid-air.
    bool emitting = true;

    // Particles per second while emitting. 0 is a pure burst emitter.
    f32 emission_rate = 30.0f;

    // Particles emitted once, the first time the system sees the emitter. This is
    // the "a gem was collected" spark, as opposed to the continuous smoke a rate
    // describes.
    u32 burst = 0;

    // Lifetime range, in seconds. Each particle samples uniformly between them, so
    // a burst does not all die on the same frame and read as a single flicker.
    f32 lifetime_min = 0.6f;
    f32 lifetime_max = 1.2f;

    // Initial speed, in world units per second, and the symmetric spread around
    // it. A particle's speed is `start_speed +/- speed_spread`.
    f32 start_speed = 2.0f;
    f32 speed_spread = 0.5f;

    // Half-angle of the emission cone, in degrees, around `direction`.
    f32 spread_degrees = 20.0f;

    // Emission axis in the node's local space. Rotating the node aims the
    // emitter, which is why this is not a world-space vector.
    Vec3 direction{0.0f, 1.0f, 0.0f};

    // Constant acceleration applied to every live particle, in world units per
    // second squared. Downward by default, so a fountain falls.
    Vec3 gravity{0.0f, -9.8f, 0.0f};

    // Size in world units at birth and at death. A size of 0 at the end shrinks
    // the particle away instead of popping it out.
    f32 start_size = 0.15f;
    f32 end_size = 0.02f;

    // Colour and alpha at birth and at death, interpolated over the lifetime.
    Vec4 start_color{1.0f, 1.0f, 1.0f, 1.0f};
    Vec4 end_color{1.0f, 1.0f, 1.0f, 0.0f};

    // Hard cap on live particles. Emission past it is dropped rather than
    // overwriting a live particle, so a rate that is too high for the cap shows up
    // as a thinner stream, not as particles that never finish their life.
    u32 max_particles = 256;

    // Seed for the emitter's random stream. Two emitters with the same seed,
    // parameters, and time steps produce the same particles, which is what makes
    // the simulation testable frame for frame.
    u32 seed = 1;

    // --- Runtime -----------------------------------------------------------
    // Advanced by `ParticleSystem`, read by the renderer. Never serialized: see
    // `Particle`.
    std::vector<Particle> particles;

    // Fractional particle carried between frames, so a rate of 30 at 60 Hz emits
    // one every other frame instead of rounding to zero.
    f32 emit_accumulator = 0.0f;

    // Whether the first update has run, which is when `burst` is emitted and the
    // random stream is seeded.
    bool started = false;

    // The emitter's random stream. Advanced only by the system.
    u32 rng_state = 0;
};

} // namespace fe
