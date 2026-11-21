// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Types.h"
#include "fe/systems/System.h"

#include <string_view>

namespace fe {

class ParticleEmitter3D;

// Advances every `ParticleEmitter3D` in the scene.
//
// The system owns the simulation and the emitter owns the data: for each emitter
// it spawns the particles this frame's time asks for, integrates the live ones,
// and drops the ones whose life has run out. The renderer never simulates, it
// only reads the resulting array, so a headless test steps this system and
// asserts on positions with no GPU, and a paused editor can stop stepping it to
// freeze the effect.
//
// Determinism: the random stream is a per-emitter xorshift seeded from the
// emitter's `seed`, advanced only by this system. Two runs with the same seed,
// parameters, transforms, and time steps produce identical particles, which is
// what the tests assert. `FrameContext::delta_time` is clamped to a quarter of a
// second so a debugger break or a window drag cannot teleport a burst across the
// scene.
class ParticleSystem final : public System {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "ParticleSystem"; }

    void update(const FrameContext& context, Scene& scene) override;

    // The number of live particles across the whole scene after the last update.
    // Reported so a test can assert emission and ageing without walking the scene
    // itself.
    [[nodiscard]] usize last_particle_count() const noexcept { return last_particle_count_; }

    // The largest delta the system will integrate in one step, in seconds.
    static constexpr f32 kMaxStepSeconds = 0.25f;

private:
    usize last_particle_count_ = 0;
};

} // namespace fe
