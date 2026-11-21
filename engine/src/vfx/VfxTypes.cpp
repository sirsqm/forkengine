// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/vfx/VfxTypes.h"

#include "fe/reflect/PropertyBuilders.h"
#include "fe/reflect/Registry.h"
#include "fe/vfx/ParticleEmitter3D.h"

#include <memory>

// The particle emitter's registration, in the same style as the scene3d block:
// the component's authored fields are described once here, and the serializer and
// the inspector read that description rather than naming the type. The runtime
// fields (`particles`, `emit_accumulator`, `started`, `rng_state`) are
// deliberately absent, so a saved scene carries parameters and not live
// particles.

namespace fe::reflect {
namespace {

void register_particle_emitter3d(Registry& registry) {
    ParticleEmitter3D defaults;

    TypeDesc desc;
    desc.name = ParticleEmitter3D::kTypeName;
    desc.factory = [] { return std::unique_ptr<Component>(std::make_unique<ParticleEmitter3D>()); };

    desc.properties.push_back(
        make_scalar<ParticleEmitter3D, bool>("emitting", &ParticleEmitter3D::emitting, ValueKind::Bool, &defaults));
    desc.properties.push_back(make_scalar<ParticleEmitter3D, f32>("emission_rate", &ParticleEmitter3D::emission_rate,
                                                                  ValueKind::Float, &defaults));
    desc.properties.push_back(
        make_scalar<ParticleEmitter3D, u32>("burst", &ParticleEmitter3D::burst, ValueKind::Int, &defaults));
    desc.properties.push_back(make_scalar<ParticleEmitter3D, f32>("lifetime_min", &ParticleEmitter3D::lifetime_min,
                                                                  ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<ParticleEmitter3D, f32>("lifetime_max", &ParticleEmitter3D::lifetime_max,
                                                                  ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<ParticleEmitter3D, f32>("start_speed", &ParticleEmitter3D::start_speed,
                                                                  ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<ParticleEmitter3D, f32>("speed_spread", &ParticleEmitter3D::speed_spread,
                                                                  ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<ParticleEmitter3D, f32>("spread_degrees", &ParticleEmitter3D::spread_degrees,
                                                                  ValueKind::Float, &defaults));
    desc.properties.push_back(make_vector_group<ParticleEmitter3D, Vec3>("direction", &ParticleEmitter3D::direction,
                                                                         "Vec3", {"x", "y", "z"}));
    desc.properties.push_back(
        make_vector_group<ParticleEmitter3D, Vec3>("gravity", &ParticleEmitter3D::gravity, "Vec3", {"x", "y", "z"}));
    desc.properties.push_back(
        make_scalar<ParticleEmitter3D, f32>("start_size", &ParticleEmitter3D::start_size, ValueKind::Float, &defaults));
    desc.properties.push_back(
        make_scalar<ParticleEmitter3D, f32>("end_size", &ParticleEmitter3D::end_size, ValueKind::Float, &defaults));
    desc.properties.push_back(make_vector_group<ParticleEmitter3D, Vec4>("start_color", &ParticleEmitter3D::start_color,
                                                                         "Vec4", {"r", "g", "b", "a"}));
    desc.properties.push_back(make_vector_group<ParticleEmitter3D, Vec4>("end_color", &ParticleEmitter3D::end_color,
                                                                         "Vec4", {"r", "g", "b", "a"}));
    desc.properties.push_back(make_scalar<ParticleEmitter3D, u32>("max_particles", &ParticleEmitter3D::max_particles,
                                                                  ValueKind::Int, &defaults));
    desc.properties.push_back(
        make_scalar<ParticleEmitter3D, u32>("seed", &ParticleEmitter3D::seed, ValueKind::Int, &defaults));

    registry.register_type(std::move(desc));
}

} // namespace

void register_vfx_types() {
    Registry& registry = Registry::instance();
    register_particle_emitter3d(registry);
}

} // namespace fe::reflect
