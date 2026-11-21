// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// CPU-side tests for the particle path: the emitter's simulation, the
// extraction step that turns live particles into billboards, and the
// serialization boundary between authored parameters and runtime state. The
// billboards' pixels are covered by the graphics tests, which need a context.

#include <doctest/doctest.h>

#include "fe/core/Frustum.h"
#include "fe/render/RenderExtract.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/scene3d/Camera3D.h"
#include "fe/serialization/SceneSerializer.h"
#include "fe/systems/SceneSchedule.h"
#include "fe/vfx/ParticleEmitter3D.h"
#include "fe/vfx/ParticleSystem.h"
#include "fe/vfx/VfxTypes.h"

#include <vector>

using namespace fe;

namespace {

// Steps a schedule holding one ParticleSystem `steps` times with a fixed delta,
// which is the only way the simulation advances: there is no per-node update.
void step(SceneSchedule& schedule, Scene& scene, u32 steps, f32 delta) {
    FrameContext context;
    context.delta_time = delta;
    for (u32 i = 0; i < steps; ++i) {
        schedule.update(context, scene);
    }
}

} // namespace

TEST_CASE("an emitter emits at its rate over time") {
    Scene scene;
    Node* node = scene.create_node("Emitter");
    auto* emitter = node->add_component<ParticleEmitter3D>();
    emitter->emission_rate = 60.0f;
    emitter->lifetime_min = 10.0f;
    emitter->lifetime_max = 10.0f;
    emitter->start_speed = 0.0f;
    emitter->gravity = Vec3{};
    emitter->spread_degrees = 0.0f;

    SceneSchedule schedule;
    auto* system = schedule.add_system<ParticleSystem>();
    step(schedule, scene, 10, 1.0f / 60.0f);

    CHECK(system->last_particle_count() == 10);
    REQUIRE(emitter->particles.size() == 10);
}

TEST_CASE("a burst emits once, on the first update, and a rate of zero emits nothing after it") {
    Scene scene;
    Node* node = scene.create_node("Emitter");
    auto* emitter = node->add_component<ParticleEmitter3D>();
    emitter->burst = 5;
    emitter->emission_rate = 0.0f;
    emitter->lifetime_min = 10.0f;
    emitter->lifetime_max = 10.0f;

    SceneSchedule schedule;
    auto* system = schedule.add_system<ParticleSystem>();
    step(schedule, scene, 1, 1.0f / 60.0f);
    CHECK(system->last_particle_count() == 5);

    step(schedule, scene, 5, 1.0f / 60.0f);
    CHECK(system->last_particle_count() == 5);
}

TEST_CASE("a particle dies when its lifetime runs out") {
    Scene scene;
    Node* node = scene.create_node("Emitter");
    auto* emitter = node->add_component<ParticleEmitter3D>();
    emitter->burst = 4;
    emitter->emission_rate = 0.0f;
    emitter->lifetime_min = 0.5f;
    emitter->lifetime_max = 0.5f;

    SceneSchedule schedule;
    auto* system = schedule.add_system<ParticleSystem>();
    // Three steps of a quarter second take the particles past their half-second
    // life. The step is the system's maximum, so this also pins the clamp.
    step(schedule, scene, 1, ParticleSystem::kMaxStepSeconds);
    CHECK(system->last_particle_count() == 4);

    step(schedule, scene, 2, ParticleSystem::kMaxStepSeconds);
    CHECK(system->last_particle_count() == 0);
    CHECK(emitter->particles.empty());
}

TEST_CASE("gravity pulls a particle down over time") {
    Scene scene;
    Node* node = scene.create_node("Emitter");
    auto* emitter = node->add_component<ParticleEmitter3D>();
    emitter->burst = 1;
    emitter->emission_rate = 0.0f;
    emitter->lifetime_min = 10.0f;
    emitter->lifetime_max = 10.0f;
    emitter->start_speed = 0.0f;
    emitter->speed_spread = 0.0f;
    emitter->gravity = Vec3{0.0f, -10.0f, 0.0f};

    SceneSchedule schedule;
    schedule.add_system<ParticleSystem>();
    step(schedule, scene, 1, 0.1f);

    REQUIRE(emitter->particles.size() == 1);
    // Semi-implicit Euler: v = -1 m/s after the step, so the particle moves
    // -0.1 m in that same step.
    CHECK(emitter->particles[0].position.y == doctest::Approx(-0.1f));
    CHECK(emitter->particles[0].velocity.y == doctest::Approx(-1.0f));
}

TEST_CASE("an emitter aims along its node's local direction") {
    Scene scene;
    Node* node = scene.create_node("Emitter");
    // A quarter turn about X maps local +Y onto world +Z.
    node->transform().rotation = Vec3{3.14159265f * 0.5f, 0.0f, 0.0f};
    auto* emitter = node->add_component<ParticleEmitter3D>();
    emitter->burst = 1;
    emitter->emission_rate = 0.0f;
    emitter->lifetime_min = 10.0f;
    emitter->lifetime_max = 10.0f;
    emitter->start_speed = 1.0f;
    emitter->speed_spread = 0.0f;
    emitter->spread_degrees = 0.0f;
    emitter->gravity = Vec3{};

    SceneSchedule schedule;
    schedule.add_system<ParticleSystem>();
    step(schedule, scene, 1, 1.0f / 60.0f);

    REQUIRE(emitter->particles.size() == 1);
    const Vec3 velocity = emitter->particles[0].velocity;
    CHECK(velocity.x == doctest::Approx(0.0f).epsilon(1e-4f));
    CHECK(velocity.y == doctest::Approx(0.0f).epsilon(1e-4f));
    CHECK(velocity.z == doctest::Approx(1.0f).epsilon(1e-4f));
}

TEST_CASE("the same seed and the same steps produce the same particles") {
    // The determinism the tests rely on: a burst with a spread and a speed range
    // is random per particle but reproducible per emitter, which is what makes a
    // headless assertion on positions meaningful.
    const auto build = [] {
        auto scene = std::make_unique<Scene>();
        Node* node = scene->create_node("Emitter");
        auto* emitter = node->add_component<ParticleEmitter3D>();
        emitter->burst = 16;
        emitter->emission_rate = 20.0f;
        emitter->seed = 7;
        emitter->spread_degrees = 35.0f;
        emitter->start_speed = 3.0f;
        emitter->speed_spread = 0.5f;
        return std::make_pair(std::move(scene), emitter);
    };

    auto [first_scene, first_emitter] = build();
    auto [second_scene, second_emitter] = build();

    SceneSchedule first_schedule;
    first_schedule.add_system<ParticleSystem>();
    SceneSchedule second_schedule;
    second_schedule.add_system<ParticleSystem>();
    step(first_schedule, *first_scene, 20, 1.0f / 60.0f);
    step(second_schedule, *second_scene, 20, 1.0f / 60.0f);

    REQUIRE(first_emitter->particles.size() == second_emitter->particles.size());
    REQUIRE_FALSE(first_emitter->particles.empty());
    for (usize i = 0; i < first_emitter->particles.size(); ++i) {
        const Particle& a = first_emitter->particles[i];
        const Particle& b = second_emitter->particles[i];
        CHECK(a.position.x == doctest::Approx(b.position.x));
        CHECK(a.position.y == doctest::Approx(b.position.y));
        CHECK(a.position.z == doctest::Approx(b.position.z));
        CHECK(a.velocity.x == doctest::Approx(b.velocity.x));
        CHECK(a.velocity.y == doctest::Approx(b.velocity.y));
        CHECK(a.velocity.z == doctest::Approx(b.velocity.z));
    }
}

TEST_CASE("extraction interpolates a billboard's size and colour by age") {
    Scene scene;
    Node* node = scene.create_node("Emitter");
    auto* emitter = node->add_component<ParticleEmitter3D>();
    emitter->start_size = 1.0f;
    emitter->end_size = 3.0f;
    emitter->start_color = Vec4{1.0f, 0.0f, 0.0f, 1.0f};
    emitter->end_color = Vec4{0.0f, 0.0f, 1.0f, 0.0f};

    Particle particle;
    particle.position = Vec3{1.0f, 2.0f, 3.0f};
    particle.age = 0.5f;
    particle.lifetime = 1.0f;
    particle.start_size = emitter->start_size;
    particle.end_size = emitter->end_size;
    particle.start_color = emitter->start_color;
    particle.end_color = emitter->end_color;
    emitter->particles.push_back(particle);

    std::vector<render::ParticleInstance> billboards;
    render::extract_particles(scene, billboards);

    REQUIRE(billboards.size() == 1);
    CHECK(billboards[0].position.x == doctest::Approx(1.0f));
    CHECK(billboards[0].position.y == doctest::Approx(2.0f));
    CHECK(billboards[0].position.z == doctest::Approx(3.0f));
    CHECK(billboards[0].size == doctest::Approx(2.0f));
    CHECK(billboards[0].color.x == doctest::Approx(0.5f));
    CHECK(billboards[0].color.z == doctest::Approx(0.5f));
    CHECK(billboards[0].color.w == doctest::Approx(0.5f));
}

TEST_CASE("extraction drops a particle that would draw nothing") {
    Scene scene;
    Node* node = scene.create_node("Emitter");
    auto* emitter = node->add_component<ParticleEmitter3D>();

    Particle invisible;
    invisible.age = 1.0f;
    invisible.lifetime = 1.0f; // At the end of its life: fully transparent.
    invisible.start_size = 0.1f;
    invisible.end_size = 0.1f;
    invisible.start_color = Vec4{1.0f, 1.0f, 1.0f, 1.0f};
    invisible.end_color = Vec4{1.0f, 1.0f, 1.0f, 0.0f};
    emitter->particles.push_back(invisible);

    std::vector<render::ParticleInstance> billboards;
    render::extract_particles(scene, billboards);
    CHECK(billboards.empty());
}

TEST_CASE("culling drops a billboard whose centre is outside the frustum") {
    Scene scene;
    Node* camera_node = scene.create_node("Camera");
    camera_node->transform().position = Vec3{0.0f, 0.0f, 3.0f};
    auto* camera = camera_node->add_component<Camera3D>();
    const Frustum frustum = Frustum::from_view_projection(camera->view_projection(64, 64));

    std::vector<render::ParticleInstance> billboards(2);
    billboards[0].position = Vec3{0.0f, 0.0f, 0.0f};    // In front of the camera.
    billboards[1].position = Vec3{1000.0f, 0.0f, 0.0f}; // Far off to the side.

    CHECK(render::cull_particles(billboards, &frustum) == 1);
    REQUIRE(billboards.size() == 1);
    CHECK(billboards[0].position.x == doctest::Approx(0.0f));
}

TEST_CASE("the emitter's authored fields survive a round trip and its particles do not") {
    reflect::register_vfx_types();

    Scene original;
    Node* node = original.create_node("Sparks");
    auto* emitter = node->add_component<ParticleEmitter3D>();
    emitter->emitting = false;
    emitter->emission_rate = 12.5f;
    emitter->burst = 9;
    emitter->lifetime_min = 0.2f;
    emitter->lifetime_max = 0.9f;
    emitter->start_speed = 4.0f;
    emitter->speed_spread = 1.5f;
    emitter->spread_degrees = 45.0f;
    emitter->direction = Vec3{0.0f, 0.0f, -1.0f};
    emitter->gravity = Vec3{0.0f, -2.0f, 0.0f};
    emitter->start_size = 0.4f;
    emitter->end_size = 0.1f;
    emitter->start_color = Vec4{1.0f, 0.5f, 0.25f, 1.0f};
    emitter->end_color = Vec4{1.0f, 0.0f, 0.0f, 0.0f};
    emitter->max_particles = 64;
    emitter->seed = 42;

    // Runtime state that must not be written to the document.
    Particle live;
    live.position = Vec3{7.0f, 7.0f, 7.0f};
    emitter->particles.push_back(live);
    emitter->emit_accumulator = 0.75f;

    auto text = serialization::scene::to_json(original);
    REQUIRE(text);

    Scene restored;
    REQUIRE_FALSE(serialization::scene::from_json(text.value(), restored));

    Node* reloaded = restored.find_node_by_name("Sparks");
    REQUIRE(reloaded != nullptr);
    const auto* loaded = reloaded->get_component<ParticleEmitter3D>();
    REQUIRE(loaded != nullptr);
    CHECK_FALSE(loaded->emitting);
    CHECK(loaded->emission_rate == doctest::Approx(12.5f));
    CHECK(loaded->burst == 9);
    CHECK(loaded->lifetime_min == doctest::Approx(0.2f));
    CHECK(loaded->lifetime_max == doctest::Approx(0.9f));
    CHECK(loaded->start_speed == doctest::Approx(4.0f));
    CHECK(loaded->speed_spread == doctest::Approx(1.5f));
    CHECK(loaded->spread_degrees == doctest::Approx(45.0f));
    CHECK(loaded->direction.z == doctest::Approx(-1.0f));
    CHECK(loaded->gravity.y == doctest::Approx(-2.0f));
    CHECK(loaded->start_size == doctest::Approx(0.4f));
    CHECK(loaded->end_size == doctest::Approx(0.1f));
    CHECK(loaded->start_color.y == doctest::Approx(0.5f));
    CHECK(loaded->end_color.w == doctest::Approx(0.0f));
    CHECK(loaded->max_particles == 64);
    CHECK(loaded->seed == 42);

    // The live particles are simulation output, not authored state: a reloaded
    // scene starts with none.
    CHECK(loaded->particles.empty());
    CHECK(loaded->emit_accumulator == doctest::Approx(0.0f));
}
