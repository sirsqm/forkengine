// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// The runtime integration, end to end: a real Application embeds an Engine,
// runs a fixed number of headless frames, and the profiler reports those frames.
// This is what proves the profiler is fed by the actual frame loop rather than
// only by hand-written test scopes. It needs a GL context, so it lives with the
// graphics tests and skips cleanly when none is available.

#include <doctest/doctest.h>

#include "fe/core/Profiler.h"
#include "fe/runtime/Application.h"
#include "fe/scene/Camera2D.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/scene/Sprite2D.h"
#include "fe/scene3d/Camera3D.h"
#include "fe/systems/System.h"

#include <array>
#include <span>
#include <string>
#include <string_view>

using namespace fe;

namespace {

// A system that does a little work so it shows up in the frame breakdown.
class SpinSystem final : public System {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "SpinSystem"; }
    void update(const FrameContext&, Scene&) override {
        volatile u64 acc = 0;
        for (u64 i = 0; i < 1000; ++i) {
            acc += i;
        }
        (void)acc;
    }
};

// A tiny application that sets up a camera, one system, and a fixed headless
// frame count. It captures the last frame report in on_stop(), while the engine
// (and the profiler) are still alive.
class ProbeApp final : public Application {
public:
    ProbeApp(u32 frames, bool profiling) : Application(make_config()) {
        set_headless(frames);
        set_profiling(profiling);
    }

    Result<FrameReport> report{Error("not run")};
    bool started = false;

protected:
    Error on_start() override {
        started = true;
        auto* camera_node = scene().create_node("Camera");
        auto* camera = camera_node->add_component<Camera2D>();
        camera->primary = true;
        schedule().add_system<SpinSystem>();
        return Error::none();
    }

    void on_stop() override { report = last_frame_report(); }

private:
    static Engine::Config make_config() {
        Engine::Config config;
        config.window.title = "fork engine profiler runtime test";
        config.window.width = 64;
        config.window.height = 64;
        config.window.visible = false;
        config.window.vsync = false;
        return config;
    }
};

} // namespace

TEST_CASE("the runtime frame loop feeds the profiler and yields a report") {
    // A GL context may be unavailable on a machine with no display; Application
    // then fails to initialise and run() returns non-zero. Report and pass, the
    // same convention the renderer tests use, so ctest stays green.
    ProbeApp app(4, true);
    const int code = app.run();
    if (!app.started) {
        MESSAGE("graphics context unavailable; runtime profiler test skipped");
        return;
    }
    REQUIRE(code == 0);

    // The loop committed one frame per iteration.
    CHECK(app.profiler().frame_count() == 4);

    const Result<FrameReport> report = app.report;
    REQUIRE(report);
    const FrameReport& r = report.value();

    // The frame was bracketed by the runtime, not by the test.
    CHECK(r.total_ms >= 0.0);
    CHECK(r.update_ms >= 0.0);
    CHECK(r.render_ms >= 0.0);

    // The two top-level regions and the per-system sample are all present,
    // proving the loop and the schedule both fed the profiler.
    bool saw_update = false;
    bool saw_render = false;
    bool saw_system = false;
    for (u32 i = 0; i < r.scope_count; ++i) {
        saw_update = saw_update || r.scopes[i].name == "frame/update";
        saw_render = saw_render || r.scopes[i].name == "frame/render";
        saw_system = saw_system || r.scopes[i].name == "system/SpinSystem";
    }
    CHECK(saw_update);
    CHECK(saw_render);
    CHECK(saw_system);

    // The breakdown is a snapshot the caller can print, headless.
    CHECK(r.summary().find("frame ") == 0);
}

TEST_CASE("an application with profiling disabled commits no frames") {
    ProbeApp app(3, false);
    const int code = app.run();
    if (!app.started) {
        MESSAGE("graphics context unavailable; runtime profiler test skipped");
        return;
    }
    REQUIRE(code == 0);
    CHECK(app.profiler().frame_count() == 0);
    CHECK_FALSE(app.report); // last_frame_report() reports that profiling is off
}

namespace {

// A 3D-only application (no Camera2D) with an empty scene, so the whole frame is
// the clear colour. It samples the centre pixel in on_render -- after every pass,
// while the frame is still the bound target -- which is where the clear colour
// actually lands.
class ClearColorApp final : public Application {
public:
    explicit ClearColorApp(Engine::Config config) : Application(config) {}

    std::array<u8, 4> centre{};
    bool started = false;

protected:
    Error on_start() override {
        started = true;
        auto* camera = scene().create_node("Camera")->add_component<Camera3D>();
        // A colour no default would produce, so a black or default clear is
        // distinguishable from the camera's own.
        camera->clear_r = 0.25f;
        camera->clear_g = 0.50f;
        camera->clear_b = 0.75f;
        return Error::none();
    }

    void on_render(f32) override {
        const u32 w = engine().window().framebuffer_width();
        const u32 h = engine().window().framebuffer_height();
        engine().device().read_pixels(w / 2, h / 2, 1, 1, std::span<u8>(centre.data(), centre.size()));
    }
};

} // namespace

TEST_CASE("a 3D-only scene clears the frame to its camera's colour") {
    Engine::Config config;
    config.window.title = "fork engine 3d clear colour test";
    config.window.width = 64;
    config.window.height = 64;
    config.window.visible = false;
    config.window.vsync = false;

    ClearColorApp app(config);
    app.set_headless(1);
    const int code = app.run();
    if (!app.started) {
        MESSAGE("graphics context unavailable; 3D clear colour test skipped");
        return;
    }
    REQUIRE(code == 0);

    // The frame's centre is the camera's clear colour, not black: 0.25/0.50/0.75
    // in [0,1] is 64/128/191 in 8-bit, with a tolerance for the exact rounding
    // the driver applies.
    CHECK(app.centre[0] == doctest::Approx(64).epsilon(0.02));
    CHECK(app.centre[1] == doctest::Approx(128).epsilon(0.02));
    CHECK(app.centre[2] == doctest::Approx(191).epsilon(0.02));
    CHECK(app.centre[3] == 255);
}
