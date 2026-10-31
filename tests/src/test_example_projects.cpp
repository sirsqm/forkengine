// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include <doctest/doctest.h>

#include "fe/renderer2d/Renderer2D.h"
#include "fe/renderer3d/Renderer3D.h"
#include "fe/runtime/ComponentRegistration.h"
#include "fe/scene/Camera2D.h"
#include "fe/scene/Scene.h"
#include "fe/scene3d/Camera3D.h"
#include "fe/serialization/SceneSerializer.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

using namespace fe;

namespace {

std::string read_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    REQUIRE_MESSAGE(in.good(), "cannot open " << path.string());
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

// Loads a shipped example project's startup scene the way the editor does: from
// its own directory, through the project file, with every component type
// registered.
void load_startup_scene(const std::string& project_dir, Scene& scene) {
    runtime::register_all_component_types();

    const std::filesystem::path root = std::filesystem::path(FE_SOURCE_DIR) / "examples" / project_dir;
    const std::string project = read_file(root / "fork.project.json");

    // The project file names the startup scene; the examples are hand-written,
    // so a plain substring read is enough to pull it out without a JSON parse.
    const std::string key = "\"startup_scene\": \"";
    const std::size_t begin = project.find(key);
    REQUIRE(begin != std::string::npos);
    const std::size_t value_begin = begin + key.size();
    const std::size_t end = project.find('"', value_begin);
    REQUIRE(end != std::string::npos);
    const std::string scene_rel = project.substr(value_begin, end - value_begin);

    const std::string document = read_file(root / scene_rel);
    const Error error = serialization::scene::from_json(document, scene);
    REQUIRE_MESSAGE(!error, "loading " << project_dir << " failed: " << error.message());
}

} // namespace

// A shipped example is the first thing a new user opens, so it must not ship in
// a state where its own content does not render. The 2D and 3D renderers both
// draw nothing when the scene has no active camera, so a scene authored for one
// of them must carry the matching camera. `examples/demo_project` shipped
// without a `Camera2D`: the 2D fallback projects in raw pixel space, so the
// sprites laid out around the origin were drawn off-centre and mostly clipped,
// which read as "the viewport is broken" (ADR-0114).
TEST_CASE("every shipped 2D example scene has an active Camera2D") {
    Scene scene;
    load_startup_scene("demo_project", scene);
    CHECK(renderer2d::Renderer2D::find_active_camera(scene) != nullptr);
}

TEST_CASE("every shipped 3D example scene has an active Camera3D") {
    Scene scene;
    load_startup_scene("demo3d_project", scene);
    CHECK(renderer3d::Renderer3D::find_active_camera(scene) != nullptr);
}

TEST_CASE("the 2D demo's camera frames the sprites it was authored around") {
    // The bug was a missing camera, not a wrong one, so guard the property that
    // made it visible: the camera's reference box is the space the sprites sit
    // in. A camera parked at the origin with no reference box would leave the
    // same off-centre result.
    Scene scene;
    load_startup_scene("demo_project", scene);
    const Camera2D* camera = renderer2d::Renderer2D::find_active_camera(scene);
    REQUIRE(camera != nullptr);
    CHECK(camera->reference_width > 0.0f);
    CHECK(camera->reference_height > 0.0f);

    const Vec3 centre = transform_point(camera->node()->world_matrix(), Vec3{});
    CHECK(centre.x > 0.0f);
    CHECK(centre.y > 0.0f);
}
