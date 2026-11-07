// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include <doctest/doctest.h>

#include "slice_scene.h"
#include "slice_state.h"

#include "fe/physics3d/Collider3D.h"
#include "fe/physics3d/RigidBody3D.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/scene3d/Camera3D.h"
#include "fe/scene3d/MeshRenderer3D.h"
#include "fe/serialization/SceneSerializer.h"

#include <filesystem>

// The collect3d vertical slice's arena tests (the M11 3D game).
//
// The arena builder is compiled from the example's own source (see
// tests/CMakeLists.txt), so this is the *shipped* layout, not a copy. A scene is
// built in memory with no window, GPU, or sound card, which is what lets this run
// in CI. A missing asset never fails a test: the builder treats the mesh and the
// font as optional, so the arena and its colliders are identical with or without
// them.

using namespace fe;

namespace {

int count_collectibles(const Scene& scene) {
    int gems = 0;
    scene.for_each_node([&](const Node& node) {
        if (node.name() == collect3d::names::kCollectible) {
            ++gems;
        }
    });
    return gems;
}

Node* first_collectible(Scene& scene) {
    Node* found = nullptr;
    scene.for_each_node([&](Node& node) {
        if (found == nullptr && node.name() == collect3d::names::kCollectible) {
            found = &node;
        }
    });
    return found;
}

} // namespace

// ---------------------------------------------------------------------------
// The arena builder: the layout the systems and the scene file share.
// ---------------------------------------------------------------------------

TEST_CASE("the built arena has a player, a full ring of gems, and a camera") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);

    CHECK(scene.find_node_by_name(collect3d::names::kPlayer) != nullptr);
    CHECK(count_collectibles(scene) == collect3d::config::kCollectibleCount);
    CHECK(scene.find_node_by_name("Camera") != nullptr);
    CHECK(scene.find_node_by_name("Floor") != nullptr);
    CHECK(scene.find_node_by_name(collect3d::names::kHudCanvas) != nullptr);

    const Node* camera = scene.find_node_by_name("Camera");
    REQUIRE(camera != nullptr);
    CHECK(camera->get_component<Camera3D>() != nullptr);
}

TEST_CASE("the player carries a collider and a gravity-free body") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);
    Node* player = scene.find_node_by_name(collect3d::names::kPlayer);
    REQUIRE(player != nullptr);
    CHECK(player->get_component<physics3d::Collider3D>() != nullptr);
    const auto* body = player->get_component<physics3d::RigidBody3D>();
    REQUIRE(body != nullptr);
    // Gravity is off because the character controller owns the vertical axis.
    CHECK_FALSE(body->affected_by_gravity);
}

TEST_CASE("a collectible has a mesh but no collider, so the player walks through it") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);
    Node* gem = first_collectible(scene);
    REQUIRE(gem != nullptr);
    CHECK(gem->get_component<MeshRenderer3D>() != nullptr);
    CHECK(gem->get_component<physics3d::Collider3D>() == nullptr);
}

TEST_CASE("the arena scene round-trips through the serializer") {
    // The scene file is a serialization of the code-built arena, so this proves
    // the file path and the code path build the same board.
    Scene built;
    collect3d::build_collect3d_scene(built);

    const std::filesystem::path path = std::filesystem::temp_directory_path() / "collect3d_roundtrip.fescene";
    REQUIRE_FALSE(serialization::scene::save(built, path.string()));
    CHECK(std::filesystem::exists(path));

    Scene loaded;
    REQUIRE_FALSE(serialization::scene::load(path.string(), loaded));
    CHECK(loaded.find_node_by_name(collect3d::names::kPlayer) != nullptr);
    CHECK(count_collectibles(loaded) == collect3d::config::kCollectibleCount);
    std::filesystem::remove(path);
}
