// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include <doctest/doctest.h>

#include "fe/physics2d/DistanceJoint2D.h"
#include "fe/physics2d/PhysicsTypes.h"
#include "fe/reflect/ComponentTypes.h"
#include "fe/scene/Camera2D.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/scene/Sprite2D.h"
#include "fe/serialization/Prefab.h"
#include "fe/serialization/SceneSerializer.h"

#include <string>

using namespace fe;

namespace {

// Registers the component types the prefab tests use: the built-ins, plus the
// physics2d joint the reference-remap test needs.
struct RegisterTypes {
    RegisterTypes() {
        reflect::register_builtin_types();
        reflect::register_physics_types();
    }
};

[[maybe_unused]] RegisterTypes register_types_once;

// The first direct child whose name matches, or nullptr. The prefab tests use
// names to pick children out of an instance.
Node* find_child(Node* parent, std::string_view name) {
    for (const auto& child : parent->children()) {
        if (child->name() == name) {
            return child.get();
        }
    }
    return nullptr;
}

// A small prefab body: a root with a sprite, a nested child, and a joint that
// references two nodes inside the prefab (so instantiation has an intra-prefab
// reference to remap).
struct PrefabFixture {
    Scene scene;
    Node* root = nullptr;
    Node* body_a = nullptr;
    Node* body_b = nullptr;
    Node* joint_node = nullptr;

    PrefabFixture() {
        scene.set_name("Source");

        root = scene.create_node("Bullet");
        auto* sprite = root->add_component<Sprite2D>();
        sprite->size = Vec2{8.0f, 8.0f};
        sprite->layer = 5;
        root->transform().position = Vec3{10.0f, 20.0f, 0.0f};

        body_a = root->create_child<Node>("A");
        body_b = root->create_child<Node>("B");
        joint_node = root->create_child<Node>("Hinge");
        auto* joint = joint_node->add_component<physics2d::DistanceJoint2D>();
        joint->body_a = body_a->id();
        joint->body_b = body_b->id();
        joint->rest_length = 3.0f;
    }
};

} // namespace

TEST_CASE("a prefab captures a subtree and instantiates it into a scene") {
    PrefabFixture fixture;
    auto prefab = serialization::Prefab::capture("Bullet", *fixture.root);
    REQUIRE(prefab);
    CHECK(prefab.value().is_valid());
    CHECK(prefab.value().name() == "Bullet");
    CHECK(prefab.value().instance_count() == 0);

    Scene target;
    auto instance = prefab.value().instantiate(target);
    REQUIRE(instance);

    Node* root = instance.value();
    CHECK(root->name() == "Bullet");
    // The definition's subtree shape is reproduced: the sprite and the three
    // children come across.
    CHECK(root->get_component<Sprite2D>() != nullptr);
    CHECK(root->child_count() == 3);
    CHECK(prefab.value().instance_count() == 1);
}

TEST_CASE("two instances share the definition but carry independent transforms") {
    PrefabFixture fixture;
    auto prefab = serialization::Prefab::capture("Bullet", *fixture.root);
    REQUIRE(prefab);

    Scene target;
    auto first = prefab.value().instantiate(target);
    auto second = prefab.value().instantiate(target);
    REQUIRE(first);
    REQUIRE(second);
    CHECK(prefab.value().instance_count() == 2);

    Node* a = first.value();
    Node* b = second.value();
    // Two independent roots, distinct identities.
    CHECK(a != b);
    CHECK((a->id() != b->id()));

    // Move one instance: the other is untouched, and the definition is too.
    a->transform().position = Vec3{100.0f, 0.0f, 0.0f};
    CHECK(b->transform().position.x == doctest::Approx(10.0f));
    CHECK(fixture.root->transform().position.x == doctest::Approx(10.0f));
}

TEST_CASE("editing one instance leaves the other unchanged") {
    PrefabFixture fixture;
    auto prefab = serialization::Prefab::capture("Bullet", *fixture.root);
    REQUIRE(prefab);

    Scene target;
    auto first = prefab.value().instantiate(target);
    auto second = prefab.value().instantiate(target);
    REQUIRE(first);
    REQUIRE(second);

    auto* first_sprite = first.value()->get_component<Sprite2D>();
    auto* second_sprite = second.value()->get_component<Sprite2D>();
    REQUIRE(first_sprite != nullptr);
    REQUIRE((second_sprite != nullptr));
    REQUIRE(first_sprite != second_sprite);
    // Distinct component identities, so an edit that names one does not name the
    // other.
    CHECK((first_sprite->id() != second_sprite->id()));

    first_sprite->size = Vec2{99.0f, 99.0f};
    CHECK(second_sprite->size.x == doctest::Approx(8.0f));
    CHECK(fixture.root->get_component<Sprite2D>()->size.x == doctest::Approx(8.0f));
}

TEST_CASE("instantiation remaps intra-prefab references to the instance's own ids") {
    PrefabFixture fixture;
    auto prefab = serialization::Prefab::capture("Rig", *fixture.root);
    REQUIRE(prefab);

    Scene target;
    auto instance = prefab.value().instantiate(target);
    REQUIRE(instance);

    Node* root = instance.value();
    Node* joint_node = find_child(root, "Hinge");
    REQUIRE(joint_node != nullptr);
    const auto* joint = joint_node->get_component<physics2d::DistanceJoint2D>();
    REQUIRE(joint != nullptr);

    Node* a = find_child(root, "A");
    Node* b = find_child(root, "B");
    REQUIRE(a != nullptr);
    REQUIRE(b != nullptr);

    // The joint names the instance's own A and B, not the definition's.
    CHECK(joint->body_a == a->id());
    CHECK(joint->body_b == b->id());
    CHECK(joint->body_a != fixture.body_a->id());
    CHECK(joint->body_b != fixture.body_b->id());
}

TEST_CASE("a prefab survives a serialize/load round-trip") {
    PrefabFixture fixture;
    auto prefab = serialization::Prefab::capture("Rig", *fixture.root);
    REQUIRE(prefab);

    // A prefab is a document, so it can be stored and reloaded like one.
    serialization::Prefab reloaded =
        serialization::Prefab::from_document("Rig", std::string{prefab.value().document()});
    CHECK(reloaded.is_valid());

    Scene target;
    auto instance = reloaded.instantiate(target);
    REQUIRE(instance);
    CHECK(instance.value()->child_count() == 3);
    CHECK(reloaded.instance_count() == 1);
}

TEST_CASE("an empty prefab reports an error instead of instantiating nothing") {
    serialization::Prefab empty = serialization::Prefab::from_document("Empty", "");
    CHECK_FALSE(empty.is_valid());

    Scene target;
    auto instance = empty.instantiate(target);
    CHECK_FALSE(instance.ok());
    CHECK(target.root_count() == 0);
}
