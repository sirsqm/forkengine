// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include <doctest/doctest.h>

#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"

using namespace fe;

TEST_CASE("a new scene is empty and can create root nodes") {
    Scene scene;
    CHECK(scene.root_count() == 0);
    CHECK(scene.node_count() == 0);

    Node* node = scene.create_node("Root");
    REQUIRE(node != nullptr);
    CHECK(node->name() == "Root");
    CHECK((node->scene() == &scene));
    CHECK(scene.root_count() == 1);
    CHECK(scene.node_count() == 1);
}

TEST_CASE("nodes are looked up by id and by name") {
    Scene scene;
    Node* parent = scene.create_node("Parent");
    Node* child = parent->create_child<Node>("Child");

    CHECK((scene.find_node(parent->id()) == parent));
    CHECK((scene.find_node(child->id()) == child));
    CHECK(scene.find_node_by_name("Child") == child);
    CHECK(scene.find_node_by_name("Missing") == nullptr);

    // The child is in the scene even though it is not a root.
    CHECK(scene.root_count() == 1);
    CHECK(scene.node_count() == 2);
}

TEST_CASE("world transform composes down the hierarchy") {
    Scene scene;
    Node* parent = scene.create_node("Parent");
    parent->transform().position = Vec3{10.0f, 0.0f, 0.0f};

    Node* child = parent->create_child<Node>("Child");
    child->transform().position = Vec3{5.0f, 2.0f, 0.0f};

    const Vec3 world = transform_point(child->world_matrix(), Vec3{});
    CHECK(world.x == doctest::Approx(15.0f));
    CHECK(world.y == doctest::Approx(2.0f));
}

TEST_CASE("a child inherits its parent's scale") {
    Scene scene;
    Node* parent = scene.create_node("Parent");
    parent->transform().scale = Vec3{2.0f, 2.0f, 1.0f};

    Node* child = parent->create_child<Node>("Child");
    child->transform().position = Vec3{10.0f, 0.0f, 0.0f};

    const Vec3 world = transform_point(child->world_matrix(), Vec3{});
    CHECK(world.x == doctest::Approx(20.0f));
}

TEST_CASE("detaching a subtree removes every descendant from the scene") {
    Scene scene;
    Node* root = scene.create_node("Root");
    Node* child = root->create_child<Node>("Child");
    child->create_child<Node>("Grandchild");
    CHECK(scene.node_count() == 3);

    auto detached = scene.remove_root(root);
    REQUIRE(detached != nullptr);
    CHECK(scene.node_count() == 0);
    CHECK(scene.root_count() == 0);
    // The detached node keeps its own subtree.
    CHECK(detached->child_count() == 1);
}

TEST_CASE("reparenting a node moves it under a new parent") {
    Scene scene;
    Node* a = scene.create_node("A");
    Node* b = scene.create_node("B");
    Node* child = a->create_child<Node>("Child");

    b->add_child(a->remove_child(child));

    CHECK((child->parent() == b));
    CHECK(a->child_count() == 0);
    CHECK(b->child_count() == 1);
    CHECK(scene.node_count() == 3);
}

namespace {

// Element-wise matrix comparison, so a failure names the element that moved.
void check_matrix_near(const Mat4& actual, const Mat4& expected, f32 epsilon = 1e-4f) {
    for (int i = 0; i < 16; ++i) {
        CHECK(actual.m[i] == doctest::Approx(expected.m[i]).epsilon(epsilon));
    }
}

} // namespace

TEST_CASE("reparenting under a new parent preserves the world transform") {
    Scene scene;
    Node* from = scene.create_node("From");
    from->transform().position = Vec3{10.0f, 0.0f, 0.0f};
    from->transform().rotation = Vec3{0.0f, 0.0f, 0.5f};
    from->transform().scale = Vec3{2.0f, 2.0f, 1.0f};

    Node* to = scene.create_node("To");
    to->transform().position = Vec3{-4.0f, 3.0f, 0.0f};
    to->transform().rotation = Vec3{0.0f, 0.0f, -0.75f};

    Node* child = from->create_child<Node>("Child");
    child->transform().position = Vec3{1.0f, 2.0f, 0.0f};

    const Mat4 before = child->world_matrix();
    CHECK((child->reparent(to) == child));

    CHECK((child->parent() == to));
    CHECK(from->child_count() == 0);
    CHECK(to->child_count() == 1);
    CHECK(scene.node_count() == 3);
    check_matrix_near(child->world_matrix(), before);

    // The local transform must have moved: the new parent sits somewhere else
    // with a different rotation and scale.
    CHECK_FALSE(child->transform().position.x == doctest::Approx(1.0f));
}

TEST_CASE("reparenting to the scene root preserves the world transform") {
    Scene scene;
    Node* parent = scene.create_node("Parent");
    parent->transform().position = Vec3{5.0f, 5.0f, 0.0f};
    Node* child = parent->create_child<Node>("Child");
    child->transform().position = Vec3{1.0f, 0.0f, 0.0f};

    const Mat4 before = child->world_matrix();
    child->reparent(nullptr);

    CHECK(child->parent() == nullptr);
    CHECK((child->scene() == &scene));
    CHECK(scene.root_count() == 2);
    CHECK(scene.node_count() == 2);
    // A root's local transform is its world transform.
    CHECK(child->transform().position.x == doctest::Approx(6.0f));
    check_matrix_near(child->world_matrix(), before);
}

TEST_CASE("reparenting to the same parent changes nothing") {
    Scene scene;
    Node* parent = scene.create_node("Parent");
    Node* child = parent->create_child<Node>("Child");
    child->transform().position = Vec3{1.0f, 2.0f, 3.0f};

    child->reparent(parent);

    CHECK((child->parent() == parent));
    CHECK(parent->child_count() == 1);
    CHECK(child->transform().position.x == doctest::Approx(1.0f));
    CHECK(child->transform().position.y == doctest::Approx(2.0f));
}

TEST_CASE("a reparent without preserve_world keeps the local transform") {
    Scene scene;
    Node* from = scene.create_node("From");
    Node* to = scene.create_node("To");
    to->transform().position = Vec3{100.0f, 0.0f, 0.0f};
    Node* child = from->create_child<Node>("Child");
    child->transform().position = Vec3{1.0f, 2.0f, 0.0f};

    child->reparent(to, false);

    CHECK((child->parent() == to));
    CHECK(child->transform().position.x == doctest::Approx(1.0f));
    CHECK(child->world_matrix().m[12] == doctest::Approx(101.0f));
}

TEST_CASE("clearing a scene detaches everything") {
    Scene scene;
    Node* root = scene.create_node("Root");
    root->create_child<Node>("Child");

    scene.clear();
    CHECK(scene.root_count() == 0);
    CHECK(scene.node_count() == 0);
}

TEST_CASE("for_each_node visits every node depth-first") {
    Scene scene;
    Node* root = scene.create_node("Root");
    root->create_child<Node>("A");
    root->create_child<Node>("B");

    std::vector<std::string> names;
    scene.for_each_node([&](Node& node) { names.push_back(node.name()); });

    REQUIRE(names.size() == 3);
    CHECK(names[0] == "Root");
    CHECK(names[1] == "A");
    CHECK(names[2] == "B");
}
