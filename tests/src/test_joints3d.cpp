// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include <doctest/doctest.h>

#include "fe/core/Math.h"
#include "fe/physics3d/Collider3D.h"
#include "fe/physics3d/FixedJoint3D.h"
#include "fe/physics3d/Geometry3D.h"
#include "fe/physics3d/JointSolver3D.h"
#include "fe/physics3d/Physics3DSystem.h"
#include "fe/physics3d/Physics3DTypes.h"
#include "fe/physics3d/PrismaticJoint3D.h"
#include "fe/physics3d/RigidBody3D.h"
#include "fe/reflect/ComponentTypes.h"
#include "fe/reflect/Registry.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/serialization/SceneSerializer.h"
#include "fe/systems/FrameContext.h"

#include <cmath>

using namespace fe;

namespace {

physics3d::JointBody3D joint_body(const Vec3& position, f32 inv_mass, f32 inv_inertia = 0.0f) {
    physics3d::JointBody3D body;
    body.position = position;
    body.inv_mass = inv_mass;
    body.inv_inertia = inv_inertia;
    return body;
}

// The angle of the rotation that takes `from` to `to`, in radians: a single
// number to assert a relative orientation against.
f32 rotation_angle_between(const Quat& from, const Quat& to) {
    return physics3d::rotation_vector(from.conjugate() * to).length();
}

FrameContext step(f32 delta) {
    FrameContext context;
    context.delta_time = delta;
    context.elapsed_time = static_cast<f64>(delta);
    return context;
}

void run(physics3d::Physics3DSystem& system, Scene& scene, f32 delta, int steps) {
    for (int i = 0; i < steps; ++i) {
        system.update(step(delta), scene);
    }
}

// A dynamic body with an explicit inertia and no collider, so a joint is the
// only thing acting on it and its rotation is free to be driven.
Node* add_bare_body(Scene& scene, const Vec3& position, f32 mass, f32 inertia) {
    Node* node = scene.create_node("Body");
    auto* body = node->add_component<physics3d::RigidBody3D>();
    body->mass = mass;
    body->inertia = inertia;
    node->transform().position = position;
    return node;
}

} // namespace

// ---------------------------------------------------------------------------
// rotation_vector / apply_rotation_delta: the shared angular helpers.
// ---------------------------------------------------------------------------

TEST_CASE("rotation_vector of the identity is zero") {
    CHECK(physics3d::rotation_vector(Quat::identity()).length() == doctest::Approx(0.0f));
}

TEST_CASE("rotation_vector recovers the axis-angle it was built from") {
    const Vec3 axis = Vec3{0.0f, 1.0f, 0.0f};
    const f32 angle = 1.0f;
    const Quat q = Quat::from_axis_angle(axis, angle);
    const Vec3 rv = physics3d::rotation_vector(q);
    CHECK(rv.y == doctest::Approx(angle).epsilon(1e-4));
    CHECK(rv.x == doctest::Approx(0.0f).epsilon(1e-4));
    CHECK(rv.z == doctest::Approx(0.0f).epsilon(1e-4));
}

TEST_CASE("rotation_vector takes the shorter arc for a negated quaternion") {
    // q and -q are the same rotation; the rotation vector must be the same too.
    const Quat q = Quat::from_axis_angle(Vec3{1.0f, 0.0f, 0.0f}, 2.0f);
    const Quat negated = q * -1.0f;
    CHECK(physics3d::rotation_vector(negated).x == doctest::Approx(physics3d::rotation_vector(q).x).epsilon(1e-4));
}

TEST_CASE("apply_rotation_delta turns a body by the requested angle about the axis") {
    const Quat start = Quat::identity();
    const Vec3 delta = Vec3{0.0f, 0.0f, 0.5f}; // half a radian about +z
    const Quat turned = physics3d::apply_rotation_delta(start, delta);
    CHECK(rotation_angle_between(start, turned) == doctest::Approx(0.5f).epsilon(1e-4));
    // Turning about +z leaves the local x axis rotated towards +y.
    const Vec3 x_axis = physics3d::rotate_by_quat(Vec3{1.0f, 0.0f, 0.0f}, turned);
    CHECK(x_axis.y == doctest::Approx(std::sin(0.5f)).epsilon(1e-4));
}

TEST_CASE("apply_rotation_delta with a zero delta is a no-op") {
    const Quat start = Quat::from_axis_angle(Vec3{0.0f, 1.0f, 0.0f}, 0.3f);
    CHECK(rotation_angle_between(start, physics3d::apply_rotation_delta(start, Vec3{})) == doctest::Approx(0.0f));
}

// ---------------------------------------------------------------------------
// perpendicular_basis: the two directions a prismatic joint removes.
// ---------------------------------------------------------------------------

TEST_CASE("perpendicular_basis returns a right-handed orthonormal frame") {
    const Vec3 axis = Vec3{0.0f, 0.0f, 1.0f};
    Vec3 a;
    Vec3 b;
    physics3d::perpendicular_basis(axis, a, b);
    CHECK(a.dot(axis) == doctest::Approx(0.0f).epsilon(1e-4));
    CHECK(b.dot(axis) == doctest::Approx(0.0f).epsilon(1e-4));
    CHECK(a.dot(b) == doctest::Approx(0.0f).epsilon(1e-4));
    CHECK(a.length() == doctest::Approx(1.0f).epsilon(1e-4));
    CHECK(b.length() == doctest::Approx(1.0f).epsilon(1e-4));
    // The three form a right-handed set: a x b == axis.
    CHECK(a.cross(b).z == doctest::Approx(1.0f).epsilon(1e-4));
}

// ---------------------------------------------------------------------------
// Fixed joint rows: pin two anchors and lock the relative rotation.
// ---------------------------------------------------------------------------

TEST_CASE("fixed joint rows pin the anchors and lock the rotation") {
    // Two bodies with the same orientation and coincident anchors: the joint is
    // already satisfied, so every row reports zero error.
    physics3d::JointBody3D a = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    physics3d::JointBody3D b = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    physics3d::FixedJoint3DParams params;
    params.reference_relative = physics3d::fixed_reference_relative(a, b);

    physics3d::ConstraintRow3D rows[physics3d::kFixedJoint3DRows];
    auto built = physics3d::build_fixed_rows(a, b, params, 1.0f / 60.0f, rows);
    REQUIRE(built);
    for (usize i = 0; i < physics3d::kFixedJoint3DRows; ++i) {
        CHECK(rows[i].position_error == doctest::Approx(0.0f).epsilon(1e-4));
        CHECK(rows[i].effective_mass > 0.0f);
    }
}

TEST_CASE("fixed joint rows report a separation on the pinning rows") {
    physics3d::JointBody3D a = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    physics3d::JointBody3D b = joint_body(Vec3{0.5f, 0.0f, 0.0f}, 1.0f, 1.0f);
    physics3d::FixedJoint3DParams params;
    params.reference_relative = Quat::identity();

    physics3d::ConstraintRow3D rows[physics3d::kFixedJoint3DRows];
    auto built = physics3d::build_fixed_rows(a, b, params, 1.0f / 60.0f, rows);
    REQUIRE(built);
    // The x pinning row (row 0) carries the 0.5 separation; the others are clean.
    CHECK(std::abs(rows[0].position_error) == doctest::Approx(0.5f).epsilon(1e-4));
    CHECK(rows[1].position_error == doctest::Approx(0.0f).epsilon(1e-4));
    CHECK(rows[2].position_error == doctest::Approx(0.0f).epsilon(1e-4));
}

TEST_CASE("fixed joint rows report a relative rotation error") {
    physics3d::JointBody3D a = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    physics3d::JointBody3D b = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    b.orientation = Quat::from_axis_angle(Vec3{0.0f, 1.0f, 0.0f}, 0.4f);
    physics3d::FixedJoint3DParams params;
    params.reference_relative = Quat::identity(); // weld to a common orientation

    physics3d::ConstraintRow3D rows[physics3d::kFixedJoint3DRows];
    auto built = physics3d::build_fixed_rows(a, b, params, 1.0f / 60.0f, rows);
    REQUIRE(built);
    // The rotation rows (3-5) carry the 0.4 error; the pinning rows are clean.
    const f32 rotation_error =
        std::sqrt(rows[3].position_error * rows[3].position_error + rows[4].position_error * rows[4].position_error +
                  rows[5].position_error * rows[5].position_error);
    CHECK(rotation_error == doctest::Approx(0.4f).epsilon(1e-3));
    CHECK(rows[0].position_error == doctest::Approx(0.0f).epsilon(1e-4));
}

TEST_CASE("fixed joint rows between two immovable bodies are unsolvable") {
    physics3d::JointBody3D a = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    physics3d::JointBody3D b = joint_body(Vec3{1.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    physics3d::FixedJoint3DParams params;
    physics3d::ConstraintRow3D rows[physics3d::kFixedJoint3DRows];
    CHECK_FALSE(physics3d::build_fixed_rows(a, b, params, 1.0f / 60.0f, rows));
}

TEST_CASE("applying the fixed rows pulls a separated pair together and squares a twisted pair") {
    physics3d::JointBody3D a = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    physics3d::JointBody3D b = joint_body(Vec3{0.5f, 0.0f, 0.0f}, 1.0f, 1.0f);
    b.orientation = Quat::from_axis_angle(Vec3{0.0f, 1.0f, 0.0f}, 0.4f);
    physics3d::FixedJoint3DParams params;
    params.reference_relative = Quat::identity();

    physics3d::ConstraintRow3D rows[physics3d::kFixedJoint3DRows];
    REQUIRE(physics3d::build_fixed_rows(a, b, params, 1.0f / 60.0f, rows));
    // The system's convention: rows 0-2 are translation and are corrected by
    // apply_row_position (which also carries their lever-arm angular term); rows
    // 3-5 are pure rotation and are corrected by apply_row_orientation.
    for (usize i = 0; i < physics3d::kFixedJoint3DRows; ++i) {
        physics3d::apply_row(rows[i], a, b);
        if (i < physics3d::kRevoluteJoint3DRows) {
            physics3d::apply_row_position(rows[i], a, b);
        } else {
            physics3d::apply_row_orientation(rows[i], a, b);
        }
    }

    // The bodies moved together along x and the relative rotation shrank.
    CHECK(std::abs(b.position.x - a.position.x) < 0.5f);
    CHECK(rotation_angle_between(a.orientation, b.orientation) < 0.4f);
}

// ---------------------------------------------------------------------------
// Prismatic joint rows: two translation locks plus the rotation lock.
// ---------------------------------------------------------------------------

TEST_CASE("prismatic rows leave sliding along the axis free") {
    // Two bodies aligned on the z axis, one unit apart, both at identity
    // orientation. A prismatic joint along z must find no error, because the
    // separation is exactly the one free degree of freedom.
    physics3d::JointBody3D a = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    physics3d::JointBody3D b = joint_body(Vec3{0.0f, 0.0f, 1.0f}, 1.0f, 1.0f);
    physics3d::PrismaticJoint3DParams params;
    params.axis_a = Vec3{0.0f, 0.0f, 1.0f};
    params.reference_relative = Quat::identity();

    physics3d::ConstraintRow3D rows[physics3d::kPrismaticJoint3DRows];
    auto built = physics3d::build_prismatic_rows(a, b, params, 1.0f / 60.0f, rows);
    REQUIRE(built);
    // The two translation rows remove only motion perpendicular to z, so the
    // 1-unit z separation contributes no error.
    for (usize i = 0; i < physics3d::kPrismaticJoint3DRows; ++i) {
        CHECK(rows[i].position_error == doctest::Approx(0.0f).epsilon(1e-4));
    }
}

TEST_CASE("prismatic rows report separation perpendicular to the axis") {
    physics3d::JointBody3D a = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    physics3d::JointBody3D b = joint_body(Vec3{0.5f, 0.0f, 1.0f}, 1.0f, 1.0f);
    physics3d::PrismaticJoint3DParams params;
    params.axis_a = Vec3{0.0f, 0.0f, 1.0f};
    params.reference_relative = Quat::identity();

    physics3d::ConstraintRow3D rows[physics3d::kPrismaticJoint3DRows];
    REQUIRE(physics3d::build_prismatic_rows(a, b, params, 1.0f / 60.0f, rows));
    // The 0.5 offset on x is perpendicular to the z axis, so one of the two
    // translation rows carries it; the rotation rows are clean.
    const f32 translation_error =
        std::sqrt(rows[0].position_error * rows[0].position_error + rows[1].position_error * rows[1].position_error);
    CHECK(translation_error == doctest::Approx(0.5f).epsilon(1e-3));
    CHECK(rows[2].position_error == doctest::Approx(0.0f).epsilon(1e-4));
}

TEST_CASE("prismatic rows lock the relative rotation") {
    physics3d::JointBody3D a = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    physics3d::JointBody3D b = joint_body(Vec3{0.0f, 0.0f, 1.0f}, 1.0f, 1.0f);
    b.orientation = Quat::from_axis_angle(Vec3{1.0f, 0.0f, 0.0f}, 0.3f);
    physics3d::PrismaticJoint3DParams params;
    params.axis_a = Vec3{0.0f, 0.0f, 1.0f};
    params.reference_relative = Quat::identity();

    physics3d::ConstraintRow3D rows[physics3d::kPrismaticJoint3DRows];
    REQUIRE(physics3d::build_prismatic_rows(a, b, params, 1.0f / 60.0f, rows));
    const f32 rotation_error =
        std::sqrt(rows[2].position_error * rows[2].position_error + rows[3].position_error * rows[3].position_error +
                  rows[4].position_error * rows[4].position_error);
    CHECK(rotation_error == doctest::Approx(0.3f).epsilon(1e-3));
}

TEST_CASE("a prismatic joint with a degenerate axis is unsolvable") {
    physics3d::JointBody3D a = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    physics3d::JointBody3D b = joint_body(Vec3{0.0f, 0.0f, 1.0f}, 1.0f, 1.0f);
    physics3d::PrismaticJoint3DParams params;
    params.axis_a = Vec3{0.0f, 0.0f, 0.0f};
    physics3d::ConstraintRow3D rows[physics3d::kPrismaticJoint3DRows];
    CHECK_FALSE(physics3d::build_prismatic_rows(a, b, params, 1.0f / 60.0f, rows));
}

TEST_CASE("prismatic_world_axis rotates the local axis by the body orientation") {
    physics3d::JointBody3D a = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f);
    a.orientation = Quat::from_axis_angle(Vec3{1.0f, 0.0f, 0.0f}, 1.5707963f); // 90 degrees about x
    physics3d::PrismaticJoint3DParams params;
    params.axis_a = Vec3{0.0f, 0.0f, 1.0f};
    const Vec3 axis = physics3d::prismatic_world_axis(a, params);
    // A quarter turn about +x takes +z to -y.
    CHECK(axis.y == doctest::Approx(-1.0f).epsilon(1e-3));
    CHECK(std::abs(axis.z) < 1e-3f);
}

// ---------------------------------------------------------------------------
// System wiring: a fixed joint welds a body to a pivot, a prismatic joint lets
// a body slide on one axis, and both round-trip through the serializer.
// ---------------------------------------------------------------------------

TEST_CASE("a fixed joint welds a dynamic body to a static pivot") {
    Scene scene;
    Node* pivot = scene.create_node("Pivot");
    pivot->transform().position = Vec3{0.0f, 0.0f, 0.0f};

    // A body placed off the pivot with gravity on: the weld must hold it in
    // place instead of letting it fall.
    Node* body_node = add_bare_body(scene, Vec3{1.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = body_node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = true;

    Node* host = scene.create_node("Weld");
    auto* joint = host->add_component<physics3d::FixedJoint3D>();
    joint->body_a = pivot->id();
    joint->body_b = body_node->id();
    // Weld the body's left end (1 unit from its centre) to the pivot, so the
    // weld holds the body out at x = 1 rather than pulling its centre to the
    // pivot.
    joint->anchor_b = Vec3{-1.0f, 0.0f, 0.0f};
    joint->reference_relative = Quat::identity();

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 240);

    CHECK(system.joint_count() == 1);
    // The weld held the body at its authored offset: gravity could not pull it
    // down, because the anchor is pinned to the static pivot.
    CHECK(body_node->transform().position.x == doctest::Approx(1.0f).epsilon(0.1));
    CHECK(std::abs(body_node->transform().position.y) < 0.1f);
}

TEST_CASE("a fixed joint squares a body that starts rotated against the reference") {
    Scene scene;
    Node* pivot = scene.create_node("Pivot");

    Node* body_node = add_bare_body(scene, Vec3{1.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = body_node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = false;
    // Starts twisted about y; the joint's reference is the identity, so it must
    // turn the body back to a common orientation.
    body_node->transform().rotation = Vec3{0.0f, 0.5f, 0.0f};

    Node* host = scene.create_node("Weld");
    auto* joint = host->add_component<physics3d::FixedJoint3D>();
    joint->body_a = pivot->id();
    joint->body_b = body_node->id();
    joint->reference_relative = Quat::identity();

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 180);

    // The relative rotation collapsed to (nearly) the identity.
    CHECK(std::abs(body_node->transform().rotation.y) < 0.05f);
}

TEST_CASE("a prismatic joint lets a body slide along its axis but not off it") {
    Scene scene;
    Node* rail = scene.create_node("Rail");
    rail->transform().position = Vec3{0.0f, 0.0f, 0.0f};

    // A body pushed along +x, off the slide axis (+z): the joint must hold the x
    // separation at zero while leaving z free.
    Node* slider_node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = slider_node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = false;
    body->velocity = Vec3{2.0f, 0.0f, 2.0f}; // half along the axis, half across it

    Node* host = scene.create_node("Slider");
    auto* joint = host->add_component<physics3d::PrismaticJoint3D>();
    joint->body_a = rail->id();
    joint->body_b = slider_node->id();
    joint->axis = Vec3{0.0f, 0.0f, 1.0f};
    joint->reference_relative = Quat::identity();

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 120);

    CHECK(system.joint_count() == 1);
    // The across-axis (x) motion was removed; the along-axis (z) motion was not.
    CHECK(std::abs(slider_node->transform().position.x) < 0.1f);
    CHECK(slider_node->transform().position.z > 1.0f);
}

TEST_CASE("a fixed joint round-trips through the serializer") {
    reflect::register_builtin_types();
    reflect::register_physics3d_types();

    Scene scene;
    Node* a = scene.create_node("A");
    Node* b = scene.create_node("B");
    Node* joint_node = scene.create_node("Weld");
    auto* joint = joint_node->add_component<physics3d::FixedJoint3D>();
    joint->body_a = a->id();
    joint->body_b = b->id();
    joint->anchor_a = Vec3{0.1f, 0.2f, 0.3f};
    joint->anchor_b = Vec3{-0.4f, 0.5f, -0.6f};
    joint->reference_relative = Quat::from_axis_angle(Vec3{0.0f, 1.0f, 0.0f}, 0.5f);

    auto json = serialization::scene::to_json(scene);
    REQUIRE(json);

    Scene loaded;
    REQUIRE_FALSE(serialization::scene::from_json(json.value(), loaded));

    Node* loaded_joint = loaded.find_node_by_name("Weld");
    REQUIRE(loaded_joint != nullptr);
    const auto* loaded_component = loaded_joint->get_component<physics3d::FixedJoint3D>();
    REQUIRE(loaded_component != nullptr);
    CHECK(loaded_component->body_a == a->id());
    CHECK(loaded_component->body_b == b->id());
    CHECK(loaded_component->anchor_a.y == doctest::Approx(0.2f));
    CHECK(loaded_component->reference_relative.y == doctest::Approx(joint->reference_relative.y).epsilon(1e-4));
}

TEST_CASE("a prismatic joint round-trips through the serializer") {
    reflect::register_builtin_types();
    reflect::register_physics3d_types();

    Scene scene;
    Node* a = scene.create_node("A");
    Node* b = scene.create_node("B");
    Node* joint_node = scene.create_node("Slider");
    auto* joint = joint_node->add_component<physics3d::PrismaticJoint3D>();
    joint->body_a = a->id();
    joint->body_b = b->id();
    joint->anchor_a = Vec3{1.0f, 0.0f, 0.0f};
    joint->axis = Vec3{0.0f, 1.0f, 0.0f};

    auto json = serialization::scene::to_json(scene);
    REQUIRE(json);

    Scene loaded;
    REQUIRE_FALSE(serialization::scene::from_json(json.value(), loaded));

    Node* loaded_joint = loaded.find_node_by_name("Slider");
    REQUIRE(loaded_joint != nullptr);
    const auto* loaded_component = loaded_joint->get_component<physics3d::PrismaticJoint3D>();
    REQUIRE(loaded_component != nullptr);
    CHECK(loaded_component->body_a == a->id());
    CHECK(loaded_component->body_b == b->id());
    CHECK(loaded_component->axis.y == doctest::Approx(1.0f));
}

TEST_CASE("the fixed and prismatic 3D joints are registered with reflection") {
    reflect::register_physics3d_types();
    CHECK(reflect::Registry::instance().contains("FixedJoint3D"));
    CHECK(reflect::Registry::instance().contains("PrismaticJoint3D"));
}
