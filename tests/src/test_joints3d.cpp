// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include <doctest/doctest.h>

#include "fe/core/Math.h"
#include "fe/physics3d/Collider3D.h"
#include "fe/physics3d/DistanceJoint3D.h"
#include "fe/physics3d/FixedJoint3D.h"
#include "fe/physics3d/Geometry3D.h"
#include "fe/physics3d/JointSolver3D.h"
#include "fe/physics3d/Physics3DSystem.h"
#include "fe/physics3d/Physics3DTypes.h"
#include "fe/physics3d/PrismaticJoint3D.h"
#include "fe/physics3d/RevoluteJoint3D.h"
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

    // Starts 1 unit off the slide axis (+z) and is pushed straight along the
    // axis, so only the perpendicular position rows can pull it back onto the
    // rail: a velocity-only constraint would leave it 1 unit out forever.
    Node* slider_node = add_bare_body(scene, Vec3{1.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = slider_node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = false;
    body->velocity = Vec3{0.0f, 0.0f, 2.0f}; // straight along the +z slide axis

    Node* host = scene.create_node("Slider");
    auto* joint = host->add_component<physics3d::PrismaticJoint3D>();
    joint->body_a = rail->id();
    joint->body_b = slider_node->id();
    joint->axis = Vec3{0.0f, 0.0f, 1.0f};
    joint->reference_relative = Quat::identity();

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 120);

    CHECK(system.joint_count() == 1);
    // The off-axis start was pulled back to the rail; the along-axis motion was
    // left free to carry the body down the axis.
    CHECK(slider_node->transform().position.x < 0.1f);
    CHECK(slider_node->transform().position.z > 1.0f);
}

TEST_CASE("a 3D prismatic joint's translation limits stop the slide at its bound") {
    Scene scene;
    Node* rail = scene.create_node("Rail");
    rail->transform().position = Vec3{0.0f, 0.0f, 0.0f};
    Node* slider_node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = slider_node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = false;
    body->velocity = Vec3{0.0f, 0.0f, 5.0f}; // drive into the upper stop

    Node* host = scene.create_node("Slider");
    auto* joint = host->add_component<physics3d::PrismaticJoint3D>();
    joint->body_a = rail->id();
    joint->body_b = slider_node->id();
    joint->axis = Vec3{0.0f, 0.0f, 1.0f};
    joint->enable_limits = true;
    joint->lower_translation = -1.0f;
    joint->upper_translation = 2.0f;

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 240);

    CHECK(system.joint_count() == 1);
    CHECK(slider_node->transform().position.z == doctest::Approx(2.0f).epsilon(0.05));
}

TEST_CASE("a 3D prismatic joint's linear motor drives the slide at its target speed") {
    Scene scene;
    Node* rail = scene.create_node("Rail");
    Node* slider_node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = slider_node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = false;

    Node* host = scene.create_node("Piston");
    auto* joint = host->add_component<physics3d::PrismaticJoint3D>();
    joint->body_a = rail->id();
    joint->body_b = slider_node->id();
    joint->axis = Vec3{0.0f, 0.0f, 1.0f};
    joint->motor_speed = 2.0f;
    joint->motor_max_force = 1000.0f;

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 120);

    CHECK(system.joint_count() == 1);
    CHECK(body->velocity.z == doctest::Approx(2.0f).epsilon(0.05));
}

TEST_CASE("a 3D prismatic joint's position motor row drives the slide towards its target") {
    physics3d::JointBody3D a;
    a.inv_mass = 1.0f;
    physics3d::JointBody3D b;
    b.inv_mass = 1.0f;

    physics3d::PrismaticJoint3DParams params;
    params.axis_a = Vec3{0.0f, 0.0f, 1.0f};

    physics3d::JointMotor3D motor;
    motor.max_impulse = 1000.0f;
    motor.use_target = true;
    motor.target_position = 1.0f;
    motor.stiffness = 2.0f;

    // The slide is at 2.0 and the target 1.0, so the error (current minus target)
    // is +1 and the bias +2. The row's impulse is negative, so B is driven back
    // down the axis: the relative velocity becomes -2.
    auto built = physics3d::build_prismatic_position_motor_row(a, b, params, motor, 2.0f);
    REQUIRE(built);
    physics3d::ConstraintRow3D row = built.value();
    physics3d::apply_row(row, a, b);

    CHECK(b.velocity.z - a.velocity.z == doctest::Approx(-2.0f).epsilon(1e-3));
}

TEST_CASE("a 3D prismatic joint's position motor gives way under a load it cannot hold") {
    physics3d::JointBody3D a;
    a.inv_mass = 1.0f;
    physics3d::JointBody3D b;
    b.inv_mass = 1.0f;

    physics3d::PrismaticJoint3DParams params;
    params.axis_a = Vec3{0.0f, 0.0f, 1.0f};

    physics3d::JointMotor3D motor;
    motor.max_impulse = 0.25f; // a weak servo
    motor.use_target = true;
    motor.target_position = 1.0f;
    motor.stiffness = 2.0f;

    // The unclamped impulse would drive +2; the clamp holds it to 0.25 on each
    // unit-mass body, so the relative velocity changes by only 0.5.
    auto built = physics3d::build_prismatic_position_motor_row(a, b, params, motor, 0.0f);
    REQUIRE(built);
    physics3d::ConstraintRow3D row = built.value();
    physics3d::apply_row(row, a, b);

    CHECK(b.velocity.z - a.velocity.z == doctest::Approx(0.5f).epsilon(1e-3));
}

TEST_CASE("a 3D prismatic joint's position motor holds the slide at its target") {
    Scene scene;
    Node* rail = scene.create_node("Rail");
    Node* slider_node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = slider_node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = true;

    Node* host = scene.create_node("Lift");
    auto* joint = host->add_component<physics3d::PrismaticJoint3D>();
    joint->body_a = rail->id();
    joint->body_b = slider_node->id();
    joint->axis = Vec3{0.0f, 0.0f, 1.0f};
    joint->motor_use_target = true;
    joint->motor_target_translation = 1.0f; // hold the slider one unit up
    joint->motor_stiffness = 50.0f;
    joint->motor_damping = 0.5f;
    joint->motor_max_force = 1000.0f;

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 240);

    CHECK(system.joint_count() == 1);
    // The servo holds the slider against gravity at its target translation. The
    // slide axis is +z and gravity pulls -y, so the perpendicular rows hold y.
    CHECK(slider_node->transform().position.z == doctest::Approx(1.0f).epsilon(0.02));
}

TEST_CASE("a 3D prismatic joint's position motor round-trips through the serializer") {
    reflect::register_builtin_types();
    reflect::register_physics3d_types();

    Scene scene;
    Node* a = scene.create_node("A");
    Node* b = scene.create_node("B");

    Node* host = scene.create_node("Lift");
    auto* joint = host->add_component<physics3d::PrismaticJoint3D>();
    joint->body_a = a->id();
    joint->body_b = b->id();
    joint->motor_use_target = true;
    joint->motor_target_translation = 2.5f;
    joint->motor_stiffness = 40.0f;
    joint->motor_damping = 0.8f;
    joint->motor_max_force = 25.0f;

    auto json = serialization::scene::to_json(scene);
    REQUIRE(json);

    Scene loaded;
    REQUIRE_FALSE(serialization::scene::from_json(json.value(), loaded));

    Node* loaded_host = loaded.find_node_by_name("Lift");
    REQUIRE(loaded_host != nullptr);
    const auto* loaded_joint = loaded_host->get_component<physics3d::PrismaticJoint3D>();
    REQUIRE(loaded_joint != nullptr);
    CHECK(loaded_joint->motor_use_target);
    CHECK(loaded_joint->motor_target_translation == doctest::Approx(2.5f));
    CHECK(loaded_joint->motor_stiffness == doctest::Approx(40.0f));
    CHECK(loaded_joint->motor_damping == doctest::Approx(0.8f));
}

TEST_CASE("a 3D prismatic joint's limits and motor round-trip through the serializer") {
    reflect::register_builtin_types();
    reflect::register_physics3d_types();

    Scene scene;
    Node* a = scene.create_node("A");
    Node* b = scene.create_node("B");

    Node* host = scene.create_node("Piston");
    auto* joint = host->add_component<physics3d::PrismaticJoint3D>();
    joint->body_a = a->id();
    joint->body_b = b->id();
    joint->enable_limits = true;
    joint->lower_translation = -0.5f;
    joint->upper_translation = 1.5f;
    joint->motor_speed = 3.0f;
    joint->motor_max_force = 7.0f;

    auto json = serialization::scene::to_json(scene);
    REQUIRE(json);

    Scene loaded;
    REQUIRE_FALSE(serialization::scene::from_json(json.value(), loaded));

    Node* loaded_host = loaded.find_node_by_name("Piston");
    REQUIRE(loaded_host != nullptr);
    const auto* loaded_joint = loaded_host->get_component<physics3d::PrismaticJoint3D>();
    REQUIRE(loaded_joint != nullptr);
    CHECK(loaded_joint->enable_limits);
    CHECK(loaded_joint->lower_translation == doctest::Approx(-0.5f));
    CHECK(loaded_joint->upper_translation == doctest::Approx(1.5f));
    CHECK(loaded_joint->motor_speed == doctest::Approx(3.0f));
    CHECK(loaded_joint->motor_max_force == doctest::Approx(7.0f));
}

// ---------------------------------------------------------------------------
// Joint motors (ADR-0134): a joint drives its free degree of freedom towards a
// target speed, up to a maximum force.
// ---------------------------------------------------------------------------

TEST_CASE("a 3D revolute motor row drives the relative spin about its axis") {
    physics3d::JointBody3D a;
    a.inv_inertia = 1.0f;
    physics3d::JointBody3D b;
    b.inv_inertia = 1.0f;

    physics3d::JointMotor3D motor;
    motor.axis = Vec3{0.0f, 1.0f, 0.0f};
    motor.target_speed = 2.0f;
    motor.max_impulse = 100.0f;

    auto built = physics3d::build_revolute_motor_row(a, b, motor);
    REQUIRE(built);
    physics3d::ConstraintRow3D row = built.value();
    physics3d::apply_row(row, a, b);

    // The spin is about the motor axis; the other two components stay zero.
    CHECK(b.angular_velocity.y - a.angular_velocity.y == doctest::Approx(2.0f).epsilon(1e-3));
    CHECK(b.angular_velocity.x - a.angular_velocity.x == doctest::Approx(0.0f));
    CHECK(b.angular_velocity.z - a.angular_velocity.z == doctest::Approx(0.0f));
}

TEST_CASE("a 3D revolute motor's maximum torque caps the impulse it applies") {
    physics3d::JointBody3D a;
    a.inv_inertia = 1.0f;
    physics3d::JointBody3D b;
    b.inv_inertia = 1.0f;

    physics3d::JointMotor3D motor;
    motor.axis = Vec3{0.0f, 1.0f, 0.0f};
    motor.target_speed = 2.0f;
    motor.max_impulse = 0.25f;

    auto built = physics3d::build_revolute_motor_row(a, b, motor);
    REQUIRE(built);
    physics3d::ConstraintRow3D row = built.value();
    physics3d::apply_row(row, a, b);

    CHECK(b.angular_velocity.y - a.angular_velocity.y == doctest::Approx(0.5f).epsilon(1e-3));
}

TEST_CASE("a 3D revolute motor with a degenerate axis is unsolvable") {
    physics3d::JointBody3D a;
    a.inv_inertia = 1.0f;
    physics3d::JointBody3D b;
    b.inv_inertia = 1.0f;
    physics3d::JointMotor3D motor;
    motor.axis = Vec3{0.0f, 0.0f, 0.0f};
    motor.max_impulse = 10.0f;
    CHECK_FALSE(physics3d::build_revolute_motor_row(a, b, motor));
}

TEST_CASE("a motor on a 3D revolute joint spins a body about its pivot") {
    Scene scene;
    Node* pivot = scene.create_node("Pivot"); // a bare, immovable anchor
    Node* arm = add_bare_body(scene, Vec3{1.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = arm->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = false;

    Node* host = scene.create_node("Axle");
    auto* joint = host->add_component<physics3d::RevoluteJoint3D>();
    joint->body_a = pivot->id();
    joint->body_b = arm->id();
    joint->anchor_b = Vec3{-1.0f, 0.0f, 0.0f};  // the pivot end of the arm
    joint->motor_axis = Vec3{0.0f, 0.0f, 1.0f}; // spin about +z
    joint->motor_speed = 3.0f;
    joint->motor_max_torque = 100.0f;

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 120);

    CHECK(system.joint_count() == 1);
    CHECK(body->angular_velocity.z == doctest::Approx(3.0f).epsilon(0.1));
    // The pin still holds the axle: the arm's centre stays one unit from the
    // pivot, wherever the spin has carried it around the circle.
    CHECK(arm->transform().position.length() == doctest::Approx(1.0f).epsilon(0.05));
}

TEST_CASE("a 3D revolute position motor row drives the swing towards its target") {
    physics3d::JointBody3D a;
    a.inv_inertia = 1.0f;
    physics3d::JointBody3D b;
    b.inv_inertia = 1.0f;

    physics3d::JointMotor3D motor;
    motor.axis = Vec3{0.0f, 1.0f, 0.0f};
    motor.max_impulse = 1000.0f;
    motor.use_target = true;
    motor.target_position = 1.0f;
    motor.stiffness = 2.0f;

    // The swing is 2.0 and the target 1.0, so the error (current minus target) is
    // +1 and the bias is +2. The row's impulse is negative about the axis, so the
    // relative spin about y is driven to -2 -- the swing falls back.
    auto built = physics3d::build_revolute_position_motor_row(a, b, motor, 2.0f);
    REQUIRE(built);
    physics3d::ConstraintRow3D row = built.value();
    physics3d::apply_row(row, a, b);

    CHECK(b.angular_velocity.y - a.angular_velocity.y == doctest::Approx(-2.0f).epsilon(1e-3));
    CHECK(b.angular_velocity.x - a.angular_velocity.x == doctest::Approx(0.0f));
}

TEST_CASE("a 3D revolute position motor's maximum torque caps the impulse it applies") {
    physics3d::JointBody3D a;
    a.inv_inertia = 1.0f;
    physics3d::JointBody3D b;
    b.inv_inertia = 1.0f;

    physics3d::JointMotor3D motor;
    motor.axis = Vec3{0.0f, 1.0f, 0.0f};
    motor.max_impulse = 0.25f;
    motor.use_target = true;
    motor.target_position = 1.0f;
    motor.stiffness = 2.0f;

    // The unclamped impulse would drive +2; the clamp holds it to 0.25 on each
    // unit-inertia body, so the relative spin changes by only 0.5.
    auto built = physics3d::build_revolute_position_motor_row(a, b, motor, 0.0f);
    REQUIRE(built);
    physics3d::ConstraintRow3D row = built.value();
    physics3d::apply_row(row, a, b);

    CHECK(b.angular_velocity.y - a.angular_velocity.y == doctest::Approx(0.5f).epsilon(1e-3));
}

TEST_CASE("a 3D revolute position motor with a degenerate axis is unsolvable") {
    physics3d::JointBody3D a;
    a.inv_inertia = 1.0f;
    physics3d::JointBody3D b;
    b.inv_inertia = 1.0f;
    physics3d::JointMotor3D motor;
    motor.axis = Vec3{0.0f, 0.0f, 0.0f};
    motor.max_impulse = 10.0f;
    motor.use_target = true;
    motor.stiffness = 2.0f;
    CHECK_FALSE(physics3d::build_revolute_position_motor_row(a, b, motor, 0.0f));
}

TEST_CASE("a 3D revolute position motor holds an arm at its target angle") {
    Scene scene;
    Node* pivot = scene.create_node("Pivot"); // a bare, immovable anchor
    Node* arm = add_bare_body(scene, Vec3{1.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = arm->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = true;

    Node* host = scene.create_node("Servo");
    auto* joint = host->add_component<physics3d::RevoluteJoint3D>();
    joint->body_a = pivot->id();
    joint->body_b = arm->id();
    joint->anchor_b = Vec3{-1.0f, 0.0f, 0.0f}; // the pivot end of the arm
    joint->motor_axis = Vec3{0.0f, 0.0f, 1.0f};
    joint->motor_use_target = true;
    joint->motor_target_angle = 0.5f; // hold the arm 0.5 rad above horizontal
    joint->motor_stiffness = 30.0f;
    joint->motor_damping = 0.9f;
    joint->motor_max_torque = 1000.0f;

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 240);

    CHECK(system.joint_count() == 1);
    // The servo holds the arm against gravity at the target swing about +z: the
    // arm's own orientation is the relative rotation, since the pivot is fixed.
    const f32 swing = physics3d::rotation_vector(Quat::from_euler(arm->transform().rotation)).z;
    CHECK(swing == doctest::Approx(0.5f).epsilon(0.05));
}

TEST_CASE("a 3D revolute position motor's target round-trips through the serializer") {
    reflect::register_builtin_types();
    reflect::register_physics3d_types();

    Scene scene;
    Node* a = scene.create_node("A");
    Node* b = scene.create_node("B");

    Node* host = scene.create_node("Servo");
    auto* joint = host->add_component<physics3d::RevoluteJoint3D>();
    joint->body_a = a->id();
    joint->body_b = b->id();
    joint->motor_use_target = true;
    joint->motor_target_angle = 0.75f;
    joint->motor_stiffness = 40.0f;
    joint->motor_damping = 0.8f;
    joint->motor_max_torque = 25.0f;

    auto json = serialization::scene::to_json(scene);
    REQUIRE(json);

    Scene loaded;
    REQUIRE_FALSE(serialization::scene::from_json(json.value(), loaded));

    Node* loaded_host = loaded.find_node_by_name("Servo");
    REQUIRE(loaded_host != nullptr);
    const auto* loaded_joint = loaded_host->get_component<physics3d::RevoluteJoint3D>();
    REQUIRE(loaded_joint != nullptr);
    CHECK(loaded_joint->motor_use_target);
    CHECK(loaded_joint->motor_target_angle == doctest::Approx(0.75f));
    CHECK(loaded_joint->motor_stiffness == doctest::Approx(40.0f));
    CHECK(loaded_joint->motor_damping == doctest::Approx(0.8f));
}

TEST_CASE("a 3D revolute joint's angular limits build a one-sided row at the bound it is past") {
    physics3d::JointBody3D a = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    physics3d::JointBody3D b = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    b.orientation = Quat::from_axis_angle(Vec3{0.0f, 0.0f, 1.0f}, 1.2f); // past the upper bound

    physics3d::RevoluteJoint3DParams params;
    params.enable_limits = true;
    params.limit_axis = Vec3{0.0f, 0.0f, 1.0f};
    params.lower_angle = -0.5f;
    params.upper_angle = 1.0f;

    physics3d::ConstraintRow3D lower;
    physics3d::ConstraintRow3D upper;
    physics3d::build_revolute_limit_rows(a, b, params, lower, upper);

    // Past the upper bound the upper row is active, purely angular about the
    // hinge axis, and clamped to only a negative impulse so it can shrink the
    // swing but never grow it. The lower row is slack.
    CHECK(upper.effective_mass > 0.0f);
    CHECK(upper.angular_a.z == doctest::Approx(-1.0f));
    CHECK(upper.angular_b.z == doctest::Approx(1.0f));
    CHECK(upper.lower_impulse < 0.0f);
    CHECK(upper.upper_impulse == doctest::Approx(0.0f));
    CHECK(lower.effective_mass == doctest::Approx(0.0f));
}

TEST_CASE("a 3D revolute limit row cancels the spin into the bound") {
    physics3d::JointBody3D a = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    physics3d::JointBody3D b = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    b.orientation = Quat::from_axis_angle(Vec3{0.0f, 0.0f, 1.0f}, 1.2f);
    b.angular_velocity = Vec3{0.0f, 0.0f, 5.0f}; // spinning further past the upper bound

    physics3d::RevoluteJoint3DParams params;
    params.enable_limits = true;
    params.limit_axis = Vec3{0.0f, 0.0f, 1.0f};
    params.lower_angle = -0.5f;
    params.upper_angle = 1.0f;

    physics3d::ConstraintRow3D lower;
    physics3d::ConstraintRow3D upper;
    physics3d::build_revolute_limit_rows(a, b, params, lower, upper);
    physics3d::apply_row(upper, a, b);

    // The row removed the relative spin about the hinge axis into the stop.
    const Vec3 relative_spin = b.angular_velocity - a.angular_velocity;
    CHECK(relative_spin.z == doctest::Approx(0.0f).epsilon(0.01));
}

TEST_CASE("a limited 3D revolute joint stops a swinging arm at its bound") {
    Scene scene;
    Node* pivot = scene.create_node("Pivot"); // a bare, immovable anchor
    Node* arm = add_bare_body(scene, Vec3{1.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = arm->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = true;

    Node* host = scene.create_node("Hinge");
    auto* joint = host->add_component<physics3d::RevoluteJoint3D>();
    joint->body_a = pivot->id();
    joint->body_b = arm->id();
    joint->anchor_b = Vec3{-1.0f, 0.0f, 0.0f}; // the pivot end of the arm
    joint->enable_limits = true;
    joint->limit_axis = Vec3{0.0f, 0.0f, 1.0f};
    joint->lower_angle = -1.0f;
    joint->upper_angle = 1.0f;

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 240);

    CHECK(system.joint_count() == 1);
    // Left free, gravity would swing the arm down to about -pi/2 (hanging
    // straight down); the lower stop holds it at -1.0 instead.
    const f32 swing = physics3d::rotation_vector(Quat::from_euler(arm->transform().rotation)).z;
    CHECK(swing >= -1.1f);
    CHECK(swing == doctest::Approx(-1.0f).epsilon(0.05));
}

TEST_CASE("a limited 3D revolute joint swings freely inside its limits") {
    Scene scene;
    Node* pivot = scene.create_node("Pivot");
    Node* arm = add_bare_body(scene, Vec3{1.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = arm->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = true;

    Node* host = scene.create_node("Hinge");
    auto* joint = host->add_component<physics3d::RevoluteJoint3D>();
    joint->body_a = pivot->id();
    joint->body_b = arm->id();
    joint->anchor_b = Vec3{-1.0f, 0.0f, 0.0f};
    joint->enable_limits = true;
    joint->limit_axis = Vec3{0.0f, 0.0f, 1.0f};
    joint->lower_angle = -3.0f; // wide enough never to touch
    joint->upper_angle = 3.0f;

    physics3d::Physics3DSystem system;
    f32 min_swing = 0.0f;
    for (int i = 0; i < 240; ++i) {
        system.update(step(1.0f / 60.0f), scene);
        min_swing = std::min(min_swing, physics3d::rotation_vector(Quat::from_euler(arm->transform().rotation)).z);
    }

    // The slack rows exerted nothing: the arm swung past the -1.0 the tighter
    // joint is stopped at, towards hanging straight down.
    CHECK(min_swing < -1.2f);
}

TEST_CASE("a 3D revolute joint's angular limits round-trip through the serializer") {
    reflect::register_builtin_types();
    reflect::register_physics3d_types();

    Scene scene;
    Node* a = scene.create_node("A");
    Node* b = scene.create_node("B");

    Node* host = scene.create_node("Hinge");
    auto* joint = host->add_component<physics3d::RevoluteJoint3D>();
    joint->body_a = a->id();
    joint->body_b = b->id();
    joint->enable_limits = true;
    joint->limit_axis = Vec3{0.0f, 1.0f, 0.0f};
    joint->lower_angle = -0.75f;
    joint->upper_angle = 1.25f;

    auto json = serialization::scene::to_json(scene);
    REQUIRE(json);

    Scene loaded;
    REQUIRE_FALSE(serialization::scene::from_json(json.value(), loaded));

    Node* loaded_host = loaded.find_node_by_name("Hinge");
    REQUIRE(loaded_host != nullptr);
    const auto* loaded_joint = loaded_host->get_component<physics3d::RevoluteJoint3D>();
    REQUIRE(loaded_joint != nullptr);
    CHECK(loaded_joint->enable_limits);
    CHECK(loaded_joint->limit_axis.y == doctest::Approx(1.0f));
    CHECK(loaded_joint->lower_angle == doctest::Approx(-0.75f));
    CHECK(loaded_joint->upper_angle == doctest::Approx(1.25f));
}

TEST_CASE("a 3D revolute joint motor round-trips through the serializer") {
    reflect::register_builtin_types();
    reflect::register_physics3d_types();

    Scene scene;
    Node* a = scene.create_node("A");
    Node* b = scene.create_node("B");

    Node* host = scene.create_node("Motor");
    auto* joint = host->add_component<physics3d::RevoluteJoint3D>();
    joint->body_a = a->id();
    joint->body_b = b->id();
    joint->motor_axis = Vec3{1.0f, 0.0f, 0.0f};
    joint->motor_speed = 4.5f;
    joint->motor_max_torque = 12.0f;

    auto json = serialization::scene::to_json(scene);
    REQUIRE(json);

    Scene loaded;
    REQUIRE_FALSE(serialization::scene::from_json(json.value(), loaded));

    Node* loaded_host = loaded.find_node_by_name("Motor");
    REQUIRE(loaded_host != nullptr);
    const auto* loaded_joint = loaded_host->get_component<physics3d::RevoluteJoint3D>();
    REQUIRE(loaded_joint != nullptr);
    CHECK(loaded_joint->motor_axis.x == doctest::Approx(1.0f));
    CHECK(loaded_joint->motor_speed == doctest::Approx(4.5f));
    CHECK(loaded_joint->motor_max_torque == doctest::Approx(12.0f));
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

// ---------------------------------------------------------------------------
// Distance-joint length limits in 3D: the pure target rule, then a leash
// through the system (ADR-0134).
// ---------------------------------------------------------------------------

TEST_CASE("a 3D distance joint with no limits is a plain rod") {
    CHECK_FALSE(physics3d::has_length_limits(physics3d::DistanceJoint3DParams{}));

    bool active = false;
    physics3d::DistanceJoint3DParams params;
    params.rest_length = 2.0f;
    CHECK(physics3d::distance_limit_target(5.0f, params, active) == doctest::Approx(2.0f));
    CHECK(active);
}

TEST_CASE("a 3D distance joint's maximum is a leash and its inside is slack") {
    physics3d::DistanceJoint3DParams params;
    params.max_length = 3.0f;

    bool active = false;
    CHECK(physics3d::distance_limit_target(4.5f, params, active) == doctest::Approx(3.0f));
    CHECK(active);
    // Inside the leash the target falls back to rest_length (0 here) and the
    // joint is slack.
    (void)physics3d::distance_limit_target(2.0f, params, active);
    CHECK_FALSE(active);
}

TEST_CASE("a 3D limited distance row reports the stretch past the bound") {
    physics3d::JointBody3D a = joint_body(Vec3{0.0f, 0.0f, 0.0f}, 1.0f);
    physics3d::JointBody3D b = joint_body(Vec3{5.0f, 0.0f, 0.0f}, 1.0f);

    physics3d::DistanceJoint3DParams params;
    params.max_length = 3.0f;

    auto row = physics3d::build_distance_row(a, b, params, 1.0f / 60.0f);
    REQUIRE(row);
    CHECK(row.value().position_error == doctest::Approx(2.0f));
}

TEST_CASE("a 3D distance joint's maximum leash stops a body from flying off") {
    Scene scene;
    Node* anchor = scene.create_node("Anchor");
    Node* flyer_node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = flyer_node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = false;
    body->velocity = Vec3{10.0f, 0.0f, 0.0f};

    Node* host = scene.create_node("Leash");
    auto* joint = host->add_component<physics3d::DistanceJoint3D>();
    joint->body_a = anchor->id();
    joint->body_b = flyer_node->id();
    joint->rest_length = 0.0f;
    joint->max_length = 3.0f;

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 240);

    CHECK(system.joint_count() == 1);
    CHECK(flyer_node->transform().position.x < 3.5f);
    CHECK(flyer_node->transform().position.x > 2.0f);
}

TEST_CASE("a distance joint's limits round-trip through the serializer") {
    reflect::register_builtin_types();
    reflect::register_physics3d_types();

    Scene scene;
    Node* a = scene.create_node("A");
    Node* b = scene.create_node("B");
    Node* joint_node = scene.create_node("Rod");
    auto* joint = joint_node->add_component<physics3d::DistanceJoint3D>();
    joint->body_a = a->id();
    joint->body_b = b->id();
    joint->min_length = 0.5f;
    joint->max_length = 4.0f;

    auto json = serialization::scene::to_json(scene);
    REQUIRE(json);

    Scene loaded;
    REQUIRE_FALSE(serialization::scene::from_json(json.value(), loaded));

    Node* loaded_joint = loaded.find_node_by_name("Rod");
    REQUIRE(loaded_joint != nullptr);
    const auto* loaded_component = loaded_joint->get_component<physics3d::DistanceJoint3D>();
    REQUIRE(loaded_component != nullptr);
    CHECK(loaded_component->min_length == doctest::Approx(0.5f));
    CHECK(loaded_component->max_length == doctest::Approx(4.0f));
}

// ---------------------------------------------------------------------------
// Joint reactions and breaking in 3D: a joint reports the force it carried and
// snaps once that force exceeds its threshold. The load is gravity on a hanging
// body, so the force is a real, sustained one.
// ---------------------------------------------------------------------------

TEST_CASE("a 3D revolute joint reports the load of the body it hangs") {
    Scene scene;
    Node* pivot = scene.create_node("Pivot");                              // static: no rigid body
    Node* bob = add_bare_body(scene, Vec3{0.0f, -2.0f, 0.0f}, 2.0f, 0.0f); // 2 kg, 2 units below

    Node* host = scene.create_node("Pin");
    auto* joint = host->add_component<physics3d::RevoluteJoint3D>();
    joint->body_a = pivot->id();
    joint->body_b = bob->id();

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 1);

    const auto reaction = system.joint_reaction(pivot->id(), bob->id());
    // The pin holds a 2 kg bob against gravity (9.81), so it carries about 19.6 N.
    CHECK(reaction.peak_force == doctest::Approx(2.0f * 9.81f).epsilon(0.15));
    CHECK(reaction.force.y == doctest::Approx(2.0f * 9.81f).epsilon(0.15));
}

TEST_CASE("a 3D joint with no break threshold never snaps under a heavy load") {
    Scene scene;
    Node* anchor = scene.create_node("Anchor");
    Node* mass = add_bare_body(scene, Vec3{0.0f, -3.0f, 0.0f}, 5.0f, 0.0f);

    Node* host = scene.create_node("Rod");
    auto* joint = host->add_component<physics3d::DistanceJoint3D>();
    joint->body_a = anchor->id();
    joint->body_b = mass->id();
    joint->rest_length = 3.0f;

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 120);

    CHECK_FALSE(joint->broken);
    CHECK(system.joint_count() == 1);
}

TEST_CASE("a 3D joint snaps when the load it carries exceeds its break force") {
    Scene scene;
    Node* anchor = scene.create_node("Anchor");
    Node* mass = add_bare_body(scene, Vec3{0.0f, -3.0f, 0.0f}, 5.0f, 0.0f);

    Node* host = scene.create_node("Rod");
    auto* joint = host->add_component<physics3d::DistanceJoint3D>();
    joint->body_a = anchor->id();
    joint->body_b = mass->id();
    joint->rest_length = 3.0f;
    joint->break_force = 10.0f; // the 5 kg bob pulls ~49 N, so this rod snaps

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 60);

    CHECK(joint->broken);
    CHECK(system.joint_count() == 0);
}

TEST_CASE("a broken 3D joint stops constraining its bodies") {
    Scene scene;
    Node* anchor = scene.create_node("Anchor");
    Node* mass = add_bare_body(scene, Vec3{0.0f, -3.0f, 0.0f}, 5.0f, 0.0f);

    Node* host = scene.create_node("Rod");
    auto* joint = host->add_component<physics3d::DistanceJoint3D>();
    joint->body_a = anchor->id();
    joint->body_b = mass->id();
    joint->rest_length = 3.0f;
    joint->break_force = 10.0f;

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 30);
    REQUIRE(joint->broken);

    const f32 before = mass->transform().position.y;
    run(system, scene, 1.0f / 60.0f, 60);
    CHECK(mass->transform().position.y < before - 3.0f);
}

TEST_CASE("a 3D fixed joint snaps under a torque it cannot hold") {
    Scene scene;
    Node* anchor = scene.create_node("Anchor");
    Node* arm = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    // Give it a spin the weld must stop: a strong torque appears on the first step.
    arm->get_component<physics3d::RigidBody3D>()->angular_velocity = Vec3{0.0f, 0.0f, 20.0f};

    Node* host = scene.create_node("Weld");
    auto* joint = host->add_component<physics3d::FixedJoint3D>();
    joint->body_a = anchor->id();
    joint->body_b = arm->id();
    joint->break_torque = 1.0f; // a weak weld

    physics3d::Physics3DSystem system;
    run(system, scene, 1.0f / 60.0f, 5);

    CHECK(joint->broken);
}

// ---------------------------------------------------------------------------
// Per-step force accumulator in 3D: a force added during the frame acts for
// that step only, through the body's inverse mass, and is cleared afterwards.
// ---------------------------------------------------------------------------

TEST_CASE("a 3D body accelerates by force over mass") {
    Scene scene;
    Node* node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 2.0f, 0.0f);
    auto* body = node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = false;
    body->can_sleep = false;

    physics3d::Physics3DSystem system;
    body->add_force(Vec3{10.0f, 0.0f, 0.0f});
    run(system, scene, 1.0f, 1);

    CHECK(body->velocity.x == doctest::Approx(5.0f).epsilon(0.01));
}

TEST_CASE("a 3D force is cleared after the step it was added") {
    Scene scene;
    Node* node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 0.0f);
    auto* body = node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = false;
    body->can_sleep = false;

    physics3d::Physics3DSystem system;
    body->add_force(Vec3{4.0f, 0.0f, 0.0f});
    run(system, scene, 0.5f, 1);
    const f32 after_first = body->velocity.x;
    CHECK(after_first == doctest::Approx(2.0f).epsilon(0.01));
    CHECK(body->force.x == doctest::Approx(0.0f));

    run(system, scene, 0.5f, 1);
    CHECK(body->velocity.x == doctest::Approx(after_first).epsilon(0.01));
}

TEST_CASE("a 3D applied torque spins a body and is cleared") {
    Scene scene;
    Node* node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 2.0f);
    auto* body = node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = false;
    body->can_sleep = false;

    physics3d::Physics3DSystem system;
    body->add_torque(Vec3{0.0f, 6.0f, 0.0f});
    run(system, scene, 1.0f, 1);

    CHECK(body->angular_velocity.y == doctest::Approx(3.0f).epsilon(0.01));
    CHECK(body->applied_torque.y == doctest::Approx(0.0f));
}

TEST_CASE("a 3D force wakes a sleeping body") {
    Scene scene;
    Node* node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 0.0f);
    auto* body = node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = false;
    body->sleep();

    physics3d::Physics3DSystem system;
    body->add_force(Vec3{3.0f, 0.0f, 0.0f});
    CHECK_FALSE(body->is_sleeping);
    run(system, scene, 1.0f, 1);
    CHECK(body->velocity.x == doctest::Approx(3.0f).epsilon(0.01));
}

TEST_CASE("a 3D off-centre force spins the body and pushes it") {
    Scene scene;
    Node* node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 2.0f, 2.0f);
    auto* body = node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = false;
    body->can_sleep = false;

    physics3d::Physics3DSystem system;
    // A +x force applied one unit above the centre: 10 N gives 5 m/s, and the
    // lever arm (r x F = (0,1,0) x (10,0,0) = (0,0,-10)) gives -10 N*m about z,
    // so 5 rad/s about -z.
    body->add_force_at_offset(Vec3{10.0f, 0.0f, 0.0f}, Vec3{0.0f, 1.0f, 0.0f});
    run(system, scene, 1.0f, 1);

    CHECK(body->velocity.x == doctest::Approx(5.0f).epsilon(0.01));
    CHECK(body->angular_velocity.z == doctest::Approx(-5.0f).epsilon(0.01));
}

TEST_CASE("a 3D force through the centre adds no spin") {
    Scene scene;
    Node* node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = node->get_component<physics3d::RigidBody3D>();
    body->affected_by_gravity = false;
    body->can_sleep = false;

    physics3d::Physics3DSystem system;
    body->add_force_at_offset(Vec3{10.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f});
    run(system, scene, 1.0f, 1);

    CHECK(body->velocity.x == doctest::Approx(10.0f).epsilon(0.01));
    CHECK(body->angular_velocity.length() == doctest::Approx(0.0f));
}

// ---------------------------------------------------------------------------
// Impulses in 3D: a one-shot change to velocity, at the centre or off it.
// ---------------------------------------------------------------------------

TEST_CASE("a 3D impulse changes velocity by impulse over mass") {
    Scene scene;
    Node* node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 4.0f, 0.0f);
    auto* body = node->get_component<physics3d::RigidBody3D>();
    body->apply_impulse(Vec3{8.0f, 0.0f, 0.0f});
    CHECK(body->velocity.x == doctest::Approx(2.0f).epsilon(0.01));
}

TEST_CASE("a 3D off-centre impulse spins the body") {
    Scene scene;
    Node* node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = node->get_component<physics3d::RigidBody3D>();
    // A +x impulse applied one unit above the centre spins it about -z.
    body->apply_impulse_at_offset(Vec3{4.0f, 0.0f, 0.0f}, Vec3{0.0f, 1.0f, 0.0f});
    CHECK(body->velocity.x == doctest::Approx(4.0f).epsilon(0.01));
    CHECK(body->angular_velocity.z == doctest::Approx(-4.0f).epsilon(0.01));
}

TEST_CASE("a 3D impulse through the centre adds no spin") {
    Scene scene;
    Node* node = add_bare_body(scene, Vec3{0.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    auto* body = node->get_component<physics3d::RigidBody3D>();
    body->apply_impulse_at_offset(Vec3{4.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f});
    CHECK(body->angular_velocity.length() == doctest::Approx(0.0f));
}
