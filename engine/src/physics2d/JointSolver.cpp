// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/physics2d/JointSolver.h"

#include <cmath>

namespace fe::physics2d {
namespace {

// Fraction of a rigid joint's position error removed per solve pass. Correcting
// the whole error at once overshoots and rings; a fraction converges smoothly.
// This is the joint counterpart of the contact solver's Baumgarte factor.
constexpr f32 kJointPositionFactor = 0.2f;

// Separation below which the distance constraint direction is undefined. Two
// coincident centres have no line to measure along, so the joint is reported as
// unsolvable rather than producing a NaN direction.
constexpr f32 kMinSeparation = 1e-6f;

// Rotates a local offset into world space by a Z rotation, matching
// Mat4::rotation_z. Used to place a joint anchor on a rotating body.
[[nodiscard]] Vec2 rotate(const Vec2& vector, f32 angle) noexcept {
    const f32 c = std::cos(angle);
    const f32 s = std::sin(angle);
    return Vec2{vector.x * c - vector.y * s, vector.x * s + vector.y * c};
}

} // namespace

Result<ConstraintRow> build_distance_row(const JointBody& a, const JointBody& b, const DistanceJointParams& params,
                                         f32 delta_time) {
    if (delta_time <= 0.0f) {
        return Error("distance joint: non-positive delta time");
    }

    const Vec2 separation = b.position - a.position;
    const f32 length = separation.length();
    if (length <= kMinSeparation) {
        return Error("distance joint: the two bodies are coincident, so the constraint has no direction");
    }

    // The row measures the separation along the unit vector from A to B: the
    // Jacobian is -n on A and +n on B, so a positive row velocity means the
    // bodies are separating.
    const Vec2 direction = separation / length;
    ConstraintRow row;
    row.jacobian_a = direction * -1.0f;
    row.jacobian_b = direction;
    row.effective_mass = row_effective_mass(row, a, b);
    if (row.effective_mass <= 0.0f) {
        return Error("distance joint: neither body can move, so the constraint is unsolvable");
    }

    const f32 error = length - params.rest_length;
    if (params.stiffness > 0.0f) {
        // A spring: the bias is a restoring velocity proportional to the
        // stretch, and `damping` is a derivative gain that opposes the current
        // relative velocity so the spring loses energy instead of ringing. The
        // position is left to the velocity response, so no direct correction.
        const f32 row_velocity = row.jacobian_a.dot(a.velocity) + row.jacobian_b.dot(b.velocity);
        row.bias = params.stiffness * error + params.damping * row_velocity;
    } else {
        // A rigid rod: cancel the relative velocity, then remove a fixed
        // fraction of the position error each pass. Reaching the error at a
        // fixed rate is frame-rate independent, where a velocity bias is not.
        row.position_error = error;
        row.position_factor = kJointPositionFactor;
    }
    return row;
}

Result<void> build_revolute_rows(const JointBody& a, const JointBody& b, const RevoluteJointParams& params,
                                 f32 delta_time, ConstraintRow out_rows[kRevoluteJointRows]) {
    if (delta_time <= 0.0f) {
        return Error("revolute joint: non-positive delta time");
    }

    // The world offset of each body's local anchor. The lever arm from the
    // centre to the anchor is what couples the anchor constraint to rotation,
    // which is what makes a pendulum swing instead of sliding.
    const Vec2 lever_a = rotate(params.anchor_a, a.rotation);
    const Vec2 lever_b = rotate(params.anchor_b, b.rotation);

    // Position error: how far apart the two anchors currently are. Each axis is
    // a separate row, so a shared anchor removes both translational degrees of
    // freedom while leaving relative rotation free.
    const Vec2 error = (a.position + lever_a) - (b.position + lever_b);
    const Vec2 axes[kRevoluteJointRows] = {Vec2{1.0f, 0.0f}, Vec2{0.0f, 1.0f}};
    const f32 errors[kRevoluteJointRows] = {error.x, error.y};

    bool any_solvable = false;
    for (usize i = 0; i < kRevoluteJointRows; ++i) {
        ConstraintRow& row = out_rows[i];
        row.jacobian_a = axes[i];
        row.jacobian_b = axes[i] * -1.0f;
        // The angular Jacobian is the lever arm crossed with the linear one:
        // moving A's centre along +axis rotates its anchor by cross(r, axis).
        row.angular_a = cross(lever_a, row.jacobian_a);
        row.angular_b = cross(lever_b, row.jacobian_b);
        row.effective_mass = row_effective_mass(row, a, b);
        row.position_error = errors[i];
        row.position_factor = kJointPositionFactor;
        any_solvable = any_solvable || row.effective_mass > 0.0f;
    }

    if (!any_solvable) {
        return Error("revolute joint: neither body can move, so the constraint is unsolvable");
    }
    return Result<void>{};
}

void apply_row(const ConstraintRow& row, JointBody& a, JointBody& b) noexcept {
    if (row.effective_mass <= 0.0f) {
        return; // unsolvable row: both bodies are immovable along it
    }

    // The relative velocity along the row, then the impulse that cancels it and
    // applies the bias. One sign convention, shared with the builder.
    const f32 row_velocity = row.jacobian_a.dot(a.velocity) + row.angular_a * a.angular_velocity +
                             row.jacobian_b.dot(b.velocity) + row.angular_b * b.angular_velocity;
    const f32 impulse = -row.effective_mass * (row_velocity + row.bias);

    a.velocity += row.jacobian_a * (impulse * a.inv_mass);
    a.angular_velocity += row.angular_a * impulse * a.inv_inertia;
    b.velocity += row.jacobian_b * (impulse * b.inv_mass);
    b.angular_velocity += row.angular_b * impulse * b.inv_inertia;
}

void apply_row_position(const ConstraintRow& row, JointBody& a, JointBody& b) noexcept {
    if (row.effective_mass <= 0.0f || row.position_factor == 0.0f) {
        return;
    }

    // Remove `position_factor` of the error along the row. Scaling by the
    // effective mass is what makes the amount removed independent of the two
    // bodies' masses: the resulting change in the constraint value is exactly
    // position_factor * position_error regardless of how the motion is shared.
    const f32 correction = row.position_factor * row.position_error * row.effective_mass;
    a.position -= row.jacobian_a * (correction * a.inv_mass);
    a.rotation -= row.angular_a * correction * a.inv_inertia;
    b.position -= row.jacobian_b * (correction * b.inv_mass);
    b.rotation -= row.angular_b * correction * b.inv_inertia;
}

Vec2 revolute_world_anchor(const JointBody& a, const RevoluteJointParams& params) noexcept {
    return a.position + rotate(params.anchor_a, a.rotation);
}

namespace {

// Fills one row that locks the relative angle at `reference_angle`. Shared by
// the fixed joint (row 2) and the prismatic joint (row 1): both hold the two
// bodies at a fixed relative angle, so the maths is one function. The row
// constrains the relative angle `b - a`, matching the revolute rows' `a - b`
// sign so `apply_row_position` splits the correction the same way.
void build_angle_lock_row(const JointBody& a, const JointBody& b, f32 reference_angle, ConstraintRow& row) {
    // Error is the angle the pair is over or under the target by: the excess of
    // `b - a` over `reference_angle`.
    row.angular_a = 1.0f;
    row.angular_b = -1.0f;
    row.effective_mass = row_effective_mass(row, a, b);
    row.position_error = reference_angle - (b.rotation - a.rotation);
    row.position_factor = kJointPositionFactor;
}

} // namespace

Result<void> build_fixed_rows(const JointBody& a, const JointBody& b, const FixedJointParams& params, f32 delta_time,
                              ConstraintRow out_rows[kFixedJointRows]) {
    if (delta_time <= 0.0f) {
        return Error("fixed joint: non-positive delta time");
    }

    // Rows 0-1 pin the two anchors together, exactly as the revolute joint does;
    // row 2 removes the relative angle. A fixed joint is a revolute joint plus
    // an angle lock.
    RevoluteJointParams pin;
    pin.anchor_a = params.anchor_a;
    pin.anchor_b = params.anchor_b;
    if (auto pinned = build_revolute_rows(a, b, pin, delta_time, out_rows); !pinned) {
        return pinned;
    }
    build_angle_lock_row(a, b, params.reference_angle, out_rows[kRevoluteJointRows]);
    return Result<void>{};
}

Result<void> build_prismatic_rows(const JointBody& a, const JointBody& b, const PrismaticJointParams& params,
                                  f32 delta_time, ConstraintRow out_rows[kPrismaticJointRows]) {
    if (delta_time <= 0.0f) {
        return Error("prismatic joint: non-positive delta time");
    }

    const Vec2 axis = rotate(params.axis, a.rotation);
    if (axis.length_squared() <= kMinSeparation) {
        return Error("prismatic joint: the slide axis is degenerate");
    }
    // The one direction the bodies may not separate in: perpendicular to the
    // slide axis.
    const Vec2 perpendicular{-axis.normalized().y, axis.normalized().x};

    const Vec2 lever_a = rotate(params.anchor_a, a.rotation);
    const Vec2 lever_b = rotate(params.anchor_b, b.rotation);
    const Vec2 anchor_error = (a.position + lever_a) - (b.position + lever_b);

    ConstraintRow& translation = out_rows[0];
    translation.jacobian_a = perpendicular;
    translation.jacobian_b = perpendicular * -1.0f;
    translation.angular_a = cross(lever_a, translation.jacobian_a);
    translation.angular_b = cross(lever_b, translation.jacobian_b);
    translation.effective_mass = row_effective_mass(translation, a, b);
    translation.position_error = anchor_error.dot(perpendicular);
    translation.position_factor = kJointPositionFactor;

    build_angle_lock_row(a, b, params.reference_angle, out_rows[1]);

    if (translation.effective_mass <= 0.0f && out_rows[1].effective_mass <= 0.0f) {
        return Error("prismatic joint: neither body can move, so the constraint is unsolvable");
    }
    return Result<void>{};
}

Vec2 prismatic_world_axis(const JointBody& a, const PrismaticJointParams& params) noexcept {
    return rotate(params.axis, a.rotation).normalized();
}

} // namespace fe::physics2d
