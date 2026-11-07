// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/physics2d/JointSolver.h"

#include <algorithm>
#include <cmath>
#include <limits>

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

    // With length limits the joint holds the nearer bound when the separation is
    // out of range and is slack inside it, so a rope can go limp instead of
    // pulling both ways. `distance_limit_target` returns the length to hold and
    // whether the joint constrains at all this step.
    bool active = true;
    const f32 target = distance_limit_target(length, params, active);
    if (!active) {
        // Slack: an all-zero row the apply functions treat as a no-op. The joint
        // is still gathered and counted, it just exerts nothing this step.
        return ConstraintRow{};
    }
    if (row.effective_mass <= 0.0f) {
        return Error("distance joint: neither body can move, so the constraint is unsolvable");
    }

    const f32 error = length - target;
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

void build_revolute_limit_rows(const JointBody& a, const JointBody& b, const RevoluteJointParams& params,
                               ConstraintRow& lower_row, ConstraintRow& upper_row) noexcept {
    // Slack rows by default: a zero row is a no-op the apply functions already
    // skip, so a joint with limits off (or one swinging inside them) is unchanged.
    lower_row = ConstraintRow{};
    upper_row = ConstraintRow{};
    if (!params.enable_limits) {
        return;
    }

    // The free coordinate is the relative angle `b - a`. The row measures its
    // rate directly (purely angular Jacobian), so a positive row velocity means
    // the angle is growing and a positive impulse grows it further -- which fixes
    // each bound's clamp to one sign, as the prismatic limits' does.
    const f32 angle = b.rotation - a.rotation;
    const f32 lower = std::min(params.lower_angle, params.upper_angle);
    const f32 upper = std::max(params.lower_angle, params.upper_angle);

    const auto build_limit_row = [&](ConstraintRow& row, f32 bound, f32 clamp_lower, f32 clamp_upper) {
        row.angular_a = -1.0f;
        row.angular_b = 1.0f;
        row.effective_mass = row_effective_mass(row, a, b);
        row.position_error = angle - bound;
        row.position_factor = kJointPositionFactor;
        row.clamp_impulse = true;
        row.lower_impulse = clamp_lower;
        row.upper_impulse = clamp_upper;
    };

    // Past the upper bound the angle must not grow, so only a negative impulse
    // (which shrinks it) is allowed; past the lower bound the opposite.
    if (angle >= upper) {
        build_limit_row(upper_row, upper, -std::numeric_limits<f32>::max(), 0.0f);
    }
    if (angle <= lower) {
        build_limit_row(lower_row, lower, 0.0f, std::numeric_limits<f32>::max());
    }
}

void apply_row(ConstraintRow& row, JointBody& a, JointBody& b) noexcept {
    if (row.effective_mass <= 0.0f) {
        return; // unsolvable row: both bodies are immovable along it
    }

    // The relative velocity along the row, then the impulse that cancels it and
    // applies the bias. One sign convention, shared with the builder.
    const f32 row_velocity = row.jacobian_a.dot(a.velocity) + row.angular_a * a.angular_velocity +
                             row.jacobian_b.dot(b.velocity) + row.angular_b * b.angular_velocity;
    f32 impulse = -row.effective_mass * (row_velocity + row.bias);

    if (row.clamp_impulse) {
        // A motor or a limit: accumulate the impulse and hold the total inside its
        // bounds, then apply only the part that is new. A maximum-force row is
        // exactly this clamp -- once the accumulated impulse reaches the bound the
        // row stops adding, so the joint gives way under a load it cannot hold.
        const f32 previous = row.accumulated_impulse;
        row.accumulated_impulse = std::clamp(previous + impulse, row.lower_impulse, row.upper_impulse);
        impulse = row.accumulated_impulse - previous;
    } else {
        // A rigid row has no bound, but its impulse is still accumulated so the
        // joint's total reaction over the step can be read (for a breaking
        // threshold or a debug overlay).
        row.accumulated_impulse += impulse;
    }

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

void row_reactions(const ConstraintRow rows[], usize count, f32 inv_dt, Vec2& out_linear, f32& out_angular,
                   f32& out_peak) noexcept {
    out_linear = Vec2{};
    out_angular = 0.0f;
    out_peak = 0.0f;

    for (usize i = 0; i < count; ++i) {
        const ConstraintRow& row = rows[i];
        if (row.effective_mass <= 0.0f) {
            continue; // never solved, so it applied nothing
        }
        // The impulse the row applied to B, as a force over the step. B is the
        // end that is usually dynamic, so the reaction reads as the force the
        // joint put on the moving body; it is zero when B is the immovable end.
        out_linear += row.jacobian_b * (row.accumulated_impulse * inv_dt);
        out_angular += row.angular_b * row.accumulated_impulse * inv_dt;
        // The magnitude the row itself carried, independent of which end is
        // movable. This is what a break threshold compares against.
        out_peak = std::max(out_peak, std::abs(row.accumulated_impulse) * inv_dt);
    }
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

    build_angle_lock_row(a, b, params.reference_angle, out_rows[kPrismaticAngleRow]);

    // The two translation-limit rows act along the slide axis. They are zero rows
    // when limits are off, which `apply_row`/`apply_row_position` treat as no-ops,
    // so the free-sliding case is unchanged. When limits are on, the row for each
    // bound is built only when the joint is at or past it, and its impulse is
    // clamped to one sign: a limit pushes the pair back inside and never pulls it
    // out, so a body that has come to rest against a stop can leave it again.
    const Vec2 direction = axis.normalized();
    out_rows[kPrismaticLowerLimitRow] = ConstraintRow{};
    out_rows[kPrismaticUpperLimitRow] = ConstraintRow{};

    if (params.enable_limits) {
        // The signed position along the axis, B's anchor relative to A's. The
        // perpendicular row already removes the across-axis part of the anchor
        // error, so this is the single free coordinate. Both limit rows share the
        // same Jacobian (-direction on A, +direction on B), so a positive row
        // velocity means the coordinate is growing and a positive impulse grows
        // it further -- which is what fixes each bound's clamp to one sign.
        const f32 along = anchor_error.dot(direction) * -1.0f;
        const f32 lower = std::min(params.lower_translation, params.upper_translation);
        const f32 upper = std::max(params.lower_translation, params.upper_translation);

        const auto build_limit_row = [&](ConstraintRow& row, f32 bound, f32 clamp_lower, f32 clamp_upper) {
            row.jacobian_a = direction * -1.0f;
            row.jacobian_b = direction;
            row.angular_a = cross(lever_a, row.jacobian_a);
            row.angular_b = cross(lever_b, row.jacobian_b);
            row.effective_mass = row_effective_mass(row, a, b);
            row.position_error = along - bound;
            row.position_factor = kJointPositionFactor;
            row.clamp_impulse = true;
            row.lower_impulse = clamp_lower;
            row.upper_impulse = clamp_upper;
        };

        // Past the upper bound the coordinate must not grow, so only a negative
        // impulse (which shrinks it) is allowed. Past the lower bound the opposite.
        if (along >= upper) {
            build_limit_row(out_rows[kPrismaticUpperLimitRow], upper, -std::numeric_limits<f32>::max(), 0.0f);
        }
        if (along <= lower) {
            build_limit_row(out_rows[kPrismaticLowerLimitRow], lower, 0.0f, std::numeric_limits<f32>::max());
        }
    }

    bool any_solvable = translation.effective_mass > 0.0f || out_rows[kPrismaticAngleRow].effective_mass > 0.0f;
    if (params.enable_limits) {
        any_solvable = any_solvable || out_rows[kPrismaticLowerLimitRow].effective_mass > 0.0f ||
                       out_rows[kPrismaticUpperLimitRow].effective_mass > 0.0f;
    }
    if (!any_solvable) {
        return Error("prismatic joint: neither body can move, so the constraint is unsolvable");
    }
    return Result<void>{};
}

Vec2 prismatic_world_axis(const JointBody& a, const PrismaticJointParams& params) noexcept {
    return rotate(params.axis, a.rotation).normalized();
}

Result<ConstraintRow> build_revolute_motor_row(const JointBody& a, const JointBody& b,
                                               const JointMotor& motor) noexcept {
    if (!motor.is_enabled()) {
        return Error("revolute motor: the maximum impulse is zero, so the motor is disabled");
    }

    // The row measures the relative angular velocity b - a (angular_a = -1,
    // angular_b = +1), so a positive impulse spins B forward relative to A. The
    // bias is set so the row is satisfied when that relative speed reaches the
    // target: with the impulse `-eff * (row_velocity + bias)`, `bias` must be the
    // negative of the target.
    ConstraintRow row;
    row.angular_a = -1.0f;
    row.angular_b = 1.0f;
    row.effective_mass = row_effective_mass(row, a, b);
    row.bias = -motor.target_speed;

    if (row.effective_mass <= 0.0f) {
        return Error("revolute motor: neither body can rotate, so the motor has nothing to turn");
    }

    // A maximum-force motor is an impulse clamp, symmetric about zero.
    row.clamp_impulse = true;
    row.lower_impulse = -motor.max_impulse;
    row.upper_impulse = motor.max_impulse;
    return row;
}

Result<ConstraintRow> build_revolute_position_motor_row(const JointBody& a, const JointBody& b, const JointMotor& motor,
                                                        f32 current_angle) noexcept {
    if (!motor.use_target) {
        return Error("revolute position motor: no position target is set");
    }

    // The same angular row as the speed motor (it measures the relative angular
    // velocity b - a), but the bias is a spring: a restoring velocity
    // proportional to the angle error plus a damping term opposing the current
    // relative velocity. The error is the shortest signed difference, so a
    // target across the +/-pi seam is approached the short way. The impulse is
    // clamped to `max_impulse`, so a servo too weak to hold its pose slips.
    ConstraintRow row;
    row.angular_a = -1.0f;
    row.angular_b = 1.0f;
    row.effective_mass = row_effective_mass(row, a, b);

    if (row.effective_mass <= 0.0f) {
        return Error("revolute position motor: neither body can rotate, so the motor has nothing to drive");
    }

    // The error is the shortest signed difference `current - target`, so a joint
    // past its target is driven back towards it (the same sign convention the
    // distance spring uses: a positive error drives a negative row velocity).
    // Taking the shortest arc means a target across the +/-pi seam is approached
    // the short way.
    const f32 error = angle_difference(motor.target_position, current_angle);
    const f32 row_velocity = a.angular_velocity * row.angular_a + b.angular_velocity * row.angular_b;
    row.bias = motor.stiffness * error + motor.damping * row_velocity;

    row.clamp_impulse = true;
    row.lower_impulse = -motor.max_impulse;
    row.upper_impulse = motor.max_impulse;
    return row;
}

Result<ConstraintRow> build_prismatic_motor_row(const JointBody& a, const JointBody& b,
                                                const PrismaticJointParams& params, const JointMotor& motor) noexcept {
    if (!motor.is_enabled()) {
        return Error("prismatic motor: the maximum impulse is zero, so the motor is disabled");
    }

    const Vec2 axis = rotate(params.axis, a.rotation);
    if (axis.length_squared() <= kMinSeparation) {
        return Error("prismatic motor: the slide axis is degenerate");
    }
    // The motor acts along the slide axis at the two centres, the same direction
    // the joint leaves free. The prismatic's rotation lock keeps the two frames
    // aligned, so a centre-based row drives the one free degree of freedom
    // exactly; the sign convention matches the distance row (positive separates
    // B from A along the axis).
    const Vec2 direction = axis.normalized();
    ConstraintRow row;
    row.jacobian_a = direction * -1.0f;
    row.jacobian_b = direction;
    row.effective_mass = row_effective_mass(row, a, b);
    row.bias = -motor.target_speed;

    if (row.effective_mass <= 0.0f) {
        return Error("prismatic motor: neither body can move along the axis, so the motor has nothing to drive");
    }

    row.clamp_impulse = true;
    row.lower_impulse = -motor.max_impulse;
    row.upper_impulse = motor.max_impulse;
    return row;
}

Result<ConstraintRow> build_prismatic_position_motor_row(const JointBody& a, const JointBody& b,
                                                         const PrismaticJointParams& params, const JointMotor& motor,
                                                         f32 current_translation) noexcept {
    if (!motor.use_target) {
        return Error("prismatic position motor: no position target is set");
    }

    const Vec2 axis = rotate(params.axis, a.rotation);
    if (axis.length_squared() <= kMinSeparation) {
        return Error("prismatic position motor: the slide axis is degenerate");
    }

    // The same row as the speed motor (it measures the rate the slide coordinate
    // grows at), but the bias is a spring: a restoring velocity proportional to
    // the translation error plus a damping term opposing the current slide
    // speed. The error is `current - target`, so a joint past its target is
    // driven back. The impulse is clamped to `max_impulse`, so a servo too weak
    // to hold its position slips.
    const Vec2 direction = axis.normalized();
    ConstraintRow row;
    row.jacobian_a = direction * -1.0f;
    row.jacobian_b = direction;
    row.effective_mass = row_effective_mass(row, a, b);

    if (row.effective_mass <= 0.0f) {
        return Error(
            "prismatic position motor: neither body can move along the axis, so the motor has nothing to drive");
    }

    const f32 error = current_translation - motor.target_position;
    const f32 row_velocity = row.jacobian_a.dot(a.velocity) + row.jacobian_b.dot(b.velocity);
    row.bias = motor.stiffness * error + motor.damping * row_velocity;

    row.clamp_impulse = true;
    row.lower_impulse = -motor.max_impulse;
    row.upper_impulse = motor.max_impulse;
    return row;
}

f32 prismatic_slide_coordinate(const JointBody& a, const JointBody& b, const PrismaticJointParams& params) noexcept {
    const Vec2 axis = rotate(params.axis, a.rotation);
    if (axis.length_squared() <= kMinSeparation) {
        return 0.0f; // degenerate axis: no slide to measure
    }
    const Vec2 lever_a = rotate(params.anchor_a, a.rotation);
    const Vec2 lever_b = rotate(params.anchor_b, b.rotation);
    const Vec2 anchor_error = (a.position + lever_a) - (b.position + lever_b);
    // The signed translation the limits use: B's anchor relative to A's along the
    // axis, positive when B is ahead of A.
    return anchor_error.dot(axis.normalized()) * -1.0f;
}

} // namespace fe::physics2d
