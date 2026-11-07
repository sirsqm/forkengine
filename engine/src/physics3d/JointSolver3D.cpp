// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/physics3d/JointSolver3D.h"

#include "fe/physics3d/Geometry3D.h"

#include <algorithm>
#include <limits>

namespace fe::physics3d {
namespace {

// Fraction of a rigid joint's position error removed per solve pass. Correcting
// the whole error at once overshoots and rings; a fraction converges smoothly.
// This is the joint counterpart of the contact solver's Baumgarte factor.
constexpr f32 kJointPositionFactor = 0.2f;

// Separation below which the distance constraint direction is undefined. Two
// coincident centres have no line to measure along, so the joint is reported as
// unsolvable rather than producing a NaN direction.
constexpr f32 kMinSeparation = 1e-6f;

} // namespace

Result<ConstraintRow3D> build_distance_row(const JointBody3D& a, const JointBody3D& b,
                                           const DistanceJoint3DParams& params, f32 delta_time) {
    if (delta_time <= 0.0f) {
        return Error("distance joint: non-positive delta time");
    }

    const Vec3 separation = b.position - a.position;
    const f32 length = separation.length();
    if (length <= kMinSeparation) {
        return Error("distance joint: the two bodies are coincident, so the constraint has no direction");
    }

    // The row measures the separation along the unit vector from A to B: the
    // Jacobian is -n on A and +n on B, so a positive row velocity means the
    // bodies are separating.
    const Vec3 direction = separation / length;
    ConstraintRow3D row;
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
        return ConstraintRow3D{};
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

void apply_row(ConstraintRow3D& row, JointBody3D& a, JointBody3D& b) noexcept {
    if (row.effective_mass <= 0.0f) {
        return; // unsolvable row: both bodies are immovable along it
    }

    // The relative velocity along the row (linear and angular), then the impulse
    // that cancels it and applies the bias. One sign convention, shared with the
    // builder.
    const f32 row_velocity = row.jacobian_a.dot(a.velocity) + row.angular_a.dot(a.angular_velocity) +
                             row.jacobian_b.dot(b.velocity) + row.angular_b.dot(b.angular_velocity);
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
    b.velocity += row.jacobian_b * (impulse * b.inv_mass);
    a.angular_velocity += row.angular_a * (impulse * a.inv_inertia);
    b.angular_velocity += row.angular_b * (impulse * b.inv_inertia);
}

void apply_row_position(const ConstraintRow3D& row, JointBody3D& a, JointBody3D& b) noexcept {
    if (row.effective_mass <= 0.0f || row.position_factor == 0.0f) {
        return;
    }

    // Remove `position_factor` of the error along the row. Scaling by the
    // effective mass is what makes the amount removed independent of the two
    // bodies' masses: the resulting change in the constraint value is exactly
    // position_factor * position_error regardless of how the motion is shared.
    const f32 correction = row.position_factor * row.position_error * row.effective_mass;
    a.position -= row.jacobian_a * (correction * a.inv_mass);
    b.position -= row.jacobian_b * (correction * b.inv_mass);
}

void row_reactions(const ConstraintRow3D rows[], usize count, f32 inv_dt, Vec3& out_linear, Vec3& out_angular,
                   f32& out_peak) noexcept {
    out_linear = Vec3{};
    out_angular = Vec3{};
    out_peak = 0.0f;

    for (usize i = 0; i < count; ++i) {
        const ConstraintRow3D& row = rows[i];
        if (row.effective_mass <= 0.0f) {
            continue; // never solved, so it applied nothing
        }
        // The impulse the row applied to B, as a force over the step. B is the
        // end that is usually dynamic, so the reaction reads as the force the
        // joint put on the moving body; it is zero when B is the immovable end.
        out_linear += row.jacobian_b * (row.accumulated_impulse * inv_dt);
        out_angular += row.angular_b * (row.accumulated_impulse * inv_dt);
        // The magnitude of the force the row itself carried, independent of which
        // end is movable. This is what a break threshold compares against.
        out_peak = std::max(out_peak, std::abs(row.accumulated_impulse) * inv_dt);
    }
}

Result<void> build_revolute_rows(const JointBody3D& a, const JointBody3D& b, const RevoluteJoint3DParams& params,
                                 f32 delta_time, ConstraintRow3D out_rows[kRevoluteJoint3DRows]) {
    if (delta_time <= 0.0f) {
        return Error("revolute joint: non-positive delta time");
    }

    // The world offset of each body's local anchor. The lever arm from the
    // centre to the anchor is what couples the anchor constraint to rotation,
    // which is what makes a pendulum swing instead of sliding.
    const Vec3 lever_a = rotate_by_quat(params.anchor_a, a.orientation);
    const Vec3 lever_b = rotate_by_quat(params.anchor_b, b.orientation);

    // Position error: how far apart the two anchors currently are. Each axis is
    // a separate row, so a shared anchor removes all three translational degrees
    // of freedom while leaving relative rotation free.
    const Vec3 error = (a.position + lever_a) - (b.position + lever_b);
    const Vec3 axes[kRevoluteJoint3DRows] = {Vec3{1.0f, 0.0f, 0.0f}, Vec3{0.0f, 1.0f, 0.0f}, Vec3{0.0f, 0.0f, 1.0f}};
    const f32 errors[kRevoluteJoint3DRows] = {error.x, error.y, error.z};

    bool any_solvable = false;
    for (usize i = 0; i < kRevoluteJoint3DRows; ++i) {
        ConstraintRow3D& row = out_rows[i];
        row.jacobian_a = axes[i];
        row.jacobian_b = axes[i] * -1.0f;
        // The angular Jacobian is the lever arm crossed with the linear one:
        // moving A's centre along +axis rotates its anchor by cross(r, axis).
        row.angular_a = lever_a.cross(row.jacobian_a);
        row.angular_b = lever_b.cross(row.jacobian_b);
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

Vec3 revolute_world_anchor(const JointBody3D& a, const RevoluteJoint3DParams& params) noexcept {
    return a.position + rotate_by_quat(params.anchor_a, a.orientation);
}

void build_revolute_limit_rows(const JointBody3D& a, const JointBody3D& b, const RevoluteJoint3DParams& params,
                               ConstraintRow3D& lower_row, ConstraintRow3D& upper_row) noexcept {
    // Slack rows by default: a zero row is a no-op the apply functions already
    // skip, so a joint with limits off (or one swinging inside them) is unchanged.
    lower_row = ConstraintRow3D{};
    upper_row = ConstraintRow3D{};
    if (!params.enable_limits) {
        return;
    }

    const Vec3 axis_world = rotate_by_quat(params.limit_axis, a.orientation);
    if (axis_world.length_squared() <= kMinSeparation) {
        return; // degenerate axis: no limit, rather than a NaN row
    }
    const Vec3 axis = axis_world.normalized();

    // The swing about the hinge axis: the rotation vector of B relative to A,
    // projected onto that axis. `rotation_vector` takes the shorter arc, so the
    // projection is the signed angle while the swing stays within a half turn.
    const Quat relative = a.orientation.conjugate() * b.orientation;
    const f32 angle = rotation_vector(relative).dot(axis);
    const f32 lower = std::min(params.lower_angle, params.upper_angle);
    const f32 upper = std::max(params.lower_angle, params.upper_angle);

    // The free coordinate grows with the relative spin about the axis, so a
    // positive row velocity means the angle is growing and a positive impulse
    // grows it further -- which fixes each bound's clamp to one sign, as the
    // motor row's does (angular_a = -axis, angular_b = +axis).
    const auto build_limit_row = [&](ConstraintRow3D& row, f32 bound, f32 clamp_lower, f32 clamp_upper) {
        row.angular_a = axis * -1.0f;
        row.angular_b = axis;
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

namespace {

// Builds the three rows that remove the relative rotation between two bodies,
// about the world x, y and z axes, driving the relative orientation to
// `reference_relative`. `first_row` is the index in `out_rows` the three rows
// are written to. Shared by the fixed joint (rows 3-5) and the prismatic joint
// (rows 2-4): both lock rotation completely, so the maths is one function.
//
// The error is the rotation vector of `q_a^-1 * q_b * reference_relative^-1`
// (B's rotation relative to A, beyond the reference), expressed in A's frame and
// rotated into world space, so each row's angular Jacobian is a world axis. Row
// `i` removes the error about world axis `i`; the three rows together remove all
// three angular degrees of freedom.
//
// The error is negated before it is stored: the row's velocity form constrains
// `axis * (omega_a - omega_b)`, so the position error must measure A's rotation
// relative to B, which is the opposite sign to B-relative-to-A.
void build_rotation_lock_rows(const JointBody3D& a, const JointBody3D& b, const Quat& reference_relative,
                              ConstraintRow3D out_rows[], usize first_row) {
    // The relative orientation error, in A's local frame: the rotation that
    // takes the current relative orientation to the reference one.
    const Quat relative = a.orientation.conjugate() * b.orientation;
    const Quat error_local = relative * reference_relative.conjugate();
    const Vec3 error_world = rotate_by_quat(rotation_vector(error_local), a.orientation);

    const Vec3 axes[3] = {Vec3{1.0f, 0.0f, 0.0f}, Vec3{0.0f, 1.0f, 0.0f}, Vec3{0.0f, 0.0f, 1.0f}};
    const f32 errors[3] = {-error_world.x, -error_world.y, -error_world.z};
    for (usize i = 0; i < 3; ++i) {
        ConstraintRow3D& row = out_rows[first_row + i];
        // A purely angular row: the angular Jacobian is the world axis on A and
        // its negative on B, so the row removes the relative rotation about that
        // axis. The linear Jacobians stay zero.
        row.angular_a = axes[i];
        row.angular_b = axes[i] * -1.0f;
        row.effective_mass = row_effective_mass(row, a, b);
        row.position_error = errors[i];
        row.position_factor = kJointPositionFactor;
    }
}

} // namespace

Result<void> build_fixed_rows(const JointBody3D& a, const JointBody3D& b, const FixedJoint3DParams& params,
                              f32 delta_time, ConstraintRow3D out_rows[kFixedJoint3DRows]) {
    if (delta_time <= 0.0f) {
        return Error("fixed joint: non-positive delta time");
    }

    // Rows 0-2 pin the two anchors together, exactly as the revolute joint does;
    // rows 3-5 remove the relative rotation. A fixed joint is therefore a
    // revolute joint plus a rotation lock.
    RevoluteJoint3DParams pin;
    pin.anchor_a = params.anchor_a;
    pin.anchor_b = params.anchor_b;
    if (auto pinned = build_revolute_rows(a, b, pin, delta_time, out_rows); !pinned) {
        return pinned;
    }
    build_rotation_lock_rows(a, b, params.reference_relative, out_rows, kRevoluteJoint3DRows);
    return Result<void>{};
}

Result<void> build_prismatic_rows(const JointBody3D& a, const JointBody3D& b, const PrismaticJoint3DParams& params,
                                  f32 delta_time, ConstraintRow3D out_rows[kPrismaticJoint3DRows]) {
    if (delta_time <= 0.0f) {
        return Error("prismatic joint: non-positive delta time");
    }

    const Vec3 axis = rotate_by_quat(params.axis_a, a.orientation);
    if (axis.length_squared() <= kMinSeparation) {
        return Error("prismatic joint: the slide axis is degenerate");
    }
    const Vec3 unit_axis = axis.normalized();

    // Rows 0-1 remove translation perpendicular to the axis: two directions that
    // span the plane the bodies may not separate in. Sliding along the axis is
    // the one remaining degree of freedom.
    Vec3 perp_a;
    Vec3 perp_b;
    perpendicular_basis(unit_axis, perp_a, perp_b);

    const Vec3 lever_a = rotate_by_quat(params.anchor_a, a.orientation);
    const Vec3 lever_b = rotate_by_quat(params.anchor_b, b.orientation);
    const Vec3 anchor_error = (a.position + lever_a) - (b.position + lever_b);
    const Vec3 axes[2] = {perp_a, perp_b};
    const f32 errors[2] = {anchor_error.dot(perp_a), anchor_error.dot(perp_b)};

    bool any_solvable = false;
    for (usize i = 0; i < 2; ++i) {
        ConstraintRow3D& row = out_rows[i];
        row.jacobian_a = axes[i];
        row.jacobian_b = axes[i] * -1.0f;
        row.angular_a = lever_a.cross(row.jacobian_a);
        row.angular_b = lever_b.cross(row.jacobian_b);
        row.effective_mass = row_effective_mass(row, a, b);
        row.position_error = errors[i];
        row.position_factor = kJointPositionFactor;
        any_solvable = any_solvable || row.effective_mass > 0.0f;
    }

    // Rows 2-4 lock the relative rotation, so the slider's axis is a shared
    // frame rather than a world direction that drifts as the bodies turn.
    build_rotation_lock_rows(a, b, params.reference_relative, out_rows, kPrismatic3DRotationRow);
    for (usize i = kPrismatic3DRotationRow; i < kPrismatic3DLowerLimitRow; ++i) {
        any_solvable = any_solvable || out_rows[i].effective_mass > 0.0f;
    }

    // Rows 5-6 are the translation limits, zero rows when limits are off. Each is
    // built only when the joint reaches its bound and is clamped to one sign, so a
    // stop pushes the pair back inside and never pulls it out.
    out_rows[kPrismatic3DLowerLimitRow] = ConstraintRow3D{};
    out_rows[kPrismatic3DUpperLimitRow] = ConstraintRow3D{};
    if (params.enable_limits) {
        const f32 along = anchor_error.dot(unit_axis) * -1.0f;
        const f32 lower = std::min(params.lower_translation, params.upper_translation);
        const f32 upper = std::max(params.lower_translation, params.upper_translation);

        const auto build_limit_row = [&](ConstraintRow3D& row, f32 bound, f32 clamp_lower, f32 clamp_upper) {
            row.jacobian_a = unit_axis * -1.0f;
            row.jacobian_b = unit_axis;
            row.angular_a = lever_a.cross(row.jacobian_a);
            row.angular_b = lever_b.cross(row.jacobian_b);
            row.effective_mass = row_effective_mass(row, a, b);
            row.position_error = along - bound;
            row.position_factor = kJointPositionFactor;
            row.clamp_impulse = true;
            row.lower_impulse = clamp_lower;
            row.upper_impulse = clamp_upper;
        };

        if (along >= upper) {
            build_limit_row(out_rows[kPrismatic3DUpperLimitRow], upper, -std::numeric_limits<f32>::max(), 0.0f);
        }
        if (along <= lower) {
            build_limit_row(out_rows[kPrismatic3DLowerLimitRow], lower, 0.0f, std::numeric_limits<f32>::max());
        }
        any_solvable = any_solvable || out_rows[kPrismatic3DLowerLimitRow].effective_mass > 0.0f ||
                       out_rows[kPrismatic3DUpperLimitRow].effective_mass > 0.0f;
    }

    if (!any_solvable) {
        return Error("prismatic joint: neither body can move, so the constraint is unsolvable");
    }
    return Result<void>{};
}

Vec3 prismatic_world_axis(const JointBody3D& a, const PrismaticJoint3DParams& params) noexcept {
    return rotate_by_quat(params.axis_a, a.orientation).normalized();
}

void apply_row_orientation(const ConstraintRow3D& row, JointBody3D& a, JointBody3D& b) noexcept {
    if (row.effective_mass <= 0.0f || row.position_factor == 0.0f) {
        return;
    }
    // Remove `position_factor` of the rotation error about the row's angular
    // Jacobian, split by inverse inertia, exactly as `apply_row_position` splits
    // a translation error by inverse mass.
    const f32 correction = row.position_factor * row.position_error * row.effective_mass;
    const Vec3 delta_a = row.angular_a * (correction * a.inv_inertia);
    const Vec3 delta_b = row.angular_b * (correction * b.inv_inertia);
    a.orientation = apply_rotation_delta(a.orientation, delta_a * -1.0f);
    b.orientation = apply_rotation_delta(b.orientation, delta_b * -1.0f);
}

Result<ConstraintRow3D> build_revolute_motor_row(const JointBody3D& a, const JointBody3D& b,
                                                 const JointMotor3D& motor) noexcept {
    if (!motor.is_enabled()) {
        return Error("revolute motor: the maximum impulse is zero, so the motor is disabled");
    }

    const f32 axis_length = motor.axis.length();
    if (axis_length <= 1e-6f) {
        return Error("revolute motor: the spin axis is degenerate");
    }
    const Vec3 axis = motor.axis / axis_length;

    // The row measures the relative angular velocity b - a projected onto the
    // spin axis (angular_a = -axis, angular_b = +axis), so a positive impulse
    // spins B forward relative to A about that axis. The bias drives the
    // projected relative speed to the target: with the impulse
    // `-eff * (row_velocity + bias)`, `bias` must be the negative of the target.
    ConstraintRow3D row;
    row.angular_a = axis * -1.0f;
    row.angular_b = axis;
    row.effective_mass = row_effective_mass(row, a, b);
    row.bias = -motor.target_speed;

    if (row.effective_mass <= 0.0f) {
        return Error("revolute motor: neither body can rotate about the axis, so the motor has nothing to turn");
    }

    row.clamp_impulse = true;
    row.lower_impulse = -motor.max_impulse;
    row.upper_impulse = motor.max_impulse;
    return row;
}

Result<ConstraintRow3D> build_revolute_position_motor_row(const JointBody3D& a, const JointBody3D& b,
                                                          const JointMotor3D& motor, f32 current_angle) noexcept {
    if (!motor.use_target) {
        return Error("revolute position motor: no position target is set");
    }

    const f32 axis_length = motor.axis.length();
    if (axis_length <= 1e-6f) {
        return Error("revolute position motor: the spin axis is degenerate");
    }
    const Vec3 axis = motor.axis / axis_length;

    // The same angular row as the speed motor, but the bias is a spring: a
    // restoring velocity proportional to the angle error plus a damping term
    // opposing the current relative spin about the axis. The error is the
    // shortest signed difference (target - current), so a joint past its target
    // is driven back. The impulse is clamped to `max_impulse`, so a servo too
    // weak to hold its pose slips.
    ConstraintRow3D row;
    row.angular_a = axis * -1.0f;
    row.angular_b = axis;
    row.effective_mass = row_effective_mass(row, a, b);

    if (row.effective_mass <= 0.0f) {
        return Error(
            "revolute position motor: neither body can rotate about the axis, so the motor has nothing to drive");
    }

    const f32 error = angle_difference(motor.target_position, current_angle);
    const f32 row_velocity = a.angular_velocity.dot(row.angular_a) + b.angular_velocity.dot(row.angular_b);
    row.bias = motor.stiffness * error + motor.damping * row_velocity;

    row.clamp_impulse = true;
    row.lower_impulse = -motor.max_impulse;
    row.upper_impulse = motor.max_impulse;
    return row;
}

Result<ConstraintRow3D> build_prismatic_motor_row(const JointBody3D& a, const JointBody3D& b,
                                                  const PrismaticJoint3DParams& params,
                                                  const JointMotor3D& motor) noexcept {
    if (!motor.is_enabled()) {
        return Error("prismatic motor: the maximum impulse is zero, so the motor is disabled");
    }

    const Vec3 axis = rotate_by_quat(params.axis_a, a.orientation);
    if (axis.length_squared() <= 1e-12f) {
        return Error("prismatic motor: the slide axis is degenerate");
    }
    // The motor acts along the slide axis at the two centres, the same direction
    // the joint leaves free. The prismatic's rotation lock keeps the two frames
    // aligned, so a centre-based row drives the one free degree of freedom
    // exactly; the sign matches the distance row (positive separates B from A).
    const Vec3 direction = axis.normalized();
    ConstraintRow3D row;
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

Result<ConstraintRow3D> build_prismatic_position_motor_row(const JointBody3D& a, const JointBody3D& b,
                                                           const PrismaticJoint3DParams& params,
                                                           const JointMotor3D& motor,
                                                           f32 current_translation) noexcept {
    if (!motor.use_target) {
        return Error("prismatic position motor: no position target is set");
    }

    const Vec3 axis = rotate_by_quat(params.axis_a, a.orientation);
    if (axis.length_squared() <= 1e-12f) {
        return Error("prismatic position motor: the slide axis is degenerate");
    }

    // The same row as the speed motor (it measures the rate the slide coordinate
    // grows at), but the bias is a spring: a restoring velocity proportional to
    // the translation error plus a damping term opposing the current slide
    // speed. The error is `current - target`, so a joint past its target is
    // driven back. The impulse is clamped to `max_impulse`, so a servo too weak
    // to hold its position slips.
    const Vec3 direction = axis.normalized();
    ConstraintRow3D row;
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

f32 prismatic_slide_coordinate(const JointBody3D& a, const JointBody3D& b,
                               const PrismaticJoint3DParams& params) noexcept {
    const Vec3 axis = rotate_by_quat(params.axis_a, a.orientation);
    if (axis.length_squared() <= 1e-12f) {
        return 0.0f; // degenerate axis: no slide to measure
    }
    const Vec3 lever_a = rotate_by_quat(params.anchor_a, a.orientation);
    const Vec3 lever_b = rotate_by_quat(params.anchor_b, b.orientation);
    const Vec3 anchor_error = (a.position + lever_a) - (b.position + lever_b);
    // The signed translation the limits use: B's anchor relative to A's along the
    // axis, positive when B is ahead of A.
    return anchor_error.dot(axis.normalized()) * -1.0f;
}

} // namespace fe::physics3d
