// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Error.h"
#include "fe/core/Math.h"
#include "fe/core/Types.h"

#include <algorithm>
#include <cmath>

namespace fe::physics3d {

// The pure half of joint solving for 3D: the per-body state a constraint row
// reads and writes, and the distance-constraint row built from it.
//
// This header has no scene, no components, and no allocation, for the same
// reason Geometry3D.h has none: the maths is the part worth testing
// exhaustively, and it must be callable directly without a world.
// `Physics3DSystem` gathers the scene into these values, calls build_*, and
// applies the returned row to its bodies; a test can call the same functions
// with hand-written inputs and assert the numbers.
//
// The row is solved by sequential impulses with a velocity bias, the same shape
// as the contact solver (ADR-0009): the bias is the target relative velocity that
// drives the constraint error to zero over the next steps, so a joint and a
// contact can be solved by one loop without it knowing which is which.
//
// The row carries angular terms (ADR-0043), so a joint can spin its ends when the
// constraint acts off-centre. A distance joint measures between the two centres,
// so its angular Jacobians stay zero and it remains linear in practice; the terms
// exist so the row type is complete and a future anchor-based joint (a revolute
// or point-to-point constraint) needs no new machinery, exactly as the 2D row
// carries them for its revolute joint (ADR-0021).

// One end of a joint: the linear and angular state a constraint row reads and
// writes, plus the inverse mass and inertia that decide how the correction is
// shared. A static body carries zero inverses, so a joint to it moves only the
// other end, exactly as the contact solver treats a static collider.
struct JointBody3D {
    Vec3 position{};
    Vec3 velocity{};
    Quat orientation{};
    Vec3 angular_velocity{};
    f32 inv_mass = 0.0f;
    f32 inv_inertia = 0.0f;
};

// A scalar constraint row: a 1-DOF correction the solver applies by weighting
// each body by the row's Jacobian and the body's inverse mass.
//
// `jacobian_a`/`jacobian_b` are the row's linear Jacobians at each body centre,
// `angular_a`/`angular_b` the angular Jacobians (the lever arm crossed with the
// linear Jacobian), and `effective_mass` the inverse of the row's apparent mass
// (1 / (J M^-1 J^T)).
//
// The row carries two corrections, mirroring how the contact solver separates a
// velocity impulse from a positional one:
//   * `bias` is the target relative velocity along the row. A rigid rod leaves
//     it zero (the velocity impulse then simply cancels relative motion); a
//     spring sets it from the stretch so the joint pulls like a spring.
//   * `position_error` and `position_factor` drive a direct positional
//     correction: `position_factor` of the error is removed along the row each
//     pass. A rigid rod uses this; a spring sets `position_factor` to zero so
//     the soft bias is the only thing acting.
struct ConstraintRow3D {
    Vec3 jacobian_a{};
    Vec3 jacobian_b{};
    Vec3 angular_a{};
    Vec3 angular_b{};
    f32 effective_mass = 0.0f;
    f32 bias = 0.0f;
    f32 position_error = 0.0f;
    f32 position_factor = 0.0f;

    // Clamped-impulse support, for a motor row. When `clamp_impulse` is true the
    // row accumulates the velocity impulse it applies in `accumulated_impulse`
    // and holds it inside [lower_impulse, upper_impulse], so the joint drives its
    // relative velocity towards the bias up to a maximum force and then gives way
    // rather than spinning the bodies forever. A row that does not clamp leaves
    // these fields alone, so a pin or a rod is unchanged. Mirrors the 2D row.
    bool clamp_impulse = false;
    f32 accumulated_impulse = 0.0f;
    f32 lower_impulse = 0.0f;
    f32 upper_impulse = 0.0f;
};

// The apparent mass of a row: 1 / (J M^-1 J^T), summing the linear and angular
// terms of both bodies. A zero result means neither body can move along the row
// (both are static), which the callers report as an unsolvable constraint.
[[nodiscard]] inline f32 row_effective_mass(const ConstraintRow3D& row, const JointBody3D& a,
                                            const JointBody3D& b) noexcept {
    const f32 linear = a.inv_mass * row.jacobian_a.length_squared() + b.inv_mass * row.jacobian_b.length_squared();
    const f32 angular = a.inv_inertia * row.angular_a.length_squared() + b.inv_inertia * row.angular_b.length_squared();
    const f32 denom = linear + angular;
    return denom > 0.0f ? 1.0f / denom : 0.0f;
}

// --- Extended joint set (ADR-0134) ----------------------------------------

// The rotation vector (axis * angle, in world space) of a unit quaternion: the
// smallest rotation that takes the identity to `q`. For a tiny rotation it
// returns the small-angle limit 2 * vec(q), so a near-identity quaternion still
// reports its small error rather than collapsing to zero.
[[nodiscard]] inline Vec3 rotation_vector(const Quat& q_in) noexcept {
    Quat q = q_in.normalized();
    if (q.w < 0.0f) {
        q = q * -1.0f; // q and -q are the same rotation; take the shorter arc
    }
    const Vec3 axis{q.x, q.y, q.z};
    const f32 sin_half = axis.length();
    if (sin_half < 1e-6f) {
        return axis * 2.0f; // small-angle limit, where angle ~ 2 * |vec|
    }
    const f32 w = std::clamp(q.w, -1.0f, 1.0f);
    const f32 angle = 2.0f * std::acos(w);
    return axis / sin_half * angle;
}

// Rotates `q` by the world-space rotation vector `delta` (left-multiply, so the
// axis is a world axis). This is the positional counterpart of an angular
// velocity impulse: where `apply_row` adds to `angular_velocity`, this turns the
// orientation directly.
[[nodiscard]] inline Quat apply_rotation_delta(const Quat& q, const Vec3& delta) noexcept {
    const f32 angle = delta.length();
    if (angle < 1e-9f) {
        return q;
    }
    return (Quat::from_axis_angle(delta / angle, angle) * q).normalized();
}

// Two unit vectors orthogonal to the unit `axis`, forming a right-handed basis
// with it. Used by the prismatic joint to name the two directions it removes.
inline void perpendicular_basis(const Vec3& axis, Vec3& out_a, Vec3& out_b) noexcept {
    const Vec3 reference = std::abs(axis.x) < 0.9f ? Vec3{1.0f, 0.0f, 0.0f} : Vec3{0.0f, 1.0f, 0.0f};
    out_a = axis.cross(reference).normalized();
    out_b = axis.cross(out_a).normalized();
}

// How many rows a joint needs: one for a distance joint, three for a revolute
// joint (the x, y and z components of the shared anchor), six for a fixed joint
// (three translation plus three rotation), seven for a prismatic joint (two
// translation perpendicular to its axis, three rotation, and the two
// translation-limit rows, which are zero when limits are off).
inline constexpr usize kDistanceJoint3DRows = 1;
inline constexpr usize kRevoluteJoint3DRows = 3;
inline constexpr usize kFixedJoint3DRows = 6;
inline constexpr usize kPrismaticJoint3DRows = 7;
// The index of the prismatic joint's first rotation-lock row and its two
// translation-limit rows, so the system and the tests can name them.
inline constexpr usize kPrismatic3DRotationRow = 2;
inline constexpr usize kPrismatic3DLowerLimitRow = 5;
inline constexpr usize kPrismatic3DUpperLimitRow = 6;

// The distance-joint parameters, decoupled from the component so the maths can be
// exercised on its own. The constraint measures between the two body centres.
// `stiffness` is a frequency in Hz (0 gives a rigid rod) and `damping` is a
// fraction in [0, 1] of the current relative velocity the joint opposes, which
// lets a spring lose energy instead of ringing forever.
struct DistanceJoint3DParams {
    f32 rest_length = 0.0f;
    f32 stiffness = 0.0f;
    f32 damping = 0.0f;
    f32 min_length = 0.0f;
    f32 max_length = 0.0f;
};

// Whether the joint has a length limit enabled at all: either bound above zero.
[[nodiscard]] inline bool has_length_limits(const DistanceJoint3DParams& params) noexcept {
    return params.min_length > 0.0f || params.max_length > 0.0f;
}

// The length the two bodies are currently held to: `rest_length` when the
// separation is within the limits, otherwise the nearer bound. Returns the value
// and, through `out_active`, whether the joint constrains at all this step (a
// separation strictly inside the limits, with no spring, is left free).
[[nodiscard]] inline f32 distance_limit_target(f32 separation, const DistanceJoint3DParams& params,
                                               bool& out_active) noexcept {
    out_active = true;
    if (params.max_length > 0.0f && separation > params.max_length) {
        return params.max_length;
    }
    if (params.min_length > 0.0f && separation < params.min_length) {
        return params.min_length;
    }
    if (has_length_limits(params) && params.stiffness <= 0.0f) {
        // Inside the limits with no spring: the rod is slack and exerts nothing.
        out_active = false;
    }
    return params.rest_length;
}

// The revolute-joint parameters. `anchor_a` and `anchor_b` are the shared point
// expressed in each body's local frame, so a pin offset from a body's centre
// (the top of a pendulum bob) needs no separate world anchor to author.
struct RevoluteJoint3DParams {
    Vec3 anchor_a{};
    Vec3 anchor_b{};
};

// The fixed (weld) joint parameters. A fixed joint removes every degree of
// freedom between its two bodies: they keep the relative position *and* the
// relative orientation they had when the joint was authored. The anchors name
// the shared point in each body's local frame, exactly as the revolute joint's
// do; the `reference_*` fields hold the relative rotation the joint is to
// preserve, expressed as each body's local frame rotation relative to the
// other, so the pair is welded in the pose it was built in rather than forced
// to a common world orientation.
//
// `reference_relative` is the target rotation of B relative to A (`q_a^-1 * q_b`
// at build time). A caller that wants to weld the bodies at their *current*
// orientations passes the value `fixed_reference_relative` computes; passing the
// identity instead welds them to a common orientation.
struct FixedJoint3DParams {
    Vec3 anchor_a{};
    Vec3 anchor_b{};
    Quat reference_relative = Quat::identity();
};

// The relative rotation to preserve for a fixed joint built from two bodies in
// their current orientations: `q_a^-1 * q_b`.
[[nodiscard]] inline Quat fixed_reference_relative(const JointBody3D& a, const JointBody3D& b) noexcept {
    return a.orientation.conjugate() * b.orientation;
}

// The prismatic (slider) joint parameters. A prismatic joint allows exactly one
// translational degree of freedom: the two bodies may slide along a shared axis
// but not perpendicular to it, and they may not rotate relative to one another.
// The axis is authored in each body's local frame; a caller that wants the
// bodies to slide along a world direction sets both to that direction. As with
// the fixed joint, `reference_relative` holds the relative rotation to preserve,
// so the slider's axis is a frame the pair shares rather than a world direction
// recomputed each step.
struct PrismaticJoint3DParams {
    Vec3 anchor_a{};
    Vec3 anchor_b{};
    Vec3 axis_a{0.0f, 0.0f, 1.0f};
    Quat reference_relative = Quat::identity();

    // Translation limits along the slide axis. When `enable_limits` is set the
    // pair may slide freely between `lower_translation` and `upper_translation`
    // (B's anchor relative to A's along the axis, so positive means B is ahead)
    // and is stopped at either bound, like a drawer's stops or a piston's end
    // caps. Off by default, so the joint slides freely along its whole axis.
    bool enable_limits = false;
    f32 lower_translation = 0.0f;
    f32 upper_translation = 0.0f;
};

// Builds the single distance-constraint row for the current state of the two
// bodies, for a step of `delta_time` seconds. Returns an Error when the bodies
// are coincident (the constraint direction is undefined), when `delta_time` is
// non-positive, or when neither body can move, so the caller can skip an
// unsolvable joint rather than emit NaNs.
[[nodiscard]] Result<ConstraintRow3D> build_distance_row(const JointBody3D& a, const JointBody3D& b,
                                                         const DistanceJoint3DParams& params, f32 delta_time);

// Builds the three revolute-constraint rows that pin the two bodies' local
// anchors together. Row 0 constrains the x separation, row 1 the y, row 2 the z;
// together they remove all three translational degrees of freedom while leaving
// relative rotation free, so one body swings about the shared point. Errors for
// the same reasons as the distance row.
[[nodiscard]] Result<void> build_revolute_rows(const JointBody3D& a, const JointBody3D& b,
                                               const RevoluteJoint3DParams& params, f32 delta_time,
                                               ConstraintRow3D out_rows[kRevoluteJoint3DRows]);

// The world-space point a revolute joint currently pins, from body A's side.
// Useful for tests and for the editor's debug overlay.
[[nodiscard]] Vec3 revolute_world_anchor(const JointBody3D& a, const RevoluteJoint3DParams& params) noexcept;

// Builds the six fixed-joint rows: three pin the two anchors together (as the
// revolute rows do) and three remove the relative rotation about the x, y and z
// axes. Rows 0-2 are translation, rows 3-5 rotation. `reference_relative` is the
// relative orientation to preserve; the identity welds the bodies to a common
// world orientation.
[[nodiscard]] Result<void> build_fixed_rows(const JointBody3D& a, const JointBody3D& b,
                                            const FixedJoint3DParams& params, f32 delta_time,
                                            ConstraintRow3D out_rows[kFixedJoint3DRows]);

// Builds the five prismatic-joint rows: two remove translation perpendicular to
// the joint axis (rows 0-1) and three remove the relative rotation (rows 2-4),
// leaving sliding along the axis as the single free degree of freedom. The axis
// is taken from body A's local frame, rotated into world space.
[[nodiscard]] Result<void> build_prismatic_rows(const JointBody3D& a, const JointBody3D& b,
                                                const PrismaticJoint3DParams& params, f32 delta_time,
                                                ConstraintRow3D out_rows[kPrismaticJoint3DRows]);

// The world-space axis a prismatic joint currently slides along, from body A's
// side. Useful for tests and for the editor's debug overlay.
[[nodiscard]] Vec3 prismatic_world_axis(const JointBody3D& a, const PrismaticJoint3DParams& params) noexcept;

// A motor drives a joint's free degree of freedom towards a target speed.
// `target_speed` is in radians per second (an angular motor) or world units per
// second (a linear motor); `max_impulse` is the largest impulse the motor may
// apply in one step, which is what turns it into a *maximum force* rather than an
// infinite one -- a motor too weak to hold a load slips instead of winning. A
// zero `max_impulse` disables the motor, so a joint with no motor authored
// behaves exactly as before. Mirrors the 2D `JointMotor`, with an axis because a
// 3D revolute joint leaves rotation about any axis free and the motor must name
// which one it drives.
struct JointMotor3D {
    // The spin axis for an angular (revolute) motor, in world space. Ignored by
    // the prismatic linear motor, which acts along its own slide axis.
    Vec3 axis{0.0f, 0.0f, 1.0f};
    f32 target_speed = 0.0f;
    f32 max_impulse = 0.0f;

    [[nodiscard]] bool is_enabled() const noexcept { return max_impulse > 0.0f; }
};

// Builds the motor row for a revolute (ball-socket) joint: it drives the relative
// angular velocity (B minus A) about `motor.axis` towards `motor.target_speed`,
// clamped to `max_impulse`. A 3D revolute joint leaves rotation about every axis
// free, so the motor names the one it turns. Errors when the axis is degenerate
// or neither body can rotate.
[[nodiscard]] Result<ConstraintRow3D> build_revolute_motor_row(const JointBody3D& a, const JointBody3D& b,
                                                               const JointMotor3D& motor) noexcept;

// Builds the motor row for a prismatic joint: it drives the relative speed along
// the slide axis towards `motor.target_speed`, clamped to `max_impulse`. The row
// acts along the joint axis (body A's local frame, rotated to world), so it is a
// linear motor -- a piston that extends at a set rate, an elevator that climbs at
// a set speed. Errors when the axis is degenerate or neither body can move.
[[nodiscard]] Result<ConstraintRow3D> build_prismatic_motor_row(const JointBody3D& a, const JointBody3D& b,
                                                                const PrismaticJoint3DParams& params,
                                                                const JointMotor3D& motor) noexcept;

// Applies a solved row's velocity impulse to the two bodies: the impulse is the
// current relative velocity along the row (linear and angular) plus the row's
// bias, scaled by the row's effective mass. This is the one place the sign
// convention lives, so the solver and any test agree by construction.
//
// A clamping row (a motor) accumulates its impulse in the row it is given and
// holds it inside its bounds, so it drives the relative velocity only as hard as
// the bounds allow. The row is taken by non-const reference for that accumulator;
// a row that does not clamp is not modified.
void apply_row(ConstraintRow3D& row, JointBody3D& a, JointBody3D& b) noexcept;

// Applies a solved row's positional correction: `position_factor` of the row's
// position error is removed along the row, split by inverse mass and inertia. A
// no-op when the row has no position term (a spring) or is unsolvable.
void apply_row_position(const ConstraintRow3D& row, JointBody3D& a, JointBody3D& b) noexcept;

// The reaction a solved set of rows applied, derived from each row's accumulated
// impulse. `inv_dt` is 1 / delta_time, so the outputs are forces and torques
// rather than per-step impulses. `out_linear` is the net force on body A,
// `out_angular` the net torque on A, and `out_peak` the largest single-row force,
// which is what a breaking threshold tests. All three are zero for rows that were
// never solved.
void row_reactions(const ConstraintRow3D rows[], usize count, f32 inv_dt, Vec3& out_linear, Vec3& out_angular,
                   f32& out_peak) noexcept;

// Applies a solved row's *orientation* correction, the angular counterpart of
// `apply_row_position`: `position_factor` of the row's rotation error is removed
// about the row's angular Jacobian, split by inverse inertia. The linear
// Jacobians are ignored, so a row whose constraint is purely rotational (a weld
// or a slider's rotation lock) corrects the two orientations directly rather
// than through the velocity response. A no-op when the row has no position term
// or is unsolvable.
void apply_row_orientation(const ConstraintRow3D& row, JointBody3D& a, JointBody3D& b) noexcept;

} // namespace fe::physics3d
