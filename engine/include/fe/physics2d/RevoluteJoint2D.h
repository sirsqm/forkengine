// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Math.h"
#include "fe/core/Types.h"
#include "fe/core/Uuid.h"
#include "fe/scene/Component.h"

namespace fe::physics2d {

// A revolute (pin) joint: two bodies share a single anchor point. Their
// relative translation is removed; their relative rotation is left free, so one
// body can swing about the other -- a pendulum, a wheel on an axle, a ragdoll
// limb.
//
// The joint names its two bodies by node id, like the distance joint, so it
// serializes as a stable reference and survives a save/load cycle.
//
// The shared point is authored as a local anchor on each body. Both default to
// the body centre, which pins the two centres together; set one to a local
// offset to pin a point away from the centre. For a pendulum pinned at the top
// of a bob hanging from a fixed pivot, put the bob's local anchor at its top
// and leave the pivot's at its centre.
class RevoluteJoint2D final : public Component {
public:
    static constexpr const char* kTypeName = "RevoluteJoint2D";

    [[nodiscard]] const char* type_name() const noexcept override { return kTypeName; }

    // The two bodies the joint connects, by node id. Either may be nil (the
    // default), in which case the joint is skipped. A missing or non-dynamic
    // body is likewise skipped cleanly.
    Uuid body_a{};
    Uuid body_b{};

    // The shared anchor point expressed in each body's local frame. A local
    // offset is rotated by the body's rotation before it is constrained, so a
    // pendulum pinned off-centre swings correctly.
    Vec2 anchor_a{};
    Vec2 anchor_b{};

    // A motor that spins the joint. `motor_speed` is the relative angular
    // velocity it drives B towards, in radians per second (A's frame is the
    // reference, so a positive value turns B forward about the anchor).
    // `motor_max_torque` is the largest torque, in newton-metres, the motor may
    // apply; a torque too small to overcome the load slips rather than winning.
    // The default zero disables the motor, so a joint with no motor behaves
    // exactly as a plain pin. A wheel, a windmill, or a driven arm is this motor
    // on a revolute joint.
    f32 motor_speed = 0.0f;
    f32 motor_max_torque = 0.0f;

    // A position target turns the motor into a servo: when `motor_use_target` is
    // set the joint drives its relative angle towards `motor_target_angle` (in
    // radians, B's rotation minus A's) instead of a speed, with a proportional
    // `motor_stiffness` and derivative `motor_damping` gain, still bounded by
    // `motor_max_torque`. A stiff, damped target holds a pose against a load -- a
    // robot arm, a door held ajar, a limb posed -- where the speed motor only
    // drives a rate. Off by default, so a motor with no target is the speed motor
    // it was. Ignored while `motor_max_torque` is zero, which disables the motor.
    bool motor_use_target = false;
    f32 motor_target_angle = 0.0f;
    f32 motor_stiffness = 0.0f;
    f32 motor_damping = 0.0f;

    // Angular limits: when `enable_limits` is set the joint may swing freely
    // between `lower_angle` and `upper_angle` (the relative angle of B about the
    // anchor, in radians, measured as B's rotation minus A's) and is stopped at
    // either bound, the way a door stops at 90 degrees or a jointed limb stops at
    // its extreme. Off by default, so the joint swings through a full turn. The
    // limits are compared against the raw relative angle, so author them within
    // one turn of each other (the usual case: a swing of less than 360 degrees).
    bool enable_limits = false;
    f32 lower_angle = 0.0f;
    f32 upper_angle = 0.0f;

    // Break thresholds. When `break_force` is positive the joint snaps (stops
    // being enforced) once the force it must carry exceeds it; likewise
    // `break_torque` for the torque. Zero (the default) means the joint never
    // breaks, so a scene that does not set them is unchanged. A snapped joint
    // sets `broken` and is skipped from then on; set `broken` back to false to
    // re-arm it. This is how a rope under too much load, or a weld that gives
    // way, is authored.
    f32 break_force = 0.0f;
    f32 break_torque = 0.0f;
    bool broken = false;

    [[nodiscard]] bool is_valid() const noexcept { return !body_a.is_nil() && !body_b.is_nil() && body_a != body_b; }
};

} // namespace fe::physics2d
