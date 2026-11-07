// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Math.h"
#include "fe/core/Types.h"
#include "fe/core/Uuid.h"
#include "fe/scene/Component.h"

namespace fe::physics3d {

// A revolute (pin) joint: two bodies share a single anchor point. Their relative
// translation is removed; their relative rotation is left free, so one body can
// swing about the other -- a pendulum, a wheel on an axle, a ragdoll limb.
//
// This is the 3D counterpart of RevoluteJoint2D, and the anchor-based joint the
// 3D row type was built for (ADR-0043): the row already carries angular terms,
// so pinning an anchor off a body's centre couples the constraint to rotation
// without any new solver machinery. Where the 2D revolute has two rows (x and
// y), this has three (x, y and z), because 3D space has one more translation.
//
// The joint names its two bodies by node id, like the distance joint, so it
// serializes as a stable reference and survives a save/load cycle.
//
// The shared point is authored as a local anchor on each body. Both default to
// the body centre, which pins the two centres together; set one to a local
// offset to pin a point away from the centre. For a pendulum pinned at the top
// of a bob hanging from a fixed pivot, put the bob's local anchor at its top and
// leave the pivot's at its centre.
class RevoluteJoint3D final : public Component {
public:
    static constexpr const char* kTypeName = "RevoluteJoint3D";

    [[nodiscard]] const char* type_name() const noexcept override { return kTypeName; }

    // The two bodies the joint connects, by node id. Either may be nil (the
    // default), in which case the joint is skipped. A missing or non-dynamic
    // body is likewise skipped cleanly.
    Uuid body_a{};
    Uuid body_b{};

    // The shared anchor point expressed in each body's local frame. A local
    // offset is rotated by the body's orientation before it is constrained, so a
    // pendulum pinned off-centre swings correctly.
    Vec3 anchor_a{};
    Vec3 anchor_b{};

    // A motor that spins the joint about an axis. A 3D revolute joint leaves
    // rotation about every axis free, so the motor names the one it drives with
    // `motor_axis` (in body A's local frame; a world direction when A does not
    // turn). `motor_speed` is the relative angular velocity it drives B towards,
    // in radians per second, about that axis. `motor_max_torque` is the largest
    // torque, in newton-metres, the motor may apply; a torque too small to
    // overcome the load slips rather than winning. The default zero disables the
    // motor, so a joint with no motor behaves exactly as a plain pin.
    Vec3 motor_axis{0.0f, 0.0f, 1.0f};
    f32 motor_speed = 0.0f;
    f32 motor_max_torque = 0.0f;

    // A position target turns the motor into a servo: when `motor_use_target` is
    // set the joint drives its swing about `motor_axis` towards
    // `motor_target_angle` (radians, B's rotation relative to A about the axis)
    // instead of a speed, with a proportional `motor_stiffness` and derivative
    // `motor_damping` gain, still bounded by `motor_max_torque`. A stiff, damped
    // target holds a pose against a load -- a robot arm, a door held ajar, a limb
    // posed -- where the speed motor only drives a rate. Off by default, so a
    // motor with no target is the speed motor it was. Ignored while
    // `motor_max_torque` is zero, which disables the motor.
    bool motor_use_target = false;
    f32 motor_target_angle = 0.0f;
    f32 motor_stiffness = 0.0f;
    f32 motor_damping = 0.0f;

    // Angular limits about the hinge axis. When `enable_limits` is set the joint
    // swings freely between `lower_angle` and `upper_angle` (radians, measured
    // about `limit_axis` as the rotation of B relative to A) and is stopped at
    // either bound, the way a door stops at 90 degrees or a jointed limb stops at
    // its extreme. `limit_axis` is authored in body A's local frame, like the
    // motor's axis. Off by default, so the joint swings about every axis freely.
    bool enable_limits = false;
    Vec3 limit_axis{0.0f, 0.0f, 1.0f};
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

} // namespace fe::physics3d
