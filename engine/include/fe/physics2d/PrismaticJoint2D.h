// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Math.h"
#include "fe/core/Types.h"
#include "fe/core/Uuid.h"
#include "fe/scene/Component.h"

namespace fe::physics2d {

// A prismatic (slider) joint: it allows exactly one degree of freedom. The two
// bodies may slide along a shared axis but may not separate perpendicular to it,
// and they may not rotate relative to one another. It is the constraint behind a
// piston, a drawer, an elevator on its rail, or a platform that only moves
// vertically.
//
// It is the 2D counterpart of PrismaticJoint3D. The solver builds two rows: one
// removes the translation perpendicular to the axis and one locks the relative
// angle, so sliding along the axis is the one remaining degree of freedom.
//
// The joint names its two bodies by node id, like the other 2D joints, so it
// serializes as a stable reference and survives a save/load cycle.
//
// The slide axis is authored in body A's local frame. `reference_angle` is the
// relative angle the joint holds, as B's rotation minus A's; it keeps the axis a
// frame the pair shares rather than a world direction that drifts as the bodies
// turn. The default zero holds the bodies at a common angle.
class PrismaticJoint2D final : public Component {
public:
    static constexpr const char* kTypeName = "PrismaticJoint2D";

    [[nodiscard]] const char* type_name() const noexcept override { return kTypeName; }

    // The two bodies the joint connects, by node id. Either may be nil (the
    // default), in which case the joint is skipped. A missing or immovable body
    // is likewise skipped cleanly.
    Uuid body_a{};
    Uuid body_b{};

    // The shared anchor point expressed in each body's local frame, rotated by
    // the body's rotation before it is constrained. The anchors are pinned
    // together except along the slide axis.
    Vec2 anchor_a{};
    Vec2 anchor_b{};

    // The slide axis in body A's local frame. The default +X is a sensible
    // starting axis; set it to the track's direction.
    Vec2 axis{1.0f, 0.0f};

    // The relative angle to hold, as B's rotation minus A's. The default zero
    // holds the bodies at a common angle, so the axis is a frame the pair shares.
    f32 reference_angle = 0.0f;

    // Translation limits along the slide axis. When `enable_limits` is set the
    // pair may slide freely between `lower_translation` and `upper_translation`
    // (B's anchor relative to A's along the axis, so positive means B is ahead)
    // and is stopped at either bound, like a drawer's stops or a piston's end
    // caps. Off by default, so the joint slides freely along its whole axis.
    bool enable_limits = false;
    f32 lower_translation = 0.0f;
    f32 upper_translation = 0.0f;

    // A linear motor that drives the slide. `motor_speed` is the relative speed
    // along the axis it drives B towards, in world units per second.
    // `motor_max_force` is the largest force, in newtons, the motor may apply; a
    // force too small to overcome the load slips rather than winning. The default
    // zero disables the motor, so a joint with no motor behaves exactly as a
    // plain slider. A piston, an elevator, or a powered drawer is this motor on a
    // prismatic joint.
    f32 motor_speed = 0.0f;
    f32 motor_max_force = 0.0f;

    // A position target turns the linear motor into a servo: when
    // `motor_use_target` is set the joint drives its slide towards
    // `motor_target_translation` (the same signed coordinate the limits use:
    // B's anchor relative to A's along the axis) instead of a speed, with a
    // proportional `motor_stiffness` and derivative `motor_damping` gain, still
    // bounded by `motor_max_force`. A stiff, damped target holds a position
    // against a load -- a lift that holds a floor, a powered drawer that stays
    // shut -- where the speed motor only drives a rate. Off by default, so a
    // motor with no target is the speed motor it was. Ignored while
    // `motor_max_force` is zero, which disables the motor.
    bool motor_use_target = false;
    f32 motor_target_translation = 0.0f;
    f32 motor_stiffness = 0.0f;
    f32 motor_damping = 0.0f;

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
