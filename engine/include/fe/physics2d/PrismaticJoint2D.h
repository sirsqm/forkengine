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

    [[nodiscard]] bool is_valid() const noexcept { return !body_a.is_nil() && !body_b.is_nil() && body_a != body_b; }
};

} // namespace fe::physics2d
