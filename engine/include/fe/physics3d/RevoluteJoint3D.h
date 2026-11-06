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

    [[nodiscard]] bool is_valid() const noexcept { return !body_a.is_nil() && !body_b.is_nil() && body_a != body_b; }
};

} // namespace fe::physics3d
