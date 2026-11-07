// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Math.h"
#include "fe/core/Types.h"
#include "fe/core/Uuid.h"
#include "fe/scene/Component.h"

namespace fe::physics2d {

// A fixed (weld) joint: it removes every degree of freedom between its two
// bodies. Their relative position *and* their relative angle are held, so the
// pair behaves as a single rigid body -- a crate carried as one piece, a limb
// that must not bend, a bridge segment bolted to its neighbour.
//
// It is built from parts the solver already has: the anchor pinning of the
// revolute joint (rows 0-1) plus an angle lock (row 2). A fixed joint is a
// revolute joint that also refuses to turn.
//
// The joint names its two bodies by node id, like the other 2D joints, so it
// serializes as a stable reference and survives a save/load cycle.
//
// The shared point is authored as a local anchor on each body, exactly as the
// revolute joint's anchors are. `reference_angle` is the relative angle the
// joint holds, as B's rotation minus A's; the default zero welds the bodies at a
// common world angle. A joint dropped onto two bodies that are already rotated
// sets `capture_reference` so the angle they are in is the angle it holds.
class FixedJoint2D final : public Component {
public:
    static constexpr const char* kTypeName = "FixedJoint2D";

    [[nodiscard]] const char* type_name() const noexcept override { return kTypeName; }

    // The two bodies the joint connects, by node id. Either may be nil (the
    // default), in which case the joint is skipped. A missing or immovable body
    // is likewise skipped cleanly.
    Uuid body_a{};
    Uuid body_b{};

    // The shared anchor point expressed in each body's local frame, rotated by
    // the body's rotation before it is constrained, exactly as the revolute
    // joint's anchors are.
    Vec2 anchor_a{};
    Vec2 anchor_b{};

    // The relative angle to hold, as B's rotation minus A's. The default zero
    // welds the bodies at a common world angle, which is the usual intent when a
    // joint is dropped onto two unrotated bodies. To hold a specific relative
    // pose (two planks glued at an angle) set this to the relative angle to
    // preserve, which the helper `fixed_reference_angle` computes from two body
    // rotations.
    f32 reference_angle = 0.0f;

    [[nodiscard]] bool is_valid() const noexcept { return !body_a.is_nil() && !body_b.is_nil() && body_a != body_b; }
};

} // namespace fe::physics2d
