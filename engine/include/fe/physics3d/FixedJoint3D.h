// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Math.h"
#include "fe/core/Types.h"
#include "fe/core/Uuid.h"
#include "fe/scene/Component.h"

namespace fe::physics3d {

// A fixed (weld) joint: it removes every degree of freedom between its two
// bodies. Their relative position *and* their relative orientation are held, so
// the pair behaves as a single rigid body -- a door frame bolted to a wall, a
// limb that must not bend, two crates carried as one.
//
// It is the 3D counterpart of FixedJoint2D, and it is built from parts the
// solver already has: the anchor pinning of the revolute joint (rows 0-2) plus a
// rotation lock about the three world axes (rows 3-5). A fixed joint is a
// revolute joint that also refuses to turn.
//
// The joint names its two bodies by node id, like the other 3D joints, so it
// serializes as a stable reference and survives a save/load cycle.
//
// The shared point is authored as a local anchor on each body, exactly as the
// revolute joint's anchors are. `reference_relative` is the relative orientation
// the joint preserves, as B's rotation relative to A; the default identity welds
// the bodies to a common world orientation, which is what an author usually
// wants. A joint dropped onto two bodies that are already posed sets
// `capture_reference` so the pose they are in is the pose it holds.
class FixedJoint3D final : public Component {
public:
    static constexpr const char* kTypeName = "FixedJoint3D";

    [[nodiscard]] const char* type_name() const noexcept override { return kTypeName; }

    // The two bodies the joint connects, by node id. Either may be nil (the
    // default), in which case the joint is skipped. A missing or immovable body
    // is likewise skipped cleanly.
    Uuid body_a{};
    Uuid body_b{};

    // The shared anchor point expressed in each body's local frame, rotated by
    // the body's orientation before it is constrained, exactly as the revolute
    // joint's anchors are.
    Vec3 anchor_a{};
    Vec3 anchor_b{};

    // The relative orientation to hold, as B's rotation relative to A. The
    // default identity welds the bodies to a common world orientation, which is
    // the usual intent when a joint is dropped onto two unrotated bodies. To
    // hold a specific relative pose (two planks glued at an angle) set this to
    // the relative rotation to preserve, `q_a^-1 * q_b`, which the helper
    // `fixed_reference_relative` computes from two body orientations.
    Quat reference_relative{};

    [[nodiscard]] bool is_valid() const noexcept { return !body_a.is_nil() && !body_b.is_nil() && body_a != body_b; }
};

} // namespace fe::physics3d
