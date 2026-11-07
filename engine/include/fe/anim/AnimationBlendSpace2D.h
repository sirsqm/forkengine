// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/anim/AdditiveLayer.h"
#include "fe/anim/AnimationClip.h"
#include "fe/anim/JointMask.h"
#include "fe/anim/Skeleton.h"
#include "fe/core/Math.h"
#include "fe/core/Types.h"
#include "fe/scene/Component.h"

#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace fe {
class ResourceManager;
}

namespace fe::anim {

// A two-axis blend space over the skeletal `Animator`.
//
// `AnimationBlendTree` is a one-axis cut: two clips and a single parameter, for
// locomotion that varies with speed alone. A character that also strafes varies
// on two axes at once -- forward/back and left/right -- and the eight
// directional clips (idle, walk forward, run forward, strafe left, strafe right,
// the two diagonals) do not lie on a line. This component is that grid: each
// sample is a clip placed at a point in a two-dimensional parameter space, and
// the pose for a parameter is the blend of the samples that surround it.
//
// The blend is a triangulation. The samples are the vertices; the parameter is
// located in the triangle that contains it, and its barycentric coordinates in
// that triangle are the sample weights. Barycentric weights are the natural
// generalisation of the one-axis `lerp`: at a vertex one weight is 1 and the
// pose is that clip, on an edge two weights share it, and inside a cell three or
// more clips mix. That is what makes a four-corner cell blend all four clips at
// its centre rather than only the two ends of a diagonal.
//
// The triangulation is chosen so no sample lies inside a triangle, and a point
// on a shared edge is treated as belonging to it (a point on the diagonal of a
// cell therefore blends the two diagonal clips, the same answer either triangle
// gives). A parameter outside every triangle -- the grid does not cover the
// whole plane -- falls back to the nearest sample at full weight, so the blend
// clamps at the edge of the authored space rather than extrapolating past the
// clips.
//
// The additive layer is the same one the blend tree carries, applied after the
// base blend: a hit reaction or a lean composes over the locomotion grid rather
// than replacing it. See ADR-0124 for the one-axis tree this generalises.
//
// Serialization: the sample clips travel as their resource keys and the whole
// space serializes as one compact JSON string on the `space` property, the same
// shape the blend tree and the state machines use (reflection has no
// list-of-struct kind, and a resource reference is a property on the component,
// not on a sub-value).

// One clip of the blend space, placed at a point in the parameter plane.
struct BlendPoint {
    std::string name; // for the editor and for a test; not required to be unique
    std::shared_ptr<AnimationClip> clip;
    std::string clip_key; // the reference key the clip is loaded from and saved as
    Vec2 position{};      // the parameter value at which this clip is full weight
    f32 speed = 1.0f;     // playback-rate multiplier, so a short clip can match a long one
};

// A weight over one sample of the blend space, returned by `weights`.
struct BlendWeight {
    usize sample = 0;
    f32 weight = 0.0f;
};

// A two-axis blend space with an optional additive layer.
class AnimationBlendSpace2D final : public Component {
public:
    static constexpr const char* kTypeName = "AnimationBlendSpace2D";

    [[nodiscard]] const char* type_name() const noexcept override { return kTypeName; }

    // The grid samples. Any count is legal: none is a no-op, one is that clip,
    // two blend along the segment between them, and three or more blend over the
    // triangulation. Placing them on a regular grid (the four corners, then the
    // edge midpoints) is the common case but not required.
    std::vector<BlendPoint> samples;

    // The two-dimensional parameter gameplay drives (a movement vector, a
    // movement vector plus an aim offset). The weights are derived from it.
    Vec2 parameter{};

    // When true the base samples are blended and written onto the animator. When
    // false the space still evaluates (so an editor or a test can read the
    // weights) but writes nothing, which is what a scene that only wants the
    // additive layer sets.
    bool drive_animator = true;

    // --- The additive layer (same shape as the blend tree's) ----------------

    std::shared_ptr<AnimationClip> additive_clip;
    std::string additive_clip_key;
    f32 additive_weight = 0.0f;
    f32 additive_time = 0.0f;
    f32 additive_speed = 1.0f;

    // The joints the additive layer may write; see `AnimationBlendTree`. An
    // empty mask applies the delta to every joint.
    JointMask additive_mask;

    // A stack of further additive layers applied in order; see
    // `AnimationBlendTree::additive_layers`.
    std::vector<AdditiveLayer> additive_layers;

    // --- Derived state (not serialized) ------------------------------------

    // The animator's clock the previous frame sampled the blend at, kept so the
    // system can report the contributing clips' timeline events that playback
    // crossed between the two clocks. Zero before the first update.
    f32 previous_base_time = 0.0f;

    // --- Queried state -----------------------------------------------------

    // Writes the samples and weights that blend to `parameter` into `out`, up to
    // `capacity` entries, and returns how many were written. At most three (a
    // triangle). Zero samples writes nothing and returns 0. The weights sum to
    // 1. This is the whole geometry of the space, and it is a pure function of
    // the samples and the parameter so a test can assert on it with no scene.
    [[nodiscard]] usize weights(BlendWeight* out, usize capacity) const noexcept;

    // Sets `parameter`; the weights are derived from it on the next evaluate.
    void set_parameter(const Vec2& value) noexcept { parameter = value; }

    // --- Resource resolution ----------------------------------------------

    // Loads the sample clips and the additive clip through `manager` from their
    // keys, returning how many were resolved this call. Idempotent, like the
    // blend tree's: a clip already held is skipped and a key that does not load
    // is retried, because an asset may be mounted after the scene.
    usize resolve_clips(ResourceManager& manager);

    // --- Persistence -------------------------------------------------------

    [[nodiscard]] std::string to_json() const;
    bool from_json(std::string_view text);
};

} // namespace fe::anim
