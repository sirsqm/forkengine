// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// The directional shadow map's cascade partition and per-cascade light fit
// (ADR-0161). This lives outside Renderer3D because the arithmetic is where a
// cascade bug hides, and none of it needs a GPU to exercise.

#pragma once

#include "fe/core/Math.h"

#include <array>
#include <utility>

namespace fe::renderer3d {

// Two cascades: a near band fitted to the geometry close to the camera and a far
// band fitted to the rest. The count is fixed because the shader's atlas layout
// and its uniform array are both sized from it.
inline constexpr u32 kShadowCascades = 2;

// The partition of a view-depth band, from `near_depth` to `far_depth`, into
// `kShadowCascades` bands. Returns the band edges, so cascade i covers
// [edge[i], edge[i + 1]].
//
// `lambda` blends the two textbook schemes: 0 is a uniform partition, 1 is a
// logarithmic one (which follows how perspective spends texels on distance), and
// anything between is the practical split scheme (Zhang et al.). A degenerate
// band -- a single caster, or a caster behind the camera -- is widened to a
// usable range rather than producing a zero-width cascade.
[[nodiscard]] std::array<f32, kShadowCascades + 1> shadow_cascade_splits(f32 near_depth, f32 far_depth, f32 lambda);

// The corners of the camera frustum at two view depths, in world space. View
// depth is linear along a ray, so interpolating the near and far corners of the
// same NDC ray is exact rather than an approximation. Exposed because the fit is
// defined in terms of it, and a test can only check "the box contains its slice"
// by asking for the slice.
[[nodiscard]] std::array<Vec3, 8> shadow_frustum_slice_corners(const Mat4& inverse_view_projection, f32 near_depth,
                                                               f32 far_depth, f32 slice_near, f32 slice_far);

// The camera's near-plane depth. Nothing in front of it is drawn, so it is where
// the cascade band starts unless the nearest caster is even closer.
[[nodiscard]] f32 shadow_camera_near_depth(const Mat4& inverse_view_projection, const Vec3& camera_position);

// The view-depth range a caster's world AABB occupies: the nearest and farthest
// of its eight corners. The near corner is not `view_depth(min_corner)`: which
// corner is nearest depends on where the camera is, and reading the wrong one
// starts the cascade band behind the geometry it is meant to cover.
[[nodiscard]] std::pair<f32, f32> shadow_caster_depth_range(const Vec3& caster_min, const Vec3& caster_max,
                                                            const Vec3& camera_position, const Vec3& forward);

// The camera's forward axis, recovered from an inverse view-projection: the
// direction from the near plane's centre to the far plane's centre. This is the
// axis the split is measured along, so the shader and the fit agree on which
// band a fragment belongs to.
[[nodiscard]] Vec3 camera_forward(const Mat4& inverse_view_projection);

// The camera-space depth of a world point: the key the shader compares against
// the split to choose a cascade.
[[nodiscard]] f32 view_depth(const Vec3& world_position, const Vec3& camera_position, const Vec3& forward);

// The light-space matrix for one cascade. `slice_near`/`slice_far` bound the band
// of the camera frustum the cascade covers, in view depth. The projection's x/y
// is fitted to that band's corners -- which is what gives the near cascade its
// higher texel density -- and its depth range is extended to cover every caster,
// so a caster outside the band still shadows into it.
[[nodiscard]] Mat4 shadow_cascade_light_matrix(const Mat4& inverse_view_projection, const Vec3& camera_position,
                                               f32 slice_near, f32 slice_far, const Vec3& caster_min,
                                               const Vec3& caster_max, const Vec3& light_direction);

} // namespace fe::renderer3d
