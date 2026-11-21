// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/renderer3d/ShadowCascades.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fe::renderer3d {
namespace {

// Transforms a point by a matrix and divides by w: the inverse view-projection
// needs the perspective divide, which `transform_point` does not do.
[[nodiscard]] Vec3 unproject(const Mat4& matrix, const Vec3& ndc) {
    const f32 x = matrix.at(0, 0) * ndc.x + matrix.at(0, 1) * ndc.y + matrix.at(0, 2) * ndc.z + matrix.at(0, 3);
    const f32 y = matrix.at(1, 0) * ndc.x + matrix.at(1, 1) * ndc.y + matrix.at(1, 2) * ndc.z + matrix.at(1, 3);
    const f32 z = matrix.at(2, 0) * ndc.x + matrix.at(2, 1) * ndc.y + matrix.at(2, 2) * ndc.z + matrix.at(2, 3);
    const f32 w = matrix.at(3, 0) * ndc.x + matrix.at(3, 1) * ndc.y + matrix.at(3, 2) * ndc.z + matrix.at(3, 3);
    const f32 inverse_w = std::abs(w) > 1e-8f ? 1.0f / w : 1.0f;
    return {x * inverse_w, y * inverse_w, z * inverse_w};
}

} // namespace

std::array<Vec3, 8> shadow_frustum_slice_corners(const Mat4& inverse_view_projection, f32 near_depth, f32 far_depth,
                                                 f32 slice_near, f32 slice_far) {
    constexpr f32 kNdcXY[4][2] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}};
    const f32 span = std::max(far_depth - near_depth, 1e-4f);
    const f32 t_near = (slice_near - near_depth) / span;
    const f32 t_far = (slice_far - near_depth) / span;

    std::array<Vec3, 8> corners{};
    for (u32 i = 0; i < 4; ++i) {
        const Vec3 near_corner = unproject(inverse_view_projection, Vec3{kNdcXY[i][0], kNdcXY[i][1], -1.0f});
        const Vec3 far_corner = unproject(inverse_view_projection, Vec3{kNdcXY[i][0], kNdcXY[i][1], 1.0f});
        const Vec3 ray = far_corner - near_corner;
        corners[i] = near_corner + ray * t_near;
        corners[i + 4] = near_corner + ray * t_far;
    }
    return corners;
}

f32 shadow_camera_near_depth(const Mat4& inverse_view_projection, const Vec3& camera_position) {
    const Vec3 near_centre = unproject(inverse_view_projection, Vec3{0.0f, 0.0f, -1.0f});
    return view_depth(near_centre, camera_position, camera_forward(inverse_view_projection));
}

std::pair<f32, f32> shadow_caster_depth_range(const Vec3& caster_min, const Vec3& caster_max,
                                              const Vec3& camera_position, const Vec3& forward) {
    f32 nearest = std::numeric_limits<f32>::max();
    f32 farthest = std::numeric_limits<f32>::lowest();
    for (u32 i = 0; i < 8; ++i) {
        const Vec3 corner{(i & 1U) != 0U ? caster_max.x : caster_min.x, (i & 2U) != 0U ? caster_max.y : caster_min.y,
                          (i & 4U) != 0U ? caster_max.z : caster_min.z};
        const f32 depth = view_depth(corner, camera_position, forward);
        nearest = std::min(nearest, depth);
        farthest = std::max(farthest, depth);
    }
    return {nearest, farthest};
}

std::array<f32, kShadowCascades + 1> shadow_cascade_splits(f32 near_depth, f32 far_depth, f32 lambda) {
    const f32 near_clamped = std::max(near_depth, 1e-3f);
    const f32 far_clamped = std::max(far_depth, near_clamped * 1.001f);
    const f32 blend = std::clamp(lambda, 0.0f, 1.0f);

    std::array<f32, kShadowCascades + 1> splits{};
    splits.front() = near_clamped;
    splits.back() = far_clamped;
    for (u32 i = 1; i < kShadowCascades; ++i) {
        const f32 ratio = static_cast<f32>(i) / static_cast<f32>(kShadowCascades);
        const f32 logarithmic = near_clamped * std::pow(far_clamped / near_clamped, ratio);
        const f32 uniform = near_clamped + (far_clamped - near_clamped) * ratio;
        splits[i] = blend * logarithmic + (1.0f - blend) * uniform;
    }
    return splits;
}

Vec3 camera_forward(const Mat4& inverse_view_projection) {
    const Vec3 near_centre = unproject(inverse_view_projection, Vec3{0.0f, 0.0f, -1.0f});
    const Vec3 far_centre = unproject(inverse_view_projection, Vec3{0.0f, 0.0f, 1.0f});
    const Vec3 forward = far_centre - near_centre;
    return forward.length_squared() > 1e-12f ? forward.normalized() : Vec3{0.0f, 0.0f, -1.0f};
}

f32 view_depth(const Vec3& world_position, const Vec3& camera_position, const Vec3& forward) {
    return (world_position - camera_position).dot(forward);
}

Mat4 shadow_cascade_light_matrix(const Mat4& inverse_view_projection, const Vec3& camera_position, f32 slice_near,
                                 f32 slice_far, const Vec3& caster_min, const Vec3& caster_max,
                                 const Vec3& light_direction) {
    const Vec3 forward = camera_forward(inverse_view_projection);
    const f32 near_depth =
        view_depth(unproject(inverse_view_projection, Vec3{0.0f, 0.0f, -1.0f}), camera_position, forward);
    const f32 far_depth =
        view_depth(unproject(inverse_view_projection, Vec3{0.0f, 0.0f, 1.0f}), camera_position, forward);
    const auto slice =
        shadow_frustum_slice_corners(inverse_view_projection, near_depth, far_depth, slice_near, slice_far);

    // The light looks along the direction it travels. Its eye sits behind the
    // slice by enough to clear the caster box, so every depth below is positive.
    const Vec3 travel =
        light_direction.length_squared() > 1e-12f ? light_direction.normalized() : Vec3{0.0f, -1.0f, 0.0f};
    const Vec3 up_hint = std::abs(travel.y) > 0.99f ? Vec3{0.0f, 0.0f, 1.0f} : Vec3{0.0f, 1.0f, 0.0f};
    const f32 caster_radius = std::max((caster_max - caster_min).length() * 0.5f, 1e-3f);

    Vec3 slice_centre{};
    for (const Vec3& corner : slice) {
        slice_centre = slice_centre + corner;
    }
    slice_centre = slice_centre * (1.0f / 8.0f);

    // The eye sits up-light of the slice far enough that every caster corner is in
    // front of it: a caster far above the visible band would otherwise end up
    // behind the eye and be clipped away by the near plane.
    const Vec3 caster_centre = (caster_min + caster_max) * 0.5f;
    const f32 eye_distance = (slice_centre - caster_centre).length() + caster_radius * 2.0f + 1.0f;
    const Vec3 light_eye = slice_centre - travel * eye_distance;
    const Mat4 view = look_at(light_eye, light_eye + travel, up_hint);

    // x/y from the slice: a near band covers a small part of the world, so its
    // texels cover less of it. A slice seen edge-on is a line, so a degenerate
    // extent is widened to a texel-sized box instead of collapsing to zero.
    f32 min_x = std::numeric_limits<f32>::max();
    f32 max_x = std::numeric_limits<f32>::lowest();
    f32 min_y = min_x;
    f32 max_y = max_x;
    for (const Vec3& corner : slice) {
        const Vec3 light_space = transform_point(view, corner);
        min_x = std::min(min_x, light_space.x);
        max_x = std::max(max_x, light_space.x);
        min_y = std::min(min_y, light_space.y);
        max_y = std::max(max_y, light_space.y);
    }
    constexpr f32 kMinExtent = 1e-3f;
    if (max_x - min_x < kMinExtent) {
        const f32 centre = (min_x + max_x) * 0.5f;
        min_x = centre - kMinExtent * 0.5f;
        max_x = centre + kMinExtent * 0.5f;
    }
    if (max_y - min_y < kMinExtent) {
        const f32 centre = (min_y + max_y) * 0.5f;
        min_y = centre - kMinExtent * 0.5f;
        max_y = centre + kMinExtent * 0.5f;
    }

    // Depth from every caster corner *and* the slice's own corners. The casters
    // matter because a tall one outside the band still shadows into it; the slice
    // matters because a band can start in front of the nearest caster, and its
    // geometry would otherwise fall outside the projection's depth range.
    f32 min_z = std::numeric_limits<f32>::max();
    f32 max_z = std::numeric_limits<f32>::lowest();
    const auto include = [&](const Vec3& point) {
        const f32 z = transform_point(view, point).z;
        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);
    };
    for (const Vec3& corner : slice) {
        include(corner);
    }
    for (u32 i = 0; i < 8; ++i) {
        include(Vec3{(i & 1U) != 0U ? caster_max.x : caster_min.x, (i & 2U) != 0U ? caster_max.y : caster_min.y,
                     (i & 4U) != 0U ? caster_max.z : caster_min.z});
    }

    // The view looks down -Z, so a point at distance d has z = -d and the ortho
    // box keeps -far <= z <= -near.
    constexpr f32 kDepthMargin = 0.01f;
    const f32 near_plane = std::max(-max_z - kDepthMargin, kDepthMargin);
    const f32 far_plane = std::max(-min_z + kDepthMargin, near_plane + kDepthMargin);
    const Mat4 projection = Mat4::orthographic(min_x, max_x, min_y, max_y, near_plane, far_plane);
    return projection * view;
}

} // namespace fe::renderer3d
