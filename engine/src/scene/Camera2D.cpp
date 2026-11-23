// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/scene/Camera2D.h"

#include "fe/core/Assert.h"
#include "fe/scene/Node.h"

#include <algorithm>
#include <cmath>

namespace fe {

Mat4 Camera2D::view_projection(u32 viewport_width, u32 viewport_height) const noexcept {
    FE_CHECK(viewport_width > 0 && viewport_height > 0);
    FE_CHECK(zoom > 0.0f);

    // Centre the visible box on the owning node's world position. The camera
    // node's own scale does not change the visible extent; only `zoom` does,
    // which keeps camera behaviour predictable when a parent scales.
    const Vec3 centre = node() != nullptr ? transform_point(node()->world_matrix(), Vec3{}) : Vec3{};

    // With a reference box set, the box is scaled uniformly to fit the viewport
    // and the scale is folded into the zoom, so the projection below is
    // unchanged and one formula covers both modes. `zoom` multiplies the fit,
    // which keeps it meaningful as a zoom-in on top of the fit rather than a
    // second, conflicting scale.
    f32 effective_zoom = zoom;
    if (reference_width > 0.0f && reference_height > 0.0f) {
        const f32 fit = std::min(static_cast<f32>(viewport_width) / reference_width,
                                 static_cast<f32>(viewport_height) / reference_height);
        effective_zoom = zoom * fit;
    }

    const f32 half_width = static_cast<f32>(viewport_width) * 0.5f / effective_zoom;
    const f32 half_height = static_cast<f32>(viewport_height) * 0.5f / effective_zoom;

    // Snap the centre to the pixel grid when asked. One pixel is
    // 1 / effective_zoom world units, so rounding the centre to a multiple of
    // that makes the view translation a whole number of pixels and every texel
    // sample stable from frame to frame. The zoom and the projection are
    // untouched, so only the translation moves.
    Vec3 snapped = centre;
    if (snap_to_pixel) {
        const f32 pixel = 1.0f / effective_zoom;
        snapped.x = std::round(centre.x / pixel) * pixel;
        snapped.y = std::round(centre.y / pixel) * pixel;
    }

    const Mat4 view = Mat4::translation(Vec3{-snapped.x, -snapped.y, 0.0f});
    const Mat4 projection = Mat4::orthographic(-half_width, half_width, -half_height, half_height, -1.0f, 1.0f);
    return projection * view;
}

} // namespace fe
