// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// The cascade partition and the per-cascade light fit (ADR-0161). These are pure
// functions, so they are checked here rather than through a framebuffer: a
// cascade bug is arithmetic (a band that starts behind its own geometry, a box
// that does not contain its slice), and arithmetic is cheaper to pin down than a
// shadow that looks slightly wrong.

#include <doctest/doctest.h>

#include "fe/renderer3d/ShadowCascades.h"

#include <array>
#include <cmath>

using namespace fe;
using namespace fe::renderer3d;

namespace {

// A camera at (0, 6, 12) looking at the origin through a 60-degree vertical
// field of view. Every test uses the same one so a failure is about the cascade
// arithmetic and not about a scene.
constexpr Vec3 kEye{0.0f, 6.0f, 12.0f};
constexpr f32 kFovY = 1.0471975512f;

[[nodiscard]] Mat4 test_view_projection() {
    return perspective(kFovY, 1.0f, 0.1f, 100.0f) * look_at(kEye, Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 1.0f, 0.0f});
}

// Projects a world point through a light-space matrix and returns the NDC point.
[[nodiscard]] Vec3 to_ndc(const Mat4& matrix, const Vec3& world) {
    const f32 x = matrix.at(0, 0) * world.x + matrix.at(0, 1) * world.y + matrix.at(0, 2) * world.z + matrix.at(0, 3);
    const f32 y = matrix.at(1, 0) * world.x + matrix.at(1, 1) * world.y + matrix.at(1, 2) * world.z + matrix.at(1, 3);
    const f32 z = matrix.at(2, 0) * world.x + matrix.at(2, 1) * world.y + matrix.at(2, 2) * world.z + matrix.at(2, 3);
    const f32 w = matrix.at(3, 0) * world.x + matrix.at(3, 1) * world.y + matrix.at(3, 2) * world.z + matrix.at(3, 3);
    const f32 inverse_w = std::abs(w) > 1e-8f ? 1.0f / w : 1.0f;
    return {x * inverse_w, y * inverse_w, z * inverse_w};
}

} // namespace

TEST_CASE("the cascade splits span the band and increase") {
    const auto splits = shadow_cascade_splits(2.0f, 18.0f, 0.6f);
    CHECK(splits.front() == doctest::Approx(2.0f));
    CHECK(splits.back() == doctest::Approx(18.0f));
    CHECK(splits[1] > splits[0]);
    CHECK(splits[1] < splits[2]);
}

TEST_CASE("the split blends the uniform and logarithmic partitions") {
    // 0 is the midpoint, 1 is the geometric mean, and a blend is the weighted
    // sum of the two: 0.6 * sqrt(1 * 25) + 0.4 * 13 = 8.2.
    const auto uniform = shadow_cascade_splits(1.0f, 25.0f, 0.0f);
    CHECK(uniform[1] == doctest::Approx(13.0f));
    const auto logarithmic = shadow_cascade_splits(1.0f, 25.0f, 1.0f);
    CHECK(logarithmic[1] == doctest::Approx(5.0f));
    const auto blended = shadow_cascade_splits(1.0f, 25.0f, 0.6f);
    CHECK(blended[1] == doctest::Approx(8.2f));
}

TEST_CASE("a degenerate band is widened instead of collapsing") {
    // A single caster, or one behind the camera, would otherwise give the near
    // cascade a zero-width box and a division by zero in the fit.
    const auto splits = shadow_cascade_splits(0.0f, 0.0f, 0.6f);
    CHECK(splits[0] > 0.0f);
    CHECK(splits[1] > splits[0]);
    CHECK(splits[2] > splits[1]);

    const auto inverted = shadow_cascade_splits(10.0f, 2.0f, 0.6f);
    CHECK(inverted[1] > inverted[0]);
    CHECK(inverted[2] > inverted[1]);
}

TEST_CASE("view depth is measured along the camera's forward axis") {
    const Mat4 inverse_view_projection = inverse(test_view_projection());
    const Vec3 forward = camera_forward(inverse_view_projection);

    // The look target sits straight ahead, so its depth is its distance.
    const f32 origin_depth = view_depth(Vec3{0.0f, 0.0f, 0.0f}, kEye, forward);
    CHECK(origin_depth == doctest::Approx(13.416f).epsilon(0.001));

    // A point moved sideways keeps its depth: the split is a distance along the
    // axis, not a radius from the eye. This is what lets a wide view use one
    // cascade for a whole band across the screen.
    CHECK(view_depth(Vec3{4.0f, 0.0f, 0.0f}, kEye, forward) == doctest::Approx(origin_depth).epsilon(0.001));

    // A step along the axis changes the depth by exactly that step.
    const Vec3 ahead = Vec3{0.0f, 0.0f, 0.0f} + forward * 3.0f;
    CHECK(view_depth(ahead, kEye, forward) == doctest::Approx(origin_depth + 3.0f).epsilon(0.001));
}

TEST_CASE("the caster depth range uses the nearest corner, not the minimum one") {
    // The AABB's min corner is the *farthest* point here: the camera looks down
    // at the box from +Z, so (min.x, min.y, min.z) is behind it. Reading the
    // depth of that corner would start the near cascade band past the box.
    const Vec3 caster_min{-2.0f, -1.0f, -2.0f};
    const Vec3 caster_max{2.0f, 1.0f, 2.0f};
    const Mat4 inverse_view_projection = inverse(test_view_projection());
    const Vec3 forward = camera_forward(inverse_view_projection);

    const auto [nearest, farthest] = shadow_caster_depth_range(caster_min, caster_max, kEye, forward);
    CHECK(nearest == doctest::Approx(view_depth(Vec3{0.0f, 1.0f, 2.0f}, kEye, forward)).epsilon(0.001));
    CHECK(farthest == doctest::Approx(view_depth(Vec3{0.0f, -1.0f, -2.0f}, kEye, forward)).epsilon(0.001));
    CHECK(nearest < farthest);
    CHECK(nearest < view_depth(caster_min, kEye, forward));
}

TEST_CASE("each cascade's light box contains its slice of the frustum") {
    const Mat4 inverse_view_projection = inverse(test_view_projection());
    const Vec3 caster_min{-4.0f, -1.0f, -4.0f};
    const Vec3 caster_max{4.0f, 1.0f, 4.0f};
    const auto splits = shadow_cascade_splits(8.0f, 22.0f, 0.6f);

    for (u32 cascade = 0; cascade < kShadowCascades; ++cascade) {
        const Mat4 light_space =
            shadow_cascade_light_matrix(inverse_view_projection, kEye, splits[cascade], splits[cascade + 1], caster_min,
                                        caster_max, Vec3{0.0f, -1.0f, 0.0f});
        const auto corners =
            shadow_frustum_slice_corners(inverse_view_projection, 0.1f, 100.0f, splits[cascade], splits[cascade + 1]);
        for (const Vec3& corner : corners) {
            const Vec3 ndc = to_ndc(light_space, corner);
            // A corner on the boundary is allowed to land exactly on the edge.
            CHECK(std::abs(ndc.x) <= 1.001f);
            CHECK(std::abs(ndc.y) <= 1.001f);
            CHECK(ndc.z >= -1.001f);
            CHECK(ndc.z <= 1.001f);
        }
        // The band's own slice is *in* the box, which also means the box is not
        // a degenerate one that would make every fragment sample texel zero.
        const Vec3 centre_ndc = to_ndc(light_space, (caster_min + caster_max) * 0.5f);
        CHECK(std::abs(centre_ndc.x) <= 1.001f);
        CHECK(std::abs(centre_ndc.y) <= 1.001f);
    }
}

TEST_CASE("the near cascade resolves a smaller world step than the far one") {
    // Texel density is what a cascade buys: the near box is fitted to a slice
    // nearer the camera, so the same world step covers more of the map. Measured
    // in NDC, a 0.1-unit step must separate further through the near matrix.
    const Mat4 inverse_view_projection = inverse(test_view_projection());
    const Vec3 caster_min{-6.0f, -1.0f, -6.0f};
    const Vec3 caster_max{6.0f, 1.0f, 6.0f};
    const auto splits = shadow_cascade_splits(8.0f, 30.0f, 0.6f);

    const Mat4 near_matrix = shadow_cascade_light_matrix(inverse_view_projection, kEye, splits[0], splits[1],
                                                         caster_min, caster_max, Vec3{0.0f, -1.0f, 0.0f});
    const Mat4 far_matrix = shadow_cascade_light_matrix(inverse_view_projection, kEye, splits[1], splits[2], caster_min,
                                                        caster_max, Vec3{0.0f, -1.0f, 0.0f});

    // A point at the split, on the view axis, and a small step across it.
    const Vec3 forward = camera_forward(inverse_view_projection);
    const Vec3 point = kEye + forward * splits[1];
    const Vec3 stepped = point + Vec3{0.1f, 0.0f, 0.0f};

    const Vec3 near_a = to_ndc(near_matrix, point);
    const Vec3 near_b = to_ndc(near_matrix, stepped);
    const Vec3 far_a = to_ndc(far_matrix, point);
    const Vec3 far_b = to_ndc(far_matrix, stepped);

    const f32 near_step = std::abs(near_b.x - near_a.x);
    const f32 far_step = std::abs(far_b.x - far_a.x);
    CHECK(near_step > far_step);
    // Not a marginal difference: the near band covers a fraction of the far
    // band's width, so the step resolves at least twice as wide.
    CHECK(near_step > far_step * 1.5f);
}

TEST_CASE("the fit keeps a caster outside the slice inside the map's depth range") {
    // The classic cascade artefact: a caster just outside the band whose shadow
    // falls inside it. The box's x/y comes from the slice, but its depth range
    // must still cover the caster, or the caster is clipped out of the map and
    // its shadow disappears at the band edge.
    const Mat4 inverse_view_projection = inverse(test_view_projection());
    const Vec3 caster_min{-6.0f, -1.0f, -6.0f};
    const Vec3 caster_max{6.0f, 1.0f, 6.0f};
    const auto splits = shadow_cascade_splits(8.0f, 30.0f, 0.6f);

    const Mat4 near_matrix = shadow_cascade_light_matrix(inverse_view_projection, kEye, splits[0], splits[1],
                                                         caster_min, caster_max, Vec3{0.0f, -1.0f, 0.0f});

    // A caster sitting well behind the near band still projects inside the
    // depth range, so the depth pass draws it.
    const Vec3 behind{0.0f, 1.0f, -6.0f};
    const Vec3 ndc = to_ndc(near_matrix, behind);
    CHECK(ndc.z >= -1.001f);
    CHECK(ndc.z <= 1.001f);
}

TEST_CASE("a light pointing straight down still produces a usable light box") {
    // The up-vector fallback: a light parallel to +Y would make look_at
    // degenerate, and a NaN matrix samples nothing at all.
    const Mat4 inverse_view_projection = inverse(test_view_projection());
    const Mat4 matrix =
        shadow_cascade_light_matrix(inverse_view_projection, kEye, 2.0f, 10.0f, Vec3{-2.0f, -1.0f, -2.0f},
                                    Vec3{2.0f, 1.0f, 2.0f}, Vec3{0.0f, -1.0f, 0.0f});
    for (f32 value : matrix.m) {
        CHECK(std::isfinite(value));
    }
    const Vec3 ndc = to_ndc(matrix, Vec3{0.0f, 0.0f, 0.0f});
    CHECK(std::isfinite(ndc.x));
    CHECK(std::isfinite(ndc.y));
    CHECK(std::isfinite(ndc.z));
}
