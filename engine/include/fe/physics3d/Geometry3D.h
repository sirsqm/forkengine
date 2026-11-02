// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Math.h"
#include "fe/core/Types.h"

namespace fe::physics3d {

// A 3D collision shape: an axis-aligned box or a sphere.
//
// A collider is one of these, plus the material data that lives on the
// Collider3D component. As in 2D, the shape is a closed enum rather than a base
// class: there are exactly two and the solver wants to switch on them in a tight
// loop, not dispatch through a virtual.
enum class ShapeType3D : u8 {
    Box,
    Sphere,
};

// An axis-aligned box, stored as a half-extent around its centre.
//
// Half-extents (rather than min/max corners) are what the intersection maths
// wants, and the centre is the node's world position, so a box collider needs no
// separate offset to begin with. A box is axis-aligned: a rotated node is drawn
// rotated but collided as its unrotated box, the same limit the 2D collider has
// (ADR-0009), because there is no quaternion orientation in the solver yet.
struct Aabb3D {
    Vec3 center{};
    Vec3 half_extent{};

    [[nodiscard]] f32 min_x() const noexcept { return center.x - half_extent.x; }
    [[nodiscard]] f32 max_x() const noexcept { return center.x + half_extent.x; }
    [[nodiscard]] f32 min_y() const noexcept { return center.y - half_extent.y; }
    [[nodiscard]] f32 max_y() const noexcept { return center.y + half_extent.y; }
    [[nodiscard]] f32 min_z() const noexcept { return center.z - half_extent.z; }
    [[nodiscard]] f32 max_z() const noexcept { return center.z + half_extent.z; }
};

// A sphere, stored as a centre and a radius.
struct Sphere3D {
    Vec3 center{};
    f32 radius = 0.0f;
};

// Rotates `v` by the unit quaternion `q`. Used to place an oriented box's local
// axes in world space without materialising a matrix.
[[nodiscard]] inline Vec3 rotate_by_quat(const Vec3& v, const Quat& q) noexcept {
    const Quat p{v.x, v.y, v.z, 0.0f};
    const Quat rotated = q * p * q.conjugate();
    return Vec3{rotated.x, rotated.y, rotated.z};
}

// An oriented box: a centre, a half-extent, and an orientation.
//
// This is the general form of the box, the 3D counterpart of `Obb2D`. An
// axis-aligned box is the special case of an identity orientation, so `Aabb3D`
// is kept for the broad phase and the pure axis-aligned tests, and the oriented
// tests take an `Obb3D`. A box collider is placed as an `Obb3D` (see
// `Collider3D::oriented_box`), which is what lets a rotated node collide as its
// true shape rather than its unrotated bounding box.
struct Obb3D {
    Vec3 center{};
    Vec3 half_extent{0.5f, 0.5f, 0.5f};
    Quat orientation{};

    // The box's local axis `index` (0 = x, 1 = y, 2 = z) in world space, as a
    // unit vector. `half_extent[index]` is measured along it.
    [[nodiscard]] Vec3 axis(usize index) const noexcept {
        const Quat q = orientation.normalized();
        if (index == 0) {
            return rotate_by_quat(Vec3{1.0f, 0.0f, 0.0f}, q);
        }
        if (index == 1) {
            return rotate_by_quat(Vec3{0.0f, 1.0f, 0.0f}, q);
        }
        return rotate_by_quat(Vec3{0.0f, 0.0f, 1.0f}, q);
    }

    [[nodiscard]] f32 half(usize index) const noexcept {
        return index == 0 ? half_extent.x : (index == 1 ? half_extent.y : half_extent.z);
    }

    // The eight corners in world space, indexed by the sign of each local axis.
    [[nodiscard]] Vec3 corner(usize index) const noexcept {
        const Vec3 ax = axis(0) * half_extent.x;
        const Vec3 ay = axis(1) * half_extent.y;
        const Vec3 az = axis(2) * half_extent.z;
        const f32 sx = (index & 1u) != 0u ? 1.0f : -1.0f;
        const f32 sy = (index & 2u) != 0u ? 1.0f : -1.0f;
        const f32 sz = (index & 4u) != 0u ? 1.0f : -1.0f;
        return center + ax * sx + ay * sy + az * sz;
    }
};

// The result of a narrow-phase test between two shapes.
//
// `overlapping` is the primary answer; `normal` and `penetration` are only
// meaningful when it is true. `normal` points from shape A towards shape B (the
// direction that separates them), and `penetration` is the depth along it. That
// convention is what the impulse solver needs to push the bodies apart, and it is
// stated here so callers do not have to guess which way the normal faces.
struct Contact3D {
    bool overlapping = false;
    Vec3 normal{};
    f32 penetration = 0.0f;
};

// --- Narrow-phase tests ----------------------------------------------------
//
// These are pure geometry: no scene, no components, no allocation. They are the
// part of physics worth testing exhaustively, so they are kept free of anything
// that would make that harder.

[[nodiscard]] Contact3D intersect(const Aabb3D& a, const Aabb3D& b) noexcept;
[[nodiscard]] Contact3D intersect(const Sphere3D& a, const Sphere3D& b) noexcept;
// `a` is the sphere, `b` is the box. The returned normal still points from the
// sphere towards the box, matching the convention above.
[[nodiscard]] Contact3D intersect(const Sphere3D& a, const Aabb3D& b) noexcept;

// Oriented variants. Same contract and the same normal convention ("from A
// towards B"). An `Obb3D` with an identity orientation is exactly the matching
// `Aabb3D` test, so these generalise rather than replace the axis-aligned ones.
// This is the separating-axis test over the two boxes' fifteen candidate axes
// (three each, plus nine cross products), the standard general box-box test.
[[nodiscard]] Contact3D intersect(const Obb3D& a, const Obb3D& b) noexcept;
// `a` is the sphere, `b` is the box.
[[nodiscard]] Contact3D intersect(const Sphere3D& a, const Obb3D& b) noexcept;
[[nodiscard]] Contact3D intersect(const Obb3D& a, const Sphere3D& b) noexcept;

// The broad-phase prefilter: a cheap conservative overlap test used to skip the
// narrow phase for pairs that cannot touch. A false result is exact; a true
// result means "maybe".
[[nodiscard]] bool bounds_overlap(const Aabb3D& a, const Aabb3D& b) noexcept;
[[nodiscard]] bool bounds_overlap(const Aabb3D& box, const Sphere3D& sphere) noexcept;

// The axis-aligned bounding box that contains a shape. Used to place a sphere
// into the same broad-phase path as a box.
[[nodiscard]] Aabb3D bounds_of(const Sphere3D& sphere) noexcept;
[[nodiscard]] Aabb3D bounds_of(const Aabb3D& box) noexcept;

// Dispatches to the right overload for a pair of shape types.
[[nodiscard]] Contact3D intersect_shapes(ShapeType3D a_type, const Aabb3D& a_box, const Sphere3D& a_sphere,
                                         ShapeType3D b_type, const Aabb3D& b_box, const Sphere3D& b_sphere) noexcept;
// The oriented dispatch: the `Obb3D` form tests a rotated box as its true shape.
[[nodiscard]] Contact3D intersect_shapes(ShapeType3D a_type, const Obb3D& a_box, const Sphere3D& a_sphere,
                                         ShapeType3D b_type, const Obb3D& b_box, const Sphere3D& b_sphere) noexcept;

// --- Ray casts -------------------------------------------------------------
//
// A ray is an origin and a unit direction; the test reports the distance along
// the ray to the first intersection. The functions return a negative distance
// when there is no hit (the ray misses, or starts past the shape and points
// away), which is a total, allocation-free answer the query layer turns into an
// explicit miss.

// Distance along `direction` from `origin` to the nearest face of `box`, or -1
// if the ray misses. `direction` must be unit length. A ray that starts inside
// the box hits at distance 0.
[[nodiscard]] f32 raycast_aabb(const Vec3& origin, const Vec3& direction, const Aabb3D& box) noexcept;

// Distance along `direction` from `origin` to `sphere`, or -1 if the ray misses.
// A ray that starts inside the sphere hits at distance 0.
[[nodiscard]] f32 raycast_sphere(const Vec3& origin, const Vec3& direction, const Sphere3D& sphere) noexcept;

// Dispatches to the right ray test for a shape type.
[[nodiscard]] f32 raycast_shape(const Vec3& origin, const Vec3& direction, ShapeType3D type, const Aabb3D& box,
                                const Sphere3D& sphere) noexcept;

// The surface normal at the hit point, pointing back towards the ray origin.
// `direction` must be the unit direction used for the cast and `distance` a hit
// distance it returned. The AABB form picks the axis of the face that was
// crossed; the sphere form is the radial direction. A distance <= 0 yields a
// normal opposite the ray, which is well defined for a ray starting inside.
[[nodiscard]] Vec3 raycast_aabb_normal(const Vec3& origin, const Vec3& direction, const Aabb3D& box,
                                       f32 distance) noexcept;
[[nodiscard]] Vec3 raycast_sphere_normal(const Vec3& origin, const Vec3& direction, const Sphere3D& sphere,
                                         f32 distance) noexcept;
[[nodiscard]] Vec3 raycast_shape_normal(const Vec3& origin, const Vec3& direction, ShapeType3D type, const Aabb3D& box,
                                        const Sphere3D& sphere, f32 distance) noexcept;

// True when `point` is inside the shape.
[[nodiscard]] bool contains(const Aabb3D& box, const Vec3& point) noexcept;
[[nodiscard]] bool contains(const Sphere3D& sphere, const Vec3& point) noexcept;
[[nodiscard]] bool contains_shape(ShapeType3D type, const Aabb3D& box, const Sphere3D& sphere,
                                  const Vec3& point) noexcept;

// --- Swept (continuous) tests ----------------------------------------------

// The result of a swept test between a moving shape and a static one.
//
// `time` is the fraction of the displacement at which the two first touch, in
// [0, 1]; 0 means the moving shape already overlaps the target at the start of
// the sweep. `normal` points back towards the moving shape (the direction that
// separates them), matching `Contact3D`'s convention so a caller can stop the
// motion the same way a discrete contact would.
struct Sweep3D {
    bool hit = false;
    f32 time = 0.0f;
    Vec3 normal{};
};

// The first time a moving shape touches `target` along `displacement`.
//
// This is the continuous counterpart of `intersect`: the discrete narrow phase
// asks "do these overlap now", which a fast body can pass through between two
// steps (a bullet through a thin wall). The swept test asks "do these touch at
// any point along the motion", which cannot be tunnelled through.
//
// The shapes are taken as their axis-aligned bounds, so the sweep is exact for
// two axis-aligned boxes and conservative for a sphere or an oriented box (a box
// bound around a sphere): it may report a contact a hair early near a corner, but
// it never misses one, which is the property a continuous test must have. The
// Minkowski slab test generalises to three axes: shrinking the moving shape to
// its centre and growing the target by the moving half-extents turns the sweep
// into a ray cast against an expanded box. `displacement` is the full motion for
// the step, not a unit direction.
[[nodiscard]] Sweep3D sweep_shape(const Aabb3D& moving_bounds, const Vec3& displacement, const Aabb3D& target) noexcept;

} // namespace fe::physics3d
