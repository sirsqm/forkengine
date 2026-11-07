// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/physics3d/Geometry3D.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace fe::physics3d {
namespace {

// Below this a direction component is treated as zero, so a ray parallel to a
// slab does not divide by it. The value is well under any meaningful world unit
// and well above denormal noise.
constexpr f32 kDirectionEpsilon = 1e-8f;

// Defined below `intersect_oriented_boxes`; declared here so that function can
// call it. Builds the box-box contact point from the overlap of the two faces.
[[nodiscard]] Vec3 box_box_contact_point(const Obb3D& a, const Obb3D& b, const Vec3& normal, f32 support_a,
                                         f32 support_b) noexcept;

// Boxes are axis-aligned, so overlap on each axis is an interval test and the
// separating axis is the one with the smallest overlap. That axis is the contact
// normal, and the smallest overlap is the penetration depth. Ties prefer the
// lowest axis (x before y before z) so the result is deterministic.
Contact3D intersect_boxes(const Aabb3D& a, const Aabb3D& b) noexcept {
    const f32 overlap_x = std::min(a.max_x(), b.max_x()) - std::max(a.min_x(), b.min_x());
    if (overlap_x <= 0.0f) {
        return Contact3D{};
    }
    const f32 overlap_y = std::min(a.max_y(), b.max_y()) - std::max(a.min_y(), b.min_y());
    if (overlap_y <= 0.0f) {
        return Contact3D{};
    }
    const f32 overlap_z = std::min(a.max_z(), b.max_z()) - std::max(a.min_z(), b.min_z());
    if (overlap_z <= 0.0f) {
        return Contact3D{};
    }

    Contact3D contact;
    contact.overlapping = true;
    if (overlap_x <= overlap_y && overlap_x <= overlap_z) {
        contact.penetration = overlap_x;
        // Normal points from A towards B. If B's centre is to the right of A's,
        // that is +x; otherwise -x.
        contact.normal = Vec3{b.center.x >= a.center.x ? 1.0f : -1.0f, 0.0f, 0.0f};
    } else if (overlap_y <= overlap_z) {
        contact.penetration = overlap_y;
        contact.normal = Vec3{0.0f, b.center.y >= a.center.y ? 1.0f : -1.0f, 0.0f};
    } else {
        contact.penetration = overlap_z;
        contact.normal = Vec3{0.0f, 0.0f, b.center.z >= a.center.z ? 1.0f : -1.0f};
    }
    // The contact point sits at the centre of the overlap region -- the middle
    // of the box the two shapes share, which is where an impact effect belongs.
    contact.point = Vec3{(std::max(a.min_x(), b.min_x()) + std::min(a.max_x(), b.max_x())) * 0.5f,
                         (std::max(a.min_y(), b.min_y()) + std::min(a.max_y(), b.max_y())) * 0.5f,
                         (std::max(a.min_z(), b.min_z()) + std::min(a.max_z(), b.max_z())) * 0.5f};
    return contact;
}

Contact3D intersect_spheres(const Sphere3D& a, const Sphere3D& b) noexcept {
    const f32 radius_sum = a.radius + b.radius;
    if (radius_sum <= 0.0f) {
        return Contact3D{};
    }
    const Vec3 delta = b.center - a.center;
    const f32 distance_squared = delta.length_squared();
    if (distance_squared >= radius_sum * radius_sum) {
        return Contact3D{};
    }

    const f32 distance = std::sqrt(distance_squared);
    Contact3D contact;
    contact.overlapping = true;
    contact.penetration = radius_sum - distance;
    if (distance > 0.0f) {
        contact.normal = delta / distance;
    } else {
        // Concentric spheres have no defined direction; pick one so the solver
        // still separates them deterministically instead of dividing by zero.
        contact.normal = Vec3{0.0f, 1.0f, 0.0f};
    }
    // The contact point sits on the line between the centres, at the middle of
    // the overlap: the touching point of the two surfaces.
    contact.point = a.center + contact.normal * (a.radius - contact.penetration * 0.5f);
    return contact;
}

Contact3D intersect_sphere_box(const Sphere3D& sphere, const Aabb3D& box) noexcept {
    // Closest point on the box to the sphere centre, in box-local terms. This
    // handles the face, edge, and corner cases uniformly: if the centre is
    // inside the box, the closest point is the centre itself and the depth is
    // measured to the nearest face instead.
    const f32 closest_x = std::clamp(sphere.center.x, box.min_x(), box.max_x());
    const f32 closest_y = std::clamp(sphere.center.y, box.min_y(), box.max_y());
    const f32 closest_z = std::clamp(sphere.center.z, box.min_z(), box.max_z());
    const Vec3 closest{closest_x, closest_y, closest_z};
    const Vec3 delta = closest - sphere.center;
    const f32 distance_squared = delta.length_squared();

    if (distance_squared > sphere.radius * sphere.radius) {
        return Contact3D{};
    }

    Contact3D contact;
    contact.overlapping = true;
    if (distance_squared > 0.0f) {
        const f32 distance = std::sqrt(distance_squared);
        // Normal points from the sphere towards the box.
        contact.normal = delta / distance;
        contact.penetration = sphere.radius - distance;
    } else {
        // The sphere centre is inside the box. Separate along the nearest face.
        const f32 to_min_x = sphere.center.x - box.min_x();
        const f32 to_max_x = box.max_x() - sphere.center.x;
        const f32 to_min_y = sphere.center.y - box.min_y();
        const f32 to_max_y = box.max_y() - sphere.center.y;
        const f32 to_min_z = sphere.center.z - box.min_z();
        const f32 to_max_z = box.max_z() - sphere.center.z;
        const f32 min_x = std::min(to_min_x, to_max_x);
        const f32 min_y = std::min(to_min_y, to_max_y);
        const f32 min_z = std::min(to_min_z, to_max_z);
        if (min_x <= min_y && min_x <= min_z) {
            contact.normal = Vec3{to_max_x < to_min_x ? 1.0f : -1.0f, 0.0f, 0.0f};
            contact.penetration = min_x + sphere.radius;
        } else if (min_y <= min_z) {
            contact.normal = Vec3{0.0f, to_max_y < to_min_y ? 1.0f : -1.0f, 0.0f};
            contact.penetration = min_y + sphere.radius;
        } else {
            contact.normal = Vec3{0.0f, 0.0f, to_max_z < to_min_z ? 1.0f : -1.0f};
            contact.penetration = min_z + sphere.radius;
        }
    }
    // On the segment between the sphere's surface and the box's nearest surface,
    // half way through the overlap. `normal` points from the sphere towards the
    // box, so moving from the box's nearest point back along the normal reaches
    // the middle of the shared region.
    contact.point = closest + contact.normal * (contact.penetration * 0.5f);
    return contact;
}

// The separating-axis test between two oriented boxes. Fifteen candidate axes:
// each box's three local axes, and the nine cross products of their pairs. A
// positive overlap on every axis means the boxes touch; the axis with the
// smallest overlap is the contact normal, and that overlap is the penetration.
// This is the standard general box-box test (Ericson, Real-Time Collision
// Detection), and it is what makes a rotated box collide as its true shape
// rather than its unrotated bounding box.
Contact3D intersect_oriented_boxes(const Obb3D& a, const Obb3D& b) noexcept {
    Vec3 axes_a[3] = {a.axis(0), a.axis(1), a.axis(2)};
    Vec3 axes_b[3] = {b.axis(0), b.axis(1), b.axis(2)};

    // Rotate each axis to point away from the other box, so the separating axis
    // (if any) has a consistent sign; a box's projection is symmetric, so the
    // sign of an axis does not change the overlap, only the normal's direction.
    for (Vec3& axis : axes_a) {
        if ((b.center - a.center).dot(axis) < 0.0f) {
            axis = axis * -1.0f;
        }
    }
    for (Vec3& axis : axes_b) {
        if ((a.center - b.center).dot(axis) < 0.0f) {
            axis = axis * -1.0f;
        }
    }

    f32 best_overlap = std::numeric_limits<f32>::max();
    Vec3 best_axis{};
    bool have_axis = false;

    const auto test_axis = [&](const Vec3& raw_axis) -> bool {
        const f32 length = raw_axis.length();
        if (length < 1e-6f) {
            return true; // parallel or degenerate: this axis is redundant
        }
        const Vec3 axis = raw_axis / length;
        const f32 distance = std::abs((b.center - a.center).dot(axis));
        f32 radius_a = 0.0f;
        f32 radius_b = 0.0f;
        for (usize i = 0; i < 3; ++i) {
            radius_a += a.half(i) * std::abs(axes_a[i].dot(axis));
            radius_b += b.half(i) * std::abs(axes_b[i].dot(axis));
        }
        const f32 overlap = radius_a + radius_b - distance;
        if (overlap <= 0.0f) {
            return false; // a separating axis: the boxes do not touch
        }
        if (overlap < best_overlap) {
            best_overlap = overlap;
            // Point the normal from A towards B: the axis as oriented (A -> B),
            // or its negation if B is on the other side.
            best_axis = (b.center - a.center).dot(axis) >= 0.0f ? axis : axis * -1.0f;
            have_axis = true;
        }
        return true;
    };

    for (const Vec3& axis : axes_a) {
        if (!test_axis(axis)) {
            return Contact3D{};
        }
    }
    for (const Vec3& axis : axes_b) {
        if (!test_axis(axis)) {
            return Contact3D{};
        }
    }
    for (usize i = 0; i < 3; ++i) {
        for (usize j = 0; j < 3; ++j) {
            if (!test_axis(axes_a[i].cross(axes_b[j]))) {
                return Contact3D{};
            }
        }
    }

    if (!have_axis) {
        return Contact3D{};
    }
    Contact3D contact;
    contact.overlapping = true;
    contact.normal = best_axis;
    contact.penetration = best_overlap;
    // A representative point: the centroid of the two boxes' overlapping face
    // region perpendicular to the normal, on the touching faces where an impact
    // effect belongs.
    f32 support_a = 0.0f;
    f32 support_b = 0.0f;
    for (usize i = 0; i < 3; ++i) {
        support_a += a.half(i) * std::abs(a.axis(i).dot(best_axis));
        support_b += b.half(i) * std::abs(b.axis(i).dot(best_axis));
    }
    contact.point = box_box_contact_point(a, b, best_axis, support_a, support_b);
    return contact;
}

// The contact point for a box-box touch: the centroid of the *overlap* of the
// two boxes' faces perpendicular to the contact normal. Both boxes are convex,
// so their intersection is convex and its centroid lies on the touching faces,
// which is where an impact effect belongs. Clipping A's face polygon against
// B's four face planes gives that region; a box-box overlap is never empty, so
// the result is always at least one point. Falls back to the midpoint of the
// two support points if a degenerate face leaves nothing, which cannot normally
// happen.
[[nodiscard]] Vec3 box_box_contact_point(const Obb3D& a, const Obb3D& b, const Vec3& normal, f32 support_a,
                                         f32 support_b) noexcept {
    constexpr usize kMaxVertices = 16;
    const Vec3 origin = a.center + normal * support_a;
    const auto comp = [](const Vec3& value, int index) -> f32 {
        return index == 0 ? value.x : (index == 1 ? value.y : value.z);
    };
    const Vec3 n_abs{std::abs(normal.x), std::abs(normal.y), std::abs(normal.z)};
    // The face is perpendicular to the normal, so the axis most aligned with the
    // normal is the one the face does not span; drop it and build the face from
    // the other two.
    int dropped = 0;
    for (int axis = 1; axis < 3; ++axis) {
        if (comp(n_abs, axis) > comp(n_abs, dropped)) {
            dropped = axis;
        }
    }
    const int u = (dropped + 1) % 3;
    const int v = (dropped + 2) % 3;
    const auto basis = [](int index) -> Vec3 {
        return Vec3{index == 0 ? 1.0f : 0.0f, index == 1 ? 1.0f : 0.0f, index == 2 ? 1.0f : 0.0f};
    };

    // A's face, as four corners in the plane: centre plus the two in-plane axes
    // at their half extents.
    std::array<Vec3, kMaxVertices> poly{};
    usize count = 4;
    poly[0] = origin + basis(u) * a.half(u) + basis(v) * a.half(v);
    poly[1] = origin + basis(u) * a.half(u) - basis(v) * a.half(v);
    poly[2] = origin - basis(u) * a.half(u) - basis(v) * a.half(v);
    poly[3] = origin - basis(u) * a.half(u) + basis(v) * a.half(v);

    // Clip against each of B's four side planes. The planes are perpendicular to
    // the normal, so the third axis is free: clip only along B's other two axes,
    // where the projected half-extent is |axis_i . axis| * half_i.
    const Vec3 delta = b.center - a.center;
    for (int k = 0; k < 3; ++k) {
        if (k == dropped) {
            continue;
        }
        const Vec3 axis = b.axis(k);
        f32 half = 0.0f;
        for (usize i = 0; i < 3; ++i) {
            half += b.half(i) * std::abs(b.axis(i).dot(axis));
        }
        const f32 center = delta.dot(axis);
        for (const f32 sign : {1.0f, -1.0f}) {
            const f32 limit = center + sign * half;
            std::array<Vec3, kMaxVertices> out{};
            usize out_count = 0;
            for (usize i = 0; i < count; ++i) {
                const Vec3 cur = poly[i];
                const Vec3 next = poly[(i + 1) % count];
                const f32 d_cur = sign * (cur.dot(axis) - limit);
                const f32 d_next = sign * (next.dot(axis) - limit);
                if (d_cur <= 0.0f && out_count < kMaxVertices) {
                    out[out_count++] = cur;
                }
                if ((d_cur < 0.0f) != (d_next < 0.0f) && out_count < kMaxVertices) {
                    const f32 t = d_cur / (d_cur - d_next);
                    out[out_count++] = cur + (next - cur) * t;
                }
            }
            poly = out;
            count = out_count;
            if (count == 0) {
                break;
            }
        }
        if (count == 0) {
            break;
        }
    }

    if (count == 0) {
        return ((a.center + normal * support_a) + (b.center - normal * support_b)) * 0.5f;
    }
    Vec3 centroid{};
    for (usize i = 0; i < count; ++i) {
        centroid += poly[i];
    }
    return centroid / static_cast<f32>(count);
}

// Sphere against an oriented box: transform the sphere centre into the box's
// local frame, do the axis-aligned sphere-box test there, then rotate the
// normal back to world space. Exact, and it reuses the axis-aligned code.
Contact3D intersect_sphere_oriented_box(const Sphere3D& sphere, const Obb3D& box) noexcept {
    const Quat q = box.orientation.normalized();
    // World -> local: the box's centre-relative delta rotated by the inverse
    // (conjugate) of the orientation.
    const Vec3 delta = sphere.center - box.center;
    const Vec3 local_center = rotate_by_quat(delta, q.conjugate());
    const Aabb3D local_box{Vec3{}, box.half_extent};
    const Sphere3D local_sphere{local_center, sphere.radius};

    Contact3D contact = intersect_sphere_box(local_sphere, local_box);
    if (!contact.overlapping) {
        return contact;
    }
    // The local normal points from the sphere towards the box; rotate it back to
    // world space. The penetration is invariant under the rigid transform.
    contact.normal = rotate_by_quat(contact.normal, q);
    // The local contact point is relative to the box's centre; rotate and
    // translate it back to world space the same way the normal is rotated.
    contact.point = box.center + rotate_by_quat(contact.point, q);
    return contact;
}

} // namespace

Contact3D intersect(const Aabb3D& a, const Aabb3D& b) noexcept {
    return intersect_boxes(a, b);
}

Contact3D intersect(const Sphere3D& a, const Sphere3D& b) noexcept {
    return intersect_spheres(a, b);
}

Contact3D intersect(const Sphere3D& a, const Aabb3D& b) noexcept {
    return intersect_sphere_box(a, b);
}

Contact3D intersect(const Obb3D& a, const Obb3D& b) noexcept {
    return intersect_oriented_boxes(a, b);
}

Contact3D intersect(const Sphere3D& a, const Obb3D& b) noexcept {
    return intersect_sphere_oriented_box(a, b);
}

Contact3D intersect(const Obb3D& a, const Sphere3D& b) noexcept {
    // Compute sphere-vs-box and flip the normal so it points from the box
    // towards the sphere, matching the "from A towards B" convention.
    Contact3D contact = intersect_sphere_oriented_box(b, a);
    contact.normal = Vec3{-contact.normal.x, -contact.normal.y, -contact.normal.z};
    return contact;
}

bool bounds_overlap(const Aabb3D& a, const Aabb3D& b) noexcept {
    return a.min_x() < b.max_x() && a.max_x() > b.min_x() && a.min_y() < b.max_y() && a.max_y() > b.min_y() &&
           a.min_z() < b.max_z() && a.max_z() > b.min_z();
}

bool bounds_overlap(const Aabb3D& box, const Sphere3D& sphere) noexcept {
    return bounds_overlap(box, bounds_of(sphere));
}

Aabb3D bounds_of(const Sphere3D& sphere) noexcept {
    return Aabb3D{sphere.center, Vec3{sphere.radius, sphere.radius, sphere.radius}};
}

Aabb3D bounds_of(const Aabb3D& box) noexcept {
    return box;
}

Contact3D intersect_shapes(ShapeType3D a_type, const Aabb3D& a_box, const Sphere3D& a_sphere, ShapeType3D b_type,
                           const Aabb3D& b_box, const Sphere3D& b_sphere) noexcept {
    if (a_type == ShapeType3D::Box && b_type == ShapeType3D::Box) {
        return intersect(a_box, b_box);
    }
    if (a_type == ShapeType3D::Sphere && b_type == ShapeType3D::Sphere) {
        return intersect(a_sphere, b_sphere);
    }
    if (a_type == ShapeType3D::Sphere && b_type == ShapeType3D::Box) {
        return intersect(a_sphere, b_box);
    }
    // Box vs sphere: compute sphere-vs-box and flip the normal so it still points
    // from the first shape towards the second.
    Contact3D contact = intersect(b_sphere, a_box);
    contact.normal = Vec3{-contact.normal.x, -contact.normal.y, -contact.normal.z};
    return contact;
}

Contact3D intersect_shapes(ShapeType3D a_type, const Obb3D& a_box, const Sphere3D& a_sphere, ShapeType3D b_type,
                           const Obb3D& b_box, const Sphere3D& b_sphere) noexcept {
    if (a_type == ShapeType3D::Box && b_type == ShapeType3D::Box) {
        return intersect(a_box, b_box);
    }
    if (a_type == ShapeType3D::Sphere && b_type == ShapeType3D::Sphere) {
        return intersect(a_sphere, b_sphere);
    }
    if (a_type == ShapeType3D::Sphere && b_type == ShapeType3D::Box) {
        return intersect(a_sphere, b_box);
    }
    // Box vs sphere: compute sphere-vs-box and flip the normal so it still points
    // from the first shape towards the second.
    Contact3D contact = intersect(b_sphere, a_box);
    contact.normal = Vec3{-contact.normal.x, -contact.normal.y, -contact.normal.z};
    return contact;
}

// --- Ray casts -------------------------------------------------------------

f32 raycast_aabb(const Vec3& origin, const Vec3& direction, const Aabb3D& box) noexcept {
    // Slab test. For each axis the ray is inside the slab for t in [t_near,
    // t_far]; the ray enters the box at the larger of the near values and leaves
    // at the smaller of the far values. An axis-parallel ray (a zero direction
    // component) is inside its slab only if the origin is already between the
    // faces, which is handled without dividing by zero.
    f32 t_near = -std::numeric_limits<f32>::infinity();
    f32 t_far = std::numeric_limits<f32>::infinity();

    const f32 origin_axis[3] = {origin.x, origin.y, origin.z};
    const f32 direction_axis[3] = {direction.x, direction.y, direction.z};
    const f32 min_axis[3] = {box.min_x(), box.min_y(), box.min_z()};
    const f32 max_axis[3] = {box.max_x(), box.max_y(), box.max_z()};

    for (int axis = 0; axis < 3; ++axis) {
        const f32 o = origin_axis[axis];
        const f32 d = direction_axis[axis];
        if (std::abs(d) < kDirectionEpsilon) {
            // Parallel to this slab: a miss unless the origin is within it.
            if (o < min_axis[axis] || o > max_axis[axis]) {
                return -1.0f;
            }
            continue;
        }
        const f32 inv = 1.0f / d;
        f32 t0 = (min_axis[axis] - o) * inv;
        f32 t1 = (max_axis[axis] - o) * inv;
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        t_near = std::max(t_near, t0);
        t_far = std::min(t_far, t1);
        if (t_near > t_far) {
            return -1.0f;
        }
    }

    // The origin is inside the box when t_near < 0 < t_far; report a hit at 0
    // rather than a negative distance so callers always get a forward hit.
    if (t_far < 0.0f) {
        return -1.0f; // the box is entirely behind the ray
    }
    return t_near > 0.0f ? t_near : 0.0f;
}

f32 raycast_sphere(const Vec3& origin, const Vec3& direction, const Sphere3D& sphere) noexcept {
    if (sphere.radius <= 0.0f) {
        return -1.0f;
    }
    // Solve |origin + t*direction - center|^2 = r^2 for the smallest t >= 0.
    const Vec3 to_origin = origin - sphere.center;
    const f32 b = to_origin.dot(direction);
    const f32 c = to_origin.length_squared() - sphere.radius * sphere.radius;
    if (c > 0.0f && b > 0.0f) {
        return -1.0f; // origin outside and pointing away from the sphere
    }
    const f32 discriminant = b * b - c;
    if (discriminant < 0.0f) {
        return -1.0f;
    }
    const f32 root = std::sqrt(discriminant);
    const f32 t = -b - root;
    if (t >= 0.0f) {
        return t;
    }
    // The origin is inside the sphere; report an immediate hit.
    return 0.0f;
}

f32 raycast_shape(const Vec3& origin, const Vec3& direction, ShapeType3D type, const Aabb3D& box,
                  const Sphere3D& sphere) noexcept {
    return type == ShapeType3D::Sphere ? raycast_sphere(origin, direction, sphere)
                                       : raycast_aabb(origin, direction, box);
}

Vec3 raycast_aabb_normal(const Vec3& origin, const Vec3& direction, const Aabb3D& box, f32 distance) noexcept {
    if (distance <= 0.0f) {
        // The origin is inside (or on) the box; there is no crossed face, so the
        // normal points back along the ray.
        return Vec3{-direction.x, -direction.y, -direction.z};
    }
    const Vec3 point = origin + direction * distance;
    // Which face was crossed: the axis whose hit point sits on a face with the
    // smallest distance to that face. Ties prefer the lowest axis.
    const f32 dx = std::min(std::abs(point.x - box.min_x()), std::abs(point.x - box.max_x()));
    const f32 dy = std::min(std::abs(point.y - box.min_y()), std::abs(point.y - box.max_y()));
    const f32 dz = std::min(std::abs(point.z - box.min_z()), std::abs(point.z - box.max_z()));
    if (dx <= dy && dx <= dz) {
        const f32 sign = (point.x < box.center.x) ? -1.0f : 1.0f;
        return Vec3{sign, 0.0f, 0.0f};
    }
    if (dy <= dz) {
        const f32 sign = (point.y < box.center.y) ? -1.0f : 1.0f;
        return Vec3{0.0f, sign, 0.0f};
    }
    const f32 sign = (point.z < box.center.z) ? -1.0f : 1.0f;
    return Vec3{0.0f, 0.0f, sign};
}

Vec3 raycast_sphere_normal(const Vec3& origin, const Vec3& direction, const Sphere3D& sphere, f32 distance) noexcept {
    const Vec3 point = origin + direction * distance;
    const Vec3 outward = point - sphere.center;
    if (outward.length_squared() <= kDirectionEpsilon * kDirectionEpsilon) {
        return Vec3{-direction.x, -direction.y, -direction.z}; // degenerate: hit at the centre
    }
    return outward.normalized();
}

Vec3 raycast_shape_normal(const Vec3& origin, const Vec3& direction, ShapeType3D type, const Aabb3D& box,
                          const Sphere3D& sphere, f32 distance) noexcept {
    return type == ShapeType3D::Sphere ? raycast_sphere_normal(origin, direction, sphere, distance)
                                       : raycast_aabb_normal(origin, direction, box, distance);
}

bool contains(const Aabb3D& box, const Vec3& point) noexcept {
    return point.x >= box.min_x() && point.x <= box.max_x() && point.y >= box.min_y() && point.y <= box.max_y() &&
           point.z >= box.min_z() && point.z <= box.max_z();
}

bool contains(const Sphere3D& sphere, const Vec3& point) noexcept {
    if (sphere.radius <= 0.0f) {
        return false;
    }
    const Vec3 delta = point - sphere.center;
    return delta.length_squared() <= sphere.radius * sphere.radius;
}

bool contains_shape(ShapeType3D type, const Aabb3D& box, const Sphere3D& sphere, const Vec3& point) noexcept {
    return type == ShapeType3D::Sphere ? contains(sphere, point) : contains(box, point);
}

// --- Swept (continuous) tests ----------------------------------------------

Sweep3D sweep_shape(const Aabb3D& moving_bounds, const Vec3& displacement, const Aabb3D& target) noexcept {
    // Minkowski slab test, the 3D counterpart of `physics2d::sweep_shape`.
    // Shrinking the moving shape to its centre point and growing the target by
    // the moving half-extents turns "does the moving box overlap the target
    // during the sweep" into "does the moving *point* cross the expanded target"
    // -- a ray cast against an AABB, which is exact.
    const Aabb3D expanded{target.center, target.half_extent + moving_bounds.half_extent};

    // The point is already inside the expanded box: the shapes overlap at the
    // start of the sweep, so the contact time is zero. The normal is chosen on
    // the axis of least penetration, which is the direction that separates them
    // with the smallest push -- the same choice the discrete solver makes.
    if (contains(expanded, moving_bounds.center)) {
        const f32 dx_left = moving_bounds.center.x - expanded.min_x();
        const f32 dx_right = expanded.max_x() - moving_bounds.center.x;
        const f32 dy_bottom = moving_bounds.center.y - expanded.min_y();
        const f32 dy_top = expanded.max_y() - moving_bounds.center.y;
        const f32 dz_back = moving_bounds.center.z - expanded.min_z();
        const f32 dz_front = expanded.max_z() - moving_bounds.center.z;
        const f32 min_x = std::min(dx_left, dx_right);
        const f32 min_y = std::min(dy_bottom, dy_top);
        const f32 min_z = std::min(dz_back, dz_front);
        Sweep3D overlap;
        overlap.hit = true;
        overlap.time = 0.0f;
        if (min_x <= min_y && min_x <= min_z) {
            overlap.normal = Vec3{dx_left < dx_right ? -1.0f : 1.0f, 0.0f, 0.0f};
        } else if (min_y <= min_z) {
            overlap.normal = Vec3{0.0f, dy_bottom < dy_top ? -1.0f : 1.0f, 0.0f};
        } else {
            overlap.normal = Vec3{0.0f, 0.0f, dz_back < dz_front ? -1.0f : 1.0f};
        }
        return overlap;
    }

    // Entry times per axis. A zero displacement component means the point never
    // moves on that axis, so it can only be inside the slab for the whole sweep
    // (handled by the containment test above) or never; treat it as an infinite
    // entry time so it never becomes the limiting axis.
    f32 t_enter = 0.0f;
    int enter_axis = -1;
    f32 enter_sign = 0.0f;
    const f32 center_axis[3] = {moving_bounds.center.x, moving_bounds.center.y, moving_bounds.center.z};
    const f32 move_axis[3] = {displacement.x, displacement.y, displacement.z};
    const f32 min_axis[3] = {expanded.min_x(), expanded.min_y(), expanded.min_z()};
    const f32 max_axis[3] = {expanded.max_x(), expanded.max_y(), expanded.max_z()};

    for (int axis = 0; axis < 3; ++axis) {
        const f32 d = move_axis[axis];
        if (std::abs(d) < kDirectionEpsilon) {
            continue; // parallel to this slab and not inside it: cannot enter
        }
        const f32 inv = 1.0f / d;
        f32 t0 = (min_axis[axis] - center_axis[axis]) * inv;
        f32 t1 = (max_axis[axis] - center_axis[axis]) * inv;
        // The face the point enters through: the near slab in the direction of
        // travel. Track its sign for the normal.
        f32 near_sign = -1.0f;
        if (t0 > t1) {
            std::swap(t0, t1);
            near_sign = 1.0f;
        }
        if (t0 > t_enter) {
            t_enter = t0;
            enter_axis = axis;
            enter_sign = near_sign;
        }
    }

    if (enter_axis < 0 || t_enter > 1.0f || t_enter < 0.0f) {
        return Sweep3D{}; // never enters within the sweep
    }

    // Confirm the entry is not past the exit on the other axes: the point must
    // be inside every slab at t_enter.
    for (int axis = 0; axis < 3; ++axis) {
        const f32 d = move_axis[axis];
        if (std::abs(d) < kDirectionEpsilon) {
            continue;
        }
        const f32 inv = 1.0f / d;
        const f32 t0 = (min_axis[axis] - center_axis[axis]) * inv;
        const f32 t1 = (max_axis[axis] - center_axis[axis]) * inv;
        const f32 t_exit = std::max(t0, t1);
        if (t_exit < t_enter) {
            return Sweep3D{}; // enters one slab after leaving another
        }
    }

    Sweep3D hit;
    hit.hit = true;
    hit.time = t_enter;
    // The normal points from the target back towards the moving shape: opposite
    // the travel direction along the axis of entry.
    if (enter_axis == 0) {
        hit.normal = Vec3{enter_sign, 0.0f, 0.0f};
    } else if (enter_axis == 1) {
        hit.normal = Vec3{0.0f, enter_sign, 0.0f};
    } else {
        hit.normal = Vec3{0.0f, 0.0f, enter_sign};
    }
    return hit;
}

} // namespace fe::physics3d
