// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/physics2d/Physics2DQueries.h"

#include "fe/physics2d/Collider2D.h"
#include "fe/physics2d/Geometry.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/scene/Tilemap.h"

#include <cmath>

namespace fe::physics2d {
namespace {

// A collider's shape placed at its node's world position, orientation, and
// offset, so the pure geometry helpers can be reused without the query layer
// duplicating any shape maths. The box is oriented and the circle is offset, so
// a query sees the same shape the solver does.
struct PlacedShape {
    ShapeType type = ShapeType::Box;
    Aabb2D aabb{};
    Obb2D box{};
    Circle2D circle{};
};

[[nodiscard]] PlacedShape place(const Collider2D& collider, const Node& node) noexcept {
    const Vec3& position = node.transform().position;
    const Vec2 center{position.x, position.y};
    const f32 rotation = node.transform().rotation.z;
    PlacedShape shape;
    shape.type = collider.shape;
    shape.aabb = collider.bounds(center, rotation);
    shape.box = collider.oriented_box(center, rotation);
    shape.circle = collider.oriented_circle(center, rotation);
    return shape;
}

// True when `point` is inside the oriented box. The point is rotated into the
// box's local frame and compared to the half-extent, so a rotated box is tested
// as its true shape rather than its bounding box.
[[nodiscard]] bool contains_obb(const Obb2D& box, const Vec2& point) noexcept {
    const Vec2 delta = point - box.center;
    const f32 c = std::cos(box.rotation);
    const f32 s = std::sin(box.rotation);
    const Vec2 local{delta.x * c + delta.y * s, -delta.x * s + delta.y * c};
    return std::abs(local.x) <= box.half_extent.x && std::abs(local.y) <= box.half_extent.y;
}

[[nodiscard]] bool contains_placed(const PlacedShape& shape, const Vec2& point) noexcept {
    return shape.type == ShapeType::Circle ? contains(shape.circle, point) : contains_obb(shape.box, point);
}

} // namespace

RaycastHit raycast(const Scene& scene, const Vec2& origin, const Vec2& direction, f32 max_distance) noexcept {
    RaycastHit best;
    const f32 length = direction.length();
    if (length <= 0.0f) {
        return best; // a zero-length direction cannot hit anything
    }
    const Vec2 unit = direction / length;

    scene.for_each_node([&](Node& node) {
        const auto* collider = node.get_component<Collider2D>();
        if (collider == nullptr) {
            return;
        }
        const PlacedShape shape = place(*collider, node);
        const f32 distance = raycast_shape(origin, unit, shape.type, shape.aabb, shape.circle);
        if (distance < 0.0f) {
            return;
        }
        if (max_distance > 0.0f && distance > max_distance) {
            return;
        }
        if (best.hit && distance >= best.distance) {
            return;
        }
        best.hit = true;
        best.node = &node;
        best.distance = distance;
        best.point = origin + unit * distance;
        best.normal = raycast_shape_normal(origin, unit, shape.type, shape.aabb, shape.circle, distance);
    });
    return best;
}

usize overlap_circle(const Scene& scene, const Vec2& center, f32 radius, std::vector<OverlapHit>& out) {
    const Circle2D query{center, radius};
    usize count = 0;
    scene.for_each_node([&](Node& node) {
        const auto* collider = node.get_component<Collider2D>();
        if (collider == nullptr) {
            return;
        }
        const PlacedShape shape = place(*collider, node);
        // Query circle first, so the returned normal points from the query
        // towards the collider, matching the documented convention.
        const Contact2D contact =
            intersect_shapes(ShapeType::Circle, Obb2D{}, query, shape.type, shape.box, shape.circle);
        if (!contact.overlapping) {
            return;
        }
        out.push_back(OverlapHit{&node, contact.normal, contact.penetration});
        ++count;
    });
    return count;
}

usize overlap_box(const Scene& scene, const Vec2& center, const Vec2& half_extent, std::vector<OverlapHit>& out) {
    const Obb2D query{center, half_extent, 0.0f};
    usize count = 0;
    scene.for_each_node([&](Node& node) {
        const auto* collider = node.get_component<Collider2D>();
        if (collider == nullptr) {
            return;
        }
        const PlacedShape shape = place(*collider, node);
        const Contact2D contact =
            intersect_shapes(ShapeType::Box, query, Circle2D{}, shape.type, shape.box, shape.circle);
        if (!contact.overlapping) {
            return;
        }
        out.push_back(OverlapHit{&node, contact.normal, contact.penetration});
        ++count;
    });
    return count;
}

bool overlap_point(const Scene& scene, const Vec2& point, OverlapHit* out) noexcept {
    bool found = false;
    scene.for_each_node([&](Node& node) {
        if (found) {
            return;
        }
        const auto* collider = node.get_component<Collider2D>();
        if (collider == nullptr) {
            return;
        }
        const PlacedShape shape = place(*collider, node);
        if (!contains_placed(shape, point)) {
            return;
        }
        found = true;
        if (out != nullptr) {
            out->node = &node;
            out->normal = Vec2{};
            out->penetration = 0.0f;
        }
    });
    return found;
}

void append_tilemap_colliders(const Node& node, std::vector<TileCollider>& out) {
    const auto* map = node.get_component<Tilemap>();
    if (map == nullptr) {
        return;
    }
    if (map->cell_count() == 0 || map->tile_size.x == 0.0f || map->tile_size.y == 0.0f) {
        return;
    }

    const Mat4 world = node.world_matrix();
    // The world-space size of a cell: the local tile axes mapped through the
    // matrix's linear part. The renderer scales the tile quad by the same
    // matrix, so the box matches the drawn tile. For a rotation the two axes
    // stay perpendicular and the box is exact; a non-uniform scale shears them,
    // which an `Obb2D` approximates by its axis extents.
    const Vec3 axis_x = transform_direction(world, Vec3{map->tile_size.x, 0.0f, 0.0f});
    const Vec3 axis_y = transform_direction(world, Vec3{0.0f, map->tile_size.y, 0.0f});
    const Vec2 half{0.5f * std::sqrt(axis_x.x * axis_x.x + axis_x.y * axis_x.y),
                    0.5f * std::sqrt(axis_y.x * axis_y.x + axis_y.y * axis_y.y)};
    if (half.x == 0.0f || half.y == 0.0f) {
        return; // a degenerate cell contributes no box
    }
    // The cell's orientation is the map's own Z rotation, taken from the world
    // matrix's first column, so a rotated map's cells collide rotated.
    const f32 rotation = std::atan2(world.at(1, 0), world.at(0, 0));

    for (u32 y = 0; y < map->height; ++y) {
        for (u32 x = 0; x < map->width; ++x) {
            if (!map->is_solid(x, y)) {
                continue;
            }
            const f32 local_x = (static_cast<f32>(x) + 0.5f) * map->tile_size.x;
            const f32 local_y = -(static_cast<f32>(y) + 0.5f) * map->tile_size.y;
            const Vec3 center = transform_point(world, Vec3{local_x, local_y, 0.0f});

            TileCollider collider;
            collider.box = Obb2D{Vec2{center.x, center.y}, half, rotation};
            collider.node = const_cast<Node*>(&node);
            collider.cell_x = x;
            collider.cell_y = y;
            out.push_back(collider);
        }
    }
}

void append_tilemap_colliders(const Scene& scene, std::vector<TileCollider>& out) {
    scene.for_each_node([&](Node& node) { append_tilemap_colliders(node, out); });
}

} // namespace fe::physics2d
