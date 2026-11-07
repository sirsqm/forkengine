// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/physics2d/Physics2DSystem.h"

#include "fe/physics2d/Collider2D.h"
#include "fe/physics2d/DistanceJoint2D.h"
#include "fe/physics2d/FixedJoint2D.h"
#include "fe/physics2d/Geometry.h"
#include "fe/physics2d/PrismaticJoint2D.h"
#include "fe/physics2d/RevoluteJoint2D.h"
#include "fe/physics2d/RigidBody2D.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"

#include <algorithm>
#include <cmath>
#include <span>
#include <utility>

namespace fe::physics2d {
namespace {

// Fraction of the penetration corrected per iteration. Correcting the whole
// depth at once overshoots and makes stacks jitter; a fraction converges
// smoothly. This is the standard Baumgarte factor.
constexpr f32 kPositionCorrection = 0.8f;

// Penetration below which the positional correction is skipped, in world units.
// Without a slop, resting bodies are pushed apart a hair every frame and
// visibly vibrate.
constexpr f32 kPenetrationSlop = 0.005f;

// The "no island vertex" marker in the body-to-vertex map. A static body (one
// with infinite mass) is never a vertex in the island graph, so it maps to this.
constexpr usize kNoVertex = static_cast<usize>(-1);

// The friction a tile cell carries. A tile has no material of its own -- the
// `Tilemap` component has no per-tile friction -- so every tile uses one value,
// the conventional ground friction, which makes a body stand and walk on it.
constexpr f32 kTileFriction = 0.7f;

[[nodiscard]] f32 clamp_unit(f32 value) noexcept {
    return std::clamp(value, 0.0f, 1.0f);
}

// The 2D cross product of a vector and a scalar: the tangential velocity a body
// at offset `r` from the contact contributes from an angular velocity `w`. In
// 2D this is (-r.y * w, r.x * w), the rotation of `r` by 90 degrees scaled by w.
[[nodiscard]] Vec2 cross_scalar_vec2(f32 scalar, const Vec2& vector) noexcept {
    return Vec2{-scalar * vector.y, scalar * vector.x};
}

// The scalar 2D cross of two vectors: the z component of their 3D cross, used
// to project a linear impulse onto an angular impulse about the contact.
[[nodiscard]] f32 cross_vec2(const Vec2& a, const Vec2& b) noexcept {
    return a.x * b.y - a.y * b.x;
}

// Reads a node's current joint state: its transform for position and rotation,
// and its rigid body (if any) for velocity and the inverse mass and inertia the
// joint shares its correction by. A node with no rigid body is immovable, which
// is exactly what a fixed pivot is.
[[nodiscard]] JointBody make_joint_body(const Node& node) noexcept {
    JointBody joint;
    joint.position = Vec2{node.transform().position.x, node.transform().position.y};
    joint.rotation = node.transform().rotation.z;

    const auto* body = node.get_component<RigidBody2D>();
    if (body == nullptr) {
        return joint;
    }
    joint.velocity = body->velocity;
    joint.angular_velocity = body->angular_velocity;
    joint.inv_mass = body->inverse_mass();
    if (joint.inv_mass > 0.0f && !body->fixed_rotation) {
        // Author-set inertia wins; otherwise derive it from the collider's
        // shape, the same rule the contact solver uses. A joint end with no
        // collider falls back to the author's inertia, or to zero (a point
        // mass, which a pin still constrains translationally).
        f32 inertia = body->inertia;
        if (inertia <= 0.0f) {
            if (const auto* collider = node.get_component<Collider2D>(); collider != nullptr) {
                inertia = collider->inertia_about_center(body->mass);
            }
        }
        joint.inv_inertia = inertia > 0.0f ? 1.0f / inertia : 0.0f;
    }
    return joint;
}

// Writes a solved joint state back to the node's transform and rigid body,
// mirroring the contact solver's write-back so the two agree on who owns what.
void store_joint_body(Node& node, const JointBody& joint) noexcept {
    node.transform().position.x = joint.position.x;
    node.transform().position.y = joint.position.y;
    node.transform().rotation.z = joint.rotation;
    if (auto* body = node.get_component<RigidBody2D>(); body != nullptr) {
        body->velocity = joint.velocity;
        body->angular_velocity = joint.angular_velocity;
    }
}

// Applies `count` rows to the two bodies: the velocity impulse and the positional
// correction for each. In 2D a row carries at most one scalar angular term and
// `apply_row_position` corrects translation and rotation together, so every
// joint kind -- distance, revolute, fixed or prismatic -- solves its rows the
// same way.
void apply_rows(ConstraintRow rows[], usize count, JointBody& a, JointBody& b) noexcept {
    for (usize i = 0; i < count; ++i) {
        apply_row(rows[i], a, b);
        apply_row_position(rows[i], a, b);
    }
}

} // namespace

const Uuid& Physics2DSystem::Joint::body_a() const noexcept {
    if (distance != nullptr) {
        return distance->body_a;
    }
    if (revolute != nullptr) {
        return revolute->body_a;
    }
    if (fixed != nullptr) {
        return fixed->body_a;
    }
    return prismatic->body_a;
}

const Uuid& Physics2DSystem::Joint::body_b() const noexcept {
    if (distance != nullptr) {
        return distance->body_b;
    }
    if (revolute != nullptr) {
        return revolute->body_b;
    }
    if (fixed != nullptr) {
        return fixed->body_b;
    }
    return prismatic->body_b;
}

std::string_view Physics2DSystem::Joint::type_name() const noexcept {
    if (distance != nullptr) {
        return DistanceJoint2D::kTypeName;
    }
    if (revolute != nullptr) {
        return RevoluteJoint2D::kTypeName;
    }
    if (fixed != nullptr) {
        return FixedJoint2D::kTypeName;
    }
    return PrismaticJoint2D::kTypeName;
}

f32 Physics2DSystem::Joint::break_force() const noexcept {
    if (distance != nullptr) {
        return distance->break_force;
    }
    if (revolute != nullptr) {
        return revolute->break_force;
    }
    if (fixed != nullptr) {
        return fixed->break_force;
    }
    return prismatic->break_force;
}

f32 Physics2DSystem::Joint::break_torque() const noexcept {
    if (distance != nullptr) {
        return distance->break_torque;
    }
    if (revolute != nullptr) {
        return revolute->break_torque;
    }
    if (fixed != nullptr) {
        return fixed->break_torque;
    }
    return prismatic->break_torque;
}

bool Physics2DSystem::Joint::is_broken() const noexcept {
    if (distance != nullptr) {
        return distance->broken;
    }
    if (revolute != nullptr) {
        return revolute->broken;
    }
    if (fixed != nullptr) {
        return fixed->broken;
    }
    return prismatic->broken;
}

void Physics2DSystem::Joint::set_broken(bool value) noexcept {
    if (distance != nullptr) {
        distance->broken = value;
    } else if (revolute != nullptr) {
        revolute->broken = value;
    } else if (fixed != nullptr) {
        fixed->broken = value;
    } else {
        prismatic->broken = value;
    }
}

void Physics2DSystem::update(const FrameContext& context, Scene& scene) {
    contacts_ = 0;
    events_.clear();
    active_pairs_.clear();
    joints_solved_ = 0;
    joint_diagnostics_.clear();
    joints_.clear();
    node_index_.clear();
    body_index_.clear();
    asleep_.clear();
    sleeping_bodies_ = 0;
    continuous_hits_ = 0;

    // Gather. Walk the scene once and keep every node that can participate in a
    // collision, with its world position and material data flattened out. The
    // solver below never touches the scene graph.
    bodies_.clear();
    scene.for_each_node([&](Node& node) {
        // Index every node, not just colliders: a joint end may be a bare pivot
        // with no collider at all.
        node_index_.emplace(node.id(), &node);

        // Collect joints here too, in the same single walk. The bodies they
        // reference are resolved in solve_joints, after the whole scene is
        // indexed, so a joint may point forward to a node that appears later.
        if (auto* distance = node.get_component<DistanceJoint2D>(); distance != nullptr) {
            joints_.push_back(Joint{distance, nullptr, nullptr, nullptr, nullptr, nullptr});
        }
        if (auto* revolute = node.get_component<RevoluteJoint2D>(); revolute != nullptr) {
            joints_.push_back(Joint{nullptr, revolute, nullptr, nullptr, nullptr, nullptr});
        }
        if (auto* fixed = node.get_component<FixedJoint2D>(); fixed != nullptr) {
            joints_.push_back(Joint{nullptr, nullptr, fixed, nullptr, nullptr, nullptr});
        }
        if (auto* prismatic = node.get_component<PrismaticJoint2D>(); prismatic != nullptr) {
            joints_.push_back(Joint{nullptr, nullptr, nullptr, prismatic, nullptr, nullptr});
        }

        auto* collider = node.get_component<Collider2D>();
        auto* body = node.get_component<RigidBody2D>();
        // Simulate a node that has either a collider (so it can collide) or a
        // rigid body (so a joint can pull on it even with no collider). A node
        // with neither is a bare pivot, which a joint may still hang from: it is
        // indexed above and its end is read on demand, but it is not simulated.
        if (collider == nullptr && body == nullptr) {
            return;
        }
        const Vec3& position = node.transform().position;

        Body entry{};
        entry.collider = collider;
        entry.body = body;
        entry.node = &node;
        entry.position = Vec2{position.x, position.y};
        entry.velocity = body != nullptr ? body->velocity : Vec2{};
        entry.rotation = node.transform().rotation.z;
        entry.angular_velocity = body != nullptr ? body->angular_velocity : 0.0f;
        entry.inv_mass = body != nullptr ? body->inverse_mass() : 0.0f;
        entry.restitution = collider != nullptr ? clamp_unit(collider->restitution) : 0.0f;
        entry.friction = collider != nullptr ? std::max(collider->friction, 0.0f) : 0.0f;
        entry.fixed_rotation = body != nullptr && body->fixed_rotation;

        // Rotational inertia. An author-set inertia on the body wins; otherwise
        // it is derived from the shape and mass, so a box or disc rotates with a
        // physically sensible resistance. A non-dynamic body keeps zero. A body
        // with no collider and no author inertia is a point mass, which has zero
        // rotational inertia and cannot spin on its own.
        if (body != nullptr && entry.inv_mass > 0.0f && !body->fixed_rotation) {
            f32 inertia = body->inertia;
            if (inertia <= 0.0f && collider != nullptr) {
                inertia = collider->inertia_about_center(body->mass);
            }
            entry.inv_inertia = inertia > 0.0f ? 1.0f / inertia : 0.0f;
        } else {
            entry.inv_inertia = 0.0f;
        }

        // A sleeping body resumes from rest: its authored velocity is ignored so
        // a body that was asleep when the scene loaded does not spring into
        // motion on the first step. Its island is re-derived below and the wake
        // pass decides whether it stays asleep.
        const bool sleeping = body != nullptr && body->is_sleeping && entry.inv_mass > 0.0f;
        if (sleeping) {
            entry.velocity = Vec2{};
            entry.angular_velocity = 0.0f;
        }

        body_index_.emplace(node.id(), bodies_.size());
        asleep_.push_back(sleeping);
        if (sleeping) {
            ++sleeping_bodies_;
        }
        bodies_.push_back(entry);
    });

    node_body_count_ = bodies_.size();

    // Solid tile cells are static geometry too, gathered from the same grid the
    // renderer draws. A tile is a box collider with infinite mass, so it joins
    // the broad phase and the solver like any static collider, but it has no
    // node: its contact is attributed to the map node, and it is excluded from
    // the contact-event stream (which is keyed by node, and several tiles of one
    // map can touch the same body at once). `tile_shapes_` owns the shapes; the
    // pool is reused across steps so a steady-state step does not allocate.
    tile_colliders_.clear();
    append_tilemap_colliders(scene, tile_colliders_);
    if (tile_shapes_.size() < tile_colliders_.size()) {
        tile_shapes_.reserve(tile_colliders_.size());
    }
    while (tile_shapes_.size() < tile_colliders_.size()) {
        tile_shapes_.push_back(std::make_unique<Collider2D>());
    }
    for (usize t = 0; t < tile_colliders_.size(); ++t) {
        const TileCollider& tile = tile_colliders_[t];
        // `Collider2D` is a `Component` (non-copyable, non-movable), so its
        // fields are set in place rather than assigned from a temporary. Only
        // the fields the solver reads matter; the rest keep their defaults.
        Collider2D& shape = *tile_shapes_[t];
        shape.shape = ShapeType::Box;
        shape.half_extent = tile.box.half_extent;
        shape.radius = 0.0f;
        shape.offset = Vec2{};
        shape.restitution = 0.0f;
        shape.friction = kTileFriction;
        shape.layer = 1u;
        shape.mask = 0xFFFFFFFFu;
        shape.is_trigger = false;

        Body entry{};
        entry.collider = &shape;
        entry.body = nullptr;
        entry.node = tile.node;
        entry.position = tile.box.center;
        entry.rotation = tile.box.rotation;
        entry.inv_mass = 0.0f;
        entry.inv_inertia = 0.0f;
        entry.restitution = 0.0f;
        entry.friction = kTileFriction;
        entry.is_tile = true;
        bodies_.push_back(entry);
        asleep_.push_back(false);
    }

    if (bodies_.empty()) {
        // No colliders at all: there can be no contacts. Reconcile against an
        // empty active set so every pair that touched last frame ends cleanly.
        // Joints still run: a joint end may be a bare body with no collider.
        reconcile_contacts(scene);
        solve_joints(scene, context.delta_time);
        return;
    }

    const f32 delta_time = context.delta_time;
    if (delta_time <= 0.0f) {
        // A paused or non-positive step advances nothing and produces no
        // contacts. It also emits no events: a paused world has not changed, so
        // a contact neither begins nor ends. `previous_pairs_` is deliberately
        // left untouched so a contact that was touching before the pause is not
        // re-reported as a Begin when the world resumes. Joints are not solved
        // either: there is no step to advance them.
        return;
    }

    // Islands, then wake checks, then integration -- all before the solver. The
    // island graph is built from the contacts that exist at the start of the
    // step and from the joints, so a moving body wakes a sleeper it has just
    // reached before the solver runs and can never pass through it.
    build_islands();
    apply_wake_checks();

    // Integrate velocity: gravity, torque, then linear and angular damping. A
    // body with no dynamic motion (infinite mass) skips all of it, and so does a
    // sleeping body: it is at rest and stays exactly where it is.
    for (usize i = 0; i < bodies_.size(); ++i) {
        Body& entry = bodies_[i];
        if (entry.inv_mass <= 0.0f || asleep_[i]) {
            continue;
        }
        if (entry.body->affected_by_gravity) {
            entry.velocity += gravity * delta_time;
        }
        // A per-step force acts through the body's inverse mass, and the
        // accumulated torque is summed with the constant one. Both are cleared
        // after the step, so a force must be re-added each step to persist.
        if (entry.inv_mass > 0.0f && (entry.body->force.x != 0.0f || entry.body->force.y != 0.0f)) {
            entry.velocity += entry.body->force * (entry.inv_mass * delta_time);
        }
        const f32 total_torque = entry.body->torque + entry.body->applied_torque;
        if (total_torque != 0.0f && entry.inv_inertia > 0.0f) {
            entry.angular_velocity += total_torque * entry.inv_inertia * delta_time;
        }
        const f32 damping = clamp_unit(entry.body->linear_damping);
        if (damping > 0.0f) {
            // Frame-rate-independent decay: the velocity is multiplied by
            // (1 - damping) once per second, distributed across this frame.
            entry.velocity = entry.velocity * std::max(0.0f, 1.0f - damping * delta_time);
        }
        const f32 angular_damping = clamp_unit(entry.body->angular_damping);
        if (angular_damping > 0.0f) {
            entry.angular_velocity *= std::max(0.0f, 1.0f - angular_damping * delta_time);
        }
    }

    // Integrate position and rotation from the updated velocities. The previous
    // position is kept for the continuous pass below, which needs the start of
    // the motion, not only its end.
    previous_positions_.assign(bodies_.size(), Vec2{});
    previous_rotations_.assign(bodies_.size(), 0.0f);
    for (usize i = 0; i < bodies_.size(); ++i) {
        Body& entry = bodies_[i];
        if (entry.inv_mass <= 0.0f || asleep_[i]) {
            continue;
        }
        previous_positions_[i] = entry.position;
        previous_rotations_[i] = entry.rotation;
        entry.position += entry.velocity * delta_time;
        // A fixed-rotation body keeps its authored rotation regardless of its
        // angular velocity; the body's angular state is carried through so a
        // contact can still see it, but it is never advanced.
        if (!entry.fixed_rotation) {
            entry.rotation += entry.angular_velocity * delta_time;
        }
    }

    // Continuous collision detection, before the discrete pass. A fast body can
    // move past a thin static wall in one step; the discrete narrow phase only
    // sees the endpoints and would let the body tunnel through. Sweeping each
    // awake body's motion and clamping it to the first touch stops a bullet at
    // the wall's surface instead (ADR-0037's remaining refinement).
    apply_continuous_collisions(delta_time);

    // Broad phase: find the candidate overlapping pairs once, before the
    // iteration loop, so the solver does not re-enumerate the whole pair set on
    // every iteration. A body with no collider (a joint-only mass) has no bounds
    // and is not inserted, so it never appears in a candidate.
    //
    // `bodies_` holds colliderless bodies too, so the grid's indices are into
    // `broad_phase_bounds_`, not into `bodies_`. `bounds_owner_` maps one to the
    // other; without it a scene with a joint-only mass ahead of its colliders
    // would resolve the wrong bodies.
    broad_phase_bounds_.clear();
    bounds_owner_.clear();
    broad_phase_bounds_.reserve(bodies_.size());
    bounds_owner_.reserve(bodies_.size());
    for (usize i = 0; i < bodies_.size(); ++i) {
        const Body& entry = bodies_[i];
        if (entry.collider != nullptr) {
            // The world AABB of the oriented shape: for a rotated box this is the
            // conservative bound of its four corners, so the grid stays a
            // superset of the truly-touching pairs.
            broad_phase_bounds_.push_back(entry.collider->bounds(entry.position, entry.rotation));
            bounds_owner_.push_back(i);
        }
    }
    broad_phase_.build(broad_phase_bounds_, context.jobs);
    const std::span<const BroadPhasePair> candidates = broad_phase_.pairs();

    // Resolve. Sequential impulses over the candidate pairs, repeated for a few
    // iterations. The candidate set is computed once above; each iteration walks
    // only it, so a step costs O(candidates * iterations) rather than
    // O(bodies^2 * iterations).
    for (u32 iteration = 0; iteration < iterations; ++iteration) {
        const bool first_iteration = iteration == 0;
        for (const BroadPhasePair& candidate : candidates) {
            Body& a = bodies_[bounds_owner_[candidate.first]];
            Body& b = bodies_[bounds_owner_[candidate.second]];

            // A body with no collider (a joint-only mass) cannot collide. The
            // grid never inserts one, but the guard keeps the solver honest if a
            // caller ever passes a colliderless body through.
            if (a.collider == nullptr || b.collider == nullptr) {
                continue;
            }

            // Sharing a grid cell is necessary but not sufficient: keep the
            // existing bounds prefilter so a pair in the same cell but not
            // actually close is rejected before the narrow phase.
            if (!bounds_overlap(a.collider->bounds(a.position, a.rotation),
                                b.collider->bounds(b.position, b.rotation))) {
                continue;
            }

            // Collision filtering. A pair whose layers and masks disagree is
            // ignored entirely: it never reaches the narrow phase, is never
            // recorded as touching, and so produces no contact event. "Ignored"
            // has to mean ignored for events too, or a game would get a begin
            // event for a pair it told the engine never to touch.
            if (!a.collider->can_collide_with(*b.collider)) {
                continue;
            }

            // Place each shape by its node's world position, offset, and Z
            // rotation, so a rotated box collides as its true oriented shape.
            const Contact2D contact = intersect_shapes(
                a.collider->shape, a.collider->oriented_box(a.position, a.rotation),
                a.collider->oriented_circle(a.position, a.rotation), b.collider->shape,
                b.collider->oriented_box(b.position, b.rotation), b.collider->oriented_circle(b.position, b.rotation));
            if (!contact.overlapping) {
                continue;
            }

            // Record the touching pair once, on the first iteration, so the
            // contact-event pass can diff it against the previous frame. The
            // key is by node identity (ordered), so it is stable no matter how
            // the scene is walked. A sleeping body's contacts are still
            // recorded -- its position does not change, so they neither begin
            // nor end, and the event stream stays quiet -- but no impulse is
            // applied to it below.
            //
            // A tile pair is not recorded: a tile has no node of its own (it is
            // attributed to the map node), and several tiles of one map can
            // touch the same body at once, so a tile contact would report as a
            // Begin/End against the map node for each cell and the diff would
            // thrash. The pair is still solved; only the event is skipped.
            if (first_iteration && !a.is_tile && !b.is_tile) {
                ++contacts_;
                BodyPair pair;
                // Order the pair by raw id bytes so the key is stable and
                // independent of scene order.
                const bool a_first =
                    std::lexicographical_compare(a.node->id().bytes().begin(), a.node->id().bytes().end(),
                                                 b.node->id().bytes().begin(), b.node->id().bytes().end());
                pair.first = a_first ? a.node->id() : b.node->id();
                pair.second = a_first ? b.node->id() : a.node->id();
                active_pairs_.push_back(pair);
            }

            // A trigger is a sensor: the touching pair was recorded above (so
            // the contact-event pass reports Begin and End exactly as for a
            // solid pair), but no impulse and no positional correction is
            // applied, so bodies pass through it. This is checked after the pair
            // record so a trigger still reports contacts, and before the sleeping
            // and impulse guards so it never moves anything.
            if (a.collider->is_trigger || b.collider->is_trigger) {
                continue;
            }

            // A sleeping body is skipped by the solver. Its contacts were
            // recorded above, but it is not moved or impulsed: it is at rest
            // until something wakes it, which the wake pass did before this
            // loop if an awake body was touching it. The body indices come from
            // `bounds_owner_`, because the grid indexes its own bounds array,
            // not `bodies_`.
            const usize index_a = bounds_owner_[candidate.first];
            const usize index_b = bounds_owner_[candidate.second];
            if (asleep_[index_a] || asleep_[index_b]) {
                continue;
            }

            const f32 inv_mass_sum = a.inv_mass + b.inv_mass;
            if (inv_mass_sum <= 0.0f) {
                continue; // two immovable colliders: nothing to resolve
            }

            // Positional correction: push the pair apart along the normal.
            // Split by inverse mass, so a heavy body moves less than a light
            // one, and a static body not at all.
            const f32 depth = contact.penetration - kPenetrationSlop;
            if (depth > 0.0f) {
                const Vec2 correction = contact.normal * (kPositionCorrection * depth / inv_mass_sum);
                a.position -= correction * a.inv_mass;
                b.position += correction * b.inv_mass;
            }

            // The contact point, taken on A's surface along the normal (a
            // single-point approximation, standard for box and disc
            // contacts). Angular impulses need the lever arm from each
            // centre to this point; a contact through a centre produces no
            // torque, which is what keeps a body resting flat stable. Each
            // shape is placed by its node's offset and Z rotation, so a
            // rotated box or an offset shape reports the lever arm at its
            // true surface rather than at a shape centred on the node.
            const Vec2 center_a = a.collider->world_center(a.position, a.rotation);
            const f32 support_a =
                a.collider->shape == ShapeType::Circle
                    ? a.collider->radius
                    : support_radius(a.collider->oriented_box(a.position, a.rotation), contact.normal);
            const Vec2 contact_point = center_a + contact.normal * (support_a - contact.penetration * 0.5f);

            // Lever arms from each body centre to the contact point. The
            // body's centre of mass is the node origin, so the offset shifts
            // where the surface is and therefore the lever arm.
            const Vec2 ra = contact_point - a.position;
            const Vec2 rb = contact_point - b.position;

            // Relative velocity at the contact, including the contribution
            // of each body's rotation. `normal` points from A to B, so a
            // separating pair has a positive projection and needs no impulse.
            const Vec2 relative = (b.velocity + cross_scalar_vec2(b.angular_velocity, rb)) -
                                  (a.velocity + cross_scalar_vec2(a.angular_velocity, ra));
            const f32 normal_speed = relative.dot(contact.normal);
            if (normal_speed >= 0.0f) {
                continue;
            }

            // Effective mass along the normal, including the angular term.
            // This is what couples rotation into the linear response.
            const f32 ra_cross_n = cross_vec2(ra, contact.normal);
            const f32 rb_cross_n = cross_vec2(rb, contact.normal);
            const f32 effective_mass =
                inv_mass_sum + a.inv_inertia * ra_cross_n * ra_cross_n + b.inv_inertia * rb_cross_n * rb_cross_n;
            if (effective_mass <= 0.0f) {
                continue;
            }

            // Restitution of the pair is the larger of the two, which is the
            // conventional choice and keeps a bouncy ball bouncy against a
            // dead floor.
            const f32 restitution = std::max(a.restitution, b.restitution);
            const f32 impulse_magnitude = -(1.0f + restitution) * normal_speed / effective_mass;
            const Vec2 impulse = contact.normal * impulse_magnitude;

            a.velocity -= impulse * a.inv_mass;
            b.velocity += impulse * b.inv_mass;
            // Angular response: the impulse at a lever arm produces torque.
            a.angular_velocity -= a.inv_inertia * cross_vec2(ra, impulse);
            b.angular_velocity += b.inv_inertia * cross_vec2(rb, impulse);

            // Friction: an impulse opposing the tangential relative motion,
            // capped by the Coulomb cone. With angular dynamics it also spins
            // the bodies, which is what makes a box tumble rather than slide.
            const Vec2 tangent_raw = relative - contact.normal * normal_speed;
            const f32 tangent_length = tangent_raw.length();
            if (tangent_length > 0.0f) {
                const Vec2 tangent = tangent_raw / tangent_length;
                const f32 ra_cross_t = cross_vec2(ra, tangent);
                const f32 rb_cross_t = cross_vec2(rb, tangent);
                const f32 tangent_mass =
                    inv_mass_sum + a.inv_inertia * ra_cross_t * ra_cross_t + b.inv_inertia * rb_cross_t * rb_cross_t;
                if (tangent_mass > 0.0f) {
                    const f32 tangent_speed = relative.dot(tangent);
                    f32 friction_impulse = -tangent_speed / tangent_mass;
                    // Combined friction is the geometric mean, the rule Box2D
                    // uses and the one that keeps the endpoints meaningful: a
                    // material with friction 0 stays frictionless against a
                    // rough surface, and 1 against 1 is 1.
                    //
                    // Taking the larger of the two instead (the previous rule)
                    // makes friction 0 inexpressible -- a ball authored as
                    // frictionless picks up the wall's 0.5 -- so every glancing
                    // contact bleeds tangential speed. A ball bounced off a wall
                    // loses its vertical component a little each hit and ends up
                    // travelling dead horizontal forever.
                    const f32 max_friction = std::sqrt(a.friction * b.friction) * impulse_magnitude;
                    friction_impulse = std::clamp(friction_impulse, -max_friction, max_friction);
                    const Vec2 friction = tangent * friction_impulse;
                    a.velocity -= friction * a.inv_mass;
                    b.velocity += friction * b.inv_mass;
                    a.angular_velocity -= a.inv_inertia * cross_vec2(ra, friction);
                    b.angular_velocity += b.inv_inertia * cross_vec2(rb, friction);
                }
            }
        }
    }

    // Write back. The transform owns the node's position and rotation; the
    // rigid body owns its velocities. Writing both here is what keeps the two in
    // sync, exactly as the animation system writes UVs back onto the sprite. A
    // sleeping body is skipped: its node transform is left exactly as authored,
    // so nothing moves it while it sleeps.
    for (usize i = 0; i < bodies_.size(); ++i) {
        Body& entry = bodies_[i];
        if (entry.inv_mass <= 0.0f || asleep_[i]) {
            continue;
        }
        entry.body->velocity = entry.velocity;
        entry.body->angular_velocity = entry.angular_velocity;
        Node* node = entry.node;
        if (node != nullptr) {
            node->transform().position.x = entry.position.x;
            node->transform().position.y = entry.position.y;
            node->transform().rotation.z = entry.rotation;
        }
    }

    // Joints last, on the freshly written-back transforms and velocities. A
    // joint is solved once (not per contact iteration) because a pin needs the
    // error corrected at a fixed rate rather than crushed to zero in one step.
    // A joint with a sleeping end is skipped, so a sleeping island is not pulled
    // by its own constraint; the wake pass already woke the island if the other
    // end was moving.
    solve_joints(scene, delta_time);

    // Sleep checks last, on the velocity the solver actually left behind, so a
    // body that was pushed this step does not count as at rest.
    apply_sleep_checks(delta_time);

    reconcile_contacts(scene);

    // The per-step force and applied torque have been integrated; clear them so
    // a force that is not re-added next step does not keep acting.
    scene.for_each_node([](Node& node) {
        if (auto* body = node.get_component<RigidBody2D>(); body != nullptr) {
            body->clear_forces();
        }
    });
}

namespace {

// The smallest axis-aligned box enclosing both `a` and `b`.
Aabb2D union_bounds(const Aabb2D& a, const Aabb2D& b) noexcept {
    const Vec2 lo{std::min(a.center.x - a.half_extent.x, b.center.x - b.half_extent.x),
                  std::min(a.center.y - a.half_extent.y, b.center.y - b.half_extent.y)};
    const Vec2 hi{std::max(a.center.x + a.half_extent.x, b.center.x + b.half_extent.x),
                  std::max(a.center.y + a.half_extent.y, b.center.y + b.half_extent.y)};
    return Aabb2D{(lo + hi) * 0.5f, (hi - lo) * 0.5f};
}

// The axis-aligned box that contains the shape at every orientation in the turn
// from `start_rotation` to `end_rotation`.
//
// A rotating body's corners sweep an arc that the two endpoint boxes alone do
// not bound -- an intermediate orientation can reach past both -- so the arc is
// sampled. The bound only has to be conservative (a superset of the true swept
// region), which this is: it contains the shape at every sampled angle, and the
// samples are dense enough that the unsampled sliver between them is far smaller
// than any wall the pass is meant to catch.
Aabb2D arc_bounds(const Collider2D& collider, const Vec2& center, f32 start_rotation, f32 end_rotation) noexcept {
    constexpr int kSamples = 4;
    Aabb2D bound = collider.bounds(center, start_rotation);
    for (int s = 1; s <= kSamples; ++s) {
        const f32 t = static_cast<f32>(s) / static_cast<f32>(kSamples);
        const Aabb2D cur = collider.bounds(center, start_rotation + (end_rotation - start_rotation) * t);
        bound = union_bounds(bound, cur);
    }
    return bound;
}

// The earliest fraction of a body's turn at which its shape first reaches
// `target`.
//
// A rotating body sweeps an arc the linear test cannot see: it can turn a corner
// through a thin wall while its centre barely moves, so the discrete pass -- and
// the linear sweep, which follows only the centre -- both miss it. The turn is
// cut into a fixed number of substeps; within each, the arc the shape sweeps is
// bounded and tested against the target. The first substep whose arc reaches the
// target is the contact. A contact already present at the start of the turn is
// left to the discrete solver, exactly as the linear sweep leaves a resting body.
struct RotationSweep2D {
    bool hit = false;
    f32 time = 0.0f;
};

RotationSweep2D sweep_rotation(const Collider2D& collider, const Vec2& center, f32 start_rotation, f32 rotation_delta,
                               const Collider2D& target, const Vec2& target_center, f32 target_rotation) noexcept {
    const Aabb2D target_bounds = target.bounds(target_center, target_rotation);
    // A shape already touching the target at the start of the turn is a resting
    // contact: the discrete solver owns it, and clamping the turn here would
    // freeze a body spinning against the surface it rests on.
    if (bounds_overlap(collider.bounds(center, start_rotation), target_bounds)) {
        return {};
    }
    // Enough substeps that a corner's arc is covered without a full continuous
    // solve; the caller's threshold guarantees the arc exceeds the body's own
    // size, so a coarse cut still catches a wall the body could tunnel.
    constexpr int kSubsteps = 16;
    for (int s = 1; s <= kSubsteps; ++s) {
        const f32 t = static_cast<f32>(s) / static_cast<f32>(kSubsteps);
        const f32 prev_t = static_cast<f32>(s - 1) / static_cast<f32>(kSubsteps);
        const Aabb2D swept =
            arc_bounds(collider, center, start_rotation + rotation_delta * prev_t, start_rotation + rotation_delta * t);
        if (bounds_overlap(swept, target_bounds)) {
            return RotationSweep2D{true, t};
        }
    }
    return {};
}

} // namespace

void Physics2DSystem::apply_continuous_collisions(f32 delta_time) {
    (void)delta_time;
    continuous_hits_ = 0;
    for (usize i = 0; i < bodies_.size(); ++i) {
        Body& mover = bodies_[i];
        if (mover.inv_mass <= 0.0f || asleep_[i] || mover.collider == nullptr) {
            continue;
        }
        const Vec2 start = previous_positions_[i];
        const Vec2 displacement = mover.position - start;
        const f32 start_rotation = previous_rotations_[i];
        const f32 rotation_delta = mover.rotation - start_rotation;

        // Only a body moving a meaningful fraction of its own size in one step
        // can tunnel: the discrete pass sees both endpoints, so a slow body is
        // caught by it. Sweeping every resting body as well would double the
        // cost and, worse, clamp a body that is merely settling against the
        // ground. The threshold scales with the body, so it holds in any world
        // unit.
        const f32 size = mover.collider->shape == ShapeType::Circle
                             ? mover.collider->radius
                             : std::min(mover.collider->half_extent.x, mover.collider->half_extent.y);
        const bool linear_motion = displacement.length_squared() > size * size;

        // A spinning box can sweep a corner through a wall while its centre
        // barely moves, which the linear sweep cannot see. The corner's arc
        // length is the turn times the shape's bounding radius, so only a turn
        // whose arc exceeds the body's own size can tunnel. A circle's shape does
        // not change as it turns, so rotation alone can never tunnel one.
        const f32 spin_radius = mover.collider->half_extent.length();
        const bool rotational_motion =
            mover.collider->shape == ShapeType::Box && std::abs(rotation_delta) * spin_radius > size;
        if (!linear_motion && !rotational_motion) {
            continue;
        }

        f32 linear_earliest = 1.0f;
        Vec2 linear_normal{};
        f32 linear_restitution = 0.0f;
        bool linear_hit = false;
        f32 rotational_earliest = 1.0f;
        bool rotational_hit = false;
        for (usize j = 0; j < bodies_.size(); ++j) {
            if (j == i) {
                continue;
            }
            const Body& target = bodies_[j];
            if (target.collider == nullptr) {
                continue;
            }
            // Only static geometry is swept against for now. A moving pair is
            // still resolved by the discrete pass; sweeping both would need
            // their relative motion, which is a later refinement.
            if (target.inv_mass > 0.0f) {
                continue;
            }
            if (!mover.collider->can_collide_with(*target.collider)) {
                continue;
            }
            if (mover.collider->is_trigger || target.collider->is_trigger) {
                continue; // a sensor does not stop motion
            }

            if (linear_motion) {
                const Aabb2D mover_bounds = mover.collider->bounds(start, start_rotation);
                const Aabb2D target_bounds = target.collider->bounds(target.position, target.rotation);
                const Sweep2D sweep = sweep_shape(mover_bounds, displacement, target_bounds);
                // A sweep that starts already touching (time zero) is a resting
                // contact, not a crossing: the discrete solver owns it, and
                // clamping here would freeze a body sliding along the ground it
                // rests on. Only a contact first reached partway through the
                // motion is a tunnelling case the continuous pass must stop.
                if (sweep.hit && sweep.time > 0.0f && sweep.time < linear_earliest) {
                    linear_earliest = sweep.time;
                    linear_normal = sweep.normal;
                    linear_restitution = std::max(mover.restitution, target.restitution);
                    linear_hit = true;
                }
            }

            if (rotational_motion) {
                const RotationSweep2D spin = sweep_rotation(*mover.collider, start, start_rotation, rotation_delta,
                                                            *target.collider, target.position, target.rotation);
                if (spin.hit && spin.time < rotational_earliest) {
                    rotational_earliest = spin.time;
                    rotational_hit = true;
                }
            }
        }

        if (!linear_hit && !rotational_hit) {
            continue;
        }

        // Clamp the motion to the first touch, leaving the body at the surface,
        // and apply the same restitution rule the discrete solver uses to the
        // velocity component into the surface. Without this the body would
        // accumulate speed through the wall and be even deeper next step.
        if (linear_hit) {
            mover.position = start + displacement * linear_earliest;
            const f32 into = mover.velocity.dot(linear_normal);
            if (into < 0.0f) {
                mover.velocity -= linear_normal * ((1.0f + linear_restitution) * into);
            }
        }
        if (rotational_hit) {
            // Clamp the turn to the first touch: the corner cannot sweep past
            // the wall. The spin is shed -- a wall stops the rotation into it --
            // while any linear motion is left to the discrete solver.
            mover.rotation = start_rotation + rotation_delta * rotational_earliest;
            mover.angular_velocity = 0.0f;
        }
        ++continuous_hits_;
    }
}

void Physics2DSystem::solve_joints(Scene& scene, f32 delta_time) {
    if (joints_.empty()) {
        return;
    }
    if (delta_time <= 0.0f) {
        return;
    }

    for (Joint& joint : joints_) {
        // A joint that has broken is skipped entirely: it applied its final
        // correction the step it snapped, and from now on constrains nothing.
        if (joint.is_broken()) {
            continue;
        }

        // Resolve the two ends. A nil, missing, or self-referential reference is
        // a broken joint: record why and skip it rather than constraining
        // nothing or a body against itself.
        const Uuid& id_a = joint.body_a();
        const Uuid& id_b = joint.body_b();
        const std::string_view type = joint.type_name();

        if (id_a.is_nil() || id_b.is_nil()) {
            note_joint_skipped(type, id_a, id_b, Error("joint body reference is nil"));
            continue;
        }
        if (id_a == id_b) {
            note_joint_skipped(type, id_a, id_b, Error("joint connects a body to itself"));
            continue;
        }
        joint.a = scene.find_node(id_a);
        joint.b = scene.find_node(id_b);
        if (joint.a == nullptr || joint.b == nullptr) {
            note_joint_skipped(type, id_a, id_b, Error("joint body is not in the scene"));
            continue;
        }

        JointBody body_a = make_joint_body(*joint.a);
        JointBody body_b = make_joint_body(*joint.b);
        if (body_a.inv_mass <= 0.0f && body_b.inv_mass <= 0.0f) {
            // Two immovable ends: the constraint can never be satisfied by
            // moving anything, so report it and leave the world alone.
            note_joint_skipped(type, id_a, id_b, Error("joint has no movable body"));
            continue;
        }

        // A sleeping end behaves as an immovable one: the joint is still
        // enforced -- so joint_count() reports it and the row is built -- but it
        // cannot move the sleeper. The wake pass already woke the island if the
        // other end was moving, so a joint is only skipped here when the whole
        // island is asleep, in which case there is nothing to correct. This is
        // what keeps a pendulum that has come to rest reporting its constraint
        // rather than vanishing from the count.
        const auto index_a = body_index_.find(id_a);
        const auto index_b = body_index_.find(id_b);
        if (index_a != body_index_.end() && asleep_[index_a->second]) {
            body_a.inv_mass = 0.0f;
            body_a.inv_inertia = 0.0f;
        }
        if (index_b != body_index_.end() && asleep_[index_b->second]) {
            body_b.inv_mass = 0.0f;
            body_b.inv_inertia = 0.0f;
        }
        if (body_a.inv_mass <= 0.0f && body_b.inv_mass <= 0.0f) {
            // Both ends are immovable or asleep: nothing to correct, but the
            // joint is still counted (it was gathered and is not broken).
            ++joints_solved_;
            continue;
        }

        const f32 inv_dt = 1.0f / delta_time;
        Vec2 reaction_force{};
        f32 reaction_torque = 0.0f;
        f32 peak_force = 0.0f;

        if (joint.distance != nullptr) {
            DistanceJointParams params;
            // A positive rest length pins that length. Zero or negative means
            // "hold the separation the two bodies have right now", which is the
            // usual intent when a joint is dropped onto an already-placed pair;
            // it is recomputed each step, so it stays a constant-length rod.
            params.rest_length = joint.distance->rest_length > 0.0f ? joint.distance->rest_length
                                                                    : (body_b.position - body_a.position).length();
            params.stiffness = joint.distance->stiffness;
            params.damping = std::clamp(joint.distance->damping, 0.0f, 1.0f);
            params.min_length = joint.distance->min_length;
            params.max_length = joint.distance->max_length;

            auto row = build_distance_row(body_a, body_b, params, delta_time);
            if (!row) {
                note_joint_skipped(type, id_a, id_b, row.error());
                continue;
            }
            ConstraintRow distance_row = row.value();
            apply_row(distance_row, body_a, body_b);
            apply_row_position(distance_row, body_a, body_b);
            row_reactions(&distance_row, 1, inv_dt, reaction_force, reaction_torque, peak_force);
        } else if (joint.revolute != nullptr) {
            RevoluteJointParams params;
            params.anchor_a = joint.revolute->anchor_a;
            params.anchor_b = joint.revolute->anchor_b;

            ConstraintRow rows[kRevoluteJointRows];
            if (auto built = build_revolute_rows(body_a, body_b, params, delta_time, rows); !built) {
                note_joint_skipped(type, id_a, id_b, built.error());
                continue;
            }
            apply_rows(rows, kRevoluteJointRows, body_a, body_b);

            // The motor drives the relative spin the pin leaves free. It runs
            // after the anchor rows so the spin is applied to the pose the pin
            // just corrected. A torque is a per-step impulse bound: the row's
            // clamp is `torque * delta_time`, which is what makes the authored
            // value a torque rather than a step-dependent impulse.
            JointMotor motor;
            motor.target_speed = joint.revolute->motor_speed;
            motor.max_impulse = joint.revolute->motor_max_torque * delta_time;
            ConstraintRow motor_row{};
            if (motor.is_enabled()) {
                if (auto row = build_revolute_motor_row(body_a, body_b, motor); row) {
                    motor_row = row.value();
                    apply_row(motor_row, body_a, body_b);
                } else {
                    note_joint_skipped(type, id_a, id_b, row.error());
                }
            }
            row_reactions(rows, kRevoluteJointRows, inv_dt, reaction_force, reaction_torque, peak_force);
            f32 motor_peak = 0.0f;
            Vec2 motor_force{};
            f32 motor_torque = 0.0f;
            row_reactions(&motor_row, 1, inv_dt, motor_force, motor_torque, motor_peak);
            reaction_force += motor_force;
            reaction_torque += motor_torque;
            peak_force = std::max(peak_force, motor_peak);
        } else if (joint.fixed != nullptr) {
            FixedJointParams params;
            params.anchor_a = joint.fixed->anchor_a;
            params.anchor_b = joint.fixed->anchor_b;
            params.reference_angle = joint.fixed->reference_angle;

            ConstraintRow rows[kFixedJointRows];
            if (auto built = build_fixed_rows(body_a, body_b, params, delta_time, rows); !built) {
                note_joint_skipped(type, id_a, id_b, built.error());
                continue;
            }
            apply_rows(rows, kFixedJointRows, body_a, body_b);
            row_reactions(rows, kFixedJointRows, inv_dt, reaction_force, reaction_torque, peak_force);
        } else {
            PrismaticJointParams params;
            params.anchor_a = joint.prismatic->anchor_a;
            params.anchor_b = joint.prismatic->anchor_b;
            params.axis = joint.prismatic->axis;
            params.reference_angle = joint.prismatic->reference_angle;
            params.enable_limits = joint.prismatic->enable_limits;
            params.lower_translation = joint.prismatic->lower_translation;
            params.upper_translation = joint.prismatic->upper_translation;

            ConstraintRow rows[kPrismaticJointRows];
            if (auto built = build_prismatic_rows(body_a, body_b, params, delta_time, rows); !built) {
                note_joint_skipped(type, id_a, id_b, built.error());
                continue;
            }
            apply_rows(rows, kPrismaticJointRows, body_a, body_b);

            // The linear motor drives the slide the joint leaves free. A force is
            // a per-step impulse bound: `force * delta_time`, which makes the
            // authored value a force and not a step-dependent impulse.
            JointMotor motor;
            motor.target_speed = joint.prismatic->motor_speed;
            motor.max_impulse = joint.prismatic->motor_max_force * delta_time;
            ConstraintRow motor_row{};
            if (motor.is_enabled()) {
                if (auto row = build_prismatic_motor_row(body_a, body_b, params, motor); row) {
                    motor_row = row.value();
                    apply_row(motor_row, body_a, body_b);
                }
            }
            row_reactions(rows, kPrismaticJointRows, inv_dt, reaction_force, reaction_torque, peak_force);
            f32 motor_peak = 0.0f;
            Vec2 motor_force{};
            f32 motor_torque = 0.0f;
            row_reactions(&motor_row, 1, inv_dt, motor_force, motor_torque, motor_peak);
            reaction_force += motor_force;
            reaction_torque += motor_torque;
            peak_force = std::max(peak_force, motor_peak);
        }

        joint.reaction_force = reaction_force;
        joint.reaction_torque = reaction_torque;
        joint.peak_reaction_force = peak_force;

        // A break threshold snaps the joint: it stops being enforced from the
        // next step on, so the bodies part under a load they could not carry. A
        // joint with both thresholds at zero never breaks.
        const f32 break_force = joint.break_force();
        const f32 break_torque = joint.break_torque();
        if ((break_force > 0.0f && peak_force > break_force) ||
            (break_torque > 0.0f && std::abs(reaction_torque) > break_torque)) {
            joint.set_broken(true);
        }

        store_joint_body(*joint.a, body_a);
        store_joint_body(*joint.b, body_b);
        ++joints_solved_;
    }
}

void Physics2DSystem::note_joint_skipped(std::string_view type, const Uuid& body_a, const Uuid& body_b, Error error) {
    joint_diagnostics_.push_back(JointDiagnostic{type, body_a, body_b, std::move(error)});
}

Physics2DSystem::JointReaction Physics2DSystem::joint_reaction(const Uuid& body_a, const Uuid& body_b) const noexcept {
    for (const Joint& joint : joints_) {
        if (joint.body_a() == body_a && joint.body_b() == body_b) {
            return JointReaction{joint.reaction_force, joint.reaction_torque, joint.peak_reaction_force};
        }
    }
    return JointReaction{};
}

bool Physics2DSystem::same_island(const Uuid& a, const Uuid& b) const {
    const auto index_a = body_index_.find(a);
    const auto index_b = body_index_.find(b);
    if (index_a == body_index_.end() || index_b == body_index_.end()) {
        return false;
    }
    const usize vertex_a = body_vertex_[index_a->second];
    const usize vertex_b = body_vertex_[index_b->second];
    if (vertex_a == kNoVertex || vertex_b == kNoVertex) {
        return false;
    }
    return islands_.same_island(vertex_a, vertex_b);
}

void Physics2DSystem::build_islands() {
    // Only dynamic bodies are island vertices: static geometry is collided with
    // but never conducts motion between the bodies resting on it, so a link to a
    // static body is dropped. That is what makes a crate on the left of a floor
    // and a crate on the right separate islands even though both touch it.
    island_vertex_body_.clear();
    body_vertex_.assign(bodies_.size(), kNoVertex);
    for (usize i = 0; i < bodies_.size(); ++i) {
        if (bodies_[i].inv_mass > 0.0f) {
            body_vertex_[i] = island_vertex_body_.size();
            island_vertex_body_.push_back(i);
        }
    }

    // Edges. A link is a pair of dynamic bodies whose bounds overlap (the broad
    // phase's notion of "may be touching") or a joint between two dynamic
    // bodies. The bounds test is deliberately conservative: it may join two
    // bodies that are near but not quite touching, which only makes sleeping
    // more conservative (the group wakes and sleeps together), never less safe.
    // Using bounds rather than the exact contact is what lets the graph be built
    // before the solver runs, so a moving body wakes a sleeper in the same step
    // it reaches it.
    //
    // This enumeration is separate from the solver's uniform-grid broad phase
    // (ADR-0023): the island graph is built before integration, over the
    // pre-step bounds, while the broad phase is built after it, over the
    // post-step bounds. It is O(n^2) and is the next thing to fold into the
    // broad phase if the two passes are ever brought to the same point in the
    // step. The order is bodies in array order, so the link order -- and
    // therefore the islands -- is deterministic.
    island_links_.clear();
    // Only node bodies are considered: a tile body is static (never a vertex),
    // so including it would only add bounds tests that can never produce a
    // link. Bounding the scan at `node_body_count_` keeps the O(n^2) pass off a
    // map's thousands of tiles.
    for (usize i = 0; i < node_body_count_; ++i) {
        if (body_vertex_[i] == kNoVertex || bodies_[i].collider == nullptr) {
            continue;
        }
        const Aabb2D bounds_a = bodies_[i].collider->bounds(bodies_[i].position, bodies_[i].rotation);
        for (usize j = i + 1; j < node_body_count_; ++j) {
            if (body_vertex_[j] == kNoVertex || bodies_[j].collider == nullptr) {
                continue;
            }
            const Aabb2D bounds_b = bodies_[j].collider->bounds(bodies_[j].position, bodies_[j].rotation);
            if (bounds_overlap(bounds_a, bounds_b)) {
                island_links_.push_back(IslandLink{body_vertex_[i], body_vertex_[j]});
            }
        }
    }

    // Joints are resolved here from the node index rather than from the Joint
    // struct's cached ends, which solve_joints fills later in the step. A joint
    // end that is nil, missing, or not a simulated dynamic body contributes no
    // link (a joint to an immovable pivot does not join two islands).
    for (const Joint& joint : joints_) {
        const Uuid& id_a = joint.body_a();
        const Uuid& id_b = joint.body_b();
        const auto index_a = body_index_.find(id_a);
        const auto index_b = body_index_.find(id_b);
        if (index_a == body_index_.end() || index_b == body_index_.end()) {
            continue;
        }
        const usize vertex_a = body_vertex_[index_a->second];
        const usize vertex_b = body_vertex_[index_b->second];
        if (vertex_a == kNoVertex || vertex_b == kNoVertex) {
            continue;
        }
        island_links_.push_back(IslandLink{vertex_a, vertex_b});
    }

    islands_.build(island_vertex_body_.size(), island_links_);

    // Seed the awake flags from the bodies' current state. A body that is awake
    // (including one that can never sleep) makes its island awake.
    island_awake_.assign(islands_.island_count(), false);
    for (usize vertex = 0; vertex < island_vertex_body_.size(); ++vertex) {
        if (!asleep_[island_vertex_body_[vertex]]) {
            island_awake_[islands_.island_of_body()[vertex]] = true;
        }
    }
}

void Physics2DSystem::wake_island(usize index) {
    if (index >= islands_.island_count()) {
        return;
    }
    island_awake_[index] = true;
    for (const usize vertex : islands_.islands()[index]) {
        const usize body = island_vertex_body_[vertex];
        if (asleep_[body]) {
            asleep_[body] = false;
            --sleeping_bodies_;
            // Clear the body's own sleeping flag and restart its rest timer. An
            // already-awake body in the island keeps its timer, so a body that
            // has been settling is not sent back to the start by a neighbour
            // being woken.
            if (bodies_[body].body != nullptr) {
                bodies_[body].body->wake();
            }
        }
    }
}

void Physics2DSystem::apply_wake_checks() {
    // A contact or a joint to an *awake* dynamic body wakes the island. The
    // island awake flags were seeded from the bodies above, so a flag is true
    // exactly when some body in it was already awake; here that wake is
    // propagated across the whole island, which is what wakes a sleeping body
    // joined to a moving one or resting against a moving one.
    //
    // Because the graph already links the pair that is touching this step, a
    // moving body wakes a sleeper before the solver runs: the solver can never
    // move an awake body into a still-sleeping one.
    for (usize island = 0; island < islands_.island_count(); ++island) {
        if (island_awake_[island]) {
            wake_island(island);
        }
    }
}

void Physics2DSystem::apply_sleep_checks(f32 delta_time) {
    for (usize island = 0; island < islands_.island_count(); ++island) {
        if (island_awake_[island]) {
            // An awake island advances each body's rest timer. A body that is
            // still moving resets its own timer; a body below the thresholds
            // accumulates. The island may only sleep when every body in it has
            // accumulated long enough and can_sleep.
            bool ready = true;
            for (const usize vertex : islands_.islands()[island]) {
                const usize body_index = island_vertex_body_[vertex];
                RigidBody2D* component = bodies_[body_index].body;
                if (component == nullptr) {
                    continue;
                }
                if (!component->can_sleep) {
                    ready = false;
                    component->sleep_timer = 0.0f;
                    continue;
                }
                // The write-back above has already synced the component's
                // velocity, so the component's own rest test is current.
                if (component->is_resting()) {
                    component->sleep_timer += delta_time;
                } else {
                    component->sleep_timer = 0.0f;
                }
                if (component->sleep_timer < component->sleep_time) {
                    ready = false;
                }
            }
            if (ready) {
                // Every body in the island has been still long enough: sleep the
                // whole island at once, so the group stops as a unit rather than
                // one body at a time.
                for (const usize vertex : islands_.islands()[island]) {
                    const usize body_index = island_vertex_body_[vertex];
                    RigidBody2D* component = bodies_[body_index].body;
                    if (component == nullptr) {
                        continue;
                    }
                    if (!asleep_[body_index]) {
                        asleep_[body_index] = true;
                        ++sleeping_bodies_;
                    }
                    component->sleep();
                }
                island_awake_[island] = false;
            }
        }
    }
}

void Physics2DSystem::reconcile_contacts(Scene& scene) {
    // Diff the pairs that touch now against those that touched last frame.
    // Sorting both makes the diff a linear merge rather than an O(n^2) scan, and
    // makes the emitted order deterministic.
    std::sort(active_pairs_.begin(), active_pairs_.end());
    std::sort(previous_pairs_.begin(), previous_pairs_.end());

    usize i = 0;
    usize j = 0;
    while (i < active_pairs_.size() && j < previous_pairs_.size()) {
        if (active_pairs_[i] == previous_pairs_[j]) {
            ++i;
            ++j;
        } else if (active_pairs_[i] < previous_pairs_[j]) {
            append_event(ContactPhase::Begin, active_pairs_[i], scene);
            ++i;
        } else {
            append_event(ContactPhase::End, previous_pairs_[j], scene);
            ++j;
        }
    }
    for (; i < active_pairs_.size(); ++i) {
        append_event(ContactPhase::Begin, active_pairs_[i], scene);
    }
    for (; j < previous_pairs_.size(); ++j) {
        append_event(ContactPhase::End, previous_pairs_[j], scene);
    }

    previous_pairs_.swap(active_pairs_);
}

void Physics2DSystem::append_event(ContactPhase phase, const BodyPair& pair, Scene& scene) {
    ContactEvent event;
    event.phase = phase;
    event.a = scene.find_node(pair.first);
    event.b = scene.find_node(pair.second);
    // A node that left the scene between the previous frame and this one cannot
    // be resolved; skip the event rather than reporting a dangling pointer.
    if (event.a == nullptr || event.b == nullptr) {
        return;
    }
    events_.push_back(event);
}

} // namespace fe::physics2d
