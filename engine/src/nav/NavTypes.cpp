// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/nav/NavTypes.h"

#include "fe/nav/NavAgent.h"
#include "fe/reflect/PropertyBuilders.h"
#include "fe/reflect/Registry.h"

#include <memory>

// The nav component registrations, in the same style as PhysicsTypes.cpp: each
// block describes a component's properties once, and the serializer and the
// inspector read those descriptions rather than naming the types. The runtime
// path and the published velocity are deliberately not properties, so a saved
// scene does not reload mid-path.

namespace fe::reflect {
namespace {

void register_nav_agent_2d(Registry& registry) {
    nav::NavAgent2D defaults;

    TypeDesc desc;
    desc.name = nav::NavAgent2D::kTypeName;
    desc.factory = [] { return std::unique_ptr<Component>(std::make_unique<nav::NavAgent2D>()); };

    desc.properties.push_back(
        make_scalar<nav::NavAgent2D, Uuid>("grid_node", &nav::NavAgent2D::grid_node, ValueKind::Uuid, &defaults));
    desc.properties.push_back(
        make_scalar<nav::NavAgent2D, Uuid>("goal_node", &nav::NavAgent2D::goal_node, ValueKind::Uuid, &defaults));
    desc.properties.push_back(
        make_vector_group<nav::NavAgent2D, Vec2>("goal", &nav::NavAgent2D::goal, "Vec2", {"x", "y"}));
    desc.properties.push_back(
        make_scalar<nav::NavAgent2D, bool>("has_goal", &nav::NavAgent2D::has_goal, ValueKind::Bool, &defaults));
    desc.properties.push_back(
        make_scalar<nav::NavAgent2D, bool>("auto_move", &nav::NavAgent2D::auto_move, ValueKind::Bool, &defaults));
    desc.properties.push_back(
        make_scalar<nav::NavAgent2D, f32>("speed", &nav::NavAgent2D::speed, ValueKind::Float, &defaults));
    desc.properties.push_back(
        make_scalar<nav::NavAgent2D, f32>("acceleration", &nav::NavAgent2D::acceleration, ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<nav::NavAgent2D, f32>("waypoint_radius", &nav::NavAgent2D::waypoint_radius,
                                                                ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<nav::NavAgent2D, f32>("arrive_radius", &nav::NavAgent2D::arrive_radius,
                                                                ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<nav::NavAgent2D, f32>(
        "separation_weight", &nav::NavAgent2D::separation_weight, ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<nav::NavAgent2D, f32>("avoidance_weight", &nav::NavAgent2D::avoidance_weight,
                                                                ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<nav::NavAgent2D, f32>(
        "separation_radius", &nav::NavAgent2D::separation_radius, ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<nav::NavAgent2D, f32>("avoidance_radius", &nav::NavAgent2D::avoidance_radius,
                                                                ValueKind::Float, &defaults));

    registry.register_type(std::move(desc));
}

void register_nav_agent_3d(Registry& registry) {
    nav::NavAgent3D defaults;

    TypeDesc desc;
    desc.name = nav::NavAgent3D::kTypeName;
    desc.factory = [] { return std::unique_ptr<Component>(std::make_unique<nav::NavAgent3D>()); };

    desc.properties.push_back(
        make_scalar<nav::NavAgent3D, Uuid>("goal_node", &nav::NavAgent3D::goal_node, ValueKind::Uuid, &defaults));
    desc.properties.push_back(
        make_vector_group<nav::NavAgent3D, Vec3>("goal", &nav::NavAgent3D::goal, "Vec3", {"x", "y", "z"}));
    desc.properties.push_back(
        make_scalar<nav::NavAgent3D, bool>("has_goal", &nav::NavAgent3D::has_goal, ValueKind::Bool, &defaults));
    desc.properties.push_back(
        make_scalar<nav::NavAgent3D, bool>("auto_move", &nav::NavAgent3D::auto_move, ValueKind::Bool, &defaults));
    desc.properties.push_back(
        make_scalar<nav::NavAgent3D, f32>("speed", &nav::NavAgent3D::speed, ValueKind::Float, &defaults));
    desc.properties.push_back(
        make_scalar<nav::NavAgent3D, f32>("acceleration", &nav::NavAgent3D::acceleration, ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<nav::NavAgent3D, f32>("waypoint_radius", &nav::NavAgent3D::waypoint_radius,
                                                                ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<nav::NavAgent3D, f32>("arrive_radius", &nav::NavAgent3D::arrive_radius,
                                                                ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<nav::NavAgent3D, f32>(
        "separation_weight", &nav::NavAgent3D::separation_weight, ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<nav::NavAgent3D, f32>("avoidance_weight", &nav::NavAgent3D::avoidance_weight,
                                                                ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<nav::NavAgent3D, f32>(
        "separation_radius", &nav::NavAgent3D::separation_radius, ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<nav::NavAgent3D, f32>("avoidance_radius", &nav::NavAgent3D::avoidance_radius,
                                                                ValueKind::Float, &defaults));

    registry.register_type(std::move(desc));
}

} // namespace

void register_nav_types() {
    Registry& registry = Registry::instance();
    register_nav_agent_2d(registry);
    register_nav_agent_3d(registry);
}

} // namespace fe::reflect
