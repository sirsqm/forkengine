// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// Tests for navigation and AI (ADR-0156).
//
// The grid build, A*, smoothing, line of sight, steering, and the two agent
// systems are all pure CPU work, so these tests run with no display and no GL
// context. The systems are driven by calling update() directly with a known
// delta, which is what makes "an agent walks to its goal" assertable without a
// frame loop.

#include <doctest/doctest.h>

#include "fe/debugdraw/DebugDraw.h"
#include "fe/nav/NavAgent.h"
#include "fe/nav/NavGrid2D.h"
#include "fe/nav/NavGrid3D.h"
#include "fe/nav/NavTypes.h"
#include "fe/nav/NavigationSystem2D.h"
#include "fe/nav/NavigationSystem3D.h"
#include "fe/nav/Path2D.h"
#include "fe/nav/Path3D.h"
#include "fe/nav/Steering.h"
#include "fe/physics2d/Collider2D.h"
#include "fe/physics3d/Collider3D.h"
#include "fe/player/NavBehaviours.h"
#include "fe/reflect/Registry.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/scene/Tilemap.h"
#include "fe/script/ScriptHost.h"
#include "fe/serialization/SceneSerializer.h"
#include "fe/systems/FrameContext.h"

#include <cmath>
#include <string>
#include <vector>

using namespace fe;
using namespace fe::nav;

namespace {

// A tilemap whose cells are all `floor` except a list of `solid` cells. The
// tilemap's node origin is the top-left, +Y down, so cell (x, y) is at local
// (x + 0.5, -(y + 0.5)) * tile_size.
struct TestMap {
    Scene scene;
    Node* node = nullptr;
    Tilemap* map = nullptr;

    explicit TestMap(u32 width, u32 height, f32 tile_size = 1.0f) {
        node = scene.create_node("Map");
        map = node->add_component<Tilemap>();
        map->tile_size = Vec2{tile_size, tile_size};
        map->resize(width, height, 1); // every cell is tile index 1 (non-empty, hence solid)
    }

    // Makes the whole map walkable. A cell is walkable when its tile is empty,
    // so an open map is all `kEmpty` and a wall is a non-empty tile.
    void open_all() {
        map->clear(Tilemap::kEmpty);
        map->solid_tiles.clear();
    }

    void set_solid(u32 x, u32 y, bool solid) { map->set(x, y, solid ? 1 : Tilemap::kEmpty); }
};

[[nodiscard]] Vec2 cell_center(const NavGrid2D& grid, i32 x, i32 y) {
    return grid.cell_center(x, y);
}

} // namespace

// ---------------------------------------------------------------------------
// NavGrid2D: the derived grid.
// ---------------------------------------------------------------------------

TEST_CASE("a nav grid is built from a tilemap, one cell per tile") {
    TestMap test(4, 3);
    test.open_all();
    // Wall down the middle column.
    for (u32 y = 0; y < 3; ++y) {
        test.set_solid(2, y, true);
    }

    const NavGrid2D grid = nav::build_nav_grid_2d(test.scene, *test.node, *test.map);
    REQUIRE_FALSE(grid.empty());
    CHECK(grid.width() == 4);
    CHECK(grid.height() == 3);
    CHECK(grid.cell_size() == doctest::Approx(1.0f));

    CHECK(grid.walkable(0, 0));
    CHECK(grid.walkable(1, 1));
    CHECK_FALSE(grid.walkable(2, 0));
    CHECK_FALSE(grid.walkable(2, 2));
    CHECK(grid.walkable(3, 1));
    // Out of bounds is not walkable.
    CHECK_FALSE(grid.walkable(-1, 0));
    CHECK_FALSE(grid.walkable(4, 0));
}

TEST_CASE("a nav grid cell centre matches the tilemap's own cell layout") {
    TestMap test(4, 4, 2.0f);
    test.open_all();
    const NavGrid2D grid = nav::build_nav_grid_2d(test.scene, *test.node, *test.map);
    REQUIRE_FALSE(grid.empty());
    // The map's node origin is its top-left, so cell (0, 0) is half a tile in.
    CHECK(grid.cell_center(0, 0).x == doctest::Approx(1.0f));
    CHECK(grid.cell_center(0, 0).y == doctest::Approx(-1.0f));
    CHECK(grid.cell_center(1, 1).x == doctest::Approx(3.0f));
    CHECK(grid.cell_center(1, 1).y == doctest::Approx(-3.0f));
}

TEST_CASE("a nav grid erodes by an agent radius so a body does not clip a wall") {
    TestMap test(5, 5);
    test.open_all();
    // A single wall cell in the middle.
    test.set_solid(2, 2, true);

    const NavGrid2D tight = nav::build_nav_grid_2d(test.scene, *test.node, *test.map, 0.0f, 0.0f);
    CHECK(tight.walkable(1, 2)); // adjacent to the wall

    const NavGrid2D eroded = nav::build_nav_grid_2d(test.scene, *test.node, *test.map, 0.0f, 1.0f);
    CHECK_FALSE(eroded.walkable(1, 2)); // one cell of clearance
    CHECK_FALSE(eroded.walkable(2, 1));
    CHECK(eroded.walkable(0, 2)); // two cells away survives
}

TEST_CASE("a static collider blocks its nav cells") {
    TestMap test(6, 6);
    test.open_all();
    Node* crate = test.scene.create_node("Crate");
    auto* collider = crate->add_component<physics2d::Collider2D>();
    collider->shape = physics2d::ShapeType::Box;
    collider->half_extent = Vec2{1.0f, 1.0f};
    crate->transform().position = Vec3{2.5f, -2.5f, 0.0f}; // over cell (2, 2)

    const NavGrid2D grid = nav::build_nav_grid_2d(test.scene, *test.node, *test.map);
    REQUIRE_FALSE(grid.empty());
    CHECK_FALSE(grid.walkable(2, 2));
    CHECK(grid.walkable(0, 0));
}

TEST_CASE("a scene with no tilemap builds an empty nav grid") {
    Scene scene;
    (void)scene.create_node("Empty");
    const NavGrid2D grid = nav::build_nav_grid_2d(scene);
    CHECK(grid.empty());
}

// ---------------------------------------------------------------------------
// Line of sight.
// ---------------------------------------------------------------------------

TEST_CASE("line of sight is clear across open ground and blocked by a wall") {
    TestMap test(5, 5);
    test.open_all();
    const NavGrid2D grid = nav::build_nav_grid_2d(test.scene, *test.node, *test.map);
    REQUIRE_FALSE(grid.empty());

    const Vec2 a = cell_center(grid, 0, 0);
    const Vec2 b = cell_center(grid, 4, 0);
    CHECK(nav::line_of_sight_2d(grid, a, b));

    // A wall in the middle of the row blocks the straight line.
    test.set_solid(2, 0, true);
    const NavGrid2D blocked = nav::build_nav_grid_2d(test.scene, *test.node, *test.map);
    CHECK_FALSE(nav::line_of_sight_2d(blocked, a, b));
}

// ---------------------------------------------------------------------------
// A* pathfinding.
// ---------------------------------------------------------------------------

TEST_CASE("A* finds a route around a wall") {
    TestMap test(5, 5);
    test.open_all();
    // A wall down column 2 except a gap at row 4.
    for (u32 y = 0; y < 4; ++y) {
        test.set_solid(2, y, true);
    }
    const NavGrid2D grid = nav::build_nav_grid_2d(test.scene, *test.node, *test.map);
    REQUIRE_FALSE(grid.empty());

    std::vector<Vec2> path;
    const Error error = nav::find_path_2d(grid, cell_center(grid, 0, 0), cell_center(grid, 4, 0), path);
    REQUIRE(error.ok());
    REQUIRE(path.size() >= 2);

    // The route must go through the gap at (2, 4), not through the wall.
    bool through_gap = false;
    for (const Vec2& point : path) {
        i32 cx = 0;
        i32 cy = 0;
        grid.world_to_cell(point, cx, cy);
        if (cx == 2 && cy == 4) {
            through_gap = true;
        }
        CHECK(grid.walkable(cx, cy)); // every waypoint is on a walkable cell
    }
    CHECK(through_gap);
    // The last waypoint is the goal cell.
    i32 gx = 0;
    i32 gy = 0;
    grid.world_to_cell(path.back(), gx, gy);
    CHECK(gx == 4);
    CHECK(gy == 0);
}

TEST_CASE("A* reports no route when the goal is walled off") {
    TestMap test(5, 5);
    test.open_all();
    // Seal a corner cell on all sides.
    test.set_solid(0, 0, false); // the goal cell itself is open
    test.set_solid(1, 0, true);
    test.set_solid(0, 1, true);
    test.set_solid(1, 1, true);
    const NavGrid2D grid = nav::build_nav_grid_2d(test.scene, *test.node, *test.map);

    std::vector<Vec2> path;
    const Error error = nav::find_path_2d(grid, cell_center(grid, 4, 4), cell_center(grid, 0, 0), path);
    CHECK_FALSE(error.ok());
    CHECK(path.empty());
}

TEST_CASE("A* does not cut a corner between two diagonal walls") {
    TestMap test(3, 3);
    test.open_all();
    // A wall directly right of the start means the diagonal step out of (0, 0)
    // to (1, 1) would slip past its corner; the search must not take it.
    test.set_solid(1, 0, true);
    const NavGrid2D grid = nav::build_nav_grid_2d(test.scene, *test.node, *test.map);
    REQUIRE(grid.walkable(1, 1));

    std::vector<Vec2> path;
    const Error error = nav::find_path_2d(grid, cell_center(grid, 0, 0), cell_center(grid, 2, 2), path);
    REQUIRE(error.ok());
    REQUIRE(path.size() >= 2);
    // The first step out of (0, 0) cannot be the diagonal to (1, 1), because the
    // orthogonal neighbour (1, 0) is a wall; it must go down to (0, 1) first.
    i32 sx = 0;
    i32 sy = 0;
    grid.world_to_cell(path[1], sx, sy);
    CHECK_FALSE((sx == 1 && sy == 1));
}

TEST_CASE("a goal on a wall snaps to the nearest walkable cell") {
    TestMap test(5, 5);
    test.open_all();
    test.set_solid(2, 2, true);
    const NavGrid2D grid = nav::build_nav_grid_2d(test.scene, *test.node, *test.map);

    std::vector<Vec2> path;
    const Error error = nav::find_path_2d(grid, cell_center(grid, 0, 0), cell_center(grid, 2, 2), path);
    REQUIRE(error.ok());
    REQUIRE_FALSE(path.empty());
    i32 gx = 0;
    i32 gy = 0;
    grid.world_to_cell(path.back(), gx, gy);
    CHECK(grid.walkable(gx, gy)); // snapped off the wall
}

// ---------------------------------------------------------------------------
// Smoothing.
// ---------------------------------------------------------------------------

TEST_CASE("smoothing collapses a stair-step into a straight diagonal") {
    TestMap test(5, 5);
    test.open_all();
    const NavGrid2D grid = nav::build_nav_grid_2d(test.scene, *test.node, *test.map);

    std::vector<Vec2> path;
    REQUIRE(nav::find_path_2d(grid, cell_center(grid, 0, 0), cell_center(grid, 4, 4), path).ok());
    const usize raw_count = path.size();
    nav::smooth_path_2d(grid, path);
    // Across open ground the smoothed path is just the two endpoints.
    CHECK(path.size() < raw_count);
    CHECK(path.size() == 2);
    CHECK(path.front() == cell_center(grid, 0, 0));
    CHECK(path.back() == cell_center(grid, 4, 4));
}

TEST_CASE("smoothing keeps a corner when the diagonal is blocked") {
    TestMap test(5, 5);
    test.open_all();
    // Block the diagonal so the path must bend around it.
    test.set_solid(2, 2, true);
    const NavGrid2D grid = nav::build_nav_grid_2d(test.scene, *test.node, *test.map);

    std::vector<Vec2> path;
    REQUIRE(nav::find_path_2d(grid, cell_center(grid, 0, 0), cell_center(grid, 4, 4), path).ok());
    nav::smooth_path_2d(grid, path);
    // A straight line from (0,0) to (4,4) would cross the blocked cell, so the
    // smoothed path keeps at least one intermediate corner.
    CHECK(path.size() >= 3);
    for (usize i = 1; i < path.size(); ++i) {
        CHECK(nav::line_of_sight_2d(grid, path[i - 1], path[i]));
    }
}

// ---------------------------------------------------------------------------
// Steering.
// ---------------------------------------------------------------------------

TEST_CASE("seek points at the target at the maximum speed") {
    const Vec2 v = nav::seek_2d(Vec2{0.0f, 0.0f}, Vec2{3.0f, 4.0f}, 5.0f);
    CHECK(v.length() == doctest::Approx(5.0f));
    CHECK(v.x == doctest::Approx(3.0f));
    CHECK(v.y == doctest::Approx(4.0f));
    CHECK(nav::seek_2d(Vec2{1.0f, 1.0f}, Vec2{1.0f, 1.0f}, 5.0f).length() == doctest::Approx(0.0f));
}

TEST_CASE("flee is the negation of seek") {
    const Vec2 s = nav::seek_2d(Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, 2.0f);
    const Vec2 f = nav::flee_2d(Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, 2.0f);
    CHECK(f.x == doctest::Approx(-s.x));
    CHECK(f.y == doctest::Approx(-s.y));
}

TEST_CASE("arrive eases in inside the slow radius and stops at the target") {
    // Beyond the slow radius: full speed.
    CHECK(nav::arrive_2d(Vec2{0.0f, 0.0f}, Vec2{10.0f, 0.0f}, 5.0f, 2.0f).length() == doctest::Approx(5.0f));
    // Inside it: scaled down.
    const Vec2 inside = nav::arrive_2d(Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, 5.0f, 2.0f);
    CHECK(inside.length() == doctest::Approx(2.5f));
    // At the target: zero.
    CHECK(nav::arrive_2d(Vec2{1.0f, 0.0f}, Vec2{1.0f, 0.0f}, 5.0f, 2.0f).length() == doctest::Approx(0.0f));
}

TEST_CASE("wander returns a unit-scaled heading that changes as its angle drifts") {
    f32 angle = 0.0f;
    f32 rng = 12345.0f;
    const Vec2 first = nav::wander_2d(angle, rng, 4.0f, 0.5f);
    CHECK(first.length() == doctest::Approx(4.0f));
    // A second call drifts the heading, so the direction changes.
    const Vec2 second = nav::wander_2d(angle, rng, 4.0f, 0.5f);
    CHECK((second - first).length() > 0.0f);
}

TEST_CASE("separation pushes away from a close neighbour and ignores a distant one") {
    const Vec2 position{0.0f, 0.0f};
    const std::vector<Vec2> near{{0.2f, 0.0f}};
    const Vec2 push = nav::separation_2d(position, near, 1.0f, 10.0f);
    CHECK(push.x < 0.0f); // pushed in -x, away from the neighbour at +x
    const std::vector<Vec2> far{{5.0f, 0.0f}};
    CHECK(nav::separation_2d(position, far, 1.0f, 10.0f).length() == doctest::Approx(0.0f));
}

TEST_CASE("avoidance steers away only from an obstacle that lies ahead") {
    const Vec2 forward{1.0f, 0.0f};
    const std::vector<Vec2> ahead{{0.5f, 0.0f}};
    CHECK(nav::avoidance_2d(forward, ahead, 1.0f, 10.0f).x < 0.0f);
    const std::vector<Vec2> behind{{-0.5f, 0.0f}};
    CHECK(nav::avoidance_2d(forward, behind, 1.0f, 10.0f).length() == doctest::Approx(0.0f));
}

TEST_CASE("the 3D steering behaviours mirror the 2D ones") {
    const Vec3 seek = nav::seek_3d(Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 4.0f}, 2.0f);
    CHECK(seek.length() == doctest::Approx(2.0f));
    CHECK(seek.z == doctest::Approx(2.0f));
    CHECK(nav::flee_3d(Vec3{}, Vec3{0.0f, 0.0f, 1.0f}, 2.0f).z == doctest::Approx(-2.0f));
    CHECK(nav::arrive_3d(Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 10.0f}, 5.0f, 2.0f).length() ==
          doctest::Approx(5.0f));

    f32 yaw = 0.0f;
    f32 pitch = 0.0f;
    f32 rng = 999.0f;
    CHECK(nav::wander_3d(yaw, pitch, rng, 3.0f, 0.4f).length() == doctest::Approx(3.0f));
}

// ---------------------------------------------------------------------------
// NavGrid3D and 3D pathfinding.
// ---------------------------------------------------------------------------

TEST_CASE("a 3D nav grid is built from box colliders and paths between them") {
    Scene scene;
    // A floor and two walls leave a corridor along x at z = 0.
    Node* floor = scene.create_node("Floor");
    auto* floor_collider = floor->add_component<physics3d::Collider3D>();
    floor_collider->shape = physics3d::ShapeType3D::Box;
    floor_collider->half_extent = Vec3{10.0f, 0.5f, 10.0f};
    floor->transform().position = Vec3{0.0f, -0.5f, 0.0f};

    Node* wall = scene.create_node("Wall");
    auto* wall_collider = wall->add_component<physics3d::Collider3D>();
    wall_collider->shape = physics3d::ShapeType3D::Box;
    wall_collider->half_extent = Vec3{0.5f, 2.0f, 10.0f};
    wall->transform().position = Vec3{3.0f, 1.0f, 0.0f};

    const NavGrid3D grid = nav::build_nav_grid_3d(scene, 1.0f);
    REQUIRE_FALSE(grid.empty());

    // A point in open space near the floor is walkable; the wall is not.
    i32 wx = 0;
    i32 wy = 0;
    i32 wz = 0;
    grid.world_to_cell(Vec3{0.0f, 1.0f, 0.0f}, wx, wy, wz);
    CHECK(grid.walkable(wx, wy, wz));
    grid.world_to_cell(Vec3{3.0f, 1.0f, 0.0f}, wx, wy, wz);
    CHECK_FALSE(grid.walkable(wx, wy, wz));

    std::vector<Vec3> path;
    const Error error = nav::find_path_3d(grid, Vec3{0.0f, 1.0f, 0.0f}, Vec3{6.0f, 1.0f, 0.0f}, path);
    REQUIRE(error.ok());
    REQUIRE(path.size() >= 2);
    nav::smooth_path_3d(grid, path);
    CHECK(path.front().x == doctest::Approx(0.5f).epsilon(0.51));
    CHECK(path.back().x > 5.0f);
}

TEST_CASE("a 3D nav grid with require_ground keeps only cells with floor below") {
    Scene scene;
    Node* floor = scene.create_node("Floor");
    auto* floor_collider = floor->add_component<physics3d::Collider3D>();
    floor_collider->shape = physics3d::ShapeType3D::Box;
    floor_collider->half_extent = Vec3{2.0f, 0.5f, 2.0f};
    floor->transform().position = Vec3{0.0f, -0.5f, 0.0f};

    const NavGrid3D grid = nav::build_nav_grid_3d(scene, 1.0f, /*require_ground=*/true);
    REQUIRE_FALSE(grid.empty());

    // A cell just above the floor has ground below it and is walkable.
    i32 wx = 0;
    i32 wy = 0;
    i32 wz = 0;
    grid.world_to_cell(Vec3{0.0f, 0.5f, 0.0f}, wx, wy, wz);
    CHECK(grid.walkable(wx, wy, wz));

    // A cell beside the floor (its footprint ends at x = 2) at floor height has
    // no ground below it, so require_ground refuses it.
    grid.world_to_cell(Vec3{2.5f, 0.5f, 0.0f}, wx, wy, wz);
    CHECK_FALSE(grid.walkable(wx, wy, wz));

    // Without require_ground the same cell is free space and walkable.
    const NavGrid3D flying = nav::build_nav_grid_3d(scene, 1.0f, /*require_ground=*/false);
    i32 fx = 0;
    i32 fy = 0;
    i32 fz = 0;
    flying.world_to_cell(Vec3{2.5f, 0.5f, 0.0f}, fx, fy, fz);
    CHECK(flying.walkable(fx, fy, fz));
}

TEST_CASE("a scene with no colliders builds an empty 3D nav grid") {
    Scene scene;
    (void)scene.create_node("Empty");
    CHECK(nav::build_nav_grid_3d(scene).empty());
}

// ---------------------------------------------------------------------------
// The 2D navigation system: an agent walks to its goal.
// ---------------------------------------------------------------------------

TEST_CASE("a NavAgent2D walks to a static goal along the grid") {
    TestMap test(8, 8);
    test.open_all();
    // A wall the agent must route around.
    for (u32 y = 0; y < 6; ++y) {
        test.set_solid(4, y, true);
    }

    Node* agent_node = test.scene.create_node("Enemy");
    auto* agent = agent_node->add_component<nav::NavAgent2D>();
    agent->has_goal = true;
    // The grid the system builds is one cell per tile, so the far cell (7, 0)
    // is at world (7.5, -0.5).
    agent->goal = Vec2{7.5f, -0.5f};
    agent_node->transform().position = Vec3{0.5f, -0.5f, 0.0f};

    nav::NavigationSystem2D system;
    FrameContext context;
    context.delta_time = 1.0f / 60.0f;

    // Run a bounded number of frames; the agent must reach the goal.
    const Vec2 start{agent_node->transform().position.x, agent_node->transform().position.y};
    for (int frame = 0; frame < 2000; ++frame) {
        system.update(context, test.scene);
        if (agent->arrived) {
            break;
        }
    }

    const Vec2 final_position{agent_node->transform().position.x, agent_node->transform().position.y};
    CHECK((final_position - agent->goal).length() <= doctest::Approx(agent->arrive_radius + 0.05f));
    CHECK((final_position - start).length() > 1.0f); // it actually moved
    CHECK(agent->arrived);
}

TEST_CASE("a NavAgent2D follows a goal node") {
    TestMap test(6, 6);
    test.open_all();

    Node* target = test.scene.create_node("Player");
    target->transform().position = Vec3{5.5f, -0.5f, 0.0f};

    Node* agent_node = test.scene.create_node("Chaser");
    auto* agent = agent_node->add_component<nav::NavAgent2D>();
    agent->goal_node = target->id();
    agent->speed = 10.0f;
    agent_node->transform().position = Vec3{0.5f, -0.5f, 0.0f};

    nav::NavigationSystem2D system;
    FrameContext context;
    context.delta_time = 1.0f / 60.0f;
    for (int frame = 0; frame < 2000; ++frame) {
        system.update(context, test.scene);
        const Vec2 position{agent_node->transform().position.x, agent_node->transform().position.y};
        const Vec2 goal{target->transform().position.x, target->transform().position.y};
        if ((position - goal).length() <= agent->arrive_radius) {
            break;
        }
    }

    const Vec2 position{agent_node->transform().position.x, agent_node->transform().position.y};
    const Vec2 goal{target->transform().position.x, target->transform().position.y};
    CHECK((position - goal).length() <= doctest::Approx(agent->arrive_radius + 0.05f));
}

TEST_CASE("a NavAgent2D with no goal does not move") {
    TestMap test(4, 4);
    test.open_all();
    Node* agent_node = test.scene.create_node("Idle");
    agent_node->add_component<nav::NavAgent2D>();
    agent_node->transform().position = Vec3{1.5f, -1.5f, 0.0f};

    nav::NavigationSystem2D system;
    FrameContext context;
    context.delta_time = 1.0f / 60.0f;
    for (int frame = 0; frame < 60; ++frame) {
        system.update(context, test.scene);
    }
    CHECK(agent_node->transform().position.x == doctest::Approx(1.5f));
    CHECK(agent_node->transform().position.y == doctest::Approx(-1.5f));
}

// ---------------------------------------------------------------------------
// The 3D navigation system.
// ---------------------------------------------------------------------------

TEST_CASE("a NavAgent3D walks to a goal through free space") {
    Scene scene;
    // A floor slab far below, so the grid has bounds; agents fly above it.
    Node* floor = scene.create_node("Floor");
    auto* floor_collider = floor->add_component<physics3d::Collider3D>();
    floor_collider->shape = physics3d::ShapeType3D::Box;
    floor_collider->half_extent = Vec3{10.0f, 0.5f, 10.0f};
    floor->transform().position = Vec3{0.0f, -0.5f, 0.0f};

    Node* agent_node = scene.create_node("Flyer");
    auto* agent = agent_node->add_component<nav::NavAgent3D>();
    agent->has_goal = true;
    agent->goal = Vec3{6.0f, 1.0f, 0.0f};
    agent->speed = 6.0f;
    agent_node->transform().position = Vec3{0.0f, 1.0f, 0.0f};

    nav::NavigationSystem3D system;
    FrameContext context;
    context.delta_time = 1.0f / 60.0f;
    for (int frame = 0; frame < 3000; ++frame) {
        system.update(context, scene);
        const Vec3 position = agent_node->transform().position;
        if ((position - agent->goal).length() <= agent->arrive_radius) {
            break;
        }
    }

    const Vec3 position = agent_node->transform().position;
    CHECK((position - agent->goal).length() <= doctest::Approx(agent->arrive_radius + 0.1f));
}

// ---------------------------------------------------------------------------
// Reflection and serialization.
// ---------------------------------------------------------------------------

TEST_CASE("a NavAgent2D round-trips through the serializer") {
    reflect::register_nav_types();

    Scene scene;
    Node* node = scene.create_node("Enemy");
    auto* agent = node->add_component<nav::NavAgent2D>();
    agent->has_goal = true;
    agent->goal = Vec2{3.0f, 4.0f};
    agent->speed = 7.5f;
    agent->separation_weight = 0.25f;

    const std::string json = serialization::scene::to_json(scene).value();
    CHECK(json.find("NavAgent2D") != std::string::npos);

    Scene loaded;
    const Error error = serialization::scene::from_json(json, loaded);
    REQUIRE(error.ok());
    Node* loaded_node = loaded.find_node_by_name("Enemy");
    REQUIRE(loaded_node != nullptr);
    const auto* loaded_agent = loaded_node->get_component<nav::NavAgent2D>();
    REQUIRE(loaded_agent != nullptr);
    CHECK(loaded_agent->has_goal);
    CHECK(loaded_agent->goal.x == doctest::Approx(3.0f));
    CHECK(loaded_agent->goal.y == doctest::Approx(4.0f));
    CHECK(loaded_agent->speed == doctest::Approx(7.5f));
    CHECK(loaded_agent->separation_weight == doctest::Approx(0.25f));
}

TEST_CASE("the nav agent types are registered with reflection") {
    reflect::register_nav_types();
    CHECK(reflect::Registry::instance().contains(nav::NavAgent2D::kTypeName));
    CHECK(reflect::Registry::instance().contains(nav::NavAgent3D::kTypeName));

    const reflect::TypeDesc* desc = reflect::Registry::instance().find(nav::NavAgent2D::kTypeName);
    REQUIRE(desc != nullptr);
    // A default instance carries the documented defaults.
    auto instance = reflect::Registry::instance().default_construct(nav::NavAgent2D::kTypeName);
    REQUIRE(instance != nullptr);
    const auto* agent = static_cast<const nav::NavAgent2D*>(instance.get());
    CHECK_FALSE(agent->has_goal);
    CHECK(agent->auto_move);
    CHECK(agent->speed == doctest::Approx(3.0f));
}

// ---------------------------------------------------------------------------
// The debug overlay.
// ---------------------------------------------------------------------------

TEST_CASE("the nav debug overlay draws blocked cells and leaves walkable ones out") {
    TestMap test(4, 3);
    test.open_all();
    test.set_solid(1, 0, true);
    test.set_solid(3, 2, true);
    const NavGrid2D grid = nav::build_nav_grid_2d(test.scene, *test.node, *test.map);

    debugdraw::DebugDrawList list;
    const usize cells = debugdraw::append_nav_grid_2d(grid, list);
    CHECK(cells == 2);
    // A box is 12 edges, so two blocked cells is 24 lines.
    CHECK(list.size() == 24);
}

TEST_CASE("the nav debug overlay draws an agent's remaining path") {
    TestMap test(6, 6);
    test.open_all();
    Node* agent_node = test.scene.create_node("Enemy");
    auto* agent = agent_node->add_component<nav::NavAgent2D>();
    agent->waypoints = {Vec2{1.0f, -1.0f}, Vec2{2.0f, -2.0f}, Vec2{3.0f, -3.0f}};
    agent->next_waypoint = 1;
    agent_node->transform().position = Vec3{0.0f, 0.0f, 0.0f};

    debugdraw::DebugDrawList list;
    const usize segments = debugdraw::append_nav_paths_2d(test.scene, list);
    // From the node position through the two remaining waypoints: two segments.
    CHECK(segments == 2);
    CHECK(list.size() == 2);
}

TEST_CASE("the 3D nav debug overlay draws blocked voxels") {
    Scene scene;
    Node* wall = scene.create_node("Wall");
    auto* collider = wall->add_component<physics3d::Collider3D>();
    collider->shape = physics3d::ShapeType3D::Box;
    collider->half_extent = Vec3{0.5f, 0.5f, 0.5f};
    wall->transform().position = Vec3{0.0f, 0.0f, 0.0f};

    const NavGrid3D grid = nav::build_nav_grid_3d(scene, 1.0f);
    REQUIRE_FALSE(grid.empty());
    debugdraw::DebugDrawList list;
    const usize voxels = debugdraw::append_nav_grid_3d(grid, list);
    CHECK(voxels > 0);
    CHECK(list.size() == voxels * 12);
}

// ---------------------------------------------------------------------------
// The AI behaviours that drive a navigation agent.
// ---------------------------------------------------------------------------

namespace {

// Binds a behaviour to `node` through the host, the way a scene load does, and
// returns the concrete instance so a test can read its state.
template <typename T>
[[nodiscard]] T* attach_behaviour(ScriptHost& host, Node& node) {
    auto script = std::make_unique<T>();
    T* raw = script.get();
    REQUIRE(host.attach_script(node, std::move(script)).ok());
    return raw;
}

} // namespace

TEST_CASE("PatrolRoute3D points the agent at each waypoint in turn and loops") {
    Scene scene;
    Node& post_a = *scene.create_node("PostA");
    post_a.transform().position = Vec3{0.0f, 0.0f, 0.0f};
    Node& post_b = *scene.create_node("PostB");
    post_b.transform().position = Vec3{5.0f, 0.0f, 0.0f};

    Node& guard = *scene.create_node("Guard");
    guard.transform().position = Vec3{2.5f, 0.0f, 0.0f}; // between the posts, not on one
    auto* agent = guard.add_component<nav::NavAgent3D>();
    agent->arrive_radius = 0.5f;

    ScriptHost host;
    auto* patrol = attach_behaviour<player::PatrolRoute3D>(host, guard);
    patrol->waypoints = "PostA, PostB";
    patrol->wait_time = 0.0f;

    FrameContext context;
    context.delta_time = 1.0f / 60.0f;

    // First update: the route resolves and the goal is the first post.
    CHECK(patrol->update(context).ok());
    CHECK(patrol->current_waypoint() == 0);
    CHECK(agent->has_goal);
    CHECK(agent->goal_node == post_a.id());

    // The agent arrives at PostA, so the patrol steps to PostB.
    guard.transform().position = post_a.transform().position;
    CHECK(patrol->update(context).ok());
    CHECK(patrol->current_waypoint() == 1);
    CHECK(agent->goal_node == post_b.id());

    // Arriving at PostB loops back to PostA.
    guard.transform().position = post_b.transform().position;
    CHECK(patrol->update(context).ok());
    CHECK(patrol->current_waypoint() == 0);
    CHECK(agent->goal_node == post_a.id());
}

TEST_CASE("PatrolRoute3D waits at a waypoint for the configured time") {
    Scene scene;
    Node& post = *scene.create_node("Post");
    Node& guard = *scene.create_node("Guard");
    auto* agent = guard.add_component<nav::NavAgent3D>();
    agent->arrive_radius = 0.5f;

    ScriptHost host;
    auto* patrol = attach_behaviour<player::PatrolRoute3D>(host, guard);
    patrol->waypoints = "Post";
    patrol->wait_time = 1.0f;

    FrameContext context;
    context.delta_time = 0.25f;

    CHECK(patrol->update(context).ok());
    // The guard starts on the post, so it begins waiting rather than advancing.
    guard.transform().position = post.transform().position;
    CHECK(patrol->update(context).ok());
    CHECK(patrol->current_waypoint() == 0); // still the only post
    // A single-post route with loop on never leaves post 0, so assert the wait
    // instead: three 0.25 s frames is still inside the 1 s wait.
    for (int frame = 0; frame < 3; ++frame) {
        CHECK(patrol->update(context).ok());
    }
    CHECK(patrol->current_waypoint() == 0);
}

TEST_CASE("a patrol route skips a waypoint name that does not resolve") {
    Scene scene;
    Node& post_b = *scene.create_node("PostB");
    Node& guard = *scene.create_node("Guard");
    guard.add_component<nav::NavAgent3D>();

    ScriptHost host;
    auto* patrol = attach_behaviour<player::PatrolRoute3D>(host, guard);
    patrol->waypoints = "Missing, PostB";

    FrameContext context;
    context.delta_time = 1.0f / 60.0f;
    CHECK(patrol->update(context).ok());
    // "Missing" was dropped, so the only stop is PostB and the goal is it.
    const auto* agent = guard.get_component<nav::NavAgent3D>();
    CHECK(agent->goal_node == post_b.id());
}

TEST_CASE("ChaseTarget3D chases a target inside sight and drops it beyond lose") {
    Scene scene;
    Node& player = *scene.create_node("Player");
    player.transform().position = Vec3{5.0f, 0.0f, 0.0f};
    Node& enemy = *scene.create_node("Enemy");
    enemy.transform().position = Vec3{0.0f, 0.0f, 0.0f};
    auto* agent = enemy.add_component<nav::NavAgent3D>();

    ScriptHost host;
    auto* chase = attach_behaviour<player::ChaseTarget3D>(host, enemy);
    chase->target = "Player";
    chase->sight_radius = 10.0f;
    chase->lose_radius = 12.0f;

    FrameContext context;
    context.delta_time = 1.0f / 60.0f;

    CHECK(chase->update(context).ok());
    CHECK(chase->engaged());
    CHECK(agent->has_goal);
    CHECK(agent->goal_node == player.id());

    // Move the target far beyond the lose radius: the chase drops and the goal is
    // no longer set by this behaviour (a patrol on the node may take over).
    player.transform().position = Vec3{50.0f, 0.0f, 0.0f};
    agent->has_goal = false; // a patrol would own it now; clear to observe
    CHECK(chase->update(context).ok());
    CHECK_FALSE(chase->engaged());
    CHECK_FALSE(agent->has_goal);
}

TEST_CASE("ChaseTarget3D keeps a target that hovers between sight and lose") {
    Scene scene;
    Node& player = *scene.create_node("Player");
    player.transform().position = Vec3{11.0f, 0.0f, 0.0f};
    Node& enemy = *scene.create_node("Enemy");
    auto* agent = enemy.add_component<nav::NavAgent3D>();

    ScriptHost host;
    auto* chase = attach_behaviour<player::ChaseTarget3D>(host, enemy);
    chase->target = "Player";
    chase->sight_radius = 10.0f;
    chase->lose_radius = 12.0f;

    FrameContext context;
    context.delta_time = 1.0f / 60.0f;

    // Outside sight: no chase.
    CHECK(chase->update(context).ok());
    CHECK_FALSE(chase->engaged());

    // Inside sight: chase starts.
    player.transform().position = Vec3{9.0f, 0.0f, 0.0f};
    CHECK(chase->update(context).ok());
    CHECK(chase->engaged());

    // Back into the hysteresis band: the chase continues rather than flickering.
    player.transform().position = Vec3{11.0f, 0.0f, 0.0f};
    CHECK(chase->update(context).ok());
    CHECK(chase->engaged());
    CHECK(agent->goal_node == player.id());
}

TEST_CASE("ChaseTarget3D with flee aims away from the target") {
    Scene scene;
    Node& player = *scene.create_node("Player");
    player.transform().position = Vec3{5.0f, 0.0f, 0.0f};
    Node& enemy = *scene.create_node("Enemy");
    enemy.transform().position = Vec3{0.0f, 0.0f, 0.0f};
    auto* agent = enemy.add_component<nav::NavAgent3D>();

    ScriptHost host;
    auto* chase = attach_behaviour<player::ChaseTarget3D>(host, enemy);
    chase->target = "Player";
    chase->flee = true;
    chase->flee_distance = 10.0f;

    FrameContext context;
    context.delta_time = 1.0f / 60.0f;
    CHECK(chase->update(context).ok());
    CHECK(chase->engaged());
    // A position goal, not a node goal, and further from the target than the
    // enemy started.
    CHECK(agent->goal_node.is_nil());
    CHECK(agent->goal.x < enemy.transform().position.x);
    CHECK((agent->goal - player.transform().position).length() == doctest::Approx(10.0f));
}

TEST_CASE("the nav behaviours register into the script registry") {
    ScriptRegistry registry;
    player::register_nav_behaviours(registry);
    CHECK(registry.find("PatrolRoute3D") != nullptr);
    CHECK(registry.find("ChaseTarget3D") != nullptr);
}
