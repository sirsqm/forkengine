// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "slice_scene.h"

#include "fe/audio/AudioSource.h"
#include "fe/mesh/Mesh.h"
#include "fe/physics3d/Collider3D.h"
#include "fe/physics3d/RigidBody3D.h"
#include "fe/player/BuiltinBehaviours.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/scene3d/Camera3D.h"
#include "fe/scene3d/DirectionalLight3D.h"
#include "fe/scene3d/MeshRenderer3D.h"
#include "fe/script/ScriptHost.h"
#include "fe/ui/UiElement.h"

#include <cmath>
#include <memory>
#include <string>
#include <string_view>

namespace collect3d {
namespace {

namespace cfg = config;

// Gives an already-created node a box mesh sized to `half_extent` and, when
// `solid`, a matching box collider. The mesh and the collider are built from the
// *same* half extent, so the two cannot disagree about the shape -- the
// vertex-layout trap AGENTS.md records is the same class of bug, and one value
// feeding both is the fix.
//
// The mesh is the shared unit cube scaled by the node, which is how a scene file
// references geometry (a resource path, not a size). When no cube resource is
// available -- a headless test with no assets -- the fallback is an in-code box
// of the same half extent, so the arena has real geometry either way and the
// collider never disagrees with what is drawn.
void configure_box(fe::Node& node, const std::shared_ptr<fe::Resource>& unit_cube, const fe::Vec3& half_extent,
                   const fe::Vec4& color, fe::f32 specular, bool solid) {
    node.transform().scale = fe::Vec3{half_extent.x * 2.0f, half_extent.y * 2.0f, half_extent.z * 2.0f};
    auto* renderer = node.add_component<fe::MeshRenderer3D>();
    renderer->mesh = unit_cube != nullptr
                         ? unit_cube
                         : std::static_pointer_cast<fe::Resource>(fe::Mesh::make_box(fe::Vec3{0.5f, 0.5f, 0.5f}));
    renderer->base_color = color;
    renderer->specular = specular;
    if (solid) {
        auto* collider = node.add_component<fe::physics3d::Collider3D>();
        collider->shape = fe::physics3d::ShapeType3D::Box;
        collider->half_extent = half_extent;
    }
}

// A root box node.
fe::Node* add_box(fe::Scene& scene, const std::shared_ptr<fe::Resource>& unit_cube, std::string_view name,
                  const fe::Vec3& position, const fe::Vec3& half_extent, const fe::Vec4& color, fe::f32 specular,
                  bool solid) {
    fe::Node* node = scene.create_node(std::string(name));
    node->transform().position = position;
    configure_box(*node, unit_cube, half_extent, color, specular, solid);
    return node;
}

// A HUD text node, laid out by the UI system in screen space. `anchor` names the
// canvas corner the element is placed against and `offset` nudges it inward.
fe::Node* add_hud_label(fe::Node* canvas, std::string_view name, fe::ui::UiAnchor anchor, const fe::Vec2& offset,
                        fe::ui::UiAlign align) {
    fe::Node* node = canvas->create_child<fe::Node>(std::string(name));
    auto* element = node->add_component<fe::ui::UiElement>();
    element->width = fe::ui::UiSize::content();
    element->height = fe::ui::UiSize::content();
    element->anchor = anchor;
    element->offset = offset;
    element->text_align = align;
    element->text_vertical = fe::ui::UiAlign::Center;
    return node;
}

// A full-screen menu canvas with a centred label, hidden until its screen is
// current. One per screen; MenuSystem toggles the canvas's `visible` flag to show
// exactly one at a time.
fe::Node* add_menu_canvas(fe::Scene& scene, std::string_view canvas_name, std::string_view label_name,
                          const std::shared_ptr<fe::Resource>& font, const char* text) {
    fe::Node* canvas_node = scene.create_node(std::string(canvas_name));
    auto* canvas = canvas_node->add_component<fe::ui::UiCanvas>();
    canvas->design_width = cfg::kHudDesignWidth;
    canvas->design_height = cfg::kHudDesignHeight;
    canvas->font = font;
    canvas->text_size = cfg::kMenuTextSize;
    canvas->layer = 20;
    canvas->visible = false;

    fe::Node* label = canvas_node->create_child<fe::Node>(std::string(label_name));
    auto* element = label->add_component<fe::ui::UiElement>();
    element->width = fe::ui::UiSize::content();
    element->height = fe::ui::UiSize::content();
    element->anchor = fe::ui::UiAnchor::MiddleCenter;
    element->text_align = fe::ui::UiAlign::Center;
    element->text_vertical = fe::ui::UiAlign::Center;
    element->text = text;
    return canvas_node;
}

// One clickable line of text under `parent`. It is a UiElement with text, so it
// draws like a label and is hit tested like a button, which is what makes a tap
// and a click both work on a menu.
fe::Node* add_menu_button(fe::Node* parent, std::string_view name, const char* text, fe::ui::UiAnchor anchor,
                          const fe::Vec2& offset, fe::ui::UiAlign align) {
    fe::Node* node = parent->create_child<fe::Node>(std::string(name));
    auto* element = node->add_component<fe::ui::UiElement>();
    element->width = fe::ui::UiSize::content();
    element->height = fe::ui::UiSize::content();
    element->anchor = anchor;
    element->offset = offset;
    element->text = text;
    element->text_align = align;
    element->text_vertical = fe::ui::UiAlign::Center;
    return node;
}

// The settings screen: a title, a volume row (a "-", the value, a "+"), and a back
// button. The canvas is hidden until MenuSystem shows it.
void build_settings_canvas(fe::Scene& scene, const std::shared_ptr<fe::Resource>& font) {
    fe::Node* canvas_node = scene.create_node(std::string(names::kSettingsCanvas));
    auto* canvas = canvas_node->add_component<fe::ui::UiCanvas>();
    canvas->design_width = cfg::kHudDesignWidth;
    canvas->design_height = cfg::kHudDesignHeight;
    canvas->font = font;
    canvas->text_size = cfg::kSettingsTextSize;
    canvas->layer = 20;
    canvas->visible = false;

    fe::Node* column_node = canvas_node->create_child<fe::Node>("SettingsRows");
    auto* column = column_node->add_component<fe::ui::UiElement>();
    column->width = fe::ui::UiSize::content();
    column->height = fe::ui::UiSize::content();
    column->anchor = fe::ui::UiAnchor::MiddleCenter;
    column->direction = fe::ui::UiDirection::Column;
    column->gap = cfg::kSettingsButtonGap;
    column->align = fe::ui::UiAlign::Center;
    column->justify = fe::ui::UiJustify::Center;

    add_menu_button(column_node, names::kSettingsTitleLabel, cfg::kSettingsText, fe::ui::UiAnchor::TopCenter,
                    fe::Vec2{}, fe::ui::UiAlign::Center);

    fe::Node* row_node = column_node->create_child<fe::Node>("VolumeRow");
    auto* row = row_node->add_component<fe::ui::UiElement>();
    row->width = fe::ui::UiSize::content();
    row->height = fe::ui::UiSize::content();
    row->direction = fe::ui::UiDirection::Row;
    row->gap = cfg::kSettingsButtonGap;
    row->align = fe::ui::UiAlign::Center;
    row->justify = fe::ui::UiJustify::Center;

    const auto add_sized_button = [&](std::string_view name, const char* text) {
        fe::Node* node =
            add_menu_button(row_node, name, text, fe::ui::UiAnchor::MiddleLeft, fe::Vec2{}, fe::ui::UiAlign::Center);
        if (auto* element = node->get_component<fe::ui::UiElement>(); element != nullptr) {
            element->width = fe::ui::UiSize::fixed(cfg::kSettingsButtonSize);
            element->height = fe::ui::UiSize::fixed(cfg::kSettingsButtonSize);
        }
    };
    add_sized_button(names::kSettingsVolumeDown, "-");
    add_menu_button(row_node, names::kSettingsVolumeValue, "", fe::ui::UiAnchor::MiddleCenter, fe::Vec2{},
                    fe::ui::UiAlign::Center);
    add_sized_button(names::kSettingsVolumeUp, "+");

    add_menu_button(column_node, names::kSettingsBack, "BACK", fe::ui::UiAnchor::TopCenter, fe::Vec2{},
                    fe::ui::UiAlign::Center);
}

} // namespace

void build_collect3d_scene(fe::Scene& scene, const SceneAssets& assets) {
    scene.set_name("Collect3D");

    const std::shared_ptr<fe::Resource>& cube = assets.box;

    // The floor: a box whose top face is at y = 0, so the player's start height
    // and the collider agree. A static collider with no rigid body is the level
    // geometry the character stands on.
    add_box(scene, cube, "Floor", fe::Vec3{0.0f, -0.5f, 0.0f},
            fe::Vec3{cfg::kArenaHalfExtent, 0.5f, cfg::kArenaHalfExtent}, fe::Vec4{0.32f, 0.38f, 0.46f, 1.0f}, 0.05f,
            true);

    // The four walls, each a static box that stops the player leaving the arena.
    const fe::f32 half = cfg::kArenaHalfExtent;
    const fe::f32 wall_y = cfg::kWallHeight * 0.5f;
    const fe::Vec3 wall_half_x{half, wall_y, cfg::kWallThickness};
    const fe::Vec3 wall_half_z{cfg::kWallThickness, wall_y, half};
    const fe::Vec4 wall_color{0.5f, 0.42f, 0.4f, 1.0f};
    add_box(scene, cube, "WallNorth", fe::Vec3{0.0f, wall_y, -half - cfg::kWallThickness}, wall_half_x, wall_color,
            0.1f, true);
    add_box(scene, cube, "WallSouth", fe::Vec3{0.0f, wall_y, half + cfg::kWallThickness}, wall_half_x, wall_color, 0.1f,
            true);
    add_box(scene, cube, "WallWest", fe::Vec3{-half - cfg::kWallThickness, wall_y, 0.0f}, wall_half_z, wall_color, 0.1f,
            true);
    add_box(scene, cube, "WallEast", fe::Vec3{half + cfg::kWallThickness, wall_y, 0.0f}, wall_half_z, wall_color, 0.1f,
            true);

    // The player: a mesh, a box collider, and a gravity-free rigid body. Gravity
    // is off because the character controller owns the vertical axis, so it can
    // decide what "grounded" means and how a jump feels. fixed_rotation keeps the
    // axis-aligned collider honest.
    fe::Node* player =
        add_box(scene, cube, names::kPlayer, fe::Vec3{cfg::kPlayerStartX, cfg::kPlayerStartY, cfg::kPlayerStartZ},
                fe::Vec3{0.45f, 0.55f, 0.45f}, fe::Vec4{0.95f, 0.6f, 0.2f, 1.0f}, 0.45f, true);
    auto* body = player->add_component<fe::physics3d::RigidBody3D>();
    body->mass = 2.0f;
    body->affected_by_gravity = false;
    body->fixed_rotation = true;

    // The collectibles: a ring of gems inside the arena, evenly spaced. They carry
    // a mesh but *no* collider, so the player walks through them and the pickup is
    // by distance rather than by contact -- which is what makes a gem something to
    // collect rather than something to bump into.
    fe::Node* gem_root = scene.create_node("Collectibles");
    for (int i = 0; i < cfg::kCollectibleCount; ++i) {
        const fe::f32 angle = (2.0f * fe::kPi * static_cast<fe::f32>(i)) / static_cast<fe::f32>(cfg::kCollectibleCount);
        const fe::Vec3 position{std::cos(angle) * cfg::kCollectibleRingRadius, cfg::kCollectibleHeight,
                                std::sin(angle) * cfg::kCollectibleRingRadius};
        // A child of the gem root, so the tree is tidy and the gems are one group.
        // A gem carries no collider: the player walks through it and the pickup is
        // by distance, so a gem is something to collect rather than to bump into.
        fe::Node* gem = gem_root->create_child<fe::Node>(std::string(names::kCollectible));
        gem->transform().position = position;
        configure_box(*gem, cube,
                      fe::Vec3{cfg::kCollectibleHalfExtent, cfg::kCollectibleHalfExtent, cfg::kCollectibleHalfExtent},
                      fe::Vec4{0.3f, 0.9f, 0.7f, 1.0f}, 0.6f, false);
    }

    // The sun: a directional light, aimed down and across the arena.
    fe::Node* sun = scene.create_node("Sun");
    auto* light = sun->add_component<fe::DirectionalLight3D>();
    light->direction = fe::Vec3{-0.4f, -1.0f, -0.35f};
    light->color = fe::Vec3{1.0f, 0.97f, 0.9f};
    light->intensity = 1.0f;
    light->ambient = 0.3f;

    // The camera. It carries no behaviour here; the follow behaviour is attached
    // by attach_collect3d_behaviours, which is the wiring a scene file names.
    fe::Node* camera_node = scene.create_node("Camera");
    camera_node->transform().position = fe::Vec3{0.0f, 3.2f, cfg::kPlayerStartZ + 6.0f};
    auto* camera = camera_node->add_component<fe::Camera3D>();
    camera->fov_degrees = 60.0f;
    camera->near_plane = 0.1f;
    camera->far_plane = 200.0f;
    camera->clear_r = 0.05f;
    camera->clear_g = 0.06f;
    camera->clear_b = 0.1f;
    camera->primary = true;

    // The pickup cue: a one-shot that starts silent and is re-triggered by
    // CollectibleSystem through play(). It lives in the scene so the sound is
    // authored, not hardcoded; the application only attaches the decoded clip.
    //
    // It is positional (`spatial`) and on the "sfx" bus: CollectibleSystem moves
    // this node onto the gem it collects, so the mixer attenuates the cue against
    // the listener at the camera and pans it to the side the gem was on, and the
    // bus keeps effects off the master volume the settings screen drives. Both are
    // the M11 audio row's "positional sources and a bus layout", exercised on the
    // shipped game rather than only in a test.
    fe::Node* audio = scene.create_node(std::string(names::kAudio));
    auto* source = audio->add_component<fe::audio::AudioSource>();
    source->gain = 0.7f;
    source->loop = false;
    source->play_on_start = false;
    source->spatial = true;
    source->min_distance = cfg::kPickupMinDistance;
    source->max_distance = cfg::kPickupMaxDistance;
    source->bus_name = cfg::kSfxBusName;

    // The HUD: a screen-space UI canvas whose counters are pinned to the window's
    // corners. The text is filled in each frame by HudSystem, so the canvas is
    // authored with empty elements and the game's state is what makes them
    // readable. An empty element draws nothing.
    fe::Node* hud = scene.create_node(std::string(names::kHudCanvas));
    auto* canvas = hud->add_component<fe::ui::UiCanvas>();
    canvas->design_width = cfg::kHudDesignWidth;
    canvas->design_height = cfg::kHudDesignHeight;
    canvas->font = assets.font;
    canvas->text_size = cfg::kHudTextSize;
    canvas->layer = 10;

    add_hud_label(hud, names::kScoreLabel, fe::ui::UiAnchor::TopLeft, fe::Vec2{cfg::kHudMargin, cfg::kHudMargin},
                  fe::ui::UiAlign::Start);
    add_hud_label(hud, names::kBestScoreLabel, fe::ui::UiAnchor::TopLeft,
                  fe::Vec2{cfg::kHudMargin, cfg::kHudMargin + cfg::kHudTextSize + 8.0f}, fe::ui::UiAlign::Start);
    add_hud_label(hud, names::kCollectedLabel, fe::ui::UiAnchor::TopRight, fe::Vec2{-cfg::kHudMargin, cfg::kHudMargin},
                  fe::ui::UiAlign::End);
    add_hud_label(hud, names::kTimeLabel, fe::ui::UiAnchor::TopCenter, fe::Vec2{0.0f, cfg::kHudMargin},
                  fe::ui::UiAlign::Center);

    // The menu screens: a title, a pause, and a game-over canvas, each hidden
    // until MenuSystem shows it.
    add_menu_canvas(scene, names::kTitleCanvas, names::kTitleLabel, assets.font, cfg::kTitleText);
    add_menu_canvas(scene, names::kPauseCanvas, names::kPauseLabel, assets.font, cfg::kPauseText);
    add_menu_canvas(scene, names::kGameOverCanvas, names::kGameOverLabel, assets.font, cfg::kGameOverText);

    // A settings button on the title and the pause screens, so the settings screen
    // is reachable by a tap.
    if (fe::Node* title_canvas = scene.find_node_by_name(names::kTitleCanvas); title_canvas != nullptr) {
        add_menu_button(title_canvas, names::kSettingsButton, cfg::kSettingsButtonText, fe::ui::UiAnchor::BottomCenter,
                        fe::Vec2{0.0f, -cfg::kHudMargin}, fe::ui::UiAlign::Center);
    }
    if (fe::Node* pause_canvas = scene.find_node_by_name(names::kPauseCanvas); pause_canvas != nullptr) {
        add_menu_button(pause_canvas, names::kPauseSettingsButton, cfg::kSettingsButtonText,
                        fe::ui::UiAnchor::BottomCenter, fe::Vec2{0.0f, -cfg::kHudMargin}, fe::ui::UiAlign::Center);
    }

    build_settings_canvas(scene, assets.font);
}

void attach_collect3d_behaviours(fe::ScriptHost& host, fe::Scene& scene) {
    if (fe::Node* player = scene.find_node_by_name(names::kPlayer); player != nullptr) {
        auto controller = std::make_unique<fe::player::CharacterController3D>();
        controller->speed = 5.0f;
        static_cast<void>(host.attach_script(*player, std::move(controller)));
    }
    if (fe::Node* camera = scene.find_node_by_name("Camera"); camera != nullptr) {
        auto follow = std::make_unique<fe::player::FollowCamera3D>();
        follow->target = std::string(names::kPlayer);
        static_cast<void>(host.attach_script(*camera, std::move(follow)));
    }
    scene.for_each_node([&](fe::Node& node) {
        if (node.name() != names::kCollectible) {
            return;
        }
        auto spin = std::make_unique<fe::player::SpinNode3D>();
        spin->axis_y = 1.0f;
        spin->degrees_per_second = 90.0f;
        static_cast<void>(host.attach_script(node, std::move(spin)));
    });
}

} // namespace collect3d
