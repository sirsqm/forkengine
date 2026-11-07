// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include <doctest/doctest.h>

#include "slice_scene.h"
#include "slice_state.h"
#include "slice_systems.h"

#include "fe/audio/AudioSystem.h"
#include "fe/physics3d/Collider3D.h"
#include "fe/physics3d/Physics3DSystem.h"
#include "fe/physics3d/RigidBody3D.h"
#include "fe/platform/InputActions.h"
#include "fe/platform/InputState.h"
#include "fe/platform/UserData.h"
#include "fe/player/BuiltinBehaviours.h"
#include "fe/resources/Font.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/scene3d/Camera3D.h"
#include "fe/scene3d/MeshRenderer3D.h"
#include "fe/script/ScriptBinding.h"
#include "fe/script/ScriptHost.h"
#include "fe/script/ScriptRegistry.h"
#include "fe/script/ScriptSystem.h"
#include "fe/serialization/SceneSerializer.h"
#include "fe/systems/FrameContext.h"
#include "fe/systems/SceneSchedule.h"
#include "fe/ui/UiElement.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// The collect3d vertical slice's tests (the M11 3D game).
//
// Two layers, matching the two halves of the game:
//
//   * CollectState -- the rules. Pure, no engine, driven directly. These are the
//     numbers the game must get right.
//   * The systems and the arena builder -- compiled from the example's own
//     sources (see tests/CMakeLists.txt), so this is the *shipped* code, not a
//     copy. A scene is built in memory and the schedule is stepped, with no
//     window, GPU, or sound card, which is what lets this run in CI.
//
// A missing asset never fails a test: the builder treats textures and the font as
// optional, so the arena and its colliders are identical with or without them.

using namespace fe;

namespace {

FrameContext step(f32 delta) {
    FrameContext context;
    context.delta_time = delta;
    context.elapsed_time = static_cast<f64>(delta);
    return context;
}

int count_collectibles(const Scene& scene) {
    int gems = 0;
    scene.for_each_node([&](const Node& node) {
        if (node.name() == collect3d::names::kCollectible) {
            ++gems;
        }
    });
    return gems;
}

Node* first_collectible(Scene& scene) {
    Node* found = nullptr;
    scene.for_each_node([&](Node& node) {
        if (found == nullptr && node.name() == collect3d::names::kCollectible) {
            found = &node;
        }
    });
    return found;
}

} // namespace

// ---------------------------------------------------------------------------
// Rules: CollectState on its own.
// ---------------------------------------------------------------------------

TEST_CASE("a fresh game is playing with the full arena and the clock") {
    collect3d::CollectState state; // 8 gems, 60 seconds
    CHECK(state.result() == collect3d::CollectState::Result::Playing);
    CHECK(state.collectibles_total() == collect3d::config::kCollectibleCount);
    CHECK(state.collectibles_remaining() == collect3d::config::kCollectibleCount);
    CHECK(state.collected() == 0);
    CHECK(state.score() == 0);
    CHECK(state.time_left() == doctest::Approx(collect3d::config::kTimeLimit));
    CHECK_FALSE(state.is_over());
    CHECK(std::string(state.result_name()) == "playing");
}

TEST_CASE("collecting a gem scores and decrements the count") {
    collect3d::CollectState state(3, 60.0f);
    CHECK(state.collect_one());
    CHECK(state.collected() == 1);
    CHECK(state.collectibles_remaining() == 2);
    CHECK(state.score() == collect3d::config::kPointsPerCollectible);
    CHECK(state.result() == collect3d::CollectState::Result::Playing);
}

TEST_CASE("collecting the last gem wins and adds the goal bonus") {
    collect3d::CollectState state(2, 60.0f);
    CHECK(state.collect_one());
    CHECK_FALSE(state.is_over());
    CHECK(state.collect_one());
    CHECK(state.collectibles_remaining() == 0);
    CHECK(state.score() == 2 * collect3d::config::kPointsPerCollectible + collect3d::config::kGoalBonus);
    CHECK(state.won());
    CHECK(std::string(state.result_name()) == "win");
}

TEST_CASE("a pickup after the win does not score or collect") {
    collect3d::CollectState state(1, 60.0f);
    CHECK(state.collect_one()); // wins
    CHECK_FALSE(state.collect_one());
    CHECK(state.collected() == 1);
    CHECK(state.score() == collect3d::config::kPointsPerCollectible + collect3d::config::kGoalBonus);
}

TEST_CASE("the clock counts down only while the game is playing") {
    collect3d::CollectState state(8, 10.0f);
    state.tick(4.0f);
    CHECK(state.time_left() == doctest::Approx(6.0f));
    CHECK(state.seconds_left() == 6);

    // A paused game does not burn time.
    state.pause();
    state.tick(5.0f);
    CHECK(state.time_left() == doctest::Approx(6.0f));

    // Nor does the title.
    state.resume();
    state.return_to_title();
    state.tick(5.0f);
    CHECK(state.time_left() == doctest::Approx(10.0f)); // return_to_title reset it
}

TEST_CASE("the clock expiring loses the game") {
    collect3d::CollectState state(8, 2.0f);
    state.tick(1.5f);
    CHECK_FALSE(state.is_over());
    state.tick(1.0f); // past the limit
    CHECK(state.time_left() == doctest::Approx(0.0f));
    CHECK(state.is_over());
    CHECK_FALSE(state.won());
    CHECK(std::string(state.result_name()) == "lose");

    // A tick after the loss is ignored, so the result is sticky.
    state.tick(5.0f);
    CHECK(state.time_left() == doctest::Approx(0.0f));
}

TEST_CASE("seconds_left rounds up so a live last second reads as 1") {
    collect3d::CollectState state(8, 5.0f);
    state.tick(4.2f);
    CHECK(state.seconds_left() == 1); // 0.8s left rounds up
    state.tick(0.9f);                 // past zero, so the clock is spent
    CHECK(state.seconds_left() == 0);
}

TEST_CASE("clearing the arena before the clock runs out wins rather than losing") {
    collect3d::CollectState state(1, 5.0f);
    state.tick(1.0f);
    CHECK(state.collect_one()); // wins with 4s to spare
    CHECK(state.won());
    state.tick(10.0f); // the expired clock cannot flip a win to a loss
    CHECK(state.won());
}

TEST_CASE("the screen flow is title -> play -> game over -> title") {
    collect3d::CollectState state(2, 60.0f);
    state.show_title();
    CHECK(state.is_title());
    CHECK(std::string(state.screen_name()) == "title");

    state.start_game();
    CHECK(state.is_playing());

    state.pause();
    CHECK(state.is_paused());
    state.resume();
    CHECK(state.is_playing());

    state.collect_one();
    state.collect_one(); // wins
    state.sync_screen();
    CHECK(state.is_over());
    CHECK(std::string(state.screen_name()) == "gameover");

    state.return_to_title();
    CHECK(state.is_title());
    CHECK(state.restart_requested()); // the arena must be rebuilt
}

TEST_CASE("pause and resume are no-ops on the wrong screen") {
    collect3d::CollectState state;
    state.pause(); // already playing, this is fine
    CHECK(state.is_paused());
    state.pause(); // already paused: no-op
    CHECK(state.is_paused());
    state.return_to_title();
    state.resume(); // cannot resume the title
    CHECK(state.is_title());
}

TEST_CASE("the settings screen opens from the title or a pause and returns there") {
    collect3d::CollectState state;

    state.show_title();
    state.open_settings();
    CHECK(state.is_settings());
    state.close_settings();
    CHECK(state.is_title());

    state.start_game();
    state.pause();
    state.open_settings();
    CHECK(state.is_settings());
    state.close_settings();
    CHECK(state.is_paused());

    // Opening from a live game is refused: a setting cannot change mid-round.
    state.resume();
    state.open_settings();
    CHECK(state.is_playing());
}

TEST_CASE("reset restores the arena, the clock, and clears the restart request") {
    collect3d::CollectState state(4, 30.0f);
    state.collect_one();
    state.tick(5.0f);
    state.request_restart();
    state.reset();
    CHECK(state.score() == 0);
    CHECK(state.collected() == 0);
    CHECK(state.collectibles_remaining() == 4);
    CHECK(state.time_left() == doctest::Approx(30.0f));
    CHECK(state.result() == collect3d::CollectState::Result::Playing);
    CHECK_FALSE(state.restart_requested());
}

TEST_CASE("a restart request is consumed exactly once") {
    collect3d::CollectState state;
    state.request_restart();
    CHECK(state.consume_restart());
    CHECK_FALSE(state.consume_restart());
}

// ---------------------------------------------------------------------------
// Settings: clamping and the text round-trip.
// ---------------------------------------------------------------------------

TEST_CASE("volume clamps at both ends and steps by a tenth") {
    collect3d::Settings settings;
    settings.set_volume(2.0f);
    CHECK(settings.volume == doctest::Approx(collect3d::config::kMaxVolume));
    settings.set_volume(-1.0f);
    CHECK(settings.volume == doctest::Approx(collect3d::config::kMinVolume));
    settings.set_volume(0.5f);
    settings.adjust_volume(+1);
    CHECK(settings.volume == doctest::Approx(0.6f));
    settings.adjust_volume(-2);
    CHECK(settings.volume == doctest::Approx(0.4f));
}

TEST_CASE("settings serialize and parse back unchanged") {
    collect3d::Settings original;
    original.set_volume(0.4f);
    const std::string text = collect3d::serialize_settings(original);

    collect3d::Settings loaded;
    loaded.set_volume(1.0f);
    REQUIRE(collect3d::parse_settings(text, loaded));
    CHECK(loaded.volume == doctest::Approx(0.4f));
}

TEST_CASE("an unknown settings line is ignored rather than fatal") {
    collect3d::Settings settings;
    settings.set_volume(0.7f);
    // A save from a newer build with a field this build does not know.
    REQUIRE(collect3d::parse_settings("volume=0.250\nfuture_setting=42\n", settings));
    CHECK(settings.volume == doctest::Approx(0.25f));
}

TEST_CASE("the best score only rises and a first game records zero") {
    collect3d::CollectState state(2, 60.0f);
    CHECK_FALSE(state.has_best_score());
    CHECK(state.record_best_score()); // a first game, even at zero
    CHECK(state.has_best_score());
    CHECK(state.best_score() == 0);

    state.collect_one();
    CHECK(state.record_best_score());
    CHECK(state.best_score() == collect3d::config::kPointsPerCollectible);

    state.set_best_score(0); // a stale save cannot lower it
    CHECK(state.best_score() == collect3d::config::kPointsPerCollectible);
}

// ---------------------------------------------------------------------------
// The arena builder: the layout the systems and the scene file share.
// ---------------------------------------------------------------------------

TEST_CASE("the built arena has a player, a full ring of gems, and a camera") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);

    CHECK(scene.find_node_by_name(collect3d::names::kPlayer) != nullptr);
    CHECK(count_collectibles(scene) == collect3d::config::kCollectibleCount);
    CHECK(scene.find_node_by_name("Camera") != nullptr);
    CHECK(scene.find_node_by_name("Floor") != nullptr);
    CHECK(scene.find_node_by_name(collect3d::names::kHudCanvas) != nullptr);

    const Node* camera = scene.find_node_by_name("Camera");
    REQUIRE(camera != nullptr);
    CHECK(camera->get_component<Camera3D>() != nullptr);
}

TEST_CASE("the player carries a collider and a gravity-free body") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);
    Node* player = scene.find_node_by_name(collect3d::names::kPlayer);
    REQUIRE(player != nullptr);
    CHECK(player->get_component<physics3d::Collider3D>() != nullptr);
    const auto* body = player->get_component<physics3d::RigidBody3D>();
    REQUIRE(body != nullptr);
    // Gravity is off because the character controller owns the vertical axis.
    CHECK_FALSE(body->affected_by_gravity);
}

TEST_CASE("a collectible has a mesh but no collider, so the player walks through it") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);
    Node* gem = first_collectible(scene);
    REQUIRE(gem != nullptr);
    CHECK(gem->get_component<MeshRenderer3D>() != nullptr);
    CHECK(gem->get_component<physics3d::Collider3D>() == nullptr);
}

TEST_CASE("the arena scene round-trips through the serializer") {
    // The scene file is a serialization of the code-built arena, so this proves
    // the file path and the code path build the same board.
    Scene built;
    collect3d::build_collect3d_scene(built);

    const std::filesystem::path path = std::filesystem::temp_directory_path() / "collect3d_roundtrip.fescene";
    REQUIRE_FALSE(serialization::scene::save(built, path.string()));
    CHECK(std::filesystem::exists(path));

    Scene loaded;
    REQUIRE_FALSE(serialization::scene::load(path.string(), loaded));
    CHECK(loaded.find_node_by_name(collect3d::names::kPlayer) != nullptr);
    CHECK(count_collectibles(loaded) == collect3d::config::kCollectibleCount);
    std::filesystem::remove(path);
}

// ---------------------------------------------------------------------------
// The systems: driven through a schedule with no window.
// ---------------------------------------------------------------------------

TEST_CASE("the pickup system collects a gem the player walks onto") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);
    SceneSchedule schedule;
    collect3d::CollectState state;
    static_cast<void>(collect3d::install_collect3d_systems(schedule, state, /*autoplay=*/false));

    Node* player = scene.find_node_by_name(collect3d::names::kPlayer);
    Node* gem = first_collectible(scene);
    REQUIRE(player != nullptr);
    REQUIRE(gem != nullptr);

    const int before = count_collectibles(scene);
    // Move the player onto the gem, then run one frame.
    player->transform().position = gem->transform().position;
    schedule.update(step(1.0f / 60.0f), scene);

    CHECK(state.collected() == 1);
    CHECK(count_collectibles(scene) == before - 1);
}

TEST_CASE("the pickup system collects nothing when the game is not playing") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);
    SceneSchedule schedule;
    collect3d::CollectState state;
    static_cast<void>(collect3d::install_collect3d_systems(schedule, state, /*autoplay=*/false));
    state.show_title();

    Node* player = scene.find_node_by_name(collect3d::names::kPlayer);
    Node* gem = first_collectible(scene);
    player->transform().position = gem->transform().position;
    schedule.update(step(1.0f / 60.0f), scene);

    CHECK(state.collected() == 0);
    CHECK(count_collectibles(scene) == collect3d::config::kCollectibleCount);
}

TEST_CASE("the goal system runs the clock and ends the round on expiry") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);
    SceneSchedule schedule;
    collect3d::CollectState state(8, 0.5f);
    static_cast<void>(collect3d::install_collect3d_systems(schedule, state, /*autoplay=*/false));

    schedule.update(step(0.25f), scene);
    CHECK(state.is_playing());
    schedule.update(step(0.25f), scene);
    schedule.update(step(0.25f), scene);
    CHECK(state.is_over());
    CHECK_FALSE(state.won());
}

TEST_CASE("the HUD shows the score, the gems, and the clock") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);
    SceneSchedule schedule;
    collect3d::CollectState state(4, 30.0f);
    static_cast<void>(collect3d::install_collect3d_systems(schedule, state, /*autoplay=*/false));

    Node* score_node = scene.find_node_by_name(collect3d::names::kScoreLabel);
    Node* gems_node = scene.find_node_by_name(collect3d::names::kCollectedLabel);
    Node* time_node = scene.find_node_by_name(collect3d::names::kTimeLabel);
    REQUIRE(score_node != nullptr);
    REQUIRE(gems_node != nullptr);
    REQUIRE(time_node != nullptr);
    auto* score_element = score_node->get_component<ui::UiElement>();
    auto* gems_element = gems_node->get_component<ui::UiElement>();
    auto* time_element = time_node->get_component<ui::UiElement>();
    REQUIRE(score_element != nullptr);
    REQUIRE(gems_element != nullptr);
    REQUIRE(time_element != nullptr);

    schedule.update(step(0.0f), scene);
    CHECK(score_element->text == "SCORE 0");
    CHECK(gems_element->text == "GEMS 0/4");
    CHECK(time_element->text == "TIME 30");

    state.collect_one();
    schedule.update(step(1.0f), scene);
    CHECK(score_element->text == "SCORE 100");
    CHECK(gems_element->text == "GEMS 1/4");
    CHECK(time_element->text == "TIME 29");
}

TEST_CASE("the game-over screen reads a win differently from a loss") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);
    SceneSchedule schedule;
    collect3d::CollectState state(1, 30.0f);
    static_cast<void>(collect3d::install_collect3d_systems(schedule, state, /*autoplay=*/false));

    Node* node = scene.find_node_by_name(collect3d::names::kGameOverLabel);
    REQUIRE(node != nullptr);
    auto* element = node->get_component<ui::UiElement>();
    REQUIRE(element != nullptr);

    state.collect_one(); // wins
    state.sync_screen();
    schedule.update(step(0.0f), scene);
    CHECK(element->text == collect3d::config::kWinText);
}

TEST_CASE("the menu shows exactly one canvas per screen") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);
    SceneSchedule schedule;
    collect3d::CollectState state;
    static_cast<void>(collect3d::install_collect3d_systems(schedule, state, /*autoplay=*/false));
    state.show_title();

    const auto canvas_visible = [&scene](std::string_view name) {
        const Node* node = scene.find_node_by_name(name);
        const auto* canvas = node != nullptr ? node->get_component<ui::UiCanvas>() : nullptr;
        return canvas != nullptr && canvas->visible;
    };

    schedule.update(step(0.0f), scene);
    CHECK(canvas_visible(collect3d::names::kTitleCanvas));
    CHECK_FALSE(canvas_visible(collect3d::names::kHudCanvas));

    state.start_game();
    schedule.update(step(0.0f), scene);
    CHECK(canvas_visible(collect3d::names::kHudCanvas));
    CHECK_FALSE(canvas_visible(collect3d::names::kTitleCanvas));

    state.pause();
    schedule.update(step(0.0f), scene);
    CHECK(canvas_visible(collect3d::names::kPauseCanvas));
    CHECK_FALSE(canvas_visible(collect3d::names::kHudCanvas));
}

TEST_CASE("the menu advances the title to a live game on the confirm press") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);
    SceneSchedule schedule;
    collect3d::CollectState state;
    collect3d::CollectActions bindings;
    InputState input;
    ActionMap actions;
    bindings = collect3d::declare_collect3d_actions(actions);

    static_cast<void>(collect3d::install_collect3d_systems(schedule, state, /*autoplay=*/false, bindings));
    state.show_title();

    FrameContext context = step(1.0f / 60.0f);
    context.input = &input;
    context.actions = &actions;
    actions.update(input, Vec2{1280.0f, 720.0f});
    schedule.update(context, scene);
    CHECK(state.is_title());

    // Press the confirm (jump) key and re-evaluate the actions, as the runtime
    // does before the systems run.
    input.on_key(Key::Space, true);
    actions.update(input, Vec2{1280.0f, 720.0f});
    schedule.update(context, scene);
    CHECK(state.is_playing());
}

TEST_CASE("the best score is saved on the game-over edge and read back") {
    Scene scene;
    collect3d::build_collect3d_scene(scene);
    SceneSchedule schedule;
    collect3d::CollectState state(1, 30.0f);
    static_cast<void>(collect3d::install_collect3d_systems(schedule, state, /*autoplay=*/false));

    UserData::Desc desc;
    desc.backend = UserData::Desc::Backend::Null;
    std::unique_ptr<UserData> store = UserData::create(desc);
    REQUIRE(store != nullptr);

    FrameContext context = step(0.0f);
    context.user_data = store.get();

    state.start_game();
    state.collect_one(); // wins, ends the game
    state.sync_screen();
    schedule.update(context, scene);

    const int expected = collect3d::config::kPointsPerCollectible + collect3d::config::kGoalBonus;
    REQUIRE(store->contains(collect3d::config::kBestScoreKey));
    CHECK(store->read_text(collect3d::config::kBestScoreKey).value() == std::to_string(expected));

    // A fresh game loads the saved value on its first frame with a store, so a
    // returning player sees their best immediately.
    Scene scene2;
    collect3d::build_collect3d_scene(scene2);
    SceneSchedule schedule2;
    collect3d::CollectState reloaded;
    static_cast<void>(collect3d::install_collect3d_systems(schedule2, reloaded, /*autoplay=*/false));
    schedule2.update(context, scene2);
    CHECK(reloaded.has_best_score());
    CHECK(reloaded.best_score() == expected);
}

// ---------------------------------------------------------------------------
// The whole game: an autoplay run clears the arena.
// ---------------------------------------------------------------------------

TEST_CASE("an autoplay run clears the arena and wins") {
    // This is the integration test the roadmap asks for: the shipped scene, the
    // shipped systems, and the shipped rules, driven with no window and no
    // player, proving the 3D game reaches its goal.
    Scene scene;
    collect3d::build_collect3d_scene(scene);
    ScriptRegistry registry;
    player::register_builtin_behaviours(registry);
    ScriptHost host;
    collect3d::attach_collect3d_behaviours(host, scene);

    SceneSchedule schedule;
    collect3d::CollectState state;
    static_cast<void>(collect3d::install_collect3d_systems(schedule, state, /*autoplay=*/true, &host));

    const int max_frames = 60 * 120; // a generous bound; the run should clear sooner
    int frames = 0;
    while (!state.is_over() && frames < max_frames) {
        schedule.update(step(1.0f / 60.0f), scene);
        ++frames;
    }

    REQUIRE(state.is_over());
    CHECK(state.won());
    CHECK(state.collected() == state.collectibles_total());
    CHECK(count_collectibles(scene) == 0);
    CHECK(frames < max_frames); // it finished inside the clock, not by running out
}
