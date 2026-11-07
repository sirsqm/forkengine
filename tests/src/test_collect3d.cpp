// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include <doctest/doctest.h>

#include "slice_state.h"

#include <string>

// The collect3d vertical slice's rules tests (the M11 3D game).
//
// CollectState is pure: no scene, no physics, no GPU, no resources. That is the
// point -- the numbers the game must get right (collect every gem before the
// clock runs out, score a pickup, win when the arena is cleared, lose when the
// time is gone) are the part worth testing directly, so they are driven here with
// no engine running at all. The systems and the arena builder are tested
// separately, compiled from the example's own sources.

using namespace fe;

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
