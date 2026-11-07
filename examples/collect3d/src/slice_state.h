// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Types.h"

#include <string>
#include <string_view>

// The 3D collect game's rules and geometry (roadmap M11, the second vertical
// slice).
//
// Everything in this header is pure: no scene, no physics, no GPU, no resources.
// That is deliberate and follows the M2.5 breakout slice exactly. The rules a
// game must get right -- collect every gem before the clock runs out, score a
// pickup, win when the arena is cleared, lose when the time is gone -- are the
// part worth testing directly, so they live in a class the tests construct and
// drive with no engine running at all. The systems in slice_systems.h are the
// only things that touch the scene; they read and write this state.
//
// Where breakout proved the 2D subsystems work together, this slice is the same
// shape in 3D: a third-person character animated by a blend tree, a course of
// collectibles, physics, positional audio, a HUD and a menu, and a goal -- the
// artifact that turns the 3D engine rows into a finished game.

namespace collect3d {

// The arena geometry and the game's tuning, shared by the scene builder and the
// gameplay systems so the two never disagree about where the arena ends or what a
// pickup is worth.
//
// The authored scene (scenes/collect3d.fescene) is a serialization of exactly
// this layout, which is what keeps the file path and the code fallback building
// an identical arena.
namespace config {

// The arena is a square floor bounded by four walls. `kArenaHalfExtent` is the
// half-width of the floor and the distance from the centre to each wall.
inline constexpr fe::f32 kArenaHalfExtent = 12.0f;
inline constexpr fe::f32 kWallHeight = 2.0f;
inline constexpr fe::f32 kWallThickness = 0.5f;

// The player starts behind the centre, facing into the arena (the engine's
// forward is -Z, so a start at +Z faces the origin).
inline constexpr fe::f32 kPlayerStartX = 0.0f;
inline constexpr fe::f32 kPlayerStartY = 1.0f;
inline constexpr fe::f32 kPlayerStartZ = 9.5f;

// The collectibles are laid out on a ring inside the arena, evenly spaced. A ring
// is used rather than a random scatter so the layout is reproducible: the scene
// file and the code fallback build the same board, and a test can predict where
// each gem is.
inline constexpr int kCollectibleCount = 8;
inline constexpr fe::f32 kCollectibleRingRadius = 7.0f;
inline constexpr fe::f32 kCollectibleHeight = 1.0f;
// The gem's own half-size, so the builder and the pickup radius agree.
inline constexpr fe::f32 kCollectibleHalfExtent = 0.4f;
// How close the player's centre must come to a gem's centre to pick it up. A
// little larger than the gem so a pickup is forgiving rather than pixel-exact,
// which is what makes it feel like a game rather than a precision task.
inline constexpr fe::f32 kPickupRadius = 1.1f;

// The character controller's start height above the floor, so it settles onto the
// floor on the first frames rather than starting inside it.
inline constexpr fe::f32 kFloorTop = 0.0f;

// Scoring. Each gem is worth a fixed number of points; clearing the arena adds a
// bonus, so finishing is worth more than the sum of its parts.
inline constexpr int kPointsPerCollectible = 100;
inline constexpr int kGoalBonus = 250;

// The clock. The round is won by clearing the arena and lost when the time runs
// out, so a game has both a goal and a limit. Chosen so a competent run clears
// the ring with time to spare and a wandering run does not.
inline constexpr fe::f32 kTimeLimit = 60.0f;

// The heads-up display. The score and the clock are screen-space labels
// (UiCanvas/UiElement), so they stay pinned to a corner at any window size.
inline constexpr fe::f32 kHudDesignWidth = 1280.0f;
inline constexpr fe::f32 kHudDesignHeight = 720.0f;
inline constexpr fe::f32 kHudTextSize = 40.0f;
inline constexpr fe::f32 kHudMargin = 28.0f;
// The menu text (title, pause, game over). Larger than the HUD counters because a
// menu line is the only thing on its screen.
inline constexpr fe::f32 kMenuTextSize = 72.0f;
// The settings screen's text is smaller than the other menus because it carries
// several lines.
inline constexpr fe::f32 kSettingsTextSize = 40.0f;
inline constexpr fe::f32 kSettingsButtonSize = 72.0f;
inline constexpr fe::f32 kSettingsButtonGap = 40.0f;

// The menu strings, authored in one place next to the layout they belong to.
inline constexpr const char* kTitleText = "GEM RUSH";
inline constexpr const char* kPauseText = "PAUSED";
inline constexpr const char* kGameOverText = "GAME OVER";
inline constexpr const char* kWinText = "ARENA CLEARED";
inline constexpr const char* kSettingsText = "SETTINGS";
inline constexpr const char* kSettingsButtonText = "SETTINGS";

// The best score and the settings, persisted. The keys and the application id
// live here so the scene builder, the systems and the application name the same
// strings.
inline constexpr const char* kBestScoreKey = "best_score";
inline constexpr const char* kSettingsKey = "settings";
inline constexpr const char* kAppId = "gem_rush";

// Master volume, 0..1. The audio system's master gain is this value directly, so
// 1 is the clip at its recorded level and 0 is silence.
inline constexpr fe::f32 kDefaultVolume = 1.0f;
inline constexpr fe::f32 kMinVolume = 0.0f;
inline constexpr fe::f32 kMaxVolume = 1.0f;
inline constexpr fe::f32 kVolumeStep = 0.1f;

// The pickup cue is positional: CollectibleSystem moves its node to the gem
// that was just collected before it plays, so a gem across the arena is quieter
// than one at the player's feet and a gem to the player's left is heard on the
// left. The falloff runs from full gain at kPickupMinDistance to silence at
// kPickupMaxDistance, chosen so a cue anywhere on the gem ring is still audible
// while the player can tell which side it came from.
inline constexpr fe::f32 kPickupMinDistance = 2.0f;
inline constexpr fe::f32 kPickupMaxDistance = 20.0f;

// The mixer group bus the game's sound effects route to. The volume setting
// drives the *master* bus, so an effects bus with its own gain is the layout the
// M11 audio row asked for: the two are independent, so muting effects later does
// not have to touch the master volume the player set.
inline constexpr const char* kSfxBusName = "sfx";
inline constexpr fe::f32 kSfxBusGain = 1.0f;

} // namespace config

// The node names the scene builder and the systems share. One place, so the two
// cannot drift apart: the builder creates a node by a name and a system finds it
// by the same name, and a rename that touched only one of them would break the
// game silently.
namespace names {
inline constexpr std::string_view kPlayer = "Player";
inline constexpr std::string_view kCollectible = "Collectible";
inline constexpr std::string_view kAudio = "PickupAudio";
inline constexpr std::string_view kHudCanvas = "HudCanvas";
inline constexpr std::string_view kScoreLabel = "ScoreLabel";
inline constexpr std::string_view kTimeLabel = "TimeLabel";
inline constexpr std::string_view kCollectedLabel = "CollectedLabel";
inline constexpr std::string_view kBestScoreLabel = "BestScoreLabel";
// The menu canvases. Each is a screen-space canvas (ADR-0086) hidden by default
// and shown by MenuSystem while its screen is current, so the whole flow is one
// flag per canvas rather than a rebuild.
inline constexpr std::string_view kTitleCanvas = "TitleCanvas";
inline constexpr std::string_view kTitleLabel = "TitleLabel";
inline constexpr std::string_view kPauseCanvas = "PauseCanvas";
inline constexpr std::string_view kPauseLabel = "PauseLabel";
inline constexpr std::string_view kGameOverCanvas = "GameOverCanvas";
inline constexpr std::string_view kGameOverLabel = "GameOverLabel";
// The settings screen: a canvas of labelled buttons that change the volume.
// Every control is a hit-tested UiElement, so a tap and a click both work.
inline constexpr std::string_view kSettingsCanvas = "SettingsCanvas";
inline constexpr std::string_view kSettingsTitleLabel = "SettingsTitleLabel";
inline constexpr std::string_view kSettingsVolumeDown = "SettingsVolumeDown";
inline constexpr std::string_view kSettingsVolumeUp = "SettingsVolumeUp";
inline constexpr std::string_view kSettingsVolumeValue = "SettingsVolumeValue";
inline constexpr std::string_view kSettingsBack = "SettingsBack";
inline constexpr std::string_view kSettingsButton = "SettingsButton";
inline constexpr std::string_view kPauseSettingsButton = "PauseSettingsButton";
} // namespace names

// The player's settings: what they can change about how the game *feels*, as
// opposed to how it plays. Today there is one knob -- the master volume -- and
// the shape is a plain value so a second is one field.
//
// Pure and free of the engine, like CollectState, so the clamping and the text
// round-trip are unit-testable with no scene and no store. Persisting it is the
// SettingsSystem's job; this type only knows what a valid setting is.
struct Settings {
    fe::f32 volume = config::kDefaultVolume;

    // Clamped, so a caller cannot author an inaudible-but-nonzero volume. Every
    // write goes through this, which is what keeps a loaded save honest.
    void set_volume(fe::f32 value) noexcept;

    // One step louder/quieter. `steps` is signed, so a button press is +1 or -1
    // and a test can move several at once. Clamping happens in the setter, so a
    // step past an end simply sticks to it.
    void adjust_volume(int steps) noexcept;

    [[nodiscard]] bool operator==(const Settings&) const noexcept = default;
};

// The settings as a stable, line-oriented text form ("volume=1.000"), the shape
// the storage seam keeps. One key per line so a future field is a new line and an
// unknown line is ignored rather than fatal -- a save written by a newer build
// must not stop an older one from starting.
[[nodiscard]] std::string serialize_settings(const Settings& settings);

// Parses `text` into `out`, leaving any field it does not mention at its current
// value and clamping every field it does. Returns false when nothing recognisable
// was read, so a caller can tell "a save from another version" from "a save that
// is corrupt" if it ever needs to. A malformed line is skipped, not fatal.
bool parse_settings(std::string_view text, Settings& out) noexcept;

// The score, the collected count, the clock, and the win/lose transition.
//
// A tiny state machine with an explicit result rather than a bool pair, so
// "playing", "won", and "lost" cannot be represented inconsistently. Every
// transition is guarded: a pickup after the game is over, or a tick past the
// limit, cannot corrupt the score or flip a finished result. Those guards are
// what the unit tests pin down.
//
// It also owns the *screen* the game is on, which is the phase the presentation
// is in rather than the outcome: a game opens on a title, runs while playing, can
// be paused, and ends on a game-over screen that leads back to the title. Keeping
// the screen here rather than in the systems means the flow title -> play -> game
// over -> title is one testable object with no scene and no engine, exactly as
// the rules are.
class CollectState {
public:
    enum class Result {
        Playing,
        Won,
        Lost,
    };

    // Which screen the game is showing. This is the presentation phase, distinct
    // from Result: the game is over (Result != Playing) exactly when the screen is
    // GameOver, but a game on the Title or Paused is still "playing" in the rules
    // sense until it is won or lost.
    enum class Screen {
        Title,    // the game has not started; the arena sits idle behind the title
        Playing,  // the clock runs and the character responds
        Paused,   // the player stopped the clock; the arena is frozen
        Settings, // the player is changing volume and look sensitivity
        GameOver, // the game ended; the result is shown and a press returns to title
    };

    explicit CollectState(int collectible_count = config::kCollectibleCount,
                          fe::f32 time_limit = config::kTimeLimit) noexcept;

    // --- Queries -----------------------------------------------------------
    [[nodiscard]] int score() const noexcept { return score_; }
    [[nodiscard]] int collected() const noexcept { return collected_; }
    [[nodiscard]] int collectibles_remaining() const noexcept { return collectibles_remaining_; }
    [[nodiscard]] int collectibles_total() const noexcept { return collectibles_total_; }
    [[nodiscard]] Result result() const noexcept { return result_; }
    [[nodiscard]] bool is_over() const noexcept { return result_ != Result::Playing; }
    [[nodiscard]] bool won() const noexcept { return result_ == Result::Won; }

    // Seconds left on the clock, and the limit it started from. The clock only
    // runs while the game is on the Playing screen, so pausing freezes it exactly
    // as it freezes the character.
    [[nodiscard]] fe::f32 time_left() const noexcept { return time_left_; }
    [[nodiscard]] fe::f32 time_limit() const noexcept { return time_limit_; }
    // The time as a whole number of seconds, rounded up, so a clock reading "0"
    // means "expired" and a run always shows at least the second it is in.
    [[nodiscard]] int seconds_left() const noexcept;

    // The screen the game is on, and the questions a system asks about it. A
    // gameplay system runs only while `is_playing()`; the menu system drives the
    // transitions between the rest.
    [[nodiscard]] Screen screen() const noexcept { return screen_; }
    [[nodiscard]] bool is_title() const noexcept { return screen_ == Screen::Title; }
    [[nodiscard]] bool is_playing() const noexcept { return screen_ == Screen::Playing; }
    [[nodiscard]] bool is_paused() const noexcept { return screen_ == Screen::Paused; }
    [[nodiscard]] bool is_settings() const noexcept { return screen_ == Screen::Settings; }

    // The player's settings. They live on the state rather than in a system so the
    // menu, the camera and the save all read one value, exactly as the score does.
    // The returned reference is mutable because a menu writes it in place; the
    // setter clamps, so a caller cannot store an out-of-range value through it.
    [[nodiscard]] Settings& settings() noexcept { return settings_; }
    [[nodiscard]] const Settings& settings() const noexcept { return settings_; }

    // The best score ever reached, and whether there is one. A fresh state has no
    // best score: "no save yet" is distinct from "the best score is zero", so the
    // HUD can leave the line blank on a first run rather than claim a best of 0.
    [[nodiscard]] int best_score() const noexcept { return best_score_; }
    [[nodiscard]] bool has_best_score() const noexcept { return has_best_score_; }

    // Sets the best score, e.g. from a loaded save. Ignores a value that is not
    // above the current best, so a stale save cannot lower it. A non-negative
    // value marks a best score as known.
    void set_best_score(int value) noexcept;

    // Raises the best score to the current score if the current game has passed
    // it, and reports whether it moved. The one place the running score and the
    // best score are compared, so the HUD, the log and the save all agree on when
    // a record was set.
    [[nodiscard]] bool record_best_score() noexcept;

    // A short, stable label for the result, used in the machine-checkable log line
    // ("result=win"). Stable so a CI grep can rely on it.
    [[nodiscard]] const char* result_name() const noexcept;

    // A short, stable label for the screen ("title", "playing", "paused",
    // "settings", "gameover"), for the same machine-checkable log line.
    [[nodiscard]] const char* screen_name() const noexcept;

    // Starts a fresh game from the title (or the game-over screen): the arena is
    // reset and the screen becomes Playing. This is the "press to play" edge.
    void start_game() noexcept;

    // Returns to the title from any screen, resetting the arena so the title shows
    // a fresh board. The flow's last step: game over -> title.
    void return_to_title() noexcept;

    // Puts the game on the title *without* requesting a rebuild, for the one
    // caller that already has a fresh board: the application before its first
    // frame.
    void show_title() noexcept;

    // Pauses and resumes a live game. Both are no-ops unless the game is on the
    // screen they act on, so a stray press cannot pause the title or resume a
    // finished game.
    void pause() noexcept;
    void resume() noexcept;

    // Opens and closes the settings screen. Opening is allowed from the title and
    // a pause -- the two screens that carry a settings button, and neither of
    // which is a live game, so a setting cannot be changed mid-round. Closing
    // returns to whichever of those it was opened from. Both are no-ops on the
    // wrong screen, matching pause()/resume().
    void open_settings() noexcept;
    void close_settings() noexcept;

    // Reconciles the screen with the rules after a transition that ends the game.
    // A system calls it after touching the rules so the presentation follows them;
    // the screen becomes GameOver exactly when the result stops being Playing.
    void sync_screen() noexcept;

    // --- Transitions -------------------------------------------------------
    // Records that one collectible was picked up: one fewer remaining and
    // `kPointsPerCollectible` more score. Clearing the arena adds `kGoalBonus` and
    // wins. A no-op once the game is over, and a no-op when nothing remains, so a
    // second pickup in the same frame cannot double-score. Returns whether a
    // collectible was actually consumed, so the caller knows whether to hide one.
    bool collect_one() noexcept;

    // Advances the clock by `delta` seconds. Only runs while the game is playing,
    // so a paused or title frame does not burn time. When the clock reaches zero
    // the game is lost. A no-op once the game is over.
    void tick(fe::f32 delta) noexcept;

    // Starts a fresh game with the same collectible count and time limit the first
    // one had.
    void reset() noexcept;

    // --- Restart handshake -------------------------------------------------
    // A system sets the request (the player pressed R); whoever owns the scene
    // consumes it and rebuilds. Keeping the flag here rather than in the system
    // means the restart survives the frame it was requested on, even though the
    // systems themselves are stateless.
    void request_restart() noexcept { restart_requested_ = true; }
    [[nodiscard]] bool restart_requested() const noexcept { return restart_requested_; }
    bool consume_restart() noexcept;

private:
    void set_over(Result result) noexcept {
        if (result_ == Result::Playing) {
            result_ = result;
        }
    }

    int collectibles_total_ = 0;
    int collectibles_remaining_ = 0;
    int collected_ = 0;
    int score_ = 0;
    fe::f32 time_limit_ = config::kTimeLimit;
    fe::f32 time_left_ = config::kTimeLimit;
    // The best score survives reset() and return_to_title(): it is the record
    // across games, not part of one game's state.
    int best_score_ = 0;
    bool has_best_score_ = false;
    Result result_ = Result::Playing;
    Screen screen_ = Screen::Playing;
    // The screen the settings screen was opened from, so closing it returns there
    // rather than always to the title.
    Screen settings_return_ = Screen::Title;
    Settings settings_{};
    bool restart_requested_ = false;
};

} // namespace collect3d
