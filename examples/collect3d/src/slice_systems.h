// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "slice_state.h"

#include "fe/core/Math.h"
#include "fe/platform/InputActions.h"
#include "fe/systems/System.h"

#include <string>
#include <string_view>

namespace fe {
class Node;
class Scene;
class SceneSchedule;
class ScriptHost;
class UserData;
struct FrameContext;
namespace physics3d {
class Physics3DSystem;
}
namespace audio {
class AudioSystem;
} // namespace audio
} // namespace fe

// The 3D collect game's gameplay systems (roadmap M11).
//
// Each is a fe::System: a unit of per-frame behaviour the SceneSchedule runs, in
// the order the game adds them (ADR-0008). None of them is a node method; the
// scene is only data, and these read and write it. That split is the whole point
// of the vertical slice -- gameplay is a system you add, not a class a node
// inherits -- and it is the same shape the M2.5 breakout slice proved in 2D.
//
// Every system resolves the nodes it cares about *by name* each frame rather than
// caching a Node*, because a scene reload or a rebuild (restart) replaces every
// node. The lookup is a handful of string compares over a small scene, so it is
// not worth caching and caching would be a stale-pointer bug waiting to happen.
//
// The rules themselves live in CollectState (see slice_state.h); these systems
// only translate scene events into calls on it, and its result back into the
// scene. That is what keeps the rules unit-testable with no engine running.

namespace collect3d {

// Walks the player toward the nearest collectible, so a headless run reaches an
// outcome with no player. It is inert unless autoplay is on, so the same schedule
// serves an interactive run and an automated one.
//
// It is placed *after* the script system, so it has the last word on the player's
// velocity: the character controller a live run uses would otherwise fight it for
// the body. Setting the velocity rather than the position keeps the physics in
// charge of collisions, so an autoplay run is still collided by the arena walls.
class AutoplaySystem final : public fe::System {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "AutoplaySystem"; }

    void update(const fe::FrameContext& context, fe::Scene& scene) override;

    void set_autoplay(bool autoplay) noexcept { autoplay_ = autoplay; }
    [[nodiscard]] bool autoplay() const noexcept { return autoplay_; }
    void set_state(CollectState* state) noexcept { state_ = state; }

    // The speed the player walks at in autoplay, in world units per second.
    fe::f32 speed = 6.0f;

private:
    bool autoplay_ = false;
    CollectState* state_ = nullptr;
};

// Picks up a collectible when the player walks into it.
//
// This is the scoring logic, and it is the one place the pickup is decided. It
// reads the player's world position and each collectible's, and consumes the
// nearest one within the pickup radius. Working from positions rather than from a
// physics contact keeps the pickup independent of the solver's shape and of
// whether the character has a collider at all, which is what makes it work on a
// gem the player walks *through* rather than bumps into.
//
// It removes the collected node, so the arena visibly empties as the player
// works; the restart rebuild puts the gems back.
class CollectibleSystem final : public fe::System {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "CollectibleSystem"; }

    void update(const fe::FrameContext& context, fe::Scene& scene) override;

    void set_state(CollectState* state) noexcept { state_ = state; }

private:
    // Starts the pickup sound on the audio node, if one is present.
    void play_pickup_sound(fe::Scene& scene) const;

    CollectState* state_ = nullptr;
};

// Runs the clock, ends the round when it expires, honours the restart key, and
// freezes the character whenever the game is not live. It is the bridge between
// the rules in CollectState and the scene.
//
// The clock only advances while the game is on the Playing screen, so pausing
// stops the countdown exactly as it stops the character -- which is what makes a
// pause fair rather than a way to lose.
class GoalSystem final : public fe::System {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "GoalSystem"; }

    void update(const fe::FrameContext& context, fe::Scene& scene) override;

    void set_state(CollectState* state) noexcept { state_ = state; }

private:
    // Stops the character dead when the game is not live, so a paused or finished
    // round is visually still rather than continuing to drift behind the menu.
    static void freeze_player(fe::Scene& scene);

    CollectState* state_ = nullptr;
};

// Writes the score, the clock, and the collected count into the scene's text
// labels every frame, and the win/lose result into the game-over screen's label,
// so the player can read the game's state on screen instead of in the log. It is
// the last system in the schedule: it reads the state the systems above it
// produced, and nothing reads its output.
//
// It owns no scene data of its own. The labels are created by the scene builder
// (or loaded from the authored document) and this system only fills in their
// text, which is why it is safe to rebuild the arena without touching it.
class HudSystem final : public fe::System {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "HudSystem"; }

    void update(const fe::FrameContext& context, fe::Scene& scene) override;

    void set_state(CollectState* state) noexcept { state_ = state; }

private:
    CollectState* state_ = nullptr;
};

// Saves the best score through the storage seam, and reads it back on the first
// frame. It is the only system that touches the best-score save, and it takes the
// store from the FrameContext rather than reaching for the engine, so a headless
// run handed no store simply does not save.
//
// The save happens on the transition *into* the game-over screen, not every
// frame: that is the moment the score is final, and it is one write per game
// rather than sixty a second.
class PersistSystem final : public fe::System {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "PersistSystem"; }

    void update(const fe::FrameContext& context, fe::Scene& scene) override;

    void set_state(CollectState* state) noexcept { state_ = state; }
    void set_key(std::string_view key) noexcept { key_ = std::string(key); }

private:
    // Loads the saved best score into the rules once, on the first update. A
    // missing key (a first run) leaves the rules with no best score rather than
    // one of zero.
    void load_once(fe::UserData& store);

    CollectState* state_ = nullptr;
    std::string key_ = config::kBestScoreKey;
    CollectState::Screen last_screen_ = CollectState::Screen::Playing;
    bool loaded_ = false;
};

// Loads the player's settings, applies them to the live game, and saves them when
// they change. It takes the store from the FrameContext for the same reason
// PersistSystem does, and writes under its own key so a settings save and a
// best-score save never overwrite each other.
//
// It applies the volume to the audio system's master gain, so the mixer hears it
// the same frame and neither value needs a second copy anywhere. It saves on the
// *edge* of a change rather than every frame, which keeps a settings screen from
// writing a file sixty times a second while a button is held.
class SettingsSystem final : public fe::System {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "SettingsSystem"; }

    void update(const fe::FrameContext& context, fe::Scene& scene) override;

    void set_state(CollectState* state) noexcept { state_ = state; }
    void set_audio(fe::audio::AudioSystem* audio) noexcept { audio_ = audio; }
    void set_key(std::string_view key) noexcept { key_ = std::string(key); }

private:
    void load_once(fe::UserData& store);

    CollectState* state_ = nullptr;
    fe::audio::AudioSystem* audio_ = nullptr;
    std::string key_ = config::kSettingsKey;
    Settings last_saved_{};
    bool loaded_ = false;
    bool primed_ = false;
};

// Drives the screen flow: title -> play -> game over -> title, with a pause in
// between.
//
// It is the one system that *decides* rather than reacts, because a menu is the
// one place a press means "change what the game is doing" rather than "do
// something in the game". It reads the confirm and pause actions plus the UI
// click state UiInputSystem wrote earlier this frame, so a button is a real
// hit-tested target rather than a full-screen "any press" zone.
//
// It owns no scene data: the menu canvases are built once by the scene builder
// and this system only toggles their `visible` flag to match the screen, so
// switching screens is one bool per canvas, not a rebuild.
class MenuSystem final : public fe::System {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "MenuSystem"; }

    void update(const fe::FrameContext& context, fe::Scene& scene) override;

    void set_state(CollectState* state) noexcept { state_ = state; }
    void set_confirm_action(fe::ActionMap::Id confirm) noexcept { confirm_ = confirm; }
    void set_pause_action(fe::ActionMap::Id pause) noexcept { pause_ = pause; }

private:
    // Shows the canvas for the current screen and hides the others. Idempotent,
    // so it is safe to call every frame.
    static void apply_visibility(fe::Scene& scene, CollectState::Screen screen);

    // Writes the settings screen's value readouts from the state.
    void refresh_settings_labels(fe::Scene& scene);

    // Reads a click on a named element, false when the element or the scene is
    // not there.
    [[nodiscard]] static bool clicked(const fe::Scene& scene, std::string_view name);

    CollectState* state_ = nullptr;
    fe::ActionMap::Id confirm_ = fe::ActionMap::kNone;
    fe::ActionMap::Id pause_ = fe::ActionMap::kNone;
};

// The gameplay actions collect3d declares, resolved once so the systems and the
// application agree on the handles. `declare_collect3d_actions` installs the
// default device bindings: WASD and the arrow keys to move, Space to jump and to
// advance a menu, P to pause.
struct CollectActions {
    fe::ActionMap::Id move_forward = fe::ActionMap::kNone;
    fe::ActionMap::Id move_back = fe::ActionMap::kNone;
    fe::ActionMap::Id move_left = fe::ActionMap::kNone;
    fe::ActionMap::Id move_right = fe::ActionMap::kNone;
    fe::ActionMap::Id jump = fe::ActionMap::kNone;
    fe::ActionMap::Id confirm = fe::ActionMap::kNone;
    fe::ActionMap::Id pause = fe::ActionMap::kNone;
};

// Declares collect3d's actions on `actions` and installs their default bindings.
[[nodiscard]] CollectActions declare_collect3d_actions(fe::ActionMap& actions);

// Installs the game's systems on `schedule` in the order the game runs them
// (ADR-0008): menu, settings, the gameplay behaviours, autoplay, physics3d,
// collectible pickup, goal/clock, persist, HUD. `state` is borrowed and must
// outlive the schedule.
//
// `host` owns the behaviours the arena's nodes carry (the character controller,
// the follow camera, the collectible spin). It is borrowed and may be null: a
// test that drives the systems without behaviours gets no character movement from
// a script, which is what a pure-system test wants. When it is present, the
// script system is installed in the right place -- after the menu and the
// settings, before the physics -- so a behaviour reads this frame's decisions and
// its motion is integrated this frame.
//
// The physics system is created here and returned so the caller can read its
// contacts or tune it. The audio system is deliberately not added: the caller
// owns the device and the engine's audio system, so it adopts and appends that
// itself (last, so a cue fired this frame is mixed this frame).
//
// `autoplay` walks the player toward the nearest collectible, so a headless run
// reaches an outcome with no player.
[[nodiscard]] fe::physics3d::Physics3DSystem* install_collect3d_systems(fe::SceneSchedule& schedule,
                                                                        CollectState& state, bool autoplay,
                                                                        fe::ScriptHost* host = nullptr);

// As above, but wires the systems to `actions` so input arrives through the
// action layer. The overload above is the no-actions path a headless tool and a
// test use.
[[nodiscard]] fe::physics3d::Physics3DSystem* install_collect3d_systems(fe::SceneSchedule& schedule,
                                                                        CollectState& state, bool autoplay,
                                                                        const CollectActions& bindings,
                                                                        fe::ScriptHost* host = nullptr);

} // namespace collect3d
