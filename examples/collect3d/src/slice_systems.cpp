// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "slice_systems.h"

#include "fe/audio/AudioSource.h"
#include "fe/audio/AudioSystem.h"
#include "fe/core/Log.h"
#include "fe/physics3d/Physics3DSystem.h"
#include "fe/physics3d/RigidBody3D.h"
#include "fe/platform/InputState.h"
#include "fe/platform/UserData.h"
#include "fe/player/BuiltinBehaviours.h"
#include "fe/runtime/AudioListenerSystem.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/script/ScriptHost.h"
#include "fe/script/ScriptSystem.h"
#include "fe/systems/FrameContext.h"
#include "fe/systems/SceneSchedule.h"
#include "fe/ui/UiElement.h"
#include "fe/ui/UiInputSystem.h"

#include <charconv>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace collect3d {

namespace {

// The squared pickup radius, so the per-gem test is a comparison rather than a
// square root. Compared against the squared distance, the two agree exactly.
constexpr fe::f32 kPickupRadiusSquared = config::kPickupRadius * config::kPickupRadius;

} // namespace

void AutoplaySystem::update(const fe::FrameContext& context, fe::Scene& scene) {
    if (!autoplay_ || state_ == nullptr || !state_->is_playing()) {
        return;
    }

    fe::Node* player = scene.find_node_by_name(names::kPlayer);
    if (player == nullptr) {
        return;
    }
    auto* body = player->get_component<fe::physics3d::RigidBody3D>();
    if (body == nullptr) {
        return;
    }

    // The nearest remaining collectible, in the ground plane. The y term is
    // ignored so the walk is horizontal: the character controller owns the
    // vertical axis, and steering into the floor would only stall the run.
    fe::Node* nearest = nullptr;
    fe::f32 nearest_squared = 0.0f;
    const fe::Vec3 player_position = player->transform().position;
    scene.for_each_node([&](fe::Node& node) {
        if (node.name() != names::kCollectible) {
            return;
        }
        const fe::Vec3 delta = node.transform().position - player_position;
        const fe::f32 distance_squared = delta.x * delta.x + delta.z * delta.z;
        if (nearest == nullptr || distance_squared < nearest_squared) {
            nearest = &node;
            nearest_squared = distance_squared;
        }
    });

    fe::Vec3 direction{};
    if (nearest != nullptr) {
        const fe::Vec3 delta = nearest->transform().position - player_position;
        direction = fe::Vec3{delta.x, 0.0f, delta.z};
    }
    const fe::f32 length = direction.length();
    if (length > 1e-4f) {
        direction = direction * (speed / length);
    } else {
        direction = fe::Vec3{};
    }

    // Set the horizontal velocity; leave the vertical to the physics, so gravity
    // and the floor keep the character grounded.
    body->velocity.x = direction.x;
    body->velocity.z = direction.z;
    body->wake();
    (void)context;
}

void CollectibleSystem::play_pickup_sound(fe::Scene& scene, const fe::Vec3& at) const {
    fe::Node* node = scene.find_node_by_name(names::kAudio);
    if (node == nullptr) {
        return;
    }
    // Place the cue where the gem was before playing it. The audio node is a
    // single reusable one-shot, so moving it is how each pickup is heard from its
    // own spot rather than from wherever the last one was; a non-positional cue
    // ignores the position, so this is harmless when the scene authors it flat.
    node->transform().position = at;
    if (auto* source = node->get_component<fe::audio::AudioSource>(); source != nullptr) {
        source->play();
    }
}

void CollectibleSystem::update(const fe::FrameContext& context, fe::Scene& scene) {
    (void)context;
    // Pickups only happen in a live game: on the title, the game-over screen, and
    // while paused the arena is not being played, so walking into a gem does not
    // score.
    if (state_ == nullptr || !state_->is_playing()) {
        return;
    }

    const fe::Node* player = scene.find_node_by_name(names::kPlayer);
    if (player == nullptr) {
        return;
    }
    const fe::Vec3 player_position = player->transform().position;

    // Gather the collected nodes first, then remove them, so removing a node
    // during the walk cannot disturb it -- the same two-pass shape the breakout
    // brick break uses.
    std::vector<fe::Node*> picked;
    scene.for_each_node([&](fe::Node& node) {
        if (node.name() != names::kCollectible) {
            return;
        }
        const fe::Vec3 delta = node.transform().position - player_position;
        if (delta.length_squared() <= kPickupRadiusSquared) {
            picked.push_back(&node);
        }
    });

    for (fe::Node* node : picked) {
        // A pickup may fail (the arena was already cleared by an earlier pickup
        // in the same frame); when it does, the node stays, so the count and the
        // scene never disagree.
        if (!state_->collect_one()) {
            break;
        }
        play_pickup_sound(scene, node->transform().position);
        if (fe::Node* parent = node->parent(); parent != nullptr) {
            static_cast<void>(parent->remove_child(node));
        } else {
            static_cast<void>(scene.remove_root(node));
        }
        if (state_->is_over()) {
            // Clearing the last gem wins, so the presentation follows the rules
            // now rather than a frame later.
            state_->sync_screen();
            FE_LOG_INFO("collect3d: arena cleared (result=%s score=%d)", state_->result_name(), state_->score());
        }
    }
}

void GoalSystem::freeze_player(fe::Scene& scene) {
    fe::Node* player = scene.find_node_by_name(names::kPlayer);
    if (player == nullptr) {
        return;
    }
    if (auto* body = player->get_component<fe::physics3d::RigidBody3D>(); body != nullptr) {
        body->velocity = fe::Vec3{};
        body->angular_velocity = fe::Vec3{};
    }
}

void GoalSystem::update(const fe::FrameContext& context, fe::Scene& scene) {
    if (state_ == nullptr) {
        return;
    }

    // R restarts the round wherever the player is: the rules reset and the screen
    // returns to play, and the arena rebuild is requested so the collected gems
    // come back.
    if (const fe::InputState* input = context.input; input != nullptr) {
        if (input->was_key_pressed(fe::Key::R)) {
            state_->start_game();
            state_->request_restart();
        }
    }

    // The clock runs only while the round is live, so pausing stops the countdown
    // exactly as it stops the character.
    if (state_->is_playing()) {
        state_->tick(context.delta_time);
        if (state_->is_over()) {
            state_->sync_screen();
            FE_LOG_INFO("collect3d: time up (result=%s score=%d)", state_->result_name(), state_->score());
        }
    } else {
        // A paused or finished round freezes the character, so it is visually
        // still rather than drifting behind the menu.
        freeze_player(scene);
    }
}

void HudSystem::update(const fe::FrameContext& context, fe::Scene& scene) {
    (void)context;
    if (state_ == nullptr) {
        return;
    }

    const auto set_text = [&scene](std::string_view name, std::string text) {
        if (fe::Node* node = scene.find_node_by_name(name); node != nullptr) {
            if (auto* element = node->get_component<fe::ui::UiElement>(); element != nullptr) {
                element->text = std::move(text);
            }
        }
    };

    set_text(names::kScoreLabel, "SCORE " + std::to_string(state_->score()));
    set_text(names::kCollectedLabel,
             "GEMS " + std::to_string(state_->collected()) + "/" + std::to_string(state_->collectibles_total()));
    set_text(names::kTimeLabel, "TIME " + std::to_string(state_->seconds_left()));

    // The game-over screen's message. A win reads differently from a loss, so the
    // end state is legible in the browser without reading the console.
    if (fe::Node* node = scene.find_node_by_name(names::kGameOverLabel); node != nullptr) {
        if (auto* element = node->get_component<fe::ui::UiElement>(); element != nullptr) {
            element->text = state_->won() ? config::kWinText : config::kGameOverText;
        }
    }

    // The best score, blank until there is one, so a first run does not claim a
    // best of 0.
    set_text(names::kBestScoreLabel,
             state_->has_best_score() ? "BEST " + std::to_string(state_->best_score()) : std::string{});
}

void PersistSystem::load_once(fe::UserData& store) {
    loaded_ = true;
    const fe::Result<std::string> saved = store.read_text(key_);
    if (!saved) {
        // No save yet: a first run. Leaving has_best_score() false keeps the HUD
        // blank rather than showing a best of zero.
        return;
    }

    int value = 0;
    const char* begin = saved.value().data();
    const char* end = begin + saved.value().size();
    const auto [ptr, ec] = std::from_chars(begin, end, value);
    if (ec != std::errc{} || ptr != end || value < 0) {
        // A save that is not a plain non-negative integer is treated as absent
        // rather than fatal: a corrupt file must not stop the game from starting.
        FE_LOG_WARN("collect3d: ignoring unreadable best score '%s'", saved.value().c_str());
        return;
    }
    state_->set_best_score(value);
    FE_LOG_INFO("collect3d: loaded best score %d", value);
}

void PersistSystem::update(const fe::FrameContext& context, fe::Scene& scene) {
    (void)scene;
    if (state_ == nullptr) {
        return;
    }

    fe::UserData* store = context.user_data;
    if (!loaded_ && store != nullptr) {
        load_once(*store);
    }

    const CollectState::Screen screen = state_->screen();

    // Only a game that has actually been played counts as a record. The title and
    // the settings screen are reachable before any game, so a visit to either
    // must not turn a blank best-score line into a misleading "BEST 0".
    if (screen != CollectState::Screen::Title && screen != CollectState::Screen::Settings) {
        if (state_->record_best_score()) {
            FE_LOG_INFO("collect3d: new best score %d", state_->best_score());
        }
    }

    // Persist on the edge *into* the game-over screen: the score is final there,
    // and it is one write per game instead of one per frame.
    const bool just_ended = screen == CollectState::Screen::GameOver && last_screen_ != CollectState::Screen::GameOver;
    last_screen_ = screen;

    if (!just_ended || store == nullptr) {
        return;
    }

    const std::string text = std::to_string(state_->best_score());
    if (fe::Error error = store->write(key_, std::string_view(text)); error) {
        FE_LOG_WARN("collect3d: could not save the best score: %s", error.message().c_str());
        return;
    }
    FE_LOG_INFO("collect3d: saved best score %d", state_->best_score());
}

void SettingsSystem::load_once(fe::UserData& store) {
    loaded_ = true;
    const fe::Result<std::string> saved = store.read_text(key_);
    if (!saved) {
        // No save yet: a first run. The defaults stand, which is the game exactly
        // as it was authored.
        return;
    }

    Settings parsed = state_->settings();
    if (!parse_settings(saved.value(), parsed)) {
        FE_LOG_WARN("collect3d: ignoring unreadable settings '%s'", saved.value().c_str());
        return;
    }
    state_->settings() = parsed;
    FE_LOG_INFO("collect3d: loaded settings volume=%.2f", static_cast<double>(parsed.volume));
}

void SettingsSystem::update(const fe::FrameContext& context, fe::Scene& scene) {
    (void)scene;
    if (state_ == nullptr) {
        return;
    }

    fe::UserData* store = context.user_data;
    if (!loaded_ && store != nullptr) {
        load_once(*store);
    }

    // Mirror the volume onto the audio system's master gain every frame, so a
    // value loaded this frame is in force this frame and a value the player
    // changed on the menu is heard immediately. A null audio system (a headless
    // test) is simply nothing to apply it to.
    if (audio_ != nullptr) {
        audio_->set_master_gain(state_->settings().volume);
    }

    // Save on the edge of a change. The first frame primes the baseline without
    // writing, so opening the game does not rewrite the file it just read.
    const Settings current = state_->settings();
    if (!primed_) {
        primed_ = true;
        last_saved_ = current;
        return;
    }
    if (current == last_saved_) {
        return;
    }
    last_saved_ = current;

    if (store == nullptr) {
        FE_LOG_INFO("collect3d: settings changed with no store; not saved");
        return;
    }

    const std::string text = serialize_settings(current);
    if (fe::Error error = store->write(key_, std::string_view(text)); error) {
        FE_LOG_WARN("collect3d: could not save settings: %s", error.message().c_str());
        return;
    }
    FE_LOG_INFO("collect3d: saved settings volume=%.2f", static_cast<double>(current.volume));
}

void MenuSystem::apply_visibility(fe::Scene& scene, CollectState::Screen screen) {
    // One canvas per screen, each shown exactly on its own screen. The HUD
    // counters stay visible only while playing, so the title and the end message
    // are not competing with a score from a game that is not running.
    const auto set_canvas = [&scene](std::string_view name, bool visible) {
        if (fe::Node* node = scene.find_node_by_name(name); node != nullptr) {
            if (auto* canvas = node->get_component<fe::ui::UiCanvas>(); canvas != nullptr) {
                canvas->visible = visible;
            }
        }
    };
    set_canvas(names::kTitleCanvas, screen == CollectState::Screen::Title);
    set_canvas(names::kPauseCanvas, screen == CollectState::Screen::Paused);
    set_canvas(names::kSettingsCanvas, screen == CollectState::Screen::Settings);
    set_canvas(names::kGameOverCanvas, screen == CollectState::Screen::GameOver);
    set_canvas(names::kHudCanvas, screen == CollectState::Screen::Playing);
}

bool MenuSystem::clicked(const fe::Scene& scene, std::string_view name) {
    const fe::Node* node = scene.find_node_by_name(name);
    if (node == nullptr) {
        return false;
    }
    const auto* element = node->get_component<fe::ui::UiElement>();
    return element != nullptr && element->interaction.clicked;
}

void MenuSystem::refresh_settings_labels(fe::Scene& scene) {
    // The value readouts are written here rather than by the settings system, so
    // the drawing of a value and the value itself are not split across two
    // systems -- the same split the HUD and PersistSystem keep.
    const auto set_text = [&scene](std::string_view name, std::string text) {
        if (fe::Node* node = scene.find_node_by_name(name); node != nullptr) {
            if (auto* element = node->get_component<fe::ui::UiElement>(); element != nullptr) {
                element->text = std::move(text);
            }
        }
    };
    const Settings& settings = state_->settings();
    const int percent = static_cast<int>(settings.volume * 100.0f + 0.5f);
    set_text(names::kSettingsVolumeValue, "VOLUME " + std::to_string(percent) + "%");
}

void MenuSystem::update(const fe::FrameContext& context, fe::Scene& scene) {
    if (state_ == nullptr) {
        return;
    }

    // The presentation follows the rules: if something ended the game since the
    // last frame, the screen is the game-over screen now.
    state_->sync_screen();

    // Whether the player asked to "go" this frame: the confirm action, a UI button
    // click, or -- with no action map -- the Space/Enter fallback. One flag, so a
    // click, a tap and a key all advance a menu identically.
    bool confirm = false;
    if (context.actions != nullptr && confirm_ != fe::ActionMap::kNone) {
        confirm = context.actions->was_pressed(confirm_);
    } else if (const fe::InputState* input = context.input; input != nullptr) {
        confirm = input->was_key_pressed(fe::Key::Space) || input->was_key_pressed(fe::Key::Enter);
    }

    // A menu button is a hit-tested target: the title's "press to play" and the
    // game-over screen's "press to continue" carry a UiElement, and a click on one
    // is the same as the confirm press.
    confirm = confirm || clicked(scene, names::kTitleLabel) || clicked(scene, names::kGameOverLabel) ||
              clicked(scene, names::kPauseLabel);

    // The settings button on the title and pause screens opens the settings
    // screen. Read separately from `confirm`, so tapping "SETTINGS" does not also
    // start or resume the game.
    const bool open_settings = clicked(scene, names::kSettingsButton) || clicked(scene, names::kPauseSettingsButton);

    bool pause_toggle = false;
    if (context.actions != nullptr && pause_ != fe::ActionMap::kNone) {
        pause_toggle = context.actions->was_pressed(pause_);
    } else if (const fe::InputState* input = context.input; input != nullptr) {
        pause_toggle = input->was_key_pressed(fe::Key::P);
    }

    switch (state_->screen()) {
    case CollectState::Screen::Title:
        if (open_settings) {
            state_->open_settings();
        } else if (confirm) {
            state_->start_game();
        }
        break;
    case CollectState::Screen::Playing:
        if (pause_toggle) {
            state_->pause();
        }
        break;
    case CollectState::Screen::Paused:
        if (open_settings) {
            state_->open_settings();
        } else if (pause_toggle || confirm) {
            state_->resume();
        }
        break;
    case CollectState::Screen::Settings:
        // Each control is one click on one button. The adjustments clamp, so a
        // click past an end sticks to it; the settings system saves the change on
        // the next frame it sees it.
        if (clicked(scene, names::kSettingsVolumeDown)) {
            state_->settings().adjust_volume(-1);
        }
        if (clicked(scene, names::kSettingsVolumeUp)) {
            state_->settings().adjust_volume(+1);
        }
        if (clicked(scene, names::kSettingsBack) || pause_toggle) {
            state_->close_settings();
        }
        break;
    case CollectState::Screen::GameOver:
        if (confirm) {
            state_->return_to_title();
        }
        break;
    }

    // The settings screen's readouts follow the values just changed, so a click is
    // reflected on the same frame it happened.
    if (state_->is_settings()) {
        refresh_settings_labels(scene);
    }

    // Show the canvas for whatever screen we ended on.
    apply_visibility(scene, state_->screen());
}

CollectActions declare_collect3d_actions(fe::ActionMap& actions) {
    CollectActions bindings;
    // The movement actions reuse the engine's standard names, so the built-in
    // character controller reads the identical handles a game's own bindings
    // document would set. `declare_builtin_actions` declares and binds them
    // (WASD, the arrows, Space to jump, touch regions for a phone); this resolves
    // the handles the menu and the pause read on top of that.
    fe::player::declare_builtin_actions(actions);
    bindings.move_forward = actions.find(fe::player::actions::kMoveForward);
    bindings.move_back = actions.find(fe::player::actions::kMoveBack);
    bindings.move_left = actions.find(fe::player::actions::kMoveLeft);
    bindings.move_right = actions.find(fe::player::actions::kMoveRight);
    bindings.jump = actions.find(fe::player::actions::kJump);

    // The menu's "go" press and the pause toggle. Confirm reuses the jump action,
    // so one button jumps and advances a menu; pause is its own action with no
    // touch region (the pause canvas is advanced by the same confirm press).
    bindings.confirm = bindings.jump;
    bindings.pause = actions.add("Pause");
    actions.bind_key(bindings.pause, fe::Key::P);
    return bindings;
}

fe::physics3d::Physics3DSystem* install_collect3d_systems(fe::SceneSchedule& schedule, CollectState& state,
                                                          bool autoplay, fe::ScriptHost* host) {
    return install_collect3d_systems(schedule, state, autoplay, CollectActions{}, host);
}

fe::physics3d::Physics3DSystem* install_collect3d_systems(fe::SceneSchedule& schedule, CollectState& state,
                                                          bool autoplay, const CollectActions& bindings,
                                                          fe::ScriptHost* host) {
    // The UI input system runs first, so the interaction state a menu reads this
    // frame is already written before any gameplay system looks at it.
    static_cast<void>(schedule.add_system<fe::ui::UiInputSystem>());

    // The menu system runs before the gameplay systems: it is what decides the
    // screen, and the gameplay systems read that decision this same frame.
    auto* menu = schedule.add_system<MenuSystem>();
    menu->set_state(&state);
    menu->set_confirm_action(bindings.confirm);
    menu->set_pause_action(bindings.pause);

    // The settings system runs right after the menu, so a click that changed a
    // setting this frame is applied and saved on the same frame.
    auto* settings = schedule.add_system<SettingsSystem>();
    settings->set_state(&state);
    settings->set_key(config::kSettingsKey);

    // The gameplay behaviours (the character controller, the follow camera, the
    // collectible spin) run here when a host is present: after the menu has
    // decided the screen and before the physics integrates the motion they
    // produce. A test with no host skips this and drives the systems directly.
    if (host != nullptr) {
        static_cast<void>(schedule.add_system<fe::ScriptSystem>(*host));
    }

    // Autoplay runs after the behaviours, so its velocity is the one the physics
    // integrates.
    auto* autopilot = schedule.add_system<AutoplaySystem>();
    autopilot->set_autoplay(autoplay);
    autopilot->set_state(&state);

    auto* physics = schedule.add_system<fe::physics3d::Physics3DSystem>();

    // The pickup runs after the physics has moved the player this frame, so a
    // gem the player just reached is collected the same frame it was reached.
    auto* collectible = schedule.add_system<CollectibleSystem>();
    collectible->set_state(&state);

    auto* goal = schedule.add_system<GoalSystem>();
    goal->set_state(&state);

    // Persistence runs after the rules have settled the screen for this frame and
    // before the HUD, so a best score set this frame is drawn this frame.
    auto* persist = schedule.add_system<PersistSystem>();
    persist->set_state(&state);
    persist->set_key(config::kBestScoreKey);

    // Last, so it reads the state the systems above it produced this frame.
    auto* hud = schedule.add_system<HudSystem>();
    hud->set_state(&state);

    return physics;
}

void install_collect3d_audio(fe::SceneSchedule& schedule, fe::audio::AudioSystem& audio) {
    // The mixer group bus the game's effects route to. The settings screen drives
    // the master bus, so the effects bus is an independent layer.
    audio.set_bus_gain(config::kSfxBusName, config::kSfxBusGain);

    // The listener before the audio system: the AudioSystem mixes the sources'
    // world positions against whatever listener it holds when its own update runs,
    // so the listener must have read this frame's camera first.
    schedule.add_system<fe::AudioListenerSystem>(&audio);
}

} // namespace collect3d
