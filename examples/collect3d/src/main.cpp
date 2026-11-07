// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// Gem Rush — the M11 vertical slice (a finished 3D game).
//
// One small, complete third-person game, built entirely on the engine's public
// API, exactly as an external project would build it. It is the 3D counterpart of
// the M2.5 breakout slice and the roadmap's M11 integration test: in one artifact
// it exercises the scene graph, the resource manager, 3D physics, the built-in
// gameplay behaviours (the character controller and the follow camera — the
// behaviours a browser can run, since it has no dynamic loader), the audio mixer,
// scene serialization, input through the action layer, the UI (HUD and menus),
// and the storage seam.
//
// The gameplay rules live in CollectState (slice_state.h), a pure class with no
// engine dependency, so they are unit-tested directly. The arena layout and the
// system wiring live in slice_scene.h, shared with the tests so the tests prove
// the shipped wiring, not a copy. This file is only the application shell: it
// loads or builds the scene, opens the audio device, drives the loop, and reports
// a machine-checkable result.
//
// Usage:
//   fork_collect3d                 play interactively (needs a display)
//   fork_collect3d --headless N    run N frames with no window; autoplay clears
//                                  the arena so a run reaches win/lose and exits
//   fork_collect3d --null-window   run with no display or GL context
//   fork_collect3d --save-scene P  write the built scene to P and exit
//
// Controls (interactive):
//   WASD / arrows       walk
//   Space               jump, start the game, continue after game over
//   P                   pause and resume
//   R                   restart the round
//   Escape              quit

#include "slice_scene.h"
#include "slice_state.h"
#include "slice_systems.h"

#include "fe/audio/AudioBackend.h"
#include "fe/audio/AudioSource.h"
#include "fe/audio/AudioSystem.h"
#include "fe/core/Log.h"
#include "fe/player/BuiltinBehaviours.h"
#include "fe/renderer3d/Renderer3D.h"
#include "fe/resources/Resource.h"
#include "fe/runtime/Application.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/script/ScriptBinding.h"
#include "fe/script/ScriptHost.h"
#include "fe/script/ScriptRegistry.h"
#include "fe/serialization/SceneSerializer.h"
#include "fe/systems/SceneSchedule.h"

#include <cstdlib>
#include <memory>
#include <string>
#include <string_view>

// A browser page owns the frame schedule, so the web build cannot block in
// run()'s while loop: the module returns to the page and requestAnimationFrame
// calls back once per frame. See docs/adr/0071-web-first-2d-shipping-path.md.
#if defined(FE_PLATFORM_WEB)
#include <emscripten.h>
#endif

namespace {

// The authored scene, loaded over the code-built one when present, and the cue.
constexpr const char* kScenePath = "examples/collect3d/scenes/collect3d.fescene";
constexpr const char* kPickupPath = "wav:examples/collect3d/assets/audio/pickup.wav";
constexpr const char* kFontPath = "font:examples/collect3d/assets/fonts/hud.fefont";
constexpr const char* kBoxPath = "obj:examples/collect3d/assets/meshes/cube.obj";

// Loads the arena's mesh and its font. A missing asset is not fatal: the arena
// is built anyway (with an in-code box) and simply draws blank, which is what a
// headless run wants.
collect3d::SceneAssets load_assets(fe::ResourceManager& resources) {
    collect3d::SceneAssets assets;
    const auto load = [&resources](const char* path) -> std::shared_ptr<fe::Resource> {
        auto resource = resources.load(path);
        if (!resource) {
            FE_LOG_WARN("collect3d: could not load '%s' (%s)", path, resource.error().message().c_str());
            return nullptr;
        }
        return resource.value();
    };
    // The shared unit cube every box is scaled from, and the font that drives
    // every label. Loading them by their scheme path is what makes the arena draw.
    assets.box = load(kBoxPath);
    assets.font = load(kFontPath);
    return assets;
}

class Collect3DApp final : public fe::Application {
public:
    Collect3DApp(fe::Engine::Config config, bool headless, std::string scene_to_save)
        : fe::Application(config), headless_(headless), scene_to_save_(std::move(scene_to_save)) {}

protected:
    fe::Error on_start() override {
        // The behaviours a scene can bind by name must be registered before the
        // scene is loaded, so a `CharacterController3D` script in the document
        // resolves to a live instance.
        fe::player::register_builtin_behaviours(registry_);
        binding_ = std::make_unique<fe::script::ScriptBinding>(host_, registry_);

        // Build the canonical arena first, then (unless we are only regenerating
        // the scene file) load the authored document over the top of it. The
        // document is a serialization of exactly this layout, so both paths give
        // the same game; loading it is what exercises the real scene-load path.
        collect3d::build_collect3d_scene(scene(), load_assets(engine().resources()));
        // The code-built arena carries no scripts; the behaviours are attached
        // here, which is what a live run of the fallback path uses. The authored
        // document binds them through the ScriptBinding instead.
        collect3d::attach_collect3d_behaviours(host_, scene());

        if (!scene_to_save_.empty()) {
            if (fe::Error error =
                    fe::serialization::scene::save(scene(), scene_to_save_, &engine().resources(), binding_.get());
                error) {
                return error;
            }
            FE_LOG_INFO("collect3d: saved the built scene to '%s'", scene_to_save_.c_str());
            request_exit();
            return fe::Error::none();
        }

        if (fe::Error error =
                fe::serialization::scene::load(kScenePath, scene(), &engine().resources(), binding_.get());
            error) {
            FE_LOG_WARN("collect3d: %s; using the scene built in code", error.message().c_str());
            // The load cleared the scene before failing, so rebuild it and
            // re-attach the behaviours.
            collect3d::build_collect3d_scene(scene(), load_assets(engine().resources()));
            collect3d::attach_collect3d_behaviours(host_, scene());
        } else {
            FE_LOG_INFO("collect3d: loaded '%s' (%zu nodes)", kScenePath, scene().node_count());
        }

        if (scene().find_node_by_name(collect3d::names::kPlayer) == nullptr) {
            return fe::Error("collect3d: the scene is missing a 'Player' node");
        }

        // Declare the game's actions before installing the systems, so the systems
        // get handles to bind to. The application evaluates the map each frame and
        // hands it to the systems through FrameContext, which is what lets a touch
        // and a key drive the identical gameplay code.
        const collect3d::CollectActions bindings = collect3d::declare_collect3d_actions(actions());

        static_cast<void>(collect3d::install_collect3d_systems(schedule(), state_, headless_, bindings, &host_));
        adopt_audio_system();
        attach_sound();

        // Open on the title, unless this is a headless autoplay run, which drives a
        // live game with no player and no menu.
        if (!headless_) {
            state_.show_title();
        }

        FE_LOG_INFO("collect3d: arena %dx%d, %d gems, %.0fs",
                    static_cast<int>(collect3d::config::kArenaHalfExtent * 2.0f),
                    static_cast<int>(collect3d::config::kArenaHalfExtent * 2.0f), state_.collectibles_total(),
                    static_cast<double>(state_.time_limit()));
        return fe::Error::none();
    }

    void on_update(fe::f32 /*delta_time*/) override {
        if (!headless_ && engine().input().was_key_pressed(fe::Key::Escape)) {
            request_exit();
        }

        // A browser keeps the audio context suspended until a gesture, so the
        // first frames are silent. Report the transition once in each direction.
        if (audio_device_ != nullptr && audio_device_->needs_user_gesture() != audio_gesture_reported_) {
            audio_gesture_reported_ = audio_device_->needs_user_gesture();
            FE_LOG_INFO(audio_gesture_reported_ ? "collect3d: audio suspended until the first user gesture"
                                                : "collect3d: audio resumed by a user gesture");
        }

        // Restart: whoever owns the scene rebuilds it. The systems look their nodes
        // up by name each frame, so only the state and the scene change.
        if (state_.consume_restart()) {
            rebuild();
            FE_LOG_INFO("collect3d: restarted");
        }

        if (headless_ && state_.is_over()) {
            request_exit();
        }
    }

    void on_stop() override {
        if (audio_system_ != nullptr) {
            // The machine-checkable proof the mixer ran and produced non-silent
            // audio, whether or not a device was opened.
            FE_LOG_INFO("collect3d: audio mixed %llu frames, peak %.4f",
                        static_cast<unsigned long long>(audio_system_->mixed_frames()),
                        static_cast<double>(audio_system_->peak_max()));
        }
        // The line a headless run greps for its outcome.
        FE_LOG_INFO("collect3d: score=%d collected=%d/%d result=%s", state_.score(), state_.collected(),
                    state_.collectibles_total(), state_.result_name());
    }

    // The engine's frame draws the world first (this hook), then 2D and the UI
    // over it. render() is the single decision point: it looks up the scene's
    // primary camera and returns 0 immediately when there is none, so the game
    // does no 3D work on a screen that has no camera and needs no separate "am I
    // a 3D game" question. Drawing here rather than in on_render() is what puts
    // the HUD on top of the arena instead of under it.
    void on_render_world(fe::f32) override {
        if (!engine().has_graphics() && !engine().headless_render()) {
            return;
        }
        auto drawn = engine().renderer_3d().render(engine().device(), scene(), engine().window().framebuffer_width(),
                                                   engine().window().framebuffer_height());
        if (!drawn) {
            FE_LOG_ERROR("collect3d: 3D render failed: %s", drawn.error().message().c_str());
        }
    }

private:
    // Adopts the engine's audio system into the schedule (last, so a cue fired
    // this frame is mixed this frame) and opens the real output device if one is
    // available. A device that will not open is not an error: the mixer still
    // runs, silently.
    //
    // The listener system is added before the audio system so the mix this frame
    // uses the camera's position this frame. Without it the listener sat at the
    // world origin, so the game's positional pickup cue would be attenuated
    // against the origin rather than against the player's ear -- the very thing
    // the M11 audio row named.
    void adopt_audio_system() {
        auto system = engine().take_audio_system();
        if (system == nullptr) {
            return;
        }
        audio_system_ = system.get();

        // The listener and the "sfx" bus the game's effects route to. The listener
        // goes in before the audio system so the mix this frame uses the camera's
        // position this frame; without it the game's positional cue would be
        // attenuated against the world origin rather than the player's ear.
        collect3d::install_collect3d_audio(schedule(), *audio_system_);

        auto device = fe::audio::create_audio_device();
        if (!device) {
            FE_LOG_WARN("collect3d: no audio backend (%s); mixing silently", device.error().message().c_str());
        } else if (auto opened = device.value()->open(48000, 2); !opened) {
            FE_LOG_WARN("collect3d: could not open the audio device (%s); mixing silently",
                        opened.error().message().c_str());
        } else if (fe::Error error = device.value()->start(); error) {
            FE_LOG_WARN("collect3d: could not start playback (%s); mixing silently", error.message().c_str());
        } else {
            audio_device_ = std::move(device.value());
            audio_system_->set_device(audio_device_.get());
            FE_LOG_INFO("collect3d: opened %s backend at %u Hz", std::string(audio_device_->backend()).c_str(),
                        audio_device_->format().sample_rate);
        }

        schedule().add_system(std::move(system));

        // The settings system mirrors the volume setting onto the mixer, but the
        // engine creates its audio system after the schedule is built, so the
        // pointer is handed over here.
        if (auto* settings = static_cast<collect3d::SettingsSystem*>(schedule().find("SettingsSystem"));
            settings != nullptr) {
            settings->set_audio(audio_system_);
        }
    }

    // Attaches the decoded cue to the scene's audio node.
    void attach_sound() {
        fe::Node* node = scene().find_node_by_name(collect3d::names::kAudio);
        if (node == nullptr) {
            return;
        }
        auto* source = node->get_component<fe::audio::AudioSource>();
        if (source == nullptr) {
            return;
        }
        auto clip = engine().resources().load(kPickupPath);
        if (!clip) {
            FE_LOG_WARN("collect3d: could not load '%s' (%s); the game runs silently", kPickupPath,
                        clip.error().message().c_str());
            return;
        }
        source->clip = clip.value();
    }

    // Rebuilds the arena: clear the scene, build it again, re-attach the
    // behaviours. It does not touch the rules, because each caller has already
    // made the state transition it wants.
    void rebuild() {
        scene().clear();
        host_.clear();
        collect3d::build_collect3d_scene(scene(), load_assets(engine().resources()));
        collect3d::attach_collect3d_behaviours(host_, scene());
        attach_sound();
    }

    collect3d::CollectState state_{};
    bool headless_ = false;
    std::string scene_to_save_;

    // The behaviours the arena's nodes carry. The host is owned here and the
    // script system borrows it; the binding persists the scripts through the
    // scene serializer.
    fe::ScriptRegistry registry_;
    fe::ScriptHost host_;
    std::unique_ptr<fe::script::ScriptBinding> binding_;

    // The device is owned here; the AudioSystem (owned by the schedule) borrows
    // it. audio_system_ is a borrowed pointer used from on_stop().
    std::unique_ptr<fe::audio::AudioDevice> audio_device_;
    bool audio_gesture_reported_ = false;
    fe::audio::AudioSystem* audio_system_ = nullptr;
};

struct Options {
    bool headless = false;
    fe::u32 headless_frames = 0;
    bool null_window = false;
    std::string scene_to_save;
    fe::LogLevel log_level = fe::LogLevel::Info;
};

[[nodiscard]] Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--headless") {
            options.headless = true;
            options.headless_frames = 1800;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                options.headless_frames = static_cast<fe::u32>(std::stoul(argv[++i]));
            }
        } else if (arg == "--null-window") {
            options.null_window = true;
        } else if (arg == "--save-scene" && i + 1 < argc) {
            options.scene_to_save = argv[++i];
        } else if (arg == "--verbose") {
            options.log_level = fe::LogLevel::Debug;
        } else if (arg == "--quiet") {
            options.log_level = fe::LogLevel::Warn;
        } else if (arg == "--help" || arg == "-h") {
            std::puts("usage: fork_collect3d [--headless [frames]] [--null-window] [--save-scene PATH] "
                      "[--verbose] [--quiet]");
            std::exit(0);
        }
    }
    return options;
}

#if defined(FE_PLATFORM_WEB)
// The application the browser's frame callback drives. It is owned by a
// file-scope pointer, not a local in main(): a frame rate of 0 tells Emscripten
// to return from main() and call back from requestAnimationFrame, so a
// stack-local application would be destroyed before the first frame ran.
std::unique_ptr<Collect3DApp> web_app;

void web_frame(void* /*arg*/) {
    if (web_app == nullptr || !web_app->step_frame()) {
        emscripten_cancel_main_loop();
        if (web_app != nullptr) {
            web_app->shutdown();
            web_app.reset();
        }
    }
}
#endif

} // namespace

int main(int argc, char** argv) {
    const Options options = parse_options(argc, argv);

    fe::Engine::Config config;
    config.window.title = "Fork Engine Gem Rush";
    config.window.width = 1280;
    config.window.height = 720;
    config.log_level = options.log_level;
    // The application id names the per-user save directory (desktop) or the
    // localStorage namespace (web), which is what makes the best score survive a
    // reload. A headless run deliberately sets no id: the engine then hands out
    // an in-memory store that persists nothing, so an automated run never writes
    // into the machine's real home directory (ADR-0096).
    if (!options.headless) {
        config.app_id = collect3d::config::kAppId;
    }
    if (options.headless) {
        config.window.visible = false;
        config.window.vsync = false;
        // A fixed step makes a headless run deterministic and reproducible: the
        // autoplay outcome does not depend on how fast the machine steps frames,
        // which is what lets a CI run assert a win (ADR-0079).
        config.fixed_delta_time = 1.0f / 60.0f;
    }
    if (options.null_window) {
        config.window.backend = fe::WindowBackend::Null;
        config.window.visible = false;
        config.window.vsync = false;
    }
    if (!options.scene_to_save.empty()) {
        // Serializing the scene writes each component's reference from the
        // *loaded* resource, so the run that regenerates the file needs the same
        // loaders a real run has. The font loader resolves its atlas through the
        // texture loader, which needs a device, so a device-free save (a plain
        // `--null-window` run) writes an empty font reference and the shipped HUD
        // draws no text. A null RHI device is enough to register it, and is a
        // no-op when a window already gave us a real one.
        config.headless_render = true;
    }

#if defined(FE_PLATFORM_WEB)
    web_app = std::make_unique<Collect3DApp>(config, options.headless, options.scene_to_save);
    if (options.headless) {
        web_app->set_headless(options.headless_frames);
    }
    if (fe::Error error = web_app->initialize(); error) {
        FE_LOG_CRITICAL("collect3d: %s", error.message().c_str());
        web_app.reset();
        return 1;
    }
    emscripten_set_main_loop_arg(web_frame, nullptr, 0, 0);
    return 0;
#else
    Collect3DApp app(config, options.headless, options.scene_to_save);
    if (options.headless) {
        app.set_headless(options.headless_frames);
    }
    return app.run();
#endif
}
