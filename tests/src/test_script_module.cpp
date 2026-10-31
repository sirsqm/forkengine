// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// Tests for the C++ gameplay module loader and hot reload (M7, ADR-0044,
// ADR-0061). These load *real* shared-library images built from
// tests/modules/module_a.cpp and module_b.cpp, so the loader, the ABI check, the
// dynamic-library seam, and the reload path are all exercised end to end — no
// mock stands in for the platform.
//
// The two module versions register the same behaviour and differ only in the
// step they advance a node's X by (1.0 vs 2.0). A test observes which code is
// live by watching the node move, and by reading each image's own tick counter
// through a second handle on the same file.

#include <doctest/doctest.h>

#include "fe/core/Log.h"
#include "fe/platform/DynamicLibrary.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/script/ScriptBinding.h"
#include "fe/script/ScriptHost.h"
#include "fe/script/ScriptLibrary.h"
#include "fe/script/ScriptRegistry.h"
#include "fe/script/ScriptSystem.h"
#include "fe/serialization/SceneSerializer.h"
#include "fe/systems/FrameContext.h"
#include "fe/systems/SceneSchedule.h"

#include <string>
#include <string_view>

using namespace fe;

namespace {

// The module images are built next to the test binary; the exact paths (with the
// platform suffix and any multi-config subdirectory) are supplied by CMake.
constexpr const char* kModuleAPath = FE_TEST_MODULE_A;
constexpr const char* kModuleBPath = FE_TEST_MODULE_B;
constexpr const char* kModuleCAbiMismatchPath = FE_TEST_MODULE_C;

using TickCountFn = u32 (*)();
using AttachCountFn = u32 (*)();

// Reads an image's own counters through a fresh handle, so a test can prove
// which module version was executing without the library exposing its internals.
struct ModuleProbe {
    explicit ModuleProbe(const char* path) {
        Result<DynamicLibrary> loaded = DynamicLibrary::load(path);
        REQUIRE(loaded.ok());
        image = std::move(loaded.value());
        tick_count = image.symbol_as<TickCountFn>("fe_module_tick_count");
        attach_count = image.symbol_as<AttachCountFn>("fe_module_attach_count");
    }

    [[nodiscard]] u32 ticks() const { return tick_count != nullptr ? tick_count() : 0; }
    [[nodiscard]] u32 attaches() const { return attach_count != nullptr ? attach_count() : 0; }

    DynamicLibrary image;
    TickCountFn tick_count = nullptr;
    AttachCountFn attach_count = nullptr;
};

[[nodiscard]] f32 node_x(const Node& node) {
    return node.transform().position.x;
}

} // namespace

TEST_CASE("DynamicLibrary opens a real image and resolves a symbol") {
    Result<DynamicLibrary> loaded = DynamicLibrary::load(kModuleAPath);
    REQUIRE(loaded.ok());
    CHECK(loaded.value().is_open());
    CHECK(loaded.value().path() == std::string(kModuleAPath));

    auto abi = loaded.value().symbol_as<TickCountFn>("fe_module_tick_count");
    CHECK(abi != nullptr);
    // An absent symbol is a clean null, not an error.
    CHECK(loaded.value().symbol("no_such_symbol") == nullptr);

    loaded.value().unload();
    CHECK_FALSE(loaded.value().is_open());
}

TEST_CASE("DynamicLibrary reports a missing file as an error") {
    Result<DynamicLibrary> loaded = DynamicLibrary::load("definitely_not_here_12345");
    CHECK_FALSE(loaded.ok());
    CHECK_FALSE(loaded.error().message().empty());
}

TEST_CASE("DynamicLibrary decorates a bare name with the platform suffix") {
    const std::string decorated = DynamicLibrary::decorate("mygame");
    CHECK(decorated != "mygame");
    CHECK(DynamicLibrary::has_shared_suffix(decorated));
    // Decorating twice is idempotent.
    CHECK(DynamicLibrary::decorate(decorated) == decorated);
    CHECK_FALSE(DynamicLibrary::has_shared_suffix("mygame"));
}

TEST_CASE("ScriptLibrary loads a module and its behaviours") {
    ScriptLibrary library;
    CHECK(library.module_count() == 0);
    CHECK_FALSE(library.registry().contains("SpinBehaviour"));

    CHECK(library.load_module(kModuleAPath).ok());
    CHECK(library.module_count() == 1);
    CHECK(library.registry().contains("SpinBehaviour"));
    CHECK(library.registry().script_count() == 1);

    // Loading the same module again is a no-op, not a second registration.
    CHECK(library.load_module(kModuleAPath).ok());
    CHECK(library.module_count() == 1);
    CHECK(library.registry().script_count() == 1);
}

TEST_CASE("a module's behaviour runs through the ordinary schedule") {
    ScriptLibrary library;
    REQUIRE(library.load_module(kModuleAPath).ok());

    Scene scene;
    Node* node = scene.create_node("Mover");
    ScriptHost host;

    // The image's counters are process-global (dlopen refcounts one mapping per
    // process), so a test reads them as a delta around the work it drives. The
    // probe is opened before attaching so the attach delta is observable.
    ModuleProbe probe(kModuleAPath);
    const u32 ticks_before = probe.ticks();
    const u32 attaches_before = probe.attaches();

    REQUIRE(library.attach(host, *node, "SpinBehaviour").ok());
    CHECK(host.script_count() == 1);
    CHECK(library.attached_count() == 1);

    SceneSchedule schedule;
    schedule.add_system<ScriptSystem>(host);

    const FrameContext context;
    schedule.update(context, scene);
    schedule.update(context, scene);
    schedule.update(context, scene);

    // Version A advances by 1.0 per frame.
    CHECK(node_x(*node) == doctest::Approx(3.0f));

    // The image's own counters prove the module's code, not a copy, ran: one
    // attach and three updates.
    CHECK(probe.attaches() - attaches_before == 1);
    CHECK(probe.ticks() - ticks_before == 3);
}

TEST_CASE("attaching an unknown behaviour is an error") {
    ScriptLibrary library;
    Scene scene;
    Node* node = scene.create_node("Node");
    ScriptHost host;

    const Error error = library.attach(host, *node, "NotAThing");
    CHECK_FALSE(error.ok());
    CHECK(host.script_count() == 0);
    CHECK(library.attached_count() == 0);
}

TEST_CASE("loading a missing module is an error") {
    ScriptLibrary library;
    const Error error = library.load_module("no_such_module_at_all");
    CHECK_FALSE(error.ok());
    CHECK(library.module_count() == 0);
}

TEST_CASE("a module with the wrong ABI version is refused") {
    ScriptLibrary library;
    const Error error = library.load_module(kModuleCAbiMismatchPath);
    CHECK_FALSE(error.ok());
    // The refusal names the version mismatch and registers nothing.
    CHECK(error.message().find("ABI") != std::string::npos);
    CHECK(library.module_count() == 0);
    CHECK(library.registry().script_count() == 0);
}

TEST_CASE("reloading a module that is not loaded is an error") {
    ScriptLibrary library;
    ScriptHost host;
    const Error error = library.reload_module(host, "never_loaded");
    CHECK_FALSE(error.ok());
}

TEST_CASE("hot reload swaps a live script's code without a restart") {
    ScriptLibrary library;
    REQUIRE(library.load_module(kModuleAPath).ok());

    Scene scene;
    Node* node = scene.create_node("Mover");
    ScriptHost host;
    REQUIRE(library.attach(host, *node, "SpinBehaviour").ok());

    SceneSchedule schedule;
    schedule.add_system<ScriptSystem>(host);
    const FrameContext context;

    // Counters are process-global, so measure deltas around each phase.
    ModuleProbe old_image(kModuleAPath);
    ModuleProbe new_image(kModuleBPath);
    const u32 old_ticks_before = old_image.ticks();
    const u32 new_ticks_before = new_image.ticks();
    const u32 new_attaches_before = new_image.attaches();

    // Run the old code for two frames: +1.0 each, so x == 2.0.
    schedule.update(context, scene);
    schedule.update(context, scene);
    CHECK(node_x(*node) == doctest::Approx(2.0f));
    CHECK(old_image.ticks() - old_ticks_before == 2);
    CHECK(new_image.ticks() == new_ticks_before);

    // Hot reload: swap in version B's image. The live script is detached, the old
    // image unloaded, the new one loaded, and the node rebound to fresh code.
    REQUIRE(library.reload_module(host, kModuleAPath, kModuleBPath).ok());

    // The node still has exactly one script, and the new image was entered to
    // attach it.
    CHECK(host.script_count() == 1);
    CHECK(library.attached_count() == 1);
    CHECK(new_image.attaches() - new_attaches_before == 1);

    // Two more frames now advance by 2.0 each: x == 2.0 + 4.0 == 6.0.
    schedule.update(context, scene);
    schedule.update(context, scene);
    CHECK(node_x(*node) == doctest::Approx(6.0f));

    // The new image ran those two frames; the old image's counter is unchanged,
    // proving the old code is no longer executing.
    CHECK(new_image.ticks() - new_ticks_before == 2);
    CHECK(old_image.ticks() - old_ticks_before == 2);
}

TEST_CASE("detaching forgets the binding so a reload does not resurrect it") {
    ScriptLibrary library;
    REQUIRE(library.load_module(kModuleAPath).ok());

    Scene scene;
    Node* a = scene.create_node("A");
    Node* b = scene.create_node("B");
    ScriptHost host;
    REQUIRE(library.attach(host, *a, "SpinBehaviour").ok());
    REQUIRE(library.attach(host, *b, "SpinBehaviour").ok());
    CHECK(library.attached_count() == 2);

    CHECK(library.detach(host, a->id()));
    CHECK(library.attached_count() == 1);
    CHECK(host.script_count() == 1);

    // A reload rebinds only the nodes still bound, so the detached node stays
    // unscripted rather than being quietly resurrected.
    REQUIRE(library.reload_module(host, kModuleAPath, kModuleBPath).ok());
    CHECK(host.script_count() == 1);
    CHECK(library.attached_count() == 1);
    CHECK(host.has_script(b->id()));
    CHECK_FALSE(host.has_script(a->id()));
}

TEST_CASE("a failed reload drops the module and leaves nodes unscripted") {
    ScriptLibrary library;
    REQUIRE(library.load_module(kModuleAPath).ok());

    Scene scene;
    Node* node = scene.create_node("Mover");
    ScriptHost host;
    REQUIRE(library.attach(host, *node, "SpinBehaviour").ok());

    // Reload to an image that does not exist: the module is dropped, its
    // behaviour unregistered, and the node left unscripted rather than holding
    // instances of code that is gone.
    const Error error = library.reload_module(host, kModuleAPath, "missing_replacement_image");
    CHECK_FALSE(error.ok());
    CHECK(library.module_count() == 0);
    CHECK_FALSE(library.registry().contains("SpinBehaviour"));
    CHECK(host.script_count() == 0);
}

// The shipped shape of M7: a scene *document* records a behaviour by name, and
// that name resolves through a module the engine did not compile in. The tests
// above attach programmatically and the binding tests use an in-process registry,
// so neither exercises the path a player runs -- a scene read from a document
// whose binding is satisfied by a loaded image. This is what the queue item's
// "a game's rules load from a module the engine did not compile in" means, and it
// is the shape an exported game uses (fork_player loads its manifest's modules,
// then loads the scene through the same ScriptBinding seam).
//
// A hand-written document, not one the test serialized: the point is that a
// *game author's* file, produced by an editor that never had the module, resolves
// against the module at load. The behaviour name is the only link between them.
TEST_CASE("a scene document's binding resolves through a loaded module") {
    const std::string document = R"({"format":"fork_scene","format_version":4,"name":"ModGame","roots":[)"
                                 R"({"name":"Mover","type":"Node","scripts":[{"behaviour":"SpinBehaviour"}]}]})";

    // Load the module first, exactly as fork_player does with a manifest's
    // modules, then read the scene through the library's registry.
    ScriptLibrary library;
    REQUIRE(library.load_module(kModuleAPath).ok());

    Scene scene;
    ScriptHost host;
    script::ScriptBinding binding(host, library.registry());
    REQUIRE_FALSE(serialization::scene::from_json(document, scene, nullptr, &binding));

    // The document's binding produced a live script from the module's code.
    Node* mover = scene.find_node_by_name("Mover");
    REQUIRE(mover != nullptr);
    REQUIRE(host.has_script(mover->id()));
    CHECK(host.script_count() == 1);

    // And it runs: the module's behaviour advances X by 1.0 per frame, driven by
    // the ordinary schedule. The image's own counter proves the module's code,
    // not a copy, executed.
    ModuleProbe probe(kModuleAPath);
    const u32 ticks_before = probe.ticks();

    SceneSchedule schedule;
    schedule.add_system<ScriptSystem>(host);
    const FrameContext context;
    schedule.update(context, scene);
    schedule.update(context, scene);

    CHECK(node_x(*mover) == doctest::Approx(2.0f));
    CHECK(probe.ticks() - ticks_before == 2);
}

// The negative half: the same document with the module *not* loaded attaches
// nothing and is not fatal -- the scene still loads. This is the state a game
// with a missing module must not crash in, and it proves the previous test's
// success came from the module rather than from the document alone.
TEST_CASE("a scene document's binding is inert when its module is not loaded") {
    const std::string document = R"({"format":"fork_scene","format_version":4,"name":"ModGame","roots":[)"
                                 R"({"name":"Mover","type":"Node","scripts":[{"behaviour":"SpinBehaviour"}]}]})";

    ScriptLibrary library; // nothing loaded
    Scene scene;
    ScriptHost host;
    script::ScriptBinding binding(host, library.registry());
    REQUIRE_FALSE(serialization::scene::from_json(document, scene, nullptr, &binding));

    CHECK(scene.find_node_by_name("Mover") != nullptr);
    CHECK(host.script_count() == 0);
}
