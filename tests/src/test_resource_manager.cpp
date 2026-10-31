// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include <doctest/doctest.h>

#include "fe/core/FileSystem.h"
#include "fe/resources/AssetId.h"
#include "fe/resources/ResourceManager.h"

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

using namespace fe;

namespace {

// A minimal in-memory resource used to test the manager's caching, dispatch, and
// release behaviour without touching the filesystem or the GPU.
class FakeResource final : public Resource {
public:
    static constexpr const char* kTypeName = "FakeResource";

    explicit FakeResource(std::string path, int resource_tag) : Resource(std::move(path)), tag(resource_tag) {}

    [[nodiscard]] const char* type_name() const noexcept override { return kTypeName; }

    int tag;
};

// Counts how many times load() was invoked, so tests can prove the cache is
// actually consulted rather than re-loading.
class FakeLoader final : public ResourceLoader {
public:
    explicit FakeLoader(std::string scheme_name, int& load_counter)
        : scheme_(std::move(scheme_name)), counter_(&load_counter) {}

    [[nodiscard]] std::string_view scheme() const noexcept override { return scheme_; }
    [[nodiscard]] bool supports(std::string_view extension) const noexcept override { return extension == ".fake"; }

    [[nodiscard]] Result<std::shared_ptr<Resource>> load(std::string_view path) override {
        ++(*counter_);
        return std::static_pointer_cast<Resource>(std::make_shared<FakeResource>(std::string(path), *counter_));
    }

private:
    std::string scheme_;
    int* counter_;
};

// A unique temporary asset root per test case, removed on destruction.
class TempDir {
public:
    explicit TempDir(std::string name) {
        path_ = std::filesystem::temp_directory_path() / ("fe_rm_test_" + name + "_" + std::to_string(counter_++));
        std::filesystem::remove_all(path_);
        std::filesystem::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }

    [[nodiscard]] std::string str() const { return path_.string(); }
    [[nodiscard]] std::string file(const std::string& name) const { return (path_ / name).string(); }

private:
    std::filesystem::path path_;
    static inline int counter_ = 0;
};

} // namespace

TEST_CASE("load dispatches to a loader by extension and caches the result") {
    int load_count = 0;
    ResourceManager manager;
    manager.register_loader(std::make_unique<FakeLoader>("", load_count));

    auto first = manager.load_as<FakeResource>("hero.fake");
    REQUIRE(first);
    CHECK(first.value()->tag == 1);
    CHECK(load_count == 1);

    // Second load of the same path must hit the cache.
    auto second = manager.load_as<FakeResource>("hero.fake");
    REQUIRE(second);
    CHECK((second.value() == first.value()));
    CHECK(load_count == 1);
}

TEST_CASE("the cache key includes the scheme") {
    int default_loads = 0;
    int tex_loads = 0;
    ResourceManager manager;
    manager.register_loader(std::make_unique<FakeLoader>("", default_loads));
    manager.register_loader(std::make_unique<FakeLoader>("tex", tex_loads));

    auto image = manager.load_as<FakeResource>("hero.fake");
    auto texture = manager.load_as<FakeResource>("tex:hero.fake");

    REQUIRE(image);
    REQUIRE(texture);
    // Different schemes are different resources even for the same file.
    CHECK((image.value() != texture.value()));
    CHECK(default_loads == 1);
    CHECK(tex_loads == 1);
    CHECK(manager.cached_count() == 2);
}

TEST_CASE("a released resource is reloaded on the next request") {
    int load_count = 0;
    ResourceManager manager;
    manager.register_loader(std::make_unique<FakeLoader>("", load_count));

    {
        auto resource = manager.load_as<FakeResource>("thing.fake");
        REQUIRE(resource);
        CHECK(load_count == 1);
    } // The last reference goes away here.

    auto reloaded = manager.load_as<FakeResource>("thing.fake");
    REQUIRE(reloaded);
    CHECK(load_count == 2);

    manager.collect_garbage();
    CHECK(manager.loaded_count() == 1);
}

TEST_CASE("an unknown extension produces a clear error") {
    ResourceManager manager;
    auto result = manager.load("mystery.unknown");
    CHECK_FALSE(result);
    CHECK(result.error().message().find(".unknown") != std::string::npos);
}

TEST_CASE("an unknown scheme produces a clear error") {
    int load_count = 0;
    ResourceManager manager;
    manager.register_loader(std::make_unique<FakeLoader>("", load_count));

    auto result = manager.load("bogus:hero.fake");
    CHECK_FALSE(result);
    CHECK(result.error().message().find("bogus") != std::string::npos);
}

TEST_CASE("a guid reference resolves through the asset database and is tagged with its id") {
    TempDir dir("guid");
    int load_count = 0;
    ResourceManager manager;
    manager.register_loader(std::make_unique<FakeLoader>("tex", load_count));

    REQUIRE_FALSE(fs::write_text_file(dir.file("assets/hero.fake"), "x"));
    manager.assets().set_root(dir.str());
    auto imported = manager.assets().import("assets/hero.fake", "tex");
    REQUIRE(imported);

    auto loaded = manager.load_as<FakeResource>(to_reference(imported.value().id));
    REQUIRE(loaded);
    CHECK(load_count == 1);
    // The resource carries the stable identity, which is what a reference taken
    // from it will persist.
    CHECK(loaded.value()->has_asset_id());
    CHECK(loaded.value()->asset_id() == imported.value().id);
}

TEST_CASE("a guid reference still resolves after the asset file moves") {
    TempDir dir("move");
    int load_count = 0;
    ResourceManager manager;
    manager.register_loader(std::make_unique<FakeLoader>("tex", load_count));

    REQUIRE_FALSE(fs::write_text_file(dir.file("assets/hero.fake"), "x"));
    manager.assets().set_root(dir.str());
    auto imported = manager.assets().import("assets/hero.fake", "tex");
    REQUIRE(imported);
    const std::string reference = to_reference(imported.value().id);

    // Move the asset and its sidecar, then rebuild the index. The reference is
    // unchanged and still names the same asset.
    std::filesystem::create_directories(dir.file("assets/textures"));
    std::filesystem::rename(dir.file("assets/hero.fake"), dir.file("assets/textures/hero.fake"));
    std::filesystem::rename(dir.file("assets/hero.fake.meta"), dir.file("assets/textures/hero.fake.meta"));
    manager.assets().rescan();

    auto loaded = manager.load_as<FakeResource>(reference);
    REQUIRE(loaded);
    CHECK(loaded.value()->asset_id() == imported.value().id);
    CHECK(loaded.value()->path() == "assets/textures/hero.fake");
}

TEST_CASE("an unknown guid reference fails clearly") {
    ResourceManager manager;
    auto result = manager.load(to_reference(Uuid::generate()));
    CHECK_FALSE(result);
    CHECK(result.error().message().find("not in the asset database") != std::string::npos);
}

TEST_CASE("a malformed guid reference is reported, not treated as a path") {
    ResourceManager manager;
    auto result = manager.load("guid:not-a-uuid");
    CHECK_FALSE(result);
    CHECK(result.error().message().find("malformed guid reference") != std::string::npos);
}

TEST_CASE("resolve_reference upgrades a known path to a guid and leaves unknown paths alone") {
    TempDir dir("resolve");
    ResourceManager manager;
    REQUIRE_FALSE(fs::write_text_file(dir.file("assets/hero.fake"), "x"));
    manager.assets().set_root(dir.str());
    auto imported = manager.assets().import("assets/hero.fake", "tex");
    REQUIRE(imported);

    // A bare path and a scheme-prefixed path both resolve to the same GUID.
    CHECK(manager.resolve_reference("assets/hero.fake") == to_reference(imported.value().id));
    CHECK(manager.resolve_reference("tex:assets/hero.fake") == to_reference(imported.value().id));

    // An unknown path, and a reference that is already a GUID, are returned as-is.
    CHECK(manager.resolve_reference("assets/other.fake") == "assets/other.fake");
    CHECK(manager.resolve_reference(to_reference(imported.value().id)) == to_reference(imported.value().id));

    // With no database at all, a path is returned unchanged (path-based mode).
    ResourceManager plain;
    CHECK(plain.resolve_reference("tex:assets/hero.fake") == "tex:assets/hero.fake");
}

namespace {

// Records the import settings that were in scope for its load, so a test can
// prove the manager scopes an asset's settings around the loader call.
class SettingsProbeLoader final : public ResourceLoader {
public:
    explicit SettingsProbeLoader(int& load_counter) : counter_(&load_counter) {}

    [[nodiscard]] std::string_view scheme() const noexcept override { return "tex"; }
    [[nodiscard]] bool supports(std::string_view extension) const noexcept override { return extension == ".fake"; }

    [[nodiscard]] Result<std::shared_ptr<Resource>> load(std::string_view path) override {
        ++(*counter_);
        if (const ImportSettings* current = manager_->current_import_settings(); current != nullptr) {
            seen = *current;
        } else {
            seen = ImportSettings{};
        }
        return std::static_pointer_cast<Resource>(std::make_shared<FakeResource>(std::string(path), *counter_));
    }

    void set_manager(ResourceManager& manager) noexcept { manager_ = &manager; }

    ImportSettings seen;

private:
    int* counter_;
    ResourceManager* manager_ = nullptr;
};

} // namespace

TEST_CASE("a guid load scopes the asset's import settings around the loader call") {
    TempDir dir("scope_settings");
    int load_count = 0;
    ResourceManager manager;
    auto probe = std::make_unique<SettingsProbeLoader>(load_count);
    SettingsProbeLoader* probe_ptr = probe.get();
    manager.register_loader(std::move(probe));
    probe_ptr->set_manager(manager);

    REQUIRE_FALSE(fs::write_text_file(dir.file("assets/hero.fake"), "x"));
    manager.assets().set_root(dir.str());
    ImportSettings settings = ImportSettings::texture_defaults();
    settings.filter = rhi::TextureFilter::Nearest;
    auto imported = manager.assets().import("assets/hero.fake", "tex", settings);
    REQUIRE(imported);

    auto loaded = manager.load(to_reference(imported.value().id));
    REQUIRE(loaded);
    // The loader saw the asset's settings.
    CHECK(probe_ptr->seen.kind == ImportSettings::Kind::Texture);
    CHECK(probe_ptr->seen.filter == rhi::TextureFilter::Nearest);

    // And the settings are cleared again once the load returns.
    CHECK(manager.current_import_settings() == nullptr);
}

TEST_CASE("a path load honours the settings of a known asset, and a direct loader call sees none") {
    TempDir dir("scope_path");
    int load_count = 0;
    ResourceManager manager;
    auto probe = std::make_unique<SettingsProbeLoader>(load_count);
    SettingsProbeLoader* probe_ptr = probe.get();
    manager.register_loader(std::move(probe));
    probe_ptr->set_manager(manager);

    REQUIRE_FALSE(fs::write_text_file(dir.file("assets/hero.fake"), "x"));
    manager.assets().set_root(dir.str());
    ImportSettings settings = ImportSettings::texture_defaults();
    settings.wrap = rhi::TextureWrap::Repeat;
    REQUIRE(manager.assets().import("assets/hero.fake", "tex", settings));

    // A scheme-prefixed path resolves the same asset and sees its settings.
    auto loaded = manager.load("tex:assets/hero.fake");
    REQUIRE(loaded);
    CHECK(probe_ptr->seen.wrap == rhi::TextureWrap::Repeat);
    CHECK(manager.current_import_settings() == nullptr);

    // A loader called directly, outside load(), has no settings in scope, so a
    // loader that is invoked by hand (as a test or tool might) keeps its
    // defaults rather than inheriting a stale asset's settings.
    probe_ptr->seen = ImportSettings{};
    auto direct = probe_ptr->load("assets/hero.fake");
    REQUIRE(direct);
    CHECK(probe_ptr->seen.kind == ImportSettings::Kind::Generic);
    CHECK(probe_ptr->seen.is_default());
}

// A loader that reads its input through the source the manager gives it, the way
// a real loader does. It records whether the read succeeded, so a test can prove
// a project-relative path-keyed reference resolves from outside the project
// directory.
class ReadingLoader final : public ResourceLoader {
public:
    [[nodiscard]] std::string_view scheme() const noexcept override { return "tex"; }
    [[nodiscard]] bool supports(std::string_view extension) const noexcept override { return extension == ".fake"; }

    [[nodiscard]] Result<std::shared_ptr<Resource>> load(std::string_view path) override {
        auto bytes = read_bytes(path);
        read_ok = static_cast<bool>(bytes);
        if (bytes) {
            content.assign(bytes.value().begin(), bytes.value().end());
        }
        return std::static_pointer_cast<Resource>(std::make_shared<FakeResource>(std::string(path), 1));
    }

    bool read_ok = false;
    std::string content;
};

TEST_CASE("a path-keyed reference resolves against the reference base, not the working directory") {
    TempDir dir("refbase");
    ResourceManager manager;
    auto loader = std::make_unique<ReadingLoader>();
    ReadingLoader* loader_ptr = loader.get();
    manager.register_loader(std::move(loader));

    REQUIRE_FALSE(fs::write_text_file(dir.file("assets/hero.fake"), "payload"));

    // With no base, the reference resolves against the process working directory
    // and misses (the test's cwd is not `dir`).
    auto missed = manager.load("tex:assets/hero.fake");
    REQUIRE(missed);
    CHECK_FALSE(loader_ptr->read_ok);

    // Pointing the base at the project makes the same reference resolve, with no
    // change to the reference itself and no change to the working directory. A
    // fresh manager stands in for "the same load after the base was set", since
    // the first attempt cached its (failed) result under the reference.
    ResourceManager base_manager;
    auto base_loader = std::make_unique<ReadingLoader>();
    ReadingLoader* base_loader_ptr = base_loader.get();
    base_manager.register_loader(std::move(base_loader));
    base_manager.set_reference_base(dir.str());
    CHECK(base_manager.reference_base() == fs::normalize(dir.str()));

    auto found = base_manager.load("tex:assets/hero.fake");
    REQUIRE(found);
    CHECK(base_loader_ptr->read_ok);
    CHECK(base_loader_ptr->content == "payload");
}

TEST_CASE("the reference base does not join an absolute reference") {
    TempDir dir("refbase_abs");
    ResourceManager manager;
    auto loader = std::make_unique<ReadingLoader>();
    ReadingLoader* loader_ptr = loader.get();
    manager.register_loader(std::move(loader));

    const std::string file = dir.file("assets/hero.fake");
    REQUIRE_FALSE(fs::write_text_file(file, "absolute"));
    manager.set_reference_base(dir.str());

    // An absolute reference is used as written; joining the base onto it would
    // produce a path that does not exist.
    auto loaded = manager.load("tex:" + file);
    REQUIRE(loaded);
    CHECK(loader_ptr->read_ok);
    CHECK(loader_ptr->content == "absolute");
}

TEST_CASE("the reference base does not affect a direct manager read") {
    TempDir dir("refbase_direct");
    ResourceManager manager;
    manager.set_reference_base(dir.str());
    REQUIRE_FALSE(fs::write_text_file(dir.file("scene.fescene"), "{}"));

    // The manager's own read_text is what the scene reader uses, with a path it
    // has already resolved. The base is for loader reads only; joining it here
    // would make the scene reader look inside the project a second time.
    auto direct = manager.read_text(dir.file("scene.fescene"));
    REQUIRE(direct);
    CHECK(direct.value() == "{}");
}
