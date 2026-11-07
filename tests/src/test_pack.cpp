// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include <doctest/doctest.h>

#include "fe/core/FileSystem.h"
#include "fe/mesh/GltfLoader.h"
#include "fe/pack/AssetClosure.h"
#include "fe/pack/PackCompression.h"
#include "fe/pack/PackFile.h"
#include "fe/pack/PackSource.h"
#include "fe/pack/PackWriter.h"
#include "fe/resources/AssetDatabase.h"
#include "fe/resources/Image.h"
#include "fe/resources/ImageLoader.h"
#include "fe/resources/ResourceManager.h"
#include "fe/scene/Scene.h"
#include "fe/scene/Sprite2D.h"
#include "fe/serialization/SceneSerializer.h"

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using namespace fe;
using namespace fe::pack;

namespace {

// A unique temporary directory per test case, removed on destruction.
class TempDir {
public:
    explicit TempDir(std::string name) {
        path_ = std::filesystem::temp_directory_path() / ("fe_pack_test_" + name + "_" + std::to_string(counter_++));
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

[[nodiscard]] std::vector<u8> bytes_of(std::string_view text) {
    return std::vector<u8>(text.begin(), text.end());
}

// A 2x2 black/white checker TGA (uncompressed, 32-bit, top-left origin). A real
// image the loader can decode, so the mount test proves a packed asset is served
// through the ordinary loader path, not just read as bytes.
[[nodiscard]] std::vector<u8> checker_tga_bytes() {
    std::vector<u8> tga = {
        0, 0, 2, 0, 0,  0,    0, 0, 0, 0, 0, 0, // header: no colour map, uncompressed true-colour
        2, 0, 2, 0, 32, 0x28,                   // 2x2, 32-bit BGRA, top-left origin
    };
    const std::array<u8, 4> black{0, 0, 0, 255};
    const std::array<u8, 4> white{255, 255, 255, 255};
    for (const auto& texel : {black, white, white, black}) {
        tga.insert(tga.end(), texel.begin(), texel.end());
    }
    return tga;
}

} // namespace

TEST_CASE("a pack round-trips its entries and payloads") {
    PackFile pack;
    const Uuid id = Uuid::generate();
    pack.add_file("assets/hero.png", "tex", id, bytes_of("PNG DATA"));
    pack.add_file("assets/beep.wav", "wav", Uuid{}, bytes_of("WAVE"));

    auto bytes = pack.write();
    REQUIRE(bytes);

    auto parsed = PackFile::parse(bytes.value().data(), bytes.value().size());
    REQUIRE(parsed);
    CHECK(parsed.value().entry_count() == 2);

    const pack::PackEntry* hero = parsed.value().find_path("assets/hero.png");
    REQUIRE(hero != nullptr);
    CHECK(hero->scheme == "tex");
    CHECK(hero->guid == id);
    CHECK(hero->has_guid());
    CHECK(hero->data == bytes_of("PNG DATA"));

    const pack::PackEntry* beep = parsed.value().find_path("assets/beep.wav");
    REQUIRE(beep != nullptr);
    CHECK(beep->scheme == "wav");
    CHECK_FALSE(beep->has_guid());
    CHECK(beep->data == bytes_of("WAVE"));

    // A guid lookup resolves to the same entry; a nil id matches nothing.
    CHECK(parsed.value().find_guid(id) == hero);
    CHECK(parsed.value().find_guid(Uuid{}) == nullptr);
    CHECK(parsed.value().find_path("assets/missing.png") == nullptr);
}

TEST_CASE("a pack is written in a deterministic order") {
    PackFile first;
    first.add_file("b.txt", "", Uuid{}, bytes_of("b"));
    first.add_file("a.txt", "", Uuid{}, bytes_of("a"));

    PackFile second;
    second.add_file("a.txt", "", Uuid{}, bytes_of("a"));
    second.add_file("b.txt", "", Uuid{}, bytes_of("b"));

    auto first_bytes = first.write();
    auto second_bytes = second.write();
    REQUIRE(first_bytes);
    REQUIRE(second_bytes);
    // Insertion order must not change the output, so a rebuild is byte-stable.
    CHECK(first_bytes.value() == second_bytes.value());
}

TEST_CASE("a duplicate key is rejected rather than shadowed") {
    PackFile pack;
    pack.add_file("assets/a.txt", "", Uuid{}, bytes_of("one"));
    pack.add_file("assets/a.txt", "", Uuid{}, bytes_of("two"));

    auto bytes = pack.write();
    REQUIRE_FALSE(bytes);
}

TEST_CASE("a malformed pack is rejected, not read as garbage") {
    PackFile pack;
    pack.add_file("assets/a.txt", "", Uuid{}, bytes_of("hello"));
    auto bytes = pack.write();
    REQUIRE(bytes);
    const std::vector<u8>& valid = bytes.value();

    SUBCASE("wrong magic") {
        std::vector<u8> corrupt = valid;
        corrupt[0] = 'X';
        auto parsed = PackFile::parse(corrupt.data(), corrupt.size());
        REQUIRE_FALSE(parsed);
    }

    SUBCASE("a version from the future") {
        std::vector<u8> corrupt = valid;
        corrupt[4] = 99; // format_version low byte
        auto parsed = PackFile::parse(corrupt.data(), corrupt.size());
        REQUIRE_FALSE(parsed);
    }

    SUBCASE("a truncated buffer") {
        auto parsed = PackFile::parse(valid.data(), valid.size() / 2);
        REQUIRE_FALSE(parsed);
    }

    SUBCASE("an index offset outside the buffer") {
        std::vector<u8> corrupt = valid;
        // index_offset is the u64 at byte 12; push it far past the end.
        for (int i = 12; i < 20; ++i) {
            corrupt[static_cast<usize>(i)] = 0xFF;
        }
        auto parsed = PackFile::parse(corrupt.data(), corrupt.size());
        REQUIRE_FALSE(parsed);
    }

    SUBCASE("a payload length that runs past the end") {
        std::vector<u8> corrupt = valid;
        // The single entry's payload is the last 5 bytes ("hello"); the u64
        // immediately before it is its length field. Make that length enormous,
        // so the span cannot fit and the reader must reject it.
        const usize length_field = corrupt.size() - 5 - 8;
        for (usize i = length_field; i < length_field + 8; ++i) {
            corrupt[i] = 0xFF;
        }
        auto parsed = PackFile::parse(corrupt.data(), corrupt.size());
        REQUIRE_FALSE(parsed);
    }
}

TEST_CASE("the writer packs a project's asset tree and its scene") {
    TempDir dir("write");
    REQUIRE_FALSE(fs::write_text_file(dir.file("assets/hero.png"), "PNG"));
    // A file that is never imported: still packed, under the default scheme,
    // so a project that references it by path runs from the pack.
    REQUIRE_FALSE(fs::write_text_file(dir.file("assets/loose.txt"), "LOOSE"));
    REQUIRE_FALSE(fs::write_text_file(dir.file("scenes/main.fescene"), "{\"format\":\"fork_scene\"}"));

    AssetDatabase assets;
    assets.set_root(dir.file("assets"));
    auto imported = assets.import("hero.png", "tex");
    REQUIRE(imported);

    auto built = pack::PackWriter::build(assets, dir.file("scenes/main.fescene"), "scenes/main.fescene");
    REQUIRE(built);

    // The indexed asset is present with its scheme and id; the unindexed file is
    // present with no scheme and no id; the scene is present under its key.
    const pack::PackEntry* hero = built.value().find_path("hero.png");
    REQUIRE(hero != nullptr);
    CHECK(hero->scheme == "tex");
    CHECK(hero->guid == imported.value().id);
    CHECK(hero->data == bytes_of("PNG"));

    const pack::PackEntry* loose = built.value().find_path("loose.txt");
    REQUIRE(loose != nullptr);
    CHECK(loose->scheme.empty());
    CHECK_FALSE(loose->has_guid());
    CHECK(loose->data == bytes_of("LOOSE"));

    const pack::PackEntry* scene = built.value().find_path("scenes/main.fescene");
    REQUIRE(scene != nullptr);
    CHECK_FALSE(scene->has_guid());

    // The sidecar the import wrote is metadata, not an asset, and is not packed.
    CHECK(built.value().find_path("hero.png.meta") == nullptr);
}

TEST_CASE("pruning packs only the assets the caller lists, plus the scene") {
    TempDir dir("prune");
    REQUIRE_FALSE(fs::write_text_file(dir.file("assets/hero.png"), "PNG"));
    REQUIRE_FALSE(fs::write_text_file(dir.file("assets/unused.png"), "UNUSED"));
    REQUIRE_FALSE(fs::write_text_file(dir.file("scenes/main.fescene"), "{\"format\":\"fork_scene\"}"));

    AssetDatabase assets;
    assets.set_root(dir.file("assets"));
    auto imported = assets.import("hero.png", "tex");
    REQUIRE(imported);

    PackOptions options;
    options.prune = true;
    options.include = {"hero.png"};
    auto built = pack::PackWriter::build(assets, dir.file("scenes/main.fescene"), "scenes/main.fescene", options);
    REQUIRE(built);

    // The listed asset keeps its scheme and id; the unlisted one is gone; the
    // scene is always present, because it is the entry point.
    const pack::PackEntry* hero = built.value().find_path("hero.png");
    REQUIRE(hero != nullptr);
    CHECK(hero->scheme == "tex");
    CHECK(hero->guid == imported.value().id);
    CHECK(built.value().find_path("unused.png") == nullptr);
    CHECK(built.value().find_path("scenes/main.fescene") != nullptr);
}

TEST_CASE("expanding an asset closure adds the files a glTF names") {
    // The shipping bug this covers: a pruned pack kept the glTF a scene named and
    // dropped the .bin and the images the glTF itself names, so the mesh loaded
    // and then failed to decode and its maps never resolved.
    TempDir dir("closure");
    const std::string gltf = R"JSON({
        "asset": {"version": "2.0"},
        "buffers": [{"uri": "prop.bin", "byteLength": 4}],
        "images": [{"uri": "textures/albedo.png"}]
    })JSON";
    REQUIRE_FALSE(fs::write_text_file(dir.file("models/prop.gltf"), gltf));
    REQUIRE_FALSE(fs::write_text_file(dir.file("models/prop.bin"), "BIN!"));
    REQUIRE_FALSE(fs::write_text_file(dir.file("models/textures/albedo.png"), "PNG"));

    auto expanded = pack::expand_asset_dependencies({"models/prop.gltf"}, dir.str(), GltfLoader::dependency_reader());
    // The glTF first, then its dependencies, resolved against its own directory.
    REQUIRE(expanded.size() == 3);
    CHECK(expanded[0] == "models/prop.gltf");
    CHECK(expanded[1] == "models/prop.bin");
    CHECK(expanded[2] == "models/textures/albedo.png");
}

TEST_CASE("an asset closure leaves a self-contained asset alone") {
    TempDir dir("closure_self");
    REQUIRE_FALSE(fs::write_text_file(dir.file("assets/hero.png"), "PNG"));

    auto expanded = pack::expand_asset_dependencies({"assets/hero.png"}, dir.str(), GltfLoader::dependency_reader());
    REQUIRE(expanded.size() == 1);
    CHECK(expanded[0] == "assets/hero.png");
}

TEST_CASE("an asset closure skips a dependency that is not in the tree") {
    // The scene named the glTF, so the glTF is packed; a dependency the packer
    // cannot find is simply not added, matching how a missing asset behaves
    // elsewhere rather than failing the whole pack.
    TempDir dir("closure_missing");
    const std::string gltf = R"JSON({
        "asset": {"version": "2.0"},
        "buffers": [{"uri": "gone.bin", "byteLength": 4}]
    })JSON";
    REQUIRE_FALSE(fs::write_text_file(dir.file("models/prop.gltf"), gltf));

    auto expanded = pack::expand_asset_dependencies({"models/prop.gltf"}, dir.str(), GltfLoader::dependency_reader());
    REQUIRE(expanded.size() == 2);
    CHECK(expanded[0] == "models/prop.gltf");
    CHECK(expanded[1] == "models/gone.bin");
}

TEST_CASE("an asset closure follows a container named by a container") {
    // The worklist, not a single pass: a glTF whose image is another glTF would
    // otherwise stop one level deep.
    TempDir dir("closure_deep");
    const std::string outer = R"JSON({
        "asset": {"version": "2.0"},
        "images": [{"uri": "inner.gltf"}]
    })JSON";
    const std::string inner = R"JSON({
        "asset": {"version": "2.0"},
        "buffers": [{"uri": "leaf.bin", "byteLength": 4}]
    })JSON";
    REQUIRE_FALSE(fs::write_text_file(dir.file("outer.gltf"), outer));
    REQUIRE_FALSE(fs::write_text_file(dir.file("inner.gltf"), inner));
    REQUIRE_FALSE(fs::write_text_file(dir.file("leaf.bin"), "BIN!"));

    auto expanded = pack::expand_asset_dependencies({"outer.gltf"}, dir.str(), GltfLoader::dependency_reader());
    REQUIRE(expanded.size() == 3);
    CHECK(expanded[0] == "outer.gltf");
    CHECK(expanded[1] == "inner.gltf");
    CHECK(expanded[2] == "leaf.bin");
}

TEST_CASE("an asset closure does not repeat a dependency two assets share") {
    TempDir dir("closure_shared");
    const std::string gltf = R"JSON({
        "asset": {"version": "2.0"},
        "buffers": [{"uri": "shared.bin", "byteLength": 4}]
    })JSON";
    REQUIRE_FALSE(fs::write_text_file(dir.file("a.gltf"), gltf));
    REQUIRE_FALSE(fs::write_text_file(dir.file("b.gltf"), gltf));
    REQUIRE_FALSE(fs::write_text_file(dir.file("shared.bin"), "BIN!"));

    auto expanded = pack::expand_asset_dependencies({"a.gltf", "b.gltf"}, dir.str(), GltfLoader::dependency_reader());
    REQUIRE(expanded.size() == 3);
    CHECK(expanded[0] == "a.gltf");
    CHECK(expanded[1] == "b.gltf");
    CHECK(expanded[2] == "shared.bin");
}

TEST_CASE("the default pack options pack the whole tree, not a pruned one") {
    TempDir dir("noprune");
    REQUIRE_FALSE(fs::write_text_file(dir.file("assets/used.png"), "USED"));
    REQUIRE_FALSE(fs::write_text_file(dir.file("assets/spare.png"), "SPARE"));
    REQUIRE_FALSE(fs::write_text_file(dir.file("scenes/main.fescene"), "{\"format\":\"fork_scene\"}"));

    AssetDatabase assets;
    assets.set_root(dir.file("assets"));
    REQUIRE(assets.import("used.png", "tex"));

    // No options at all: the historical behaviour, so a caller that never heard
    // of pruning is unaffected.
    auto built = pack::PackWriter::build(assets, dir.file("scenes/main.fescene"), "scenes/main.fescene");
    REQUIRE(built);
    CHECK(built.value().find_path("used.png") != nullptr);
    CHECK(built.value().find_path("spare.png") != nullptr);
}

TEST_CASE("a pack mounts on a ResourceManager and serves loads with no loose files") {
    TempDir dir("mount");
    // A real, tiny scene with one sprite, written by the real serializer so the
    // packed bytes are a genuine scene document. The sprite carries no texture:
    // the CPU ImageLoader under the default scheme is what proves a packed
    // asset is decoded, without needing a GPU.
    Scene authored;
    authored.set_name("Packed");
    Node* node = authored.create_node("Hero");
    auto* sprite = node->add_component<Sprite2D>();
    sprite->size = Vec2{32.0f, 32.0f};
    REQUIRE_FALSE(serialization::scene::save(authored, dir.file("scenes/main.fescene")));

    const std::vector<u8> pixels = checker_tga_bytes();
    REQUIRE_FALSE(fs::write_binary_file(dir.file("assets/checker.tga"), pixels.data(), pixels.size()));

    AssetDatabase assets;
    assets.set_root(dir.file("assets"));
    auto imported = assets.import("checker.tga", "");
    REQUIRE(imported);

    // Pack it, then delete the loose files: everything below reads only from the
    // pack, which is the property a shipped build relies on.
    REQUIRE_FALSE(pack::PackWriter::write(dir.file("game.fepack"), assets, dir.file("scenes/main.fescene"),
                                          "scenes/main.fescene"));
    std::filesystem::remove_all(dir.file("assets"));
    std::filesystem::remove_all(dir.file("scenes"));

    auto source = pack::PackSource::open(dir.file("game.fepack"));
    REQUIRE(source);

    ResourceManager manager;
    manager.mount(source.value());

    // The startup scene loads through the ordinary path, from the pack.
    Scene loaded;
    REQUIRE_FALSE(serialization::scene::load("scenes/main.fescene", loaded, &manager));
    CHECK(loaded.name() == "Packed");
    REQUIRE(loaded.node_count() == 1);

    // A guid reference resolves through the pack's own index, with no asset
    // database on the manager, and the bytes decode through the real ImageLoader
    // (registered under the default scheme, which the entry records).
    manager.register_loader(std::make_unique<ImageLoader>());
    CHECK(manager.resolve_asset(imported.value().id) != nullptr);
    auto image = manager.load_as<Image>(to_reference(imported.value().id));
    REQUIRE(image);
    CHECK(image.value()->width() == 2);
    CHECK(image.value()->height() == 2);
    CHECK(image.value()->pixels().size() == 2U * 2U * 4U);

    // A missing key fails cleanly rather than reading a loose file.
    auto missing = manager.read_text("scenes/absent.fescene");
    CHECK_FALSE(missing);
}

TEST_CASE("a pack index parses without reading payload bytes and reads one entry on demand") {
    // The streaming seam: parse_index() reads only the table of contents, so a
    // large pack can be opened without touching its payload region, and
    // read_payload() materialises a single entry on demand. The entry must read
    // back byte-identical to the fully-loaded parse().
    PackFile pack;
    const std::vector<u8> a(3000, u8{0x11});
    const std::vector<u8> b = bytes_of("the quick brown fox");
    const std::vector<u8> c(2048, u8{0x7E});
    pack.add_file("assets/a.bin", "", Uuid::generate(), a);
    pack.add_file("assets/b.txt", "tex", Uuid::generate(), b);
    pack.add_file("assets/c.bin", "", Uuid{}, c);

    // A compressed pack exercises the codec path too: the index must carry the
    // codec and original length without decoding anything.
    auto bytes = pack.write(pack::PackCompression::Fast);
    REQUIRE(bytes);
    const std::vector<u8>& container = bytes.value();

    auto indexed = PackFile::parse_index(container.data(), container.size());
    REQUIRE(indexed);
    CHECK(indexed.value().entry_count() == 3);

    // Opening touched no payload: every entry's bytes are still empty, yet its
    // on-disk location and original length are known from the index alone.
    for (const PackEntry& entry : indexed.value().entries()) {
        CHECK(entry.data.empty());
        CHECK(entry.payload_length > 0);
        CHECK(entry.uncompressed_size > 0);
    }

    // The fully-loaded parse is the reference. Both must agree byte for byte.
    auto loaded = PackFile::parse(container.data(), container.size());
    REQUIRE(loaded);

    for (const std::string_view key : {"assets/a.bin", "assets/b.txt", "assets/c.bin"}) {
        const PackEntry* streamed = indexed.value().find_path(key);
        REQUIRE(streamed != nullptr);
        const PackEntry* reference = loaded.value().find_path(key);
        REQUIRE(reference != nullptr);

        auto on_demand = indexed.value().read_payload(container.data(), container.size(), *streamed);
        REQUIRE(on_demand);
        CHECK(on_demand.value() == reference->data);
        CHECK(streamed->scheme == reference->scheme);
        CHECK(streamed->codec == reference->codec);
        CHECK(streamed->uncompressed_size == reference->uncompressed_size);
    }

    // A streaming read of one entry leaves the others untouched: reading 'b'
    // did not materialise 'a' or 'c'.
    const PackEntry* a_entry = indexed.value().find_path("assets/a.bin");
    REQUIRE(a_entry != nullptr);
    CHECK(a_entry->data.empty());
}

TEST_CASE("a streaming pack read rejects a malformed container like the full parse") {
    PackFile pack;
    pack.add_file("assets/a.txt", "", Uuid{}, bytes_of("hello"));
    auto bytes = pack.write();
    REQUIRE(bytes);
    const std::vector<u8>& valid = bytes.value();

    // A wrong magic is rejected by the index parse, before any payload is read.
    std::vector<u8> corrupt = valid;
    corrupt[0] = 'X';
    CHECK_FALSE(PackFile::parse_index(corrupt.data(), corrupt.size()));

    // A truncated buffer (index itself cut short) is rejected too.
    CHECK_FALSE(PackFile::parse_index(valid.data(), valid.size() / 2));
}

TEST_CASE("opening a pack source reads the index only, and an entry is read on demand") {
    TempDir dir("lazy_open");
    // A pack with a large, compressible entry so the payload region dominates
    // the file and the difference between index-only and whole-file open is
    // unambiguous.
    PackFile pack;
    const std::vector<u8> big(64 * 1024, u8{0x42});
    const std::vector<u8> small = bytes_of("a small text asset");
    const Uuid big_id = Uuid::generate();
    pack.add_file("assets/big.bin", "", big_id, big);
    pack.add_file("scenes/main.fescene", "", Uuid{}, small);
    auto bytes = pack.write(PackCompression::Best);
    REQUIRE(bytes);
    REQUIRE_FALSE(fs::write_binary_file(dir.file("game.fepack"), bytes.value().data(), bytes.value().size()));

    auto source = PackSource::open(dir.file("game.fepack"));
    REQUIRE(source);

    // Opening read only the header and the index: no entry carries payload bytes
    // in memory, yet each entry's on-disk location is known.
    const PackFile& opened = source.value()->pack();
    CHECK(opened.entry_count() == 2);
    for (const PackEntry& entry : opened.entries()) {
        CHECK(entry.data.empty());
        CHECK(entry.payload_length > 0);
        CHECK(entry.uncompressed_size > 0);
    }

    // Each entry reads back byte-identical to the authored payload, through the
    // ordinary source API. The compressed entry decompresses transparently.
    auto read_big = source.value()->read_binary("assets/big.bin");
    REQUIRE(read_big);
    CHECK(read_big.value() == big);

    auto read_scene = source.value()->read_text("scenes/main.fescene");
    REQUIRE(read_scene);
    CHECK(read_scene.value() == std::string(small.begin(), small.end()));

    // The guid lookup resolves through the index without reading any payload.
    CHECK(source.value()->resolve_asset(AssetId{big_id}) != nullptr);

    // A second read of the same entry returns the same bytes (the memo slot),
    // and reading a different entry afterwards still works.
    auto read_again = source.value()->read_binary("assets/big.bin");
    REQUIRE(read_again);
    CHECK(read_again.value() == big);

    // A missing key fails cleanly.
    CHECK_FALSE(source.value()->read_binary("assets/absent.bin"));
}

TEST_CASE("a lazy pack source serves the same bytes as the whole-file parse") {
    TempDir dir("lazy_equivalence");
    PackFile pack;
    const std::vector<u8> compressed_well = std::vector<u8>(8192, u8{0x11});
    const std::vector<u8> incompressible = [] {
        std::vector<u8> noise(512);
        u32 state = 0x12345678U;
        for (u8& byte : noise) {
            state = state * 1664525U + 1013904223U;
            byte = static_cast<u8>(state >> 24);
        }
        return noise;
    }();
    pack.add_file("assets/repetitive.bin", "", Uuid{}, compressed_well);
    pack.add_file("assets/noise.bin", "", Uuid{}, incompressible);
    auto bytes = pack.write(PackCompression::Fast);
    REQUIRE(bytes);
    REQUIRE_FALSE(fs::write_binary_file(dir.file("game.fepack"), bytes.value().data(), bytes.value().size()));

    // The whole-file parse, as the reference.
    auto loaded = PackFile::parse(bytes.value().data(), bytes.value().size());
    REQUIRE(loaded);

    // The lazy source, reading from disk on demand.
    auto source = PackSource::open(dir.file("game.fepack"));
    REQUIRE(source);

    for (const std::string_view key : {"assets/repetitive.bin", "assets/noise.bin"}) {
        const PackEntry* reference = loaded.value().find_path(key);
        REQUIRE(reference != nullptr);
        auto streamed = source.value()->read_binary(key);
        REQUIRE(streamed);
        CHECK(streamed.value() == reference->data);
    }
}

TEST_CASE("a corrupted payload fails on read, not at open, for a lazy source") {
    TempDir dir("lazy_corrupt");
    PackFile pack;
    pack.add_file("assets/big.bin", "", Uuid{}, std::vector<u8>(4096, u8{0x33}));
    auto packed = pack.write(PackCompression::Fast);
    REQUIRE(packed);

    std::vector<u8> corrupt = packed.value();
    // Flip a byte inside the compressed payload (the last byte of the file), so
    // the stream no longer decodes.
    corrupt.back() ^= 0xFFU;
    REQUIRE_FALSE(fs::write_binary_file(dir.file("game.fepack"), corrupt.data(), corrupt.size()));

    // Opening succeeds: the index is intact and the payload is not decoded until
    // it is asked for. This is the whole point of the streaming open - a bad
    // payload is not a mount-time failure for every asset in the pack.
    auto source = PackSource::open(dir.file("game.fepack"));
    REQUIRE(source);

    // Reading the one bad entry fails, and fails cleanly rather than serving a
    // short or wrong buffer.
    auto read = source.value()->read_binary("assets/big.bin");
    CHECK_FALSE(read);

    // The whole-file parse, by contrast, rejects the same bytes up front, which
    // is what the lazy open deliberately defers.
    CHECK_FALSE(PackFile::parse(corrupt.data(), corrupt.size()));
}

TEST_CASE("DEFLATE compresses and decompresses a payload losslessly") {
    // A highly compressible input: the same byte repeated. DEFLATE must shrink
    // it, and the round trip must return the exact bytes.
    std::vector<u8> input(4096, u8{0x41});
    auto compressed = pack::compress(input, pack::PackCompression::Fast);
    REQUIRE(compressed);
    CHECK(compressed.value().size() < input.size());

    auto restored = pack::decompress(compressed.value().data(), compressed.value().size(), input.size());
    REQUIRE(restored);
    CHECK(restored.value() == input);

    // Incompressible input still round-trips: the codec is lossless whatever the
    // bytes are, so the writer's "only if smaller" rule is a size decision, not a
    // correctness one.
    std::vector<u8> noise(1024);
    u32 state = 0x12345678U;
    for (u8& b : noise) {
        state = state * 1664525U + 1013904223U; // a plain LCG, so the bytes do not repeat
        b = static_cast<u8>(state >> 24);
    }
    auto noise_compressed = pack::compress(noise, pack::PackCompression::Fast);
    REQUIRE(noise_compressed);
    auto noise_restored =
        pack::decompress(noise_compressed.value().data(), noise_compressed.value().size(), noise.size());
    REQUIRE(noise_restored);
    CHECK(noise_restored.value() == noise);
}

TEST_CASE("an empty payload round-trips through the codec") {
    const std::vector<u8> empty;
    auto compressed = pack::compress(empty, pack::PackCompression::Fast);
    REQUIRE(compressed);
    auto restored = pack::decompress(compressed.value().data(), compressed.value().size(), 0);
    REQUIRE(restored);
    CHECK(restored.value().empty());
}

TEST_CASE("a compressed stream that does not match the promised length is rejected") {
    std::vector<u8> input(512, u8{0x7});
    auto compressed = pack::compress(input, pack::PackCompression::Fast);
    REQUIRE(compressed);

    // The index promises 512 bytes; claim 511 and the reader must refuse rather
    // than hand back a short buffer a loader would misread.
    auto wrong = pack::decompress(compressed.value().data(), compressed.value().size(), input.size() - 1);
    CHECK_FALSE(wrong);

    // Garbage that is not a DEFLATE stream is rejected too.
    const std::array<u8, 4> junk{0xDE, 0xAD, 0xBE, 0xEF};
    auto bad = pack::decompress(junk.data(), junk.size(), 512);
    CHECK_FALSE(bad);
}

TEST_CASE("an uncompressed pack is byte for byte the format before compression") {
    // The compatibility guarantee: with the default codec the writer emits
    // version 1 and the same bytes it always did, so a pack built before
    // compression existed and one built now are identical.
    PackFile pack;
    pack.add_file("assets/a.txt", "", Uuid{}, bytes_of("hello"));
    pack.add_file("assets/b.txt", "tex", Uuid::generate(), bytes_of("world"));

    auto raw = pack.write(); // default: PackCompression::None
    REQUIRE(raw);

    // The header's format_version is the u32 at byte 4; it must be 1.
    const u32 version = static_cast<u32>(raw.value()[4]) | (static_cast<u32>(raw.value()[5]) << 8);
    CHECK(version == 1);

    // Explicitly asking for None gives the same bytes as the default.
    auto explicit_none = pack.write(pack::PackCompression::None);
    REQUIRE(explicit_none);
    CHECK(explicit_none.value() == raw.value());

    // And it parses back with every entry stored raw.
    auto parsed = PackFile::parse(raw.value().data(), raw.value().size());
    REQUIRE(parsed);
    REQUIRE(parsed.value().entry_count() == 2);
    CHECK(parsed.value().find_path("assets/a.txt")->codec == pack::PackCodec::None);
    CHECK(parsed.value().find_path("assets/b.txt")->data == bytes_of("world"));
}

TEST_CASE("a compressed pack round-trips its payloads and is smaller") {
    PackFile pack;
    // A compressible payload and an incompressible one, so the test covers both
    // the compressed and the stored-raw branch in one pack.
    const std::vector<u8> compressible(8192, u8{0x5A});
    std::vector<u8> noise(512);
    u32 state = 0xC0FFEEU;
    for (u8& b : noise) {
        state = state * 1664525U + 1013904223U;
        b = static_cast<u8>(state >> 24);
    }
    const Uuid id = Uuid::generate();
    pack.add_file("assets/big.bin", "tex", id, compressible);
    pack.add_file("assets/noise.bin", "", Uuid{}, noise);

    auto raw = pack.write(pack::PackCompression::None);
    auto packed = pack.write(pack::PackCompression::Best);
    REQUIRE(raw);
    REQUIRE(packed);
    CHECK(packed.value().size() < raw.value().size());

    auto parsed = PackFile::parse(packed.value().data(), packed.value().size());
    REQUIRE(parsed);
    REQUIRE(parsed.value().entry_count() == 2);

    // The compressible entry was stored with the DEFLATE codec and comes back
    // decompressed, byte for byte.
    const pack::PackEntry* big = parsed.value().find_path("assets/big.bin");
    REQUIRE(big != nullptr);
    CHECK(big->codec == pack::PackCodec::Deflate);
    CHECK(big->uncompressed_size == compressible.size());
    CHECK(big->data == compressible);
    CHECK(big->guid == id);

    // The incompressible entry is stored raw - compressing it would grow the
    // pack - and its bytes survive unchanged.
    const pack::PackEntry* noise_entry = parsed.value().find_path("assets/noise.bin");
    REQUIRE(noise_entry != nullptr);
    CHECK(noise_entry->codec == pack::PackCodec::None);
    CHECK(noise_entry->data == noise);

    // A guid lookup still resolves through a compressed pack.
    CHECK(parsed.value().find_guid(id) == big);
}

TEST_CASE("a compressed pack mounts and serves loads with no loose files") {
    TempDir dir("compressed_mount");
    Scene authored;
    authored.set_name("Compressed");
    Node* node = authored.create_node("Hero");
    node->add_component<Sprite2D>()->size = Vec2{16.0f, 16.0f};
    REQUIRE_FALSE(serialization::scene::save(authored, dir.file("scenes/main.fescene")));

    const std::vector<u8> pixels = checker_tga_bytes();
    REQUIRE_FALSE(fs::write_binary_file(dir.file("assets/checker.tga"), pixels.data(), pixels.size()));

    AssetDatabase assets;
    assets.set_root(dir.file("assets"));
    auto imported = assets.import("checker.tga", "");
    REQUIRE(imported);

    pack::PackOptions options;
    options.compression = pack::PackCompression::Best;
    REQUIRE_FALSE(pack::PackWriter::write(dir.file("game.fepack"), assets, dir.file("scenes/main.fescene"),
                                          "scenes/main.fescene", options));
    std::filesystem::remove_all(dir.file("assets"));
    std::filesystem::remove_all(dir.file("scenes"));

    auto source = pack::PackSource::open(dir.file("game.fepack"));
    REQUIRE(source);

    ResourceManager manager;
    manager.mount(source.value());

    Scene loaded;
    REQUIRE_FALSE(serialization::scene::load("scenes/main.fescene", loaded, &manager));
    CHECK(loaded.name() == "Compressed");

    // The texture decodes from the compressed pack through the real loader, so
    // decompression is transparent to the whole load path, not just to a byte
    // read.
    manager.register_loader(std::make_unique<ImageLoader>());
    auto image = manager.load_as<Image>(to_reference(imported.value().id));
    REQUIRE(image);
    CHECK(image.value()->width() == 2);
    CHECK(image.value()->height() == 2);
    CHECK(image.value()->pixels().size() == 2U * 2U * 4U);
}

TEST_CASE("a corrupted compressed payload is rejected, not served as garbage") {
    PackFile pack;
    pack.add_file("assets/big.bin", "", Uuid{}, std::vector<u8>(4096, u8{0x33}));
    auto packed = pack.write(pack::PackCompression::Fast);
    REQUIRE(packed);

    std::vector<u8> corrupt = packed.value();
    // Flip a byte inside the compressed payload (the last byte of the file) so
    // the stream no longer decodes. The reader must fail rather than return a
    // short or wrong buffer.
    corrupt.back() ^= 0xFFU;
    auto parsed = PackFile::parse(corrupt.data(), corrupt.size());
    CHECK_FALSE(parsed);
}
