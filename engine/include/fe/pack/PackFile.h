// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Error.h"
#include "fe/core/Types.h"
#include "fe/core/Uuid.h"
#include "fe/pack/PackCompression.h"

#include <string>
#include <string_view>
#include <vector>

namespace fe::pack {

// A pack: a project's assets baked into one container the runtime mounts.
//
// A pack is the shipping form of an asset tree. A game built here cannot assume
// an ordinary filesystem (a browser, a sandboxed mobile app), so its assets ship
// in one file with a table of contents, and the runtime reads them through the
// same resource paths it uses on disk. See
// docs/adr/0018-multi-platform-shipping.md and
// docs/adr/0030-asset-pack-format.md.
//
// This type is the container as a value: it knows the on-disk layout, it
// serialises itself, and it parses itself, but it never touches the filesystem
// or a graphics device. It is the pack analogue of ImportSettings and of the
// scene format - a pure value that is exhaustively testable on its own.
//
// The on-disk layout, little-endian throughout:
//
//   header    magic 'FEPK' (4 bytes), format_version u32, entry_count u32,
//             index_offset u64, payload_offset u64
//   index     per entry, in order: key length u32 + key bytes,
//             scheme length u32 + scheme bytes, guid (16 raw bytes),
//             payload offset u64, payload length u64,
//             codec u8 + uncompressed length u64  [format version 2 only]
//   payload   the file bytes, referenced by the index
//
// Format version 2 adds the per-entry codec and the original (uncompressed)
// length. A pack with no compressed entry is still written as version 1, so its
// bytes are identical to what the format defined before compression existed; a
// version-2 pack names a codec per entry and is decompressed transparently by
// the reader. A version-1 pack parses as version 1 (every entry `None`), so the
// reader accepts both.
//
// Every offset and length is validated against the buffer on parse, so a
// malformed, truncated, or wrong-version pack is rejected rather than read as
// garbage.
struct PackEntry {
    // The path a loader is given ("assets/hero.png"). This is the pack's key:
    // it is exactly the string ResourceManager hands a loader today, so a
    // mounted pack serves the same request the filesystem did.
    std::string key;
    // The loader scheme the asset is imported under ("tex", "obj", "wav", or ""
    // for the default scheme). Empty means the default scheme.
    std::string scheme;
    // The asset's stable identity, or a nil id when the entry is not an asset
    // (the startup scene, or an unindexed file). Carried so a pack is
    // self-describing and a "guid:" reference resolves without the loose
    // sidecars.
    Uuid guid{};
    // The file's bytes. Held by value so the value type is self-contained; a
    // pack is read whole into memory (streaming is later work). On a parsed
    // entry this is always the *decompressed* bytes, whatever the codec: the
    // codec is an on-disk detail the reader hides, so a caller reads `data` the
    // same way for a stored and a compressed entry.
    std::vector<u8> data;
    // The codec the payload uses on disk. `None` for a stored entry (the
    // default, and what every entry of a version-1 pack is). The writer records
    // it; the reader honours it and leaves `data` decompressed.
    PackCodec codec = PackCodec::None;
    // The payload's original length. Equals `data.size()` for a stored entry and
    // the decoded length for a compressed one; carried so the reader can size
    // its output buffer exactly and reject a stream that decodes to a different
    // length.
    u64 uncompressed_size = 0;

    [[nodiscard]] bool has_guid() const noexcept { return !guid.is_nil(); }
};

class PackFile {
public:
    // The format marker and version. The marker catches a file that is not a
    // pack; the version catches one from the future. A reader rejects both.
    //
    // Version 1 is the original format (raw payloads). Version 2 adds a per-entry
    // codec and original length. write() emits version 1 when nothing is
    // compressed and version 2 otherwise, so an uncompressed pack is unchanged
    // from the format before compression existed; parse() accepts both.
    static constexpr u32 kMagic = 0x4B504546U; // 'FEPK' little-endian
    static constexpr i32 kFormatVersion = 2;
    static constexpr i32 kFormatVersionRaw = 1;

    PackFile() = default;

    // Appends an entry. `data` is moved in. The key must be unique; a duplicate
    // key is rejected by write() rather than silently shadowed. The entry is
    // stored raw; write() applies compression.
    void add_file(std::string key, std::string scheme, Uuid guid, std::vector<u8> data);

    // The entries, in insertion order. write() sorts them by key so a rebuild
    // from the same inputs produces the same bytes.
    [[nodiscard]] const std::vector<PackEntry>& entries() const noexcept { return entries_; }
    [[nodiscard]] usize entry_count() const noexcept { return entries_.size(); }

    // Finds an entry by its exact key, or nullptr.
    [[nodiscard]] const PackEntry* find_path(std::string_view key) const;
    // Finds an entry by its stable id, or nullptr. A nil id matches nothing.
    [[nodiscard]] const PackEntry* find_guid(const Uuid& id) const;

    // Serialises to the container bytes. Entries are written sorted by key so
    // the output is deterministic. Fails on a duplicate key or an over-long
    // key/scheme (the format stores lengths as u32).
    //
    // `compression` selects the per-entry codec. The default (`None`) writes
    // version 1 and stores every payload raw, byte for byte as before. Any other
    // value writes version 2 and compresses each payload whose compressed form is
    // smaller than the raw bytes (an incompressible payload is stored raw with
    // the `None` codec, so compression never makes a pack larger).
    [[nodiscard]] Result<std::vector<u8>> write(PackCompression compression = PackCompression::None) const;

    // Parses the container bytes. Returns an Error for a wrong magic, a version
    // newer than supported, a truncated buffer, a bad length, an entry whose
    // payload lies outside the buffer, or a compressed entry that does not
    // decode. Never returns a partial pack.
    [[nodiscard]] static Result<PackFile> parse(const u8* data, usize size);

private:
    std::vector<PackEntry> entries_;
};

} // namespace fe::pack
