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
    // The on-disk payload span within the container bytes: where this entry's
    // bytes live and how many there are. Set by both parse() and parse_index();
    // together with `codec` and `uncompressed_size` this is everything
    // `read_payload` needs to materialise the bytes on demand, so a streaming
    // reader can skip the payload region entirely (see parse_index).
    u64 payload_offset = 0;
    u64 payload_length = 0;
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

    // The fixed header size: magic, format_version, entry_count, index_offset,
    // payload_offset. A streaming reader reads exactly this many bytes first to
    // learn where the index ends.
    static constexpr usize kHeaderSize = 4 + 4 + 4 + 8 + 8;

    // The container's fixed header, as read by parse_header(). Enough to know
    // where the index lives and how much of the file it spans, without reading
    // the index or any payload.
    struct Header {
        i32 format_version = 0;
        u32 entry_count = 0;
        u64 index_offset = 0;
        // The offset the payload region starts at, which is also the end of the
        // index: reading [0, payload_offset) yields the header and the index.
        u64 payload_offset = 0;
    };

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

    // Reads the whole pack file at `path` and parses it: the "read a pack whole
    // from a path" entry point. Every payload is resident afterwards. The pack
    // can then be re-pointed at the same container for on-demand reads with
    // bind_container(), so a caller holding a whole-file pack can hand its bytes
    // back and stream instead, without the two-step of parse_index() plus a
    // separately supplied PackSource path (which nothing checked agreed).
    [[nodiscard]] static Result<PackFile> parse_file(std::string_view path);

    // Parses only the container's fixed header. `data` must hold at least
    // kHeaderSize bytes. This is what a streaming reader calls first, to learn
    // how much of the file the index spans before reading it.
    //
    // `container_size` is the true size of the whole container, used to validate
    // the header's offsets; the payload region need not be present in `data`.
    // Zero means "the buffer is the whole container".
    [[nodiscard]] static Result<Header> parse_header(const u8* data, usize size, u64 container_size = 0);

    // Parses only the index of the container bytes: every entry's key, scheme,
    // guid, codec, on-disk location and original length, but not its payload
    // bytes. `data` is left empty on each entry and `read_payload` materialises
    // one on demand, so a large pack can be opened without reading the payload
    // region (ADR-0030's streaming refinement). The validation is exactly
    // parse()'s, so a malformed pack is rejected here too; the only difference is
    // that a compressed payload is not decoded until it is asked for.
    //
    // `data`/`size` must cover the header and the whole index, i.e. the first
    // `Header::payload_offset` bytes of the container. `container_size` is the
    // true size of the whole container, used only to validate each entry's
    // payload span; the payload bytes need not be present. Zero means "the
    // buffer is the whole container" (the whole-file case), which is what a
    // caller holding the complete bytes passes.
    [[nodiscard]] static Result<PackFile> parse_index(const u8* data, usize size, u64 container_size = 0);

    // Materialises one entry's payload from the container bytes `data`/`size`
    // (the same bytes the index was parsed from), decompressing transparently
    // when the entry is compressed. This is what parse() calls per entry, exposed
    // so a streaming reader can read a single payload without the others. The
    // entry must come from parse_index()/parse() of these bytes; a span that does
    // not fit is an Error, not a read of garbage.
    [[nodiscard]] Result<std::vector<u8>> read_payload(const u8* data, usize size, const PackEntry& entry) const;

    // Materialises one entry's payload without the caller holding the container
    // bytes: from the file bound by bind_container() when one is bound, and
    // otherwise from the entry's resident `data` (a whole-file parse). This is
    // the read a pack re-pointed at its container serves, so a whole-file parse
    // that has released its payloads is read on demand through the same call.
    [[nodiscard]] Result<std::vector<u8>> read_payload(const PackEntry& entry) const;

    // The same read against the container held as a vector, so a caller that
    // kept the whole-file bytes can read an entry by value. The bytes are
    // borrowed, not copied.
    [[nodiscard]] Result<std::vector<u8>> read_payload(const std::vector<u8>& container,
                                                       const PackEntry& entry) const {
        return read_payload(container.data(), container.size(), entry);
    }

    // Drops every entry's in-memory payload, keeping its on-disk span
    // (`payload_offset`/`payload_length`) and `codec`, so a pack that was read
    // whole can be served on demand from the container bytes afterwards. This is
    // the whole-file counterpart of parse_index(): parse() materialises every
    // payload, and release_payloads() lets the caller give that memory back once
    // it has decided to stream instead, reading entries through read_payload()
    // with the bytes parse() was given. An entry with no payload (length 0) is
    // unchanged: its read still yields an empty buffer.
    void release_payloads() noexcept;

    // Whether every entry's payload is currently resident in memory. True after
    // parse(), false after release_payloads() (and for a parse_index() pack).
    [[nodiscard]] bool payloads_resident() const noexcept;

    // Re-points a whole-parsed pack at the container file it came from, so its
    // payloads are served on demand from that file instead of from the resident
    // copies. This is the one-step form of "parse_index plus a PackSource path":
    // the pack and the container cannot disagree, because the path is the one
    // parse_file() read. The call reads the container's header and index back and
    // checks they match this pack (magic, format version, entry count, keys,
    // spans and codecs) before it drops any payload; a path that is not this
    // pack's container is rejected with an Error and the resident payloads are
    // left intact, so a caller that passes a wrong or truncated file keeps a
    // working pack rather than a pack whose reads silently come from the wrong
    // bytes.
    //
    // After a successful bind, payloads_resident() is false and read_payload()
    // with the container bytes still serves each entry (the span and codec are
    // kept). `path` must name the same container parse()/parse_file() read the
    // index from.
    [[nodiscard]] Result<void> bind_container(std::string_view path);

    // The container path bound by bind_container(), or empty when none is bound.
    [[nodiscard]] const std::string& container_path() const noexcept { return container_path_; }

    // Decodes one entry's payload from bytes already located at the entry's
    // payload span, decompressing when the entry names a codec. `payload_size`
    // must equal the entry's on-disk length. This is the half of read_payload()
    // that does not need the whole container, so a reader that pulled a single
    // range from disk (or a network response) decodes it without materialising
    // the rest of the pack.
    [[nodiscard]] static Result<std::vector<u8>> decode_payload(const u8* payload, usize payload_size,
                                                                const PackEntry& entry);

private:
    std::vector<PackEntry> entries_;
    // Whether every entry's payload is currently held in `data`. Set by parse()
    // and cleared by release_payloads(); a parse_index() pack starts false. It
    // tracks the whole table, not a per-entry flag, because parse() and
    // release_payloads() act on the whole pack at once.
    bool payloads_resident_ = false;
    // The container file a whole-parsed pack was re-pointed at by
    // bind_container(). Empty when none is bound, in which case reads come from
    // each entry's resident `data`.
    std::string container_path_;
};

} // namespace fe::pack
