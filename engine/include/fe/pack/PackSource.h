// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Error.h"
#include "fe/pack/PackFile.h"
#include "fe/resources/ResourceSource.h"

#include <memory>
#include <string_view>

namespace fe::pack {

// A ResourceSource backed by a pack: the runtime mount.
//
// Mounting one on a ResourceManager (`ResourceManager::mount`) makes every load
// read from the pack instead of disk, so the loaders and the scene reader serve
// the same requests they did for a loose tree. This is the "read from a pack as
// transparently as from disk" half of
// docs/adr/0018-multi-platform-shipping.md; the format and the decision are in
// docs/adr/0030-asset-pack-format.md.
//
// The source owns the parsed pack, so it is held through a shared_ptr and stays
// alive as long as the manager holds it.
//
// Opening a pack reads only its header and index; a payload is materialised on
// demand, when the asset that names it is first loaded, by reading just that
// entry's span from the container. So mounting a large pack (an audio bank, a
// big texture set) does not pull the whole payload region into memory, and an
// asset that is never played is never read (ADR-0030's streaming refinement).
class PackSource final : public ResourceSource {
public:
    // Opens a pack file: reads and validates the header and the index only, so
    // the payload region is left on disk until an entry is asked for. Returns an
    // Error when the file cannot be read or is not a valid pack, so a bad pack
    // fails at mount time with a clear message rather than at the first load.
    [[nodiscard]] static Result<std::shared_ptr<PackSource>> open(std::string_view path);

    // Wraps an already-parsed pack whose payloads are in memory (the whole-file
    // parse, e.g. a pack fetched over HTTP). Reads are answered from the entry's
    // own bytes.
    explicit PackSource(PackFile pack) : pack_(std::move(pack)) {}

    // Wraps an index-parsed pack plus the container path its payloads are read
    // from on demand. `pack` must come from `PackFile::parse_index` of the file
    // at `container_path`.
    PackSource(PackFile pack, std::string container_path)
        : pack_(std::move(pack)), container_path_(std::move(container_path)) {}

    [[nodiscard]] Result<std::vector<u8>> read_binary(std::string_view path) const override;
    [[nodiscard]] Result<std::string> read_text(std::string_view path) const override;
    [[nodiscard]] bool exists(std::string_view path) const override;
    [[nodiscard]] const AssetLocation* resolve_asset(const AssetId& id) const override;

    [[nodiscard]] const PackFile& pack() const noexcept { return pack_; }

private:
    // The bytes of one entry's payload, decompressed. Answers from the entry's
    // own `data` when the pack was parsed whole, and otherwise reads just that
    // entry's span from the container and decodes it. A single memo slot holds
    // the last payload materialised, so re-reading the same entry is a copy, not
    // a second disk read; a source is read from one thread (the RHI contract
    // already states a device is owned by one thread, and the manager follows
    // it), so one slot is enough, exactly as `location_` is below.
    [[nodiscard]] Result<std::vector<u8>> payload_for(const PackEntry& entry) const;

    PackFile pack_;
    // The container file the payload region is read from on demand. Empty when
    // the pack was wrapped whole (its payloads are already in memory).
    std::string container_path_;
    // The last payload materialised, and the key it belongs to. A payload that
    // is legitimately empty still caches: the key match, not the size, is the
    // hit test.
    mutable std::string cached_key_;
    mutable std::vector<u8> cached_payload_;
    // A resolved location is returned by pointer, so the last one is cached
    // here. A source is read from one thread (the RHI contract already states a
    // device is owned by one thread, and the manager follows it), so a single
    // slot is enough.
    mutable AssetLocation location_;
};

} // namespace fe::pack
