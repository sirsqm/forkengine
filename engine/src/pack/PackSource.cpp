// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/pack/PackSource.h"

#include "fe/core/FileSystem.h"

#include <string>
#include <utility>
#include <vector>

namespace fe::pack {

Result<std::shared_ptr<PackSource>> PackSource::open(std::string_view path) {
    // Read only the header, then only the index. The payload region - the bulk
    // of a pack - is never read here; an entry's bytes are pulled from disk when
    // the asset that names it is first loaded (see payload_for). This is what
    // keeps a large pack's resident set proportional to what the game uses, not
    // to what it ships.
    auto header_bytes = fs::read_binary_file_range(path, 0, PackFile::kHeaderSize);
    if (!header_bytes) {
        return header_bytes.error();
    }
    const u64 container_size = fs::file_size(path);
    auto header = PackFile::parse_header(header_bytes.value().data(), header_bytes.value().size(), container_size);
    if (!header) {
        return header.error();
    }

    // The header and the index are contiguous from the start of the file, so one
    // range read covers both. The index is parsed against the true container
    // size, so an entry whose payload lies outside the file is still rejected
    // even though those bytes were never read.
    const u64 index_span = header.value().payload_offset;
    if (index_span < PackFile::kHeaderSize) {
        return Error("pack: payload region starts inside the header");
    }
    auto index_bytes = fs::read_binary_file_range(path, 0, index_span);
    if (!index_bytes) {
        return index_bytes.error();
    }
    auto parsed = PackFile::parse_index(index_bytes.value().data(), index_bytes.value().size(), container_size);
    if (!parsed) {
        return parsed.error();
    }
    return std::make_shared<PackSource>(std::move(parsed.value()), std::string(path));
}

Result<std::vector<u8>> PackSource::payload_for(const PackEntry& entry) const {
    // The whole-file wrap already holds the bytes; hand them back without a
    // second read. `container_path_` empty is the "parsed whole" signal.
    if (container_path_.empty()) {
        return entry.data;
    }
    // The last payload materialised is memoised, so a load that reads the same
    // entry twice (a reload, a reference resolved again) pays for it once.
    if (cached_key_ == entry.key) {
        return cached_payload_;
    }
    auto bytes = fs::read_binary_file_range(container_path_, entry.payload_offset, entry.payload_length);
    if (!bytes) {
        return bytes.error();
    }
    auto decoded = PackFile::decode_payload(bytes.value().data(), bytes.value().size(), entry);
    if (!decoded) {
        return decoded.error();
    }
    cached_key_ = entry.key;
    cached_payload_ = decoded.value();
    return decoded.value();
}

Result<std::vector<u8>> PackSource::read_binary(std::string_view path) const {
    const PackEntry* entry = pack_.find_path(path);
    if (entry == nullptr) {
        return Error("pack: no entry '" + std::string(path) + "'");
    }
    return payload_for(*entry);
}

Result<std::string> PackSource::read_text(std::string_view path) const {
    auto bytes = read_binary(path);
    if (!bytes) {
        return bytes.error();
    }
    return std::string(reinterpret_cast<const char*>(bytes.value().data()), bytes.value().size());
}

bool PackSource::exists(std::string_view path) const {
    return pack_.find_path(path) != nullptr;
}

const ResourceSource::AssetLocation* PackSource::resolve_asset(const AssetId& id) const {
    const PackEntry* entry = pack_.find_guid(id);
    if (entry == nullptr) {
        return nullptr;
    }
    location_.path = entry->key;
    location_.scheme = entry->scheme;
    return &location_;
}

} // namespace fe::pack
