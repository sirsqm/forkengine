// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/pack/PackFile.h"

#include "fe/core/FileSystem.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <string>
#include <utility>

namespace fe::pack {
namespace {

// One index entry's fixed part, format version 1: key length, scheme length,
// guid, offset, length.
constexpr usize kEntryFixedSizeV1 = 4 + 4 + 16 + 8 + 8;
// Version 2 appends the codec (1 byte) and the original length (u64).
constexpr usize kEntryFixedSizeV2 = kEntryFixedSizeV1 + 1 + 8;

// Little-endian writers. The format is defined as little-endian on every host,
// so a pack written on one platform reads on another.
void write_u32(std::vector<u8>& out, u32 value) {
    out.push_back(static_cast<u8>(value & 0xFFU));
    out.push_back(static_cast<u8>((value >> 8) & 0xFFU));
    out.push_back(static_cast<u8>((value >> 16) & 0xFFU));
    out.push_back(static_cast<u8>((value >> 24) & 0xFFU));
}

void write_u64(std::vector<u8>& out, u64 value) {
    for (int i = 0; i < 8; ++i) {
        out.push_back(static_cast<u8>((value >> (8 * i)) & 0xFFU));
    }
}

// A cursor over a byte buffer with bounds-checked little-endian reads. Every
// read advances it; a read past the end reports failure so the caller can
// return a clear Error instead of reading garbage.
class Reader {
public:
    Reader(const u8* data, usize size) : data_(data), size_(size) {}

    [[nodiscard]] bool ok() const noexcept { return ok_; }
    [[nodiscard]] usize offset() const noexcept { return offset_; }
    [[nodiscard]] usize remaining() const noexcept { return offset_ <= size_ ? size_ - offset_ : 0; }

    bool read_bytes(void* out, usize count) {
        if (!ok_ || count > remaining()) {
            ok_ = false;
            return false;
        }
        if (count > 0) {
            std::memcpy(out, data_ + offset_, count);
        }
        offset_ += count;
        return true;
    }

    bool read_u32(u32& out) {
        u8 bytes[4];
        if (!read_bytes(bytes, sizeof(bytes))) {
            return false;
        }
        out = static_cast<u32>(bytes[0]) | (static_cast<u32>(bytes[1]) << 8) | (static_cast<u32>(bytes[2]) << 16) |
              (static_cast<u32>(bytes[3]) << 24);
        return true;
    }

    bool read_u8(u8& out) { return read_bytes(&out, 1); }

    bool read_u64(u64& out) {
        u8 bytes[8];
        if (!read_bytes(bytes, sizeof(bytes))) {
            return false;
        }
        u64 value = 0;
        for (int i = 0; i < 8; ++i) {
            value |= static_cast<u64>(bytes[i]) << (8 * i);
        }
        out = value;
        return true;
    }

    // Jumps to an absolute offset. Fails if it is outside the buffer.
    bool seek(usize target) {
        if (!ok_ || target > size_) {
            ok_ = false;
            return false;
        }
        offset_ = target;
        return true;
    }

private:
    const u8* data_;
    usize size_;
    usize offset_ = 0;
    bool ok_ = true;
};

} // namespace

void PackFile::add_file(std::string key, std::string scheme, Uuid guid, std::vector<u8> data) {
    PackEntry entry;
    entry.key = std::move(key);
    entry.scheme = std::move(scheme);
    entry.guid = guid;
    entry.data = std::move(data);
    entries_.push_back(std::move(entry));
}

const PackEntry* PackFile::find_path(std::string_view key) const {
    for (const PackEntry& entry : entries_) {
        if (entry.key == key) {
            return &entry;
        }
    }
    return nullptr;
}

const PackEntry* PackFile::find_guid(const Uuid& id) const {
    if (id.is_nil()) {
        return nullptr;
    }
    for (const PackEntry& entry : entries_) {
        if (!entry.guid.is_nil() && entry.guid == id) {
            return &entry;
        }
    }
    return nullptr;
}

Result<std::vector<u8>> PackFile::write(PackCompression compression) const {
    // Write in a stable order (by key) so a rebuild from the same inputs is
    // byte-identical, which lets a build system treat an unchanged pack as up
    // to date. Reject a duplicate key rather than letting one shadow the other.
    std::vector<const PackEntry*> ordered;
    ordered.reserve(entries_.size());
    for (const PackEntry& entry : entries_) {
        ordered.push_back(&entry);
    }
    std::sort(ordered.begin(), ordered.end(), [](const PackEntry* a, const PackEntry* b) { return a->key < b->key; });
    for (usize i = 1; i < ordered.size(); ++i) {
        if (ordered[i - 1]->key == ordered[i]->key) {
            return Error("pack: duplicate key '" + ordered[i]->key + "'");
        }
    }

    // Resolve each entry's on-disk payload and codec up front. A payload is
    // compressed only when that makes it smaller: an incompressible asset (a
    // PNG, a compressed WAV) is stored raw with the `None` codec, so enabling
    // compression never makes a pack larger than the uncompressed one.
    struct StoredPayload {
        std::vector<u8> compressed; // empty when the payload is stored raw
        PackCodec codec = PackCodec::None;
        u64 uncompressed_size = 0;
    };
    std::vector<StoredPayload> stored;
    stored.reserve(ordered.size());
    for (const PackEntry* entry : ordered) {
        StoredPayload slot;
        slot.uncompressed_size = static_cast<u64>(entry->data.size());
        if (compression != PackCompression::None && !entry->data.empty()) {
            auto compressed = compress(entry->data, compression);
            if (!compressed) {
                return compressed.error();
            }
            // Keep the compressed form only when it is actually smaller; a
            // tie stores raw, so a pack never grows by a byte.
            if (compressed.value().size() < entry->data.size()) {
                slot.compressed = std::move(compressed.value());
                slot.codec = PackCodec::Deflate;
            }
        }
        stored.push_back(std::move(slot));
    }
    // The on-disk bytes of entry `i`: the raw payload, or the compressed form.
    const auto payload_bytes = [&](usize i) -> const std::vector<u8>& {
        return stored[i].codec == PackCodec::None ? ordered[i]->data : stored[i].compressed;
    };

    // The format version follows the content: a pack with no compressed entry is
    // written as version 1 and is byte-identical to the format before compression
    // existed. Only a pack that actually uses a codec is version 2.
    bool any_compressed = false;
    for (const StoredPayload& slot : stored) {
        any_compressed = any_compressed || slot.codec != PackCodec::None;
    }
    const i32 format_version = any_compressed ? kFormatVersion : kFormatVersionRaw;
    const usize entry_fixed_size = any_compressed ? kEntryFixedSizeV2 : kEntryFixedSizeV1;

    // The index length depends only on the keys and schemes, so it can be sized
    // before the payload offset is known.
    usize index_size = 0;
    for (const PackEntry* entry : ordered) {
        if (entry->key.size() > 0xFFFFFFFFULL || entry->scheme.size() > 0xFFFFFFFFULL) {
            return Error("pack: key or scheme is too long for the format");
        }
        index_size += entry_fixed_size + entry->key.size() + entry->scheme.size();
    }

    const u64 index_offset = kHeaderSize;
    const u64 payload_offset = index_offset + index_size;

    std::vector<u8> out;
    out.reserve(static_cast<usize>(payload_offset) + 64);

    write_u32(out, kMagic);
    write_u32(out, static_cast<u32>(format_version));
    write_u32(out, static_cast<u32>(ordered.size()));
    write_u64(out, index_offset);
    write_u64(out, payload_offset);

    // The payload starts after the index; each entry's data is appended in
    // index order, so offsets are assigned as the index is written.
    std::vector<u64> data_offsets;
    data_offsets.reserve(ordered.size());
    u64 cursor = payload_offset;
    for (usize i = 0; i < ordered.size(); ++i) {
        data_offsets.push_back(cursor);
        cursor += payload_bytes(i).size();
    }

    for (usize i = 0; i < ordered.size(); ++i) {
        const PackEntry* entry = ordered[i];
        write_u32(out, static_cast<u32>(entry->key.size()));
        out.insert(out.end(), entry->key.begin(), entry->key.end());
        write_u32(out, static_cast<u32>(entry->scheme.size()));
        out.insert(out.end(), entry->scheme.begin(), entry->scheme.end());
        out.insert(out.end(), entry->guid.bytes().begin(), entry->guid.bytes().end());
        write_u64(out, data_offsets[i]);
        write_u64(out, static_cast<u64>(payload_bytes(i).size()));
        if (any_compressed) {
            out.push_back(static_cast<u8>(stored[i].codec));
            write_u64(out, stored[i].uncompressed_size);
        }
    }

    for (usize i = 0; i < ordered.size(); ++i) {
        const std::vector<u8>& bytes = payload_bytes(i);
        out.insert(out.end(), bytes.begin(), bytes.end());
    }

    return out;
}

Result<PackFile::Header> PackFile::parse_header(const u8* data, usize size, u64 container_size) {
    if (data == nullptr || size < kHeaderSize) {
        return Error("pack: buffer is smaller than the header");
    }

    Reader reader(data, size);
    u32 magic = 0;
    u32 version = 0;
    u32 entry_count = 0;
    u64 index_offset = 0;
    u64 payload_offset = 0;
    if (!reader.read_u32(magic) || !reader.read_u32(version) || !reader.read_u32(entry_count) ||
        !reader.read_u64(index_offset) || !reader.read_u64(payload_offset)) {
        return Error("pack: could not read the header");
    }

    if (magic != kMagic) {
        return Error("pack: bad magic (not a fork pack)");
    }
    if (version > static_cast<u32>(kFormatVersion)) {
        return Error("pack: format version " + std::to_string(version) + " is newer than supported version " +
                     std::to_string(kFormatVersion));
    }
    // Version 0 is not a format we ever wrote; treat anything below 1 as corrupt
    // rather than guessing a layout.
    if (version < static_cast<u32>(kFormatVersionRaw)) {
        return Error("pack: format version " + std::to_string(version) + " is not a known pack version");
    }
    // The header's offsets are validated against the true container size, which
    // the caller supplies when the buffer is only a prefix of the file. A
    // whole-file caller passes zero, and the buffer size is then the truth.
    const u64 total_size = container_size != 0 ? container_size : static_cast<u64>(size);
    if (index_offset < kHeaderSize || index_offset > total_size || payload_offset > total_size ||
        payload_offset < index_offset) {
        return Error("pack: header offsets are out of range");
    }

    Header header;
    header.format_version = static_cast<i32>(version);
    header.entry_count = entry_count;
    header.index_offset = index_offset;
    header.payload_offset = payload_offset;
    return header;
}

Result<PackFile> PackFile::parse_index(const u8* data, usize size, u64 container_size) {
    // The header carries the version and the index span; validating it here
    // means a malformed header is rejected before the index is walked, and it
    // keeps this entry point identical to the whole-file parse.
    auto header = parse_header(data, size, container_size);
    if (!header) {
        return header.error();
    }
    const bool has_codec = header.value().format_version >= kFormatVersion;
    const u64 index_offset = header.value().index_offset;
    const u64 payload_offset = header.value().payload_offset;
    const u32 entry_count = header.value().entry_count;

    // A buffer that holds the header and the index but not the payload region
    // still has to validate each entry's span against the *true* container size,
    // which the caller passes when the buffer is a prefix. Zero means the buffer
    // is the whole container.
    const u64 total_size = container_size != 0 ? container_size : static_cast<u64>(size);
    if (payload_offset > total_size) {
        return Error("pack: payload region is out of range");
    }

    PackFile pack;
    pack.entries_.reserve(entry_count);

    Reader reader(data, size);
    if (!reader.seek(static_cast<usize>(index_offset))) {
        return Error("pack: index offset is out of range");
    }

    for (u32 i = 0; i < entry_count; ++i) {
        u32 key_length = 0;
        if (!reader.read_u32(key_length)) {
            return Error("pack: truncated index (key length)");
        }
        std::string key(key_length, '\0');
        if (!reader.read_bytes(key.data(), key_length)) {
            return Error("pack: truncated index (key)");
        }

        u32 scheme_length = 0;
        if (!reader.read_u32(scheme_length)) {
            return Error("pack: truncated index (scheme length)");
        }
        std::string scheme(scheme_length, '\0');
        if (!reader.read_bytes(scheme.data(), scheme_length)) {
            return Error("pack: truncated index (scheme)");
        }

        PackEntry entry;
        entry.key = std::move(key);
        entry.scheme = std::move(scheme);
        std::array<u8, 16> guid_bytes{};
        if (!reader.read_bytes(guid_bytes.data(), guid_bytes.size())) {
            return Error("pack: truncated index (guid)");
        }
        // A nil guid is all-zero bytes, which is Uuid's default value, so this
        // round-trips an unset identity unchanged.
        entry.guid = Uuid::from_bytes(guid_bytes);

        u64 offset = 0;
        u64 length = 0;
        if (!reader.read_u64(offset) || !reader.read_u64(length)) {
            return Error("pack: truncated index (payload location)");
        }

        // Version 2 carries the codec and the payload's original length after the
        // on-disk location. A version-1 entry is stored raw, so its original
        // length is its on-disk length.
        PackCodec codec = PackCodec::None;
        u64 uncompressed_size = length;
        if (has_codec) {
            u8 raw_codec = 0;
            if (!reader.read_u8(raw_codec) || !reader.read_u64(uncompressed_size)) {
                return Error("pack: truncated index (codec)");
            }
            if (raw_codec != static_cast<u8>(PackCodec::None) && raw_codec != static_cast<u8>(PackCodec::Deflate)) {
                return Error("pack: entry '" + entry.key + "' names an unknown codec " + std::to_string(raw_codec));
            }
            codec = static_cast<PackCodec>(raw_codec);
        }

        // The payload span must lie within the container and within the payload
        // region, so a corrupt index cannot make the reader hand out bytes from
        // the header or the index itself. It is validated against the true
        // container size, not the (possibly index-only) buffer, so a streaming
        // reader that has not read the payloads still rejects a bad span.
        if (offset < payload_offset || length > total_size || offset > total_size || length > total_size - offset) {
            return Error("pack: entry '" + entry.key + "' has an out-of-range payload");
        }

        entry.codec = codec;
        entry.uncompressed_size = uncompressed_size;
        entry.payload_offset = offset;
        entry.payload_length = length;
        pack.entries_.push_back(std::move(entry));
    }

    return pack;
}

Result<std::vector<u8>> PackFile::decode_payload(const u8* payload, usize payload_size, const PackEntry& entry) {
    if (entry.codec == PackCodec::None) {
        return std::vector<u8>(payload, payload + payload_size);
    }
    // Decompress transparently, so a caller reads the bytes the same way for a
    // stored and a compressed entry. The reader rejects a stream that does not
    // decode to the length the index promised.
    auto decoded = decompress(payload, payload_size, static_cast<usize>(entry.uncompressed_size));
    if (!decoded) {
        return Error("pack: entry '" + entry.key + "': " + decoded.error().message());
    }
    return decoded.value();
}

Result<std::vector<u8>> PackFile::read_payload(const u8* data, usize size, const PackEntry& entry) const {
    if (data == nullptr || entry.payload_length > size || entry.payload_offset > size ||
        entry.payload_length > size - static_cast<usize>(entry.payload_offset)) {
        return Error("pack: entry '" + entry.key + "' has an out-of-range payload");
    }
    return decode_payload(data + entry.payload_offset, static_cast<usize>(entry.payload_length), entry);
}

Result<std::vector<u8>> PackFile::read_payload(const PackEntry& entry) const {
    // A pack bound to its container reads the entry's span from that file and
    // decodes it, so a whole-file parse that gave its bytes back is served on
    // demand. A pack with no container bound answers from the resident copy, so
    // this overload is the whole-file read too.
    if (container_path_.empty()) {
        return entry.data;
    }
    auto bytes = fs::read_binary_file_range(container_path_, entry.payload_offset, entry.payload_length);
    if (!bytes) {
        return bytes.error();
    }
    return decode_payload(bytes.value().data(), bytes.value().size(), entry);
}

Result<PackFile> PackFile::parse(const u8* data, usize size) {
    // Parse the index, then materialise every payload. A caller that wants to
    // avoid reading the payload region uses parse_index() and read_payload()
    // itself; this is the whole-file read the format always offered.
    auto indexed = parse_index(data, size);
    if (!indexed) {
        return indexed.error();
    }
    PackFile pack = std::move(indexed.value());
    for (PackEntry& entry : pack.entries_) {
        auto payload = pack.read_payload(data, size, entry);
        if (!payload) {
            return payload.error();
        }
        entry.data = std::move(payload.value());
    }
    pack.payloads_resident_ = true;
    return pack;
}

void PackFile::release_payloads() noexcept {
    // Give each entry's bytes back while keeping the span and codec it was read
    // from, so read_payload() with the container bytes still serves it. The span
    // and codec are what a streaming read needs; the resident copy is the only
    // thing dropped.
    for (PackEntry& entry : entries_) {
        entry.data.clear();
        entry.data.shrink_to_fit();
    }
    payloads_resident_ = false;
}

bool PackFile::payloads_resident() const noexcept { return payloads_resident_; }

Result<PackFile> PackFile::parse_file(std::string_view path) {
    // Read the whole container, then parse it exactly as parse() does. This is
    // the path-based entry point a caller reaches for when it wants a
    // self-contained pack value; bind_container() can then re-point it at the
    // same file for on-demand reads.
    auto bytes = fs::read_binary_file(path);
    if (!bytes) {
        return bytes.error();
    }
    return parse(bytes.value().data(), bytes.value().size());
}

Result<void> PackFile::bind_container(std::string_view path) {
    // Read the container's header and index back and check they describe this
    // same pack before anything is dropped. A path that is not this pack's
    // container (a different pack, a truncated file, a wrong-version file) is
    // rejected, and the resident payloads are left intact, so a bad bind is a
    // no-op the caller can ignore rather than a pack that silently reads the
    // wrong bytes.
    auto header_bytes = fs::read_binary_file_range(path, 0, kHeaderSize);
    if (!header_bytes) {
        return header_bytes.error();
    }
    const u64 container_size = fs::file_size(path);
    auto header = parse_header(header_bytes.value().data(), header_bytes.value().size(), container_size);
    if (!header) {
        return header.error();
    }
    if (header.value().entry_count != static_cast<u32>(entries_.size())) {
        return Error("pack: container '" + std::string(path) + "' has a different entry count");
    }
    const u64 index_span = header.value().payload_offset;
    if (index_span < kHeaderSize) {
        return Error("pack: payload region starts inside the header");
    }
    auto index_bytes = fs::read_binary_file_range(path, 0, index_span);
    if (!index_bytes) {
        return index_bytes.error();
    }
    auto indexed = parse_index(index_bytes.value().data(), index_bytes.value().size(), container_size);
    if (!indexed) {
        return indexed.error();
    }

    // The parsed index must describe the same entries: same keys in the same
    // order, and the same on-disk span, codec and original length for each. A
    // mismatch means the path is not the container this pack came from.
    const std::vector<PackEntry>& other = indexed.value().entries();
    if (other.size() != entries_.size()) {
        return Error("pack: container '" + std::string(path) + "' has a different entry count");
    }
    for (usize i = 0; i < entries_.size(); ++i) {
        const PackEntry& mine = entries_[i];
        const PackEntry& theirs = other[i];
        if (mine.key != theirs.key || mine.payload_offset != theirs.payload_offset ||
            mine.payload_length != theirs.payload_length || mine.codec != theirs.codec ||
            mine.uncompressed_size != theirs.uncompressed_size) {
            return Error("pack: container '" + std::string(path) + "' does not match entry '" + mine.key + "'");
        }
    }

    // The container is this pack's, so it can be served on demand: record the
    // path and drop the resident payloads, keeping each span and codec.
    container_path_ = std::string(path);
    release_payloads();
    return {};
}

} // namespace fe::pack
