// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Error.h"
#include "fe/core/Types.h"

#include <vector>

namespace fe::pack {

// The codecs a pack entry's payload may use.
//
// The pack format's payload was raw (ADR-0030); this adds a per-entry codec so a
// shipped pack is smaller than the sum of its assets. The codec is recorded per
// entry rather than per pack, so a caller compresses what benefits (a texture, a
// mesh) and leaves what does not (an already-compressed PNG, a tiny scene) raw,
// and the reader still serves both through one path.
//
// `None` is the identity codec: the payload bytes are stored verbatim, so a pack
// written with no compression is byte for byte the pack ADR-0030 defined, and an
// old reader reads it unchanged. `Deflate` is zlib/DEFLATE, which the vendored
// miniz provides; the reader decompresses it transparently.
enum class PackCodec : u8 {
    None = 0,
    Deflate = 1,
};

// The compression level passed to the DEFLATE encoder. `Fast` is the default:
// a pack is built once and read many times, but a build should not stall on a
// large tree, and DEFLATE's higher levels buy a few percent for a large time
// cost. A caller that wants the smallest pack can ask for `Best`.
enum class PackCompression : u8 {
    None = 0, // store entries raw (the codec is still recorded as None)
    Fast = 1, // DEFLATE at miniz's default level
    Best = 9, // DEFLATE at the highest level
};

// Compresses `input` with DEFLATE. Returns the compressed bytes, which are a
// self-describing zlib stream the reader decodes with `decompress`. Pure: no
// filesystem, no globals, so it is exhaustively testable.
//
// Returns an Error when the buffer cannot be represented (an input larger than
// the codec's 4 GiB limit, or an allocation failure), never a partial result.
[[nodiscard]] Result<std::vector<u8>> compress(const std::vector<u8>& input, PackCompression level);

// Decompresses a DEFLATE stream `input` whose original length is `uncompressed_size`.
//
// The original length is stored in the pack index, so the reader sizes the
// output buffer before decoding and does not grow it. A stream that does not
// decode to exactly `uncompressed_size` bytes is rejected: a truncated, corrupt,
// or wrong-length payload is an Error, not a best-effort read, matching the
// pack parser's strictness (ADR-0030).
[[nodiscard]] Result<std::vector<u8>> decompress(const u8* input, usize input_size, usize uncompressed_size);

// The bytes a payload of `input_size` could need at most when compressed. The
// writer uses this to reserve, so a compress call does not reallocate mid-stream.
[[nodiscard]] usize compress_bound(usize input_size) noexcept;

} // namespace fe::pack
