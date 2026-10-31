// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/pack/PackCompression.h"

#include <miniz.h>

#include <limits>

namespace fe::pack {

usize compress_bound(usize input_size) noexcept {
    // miniz works in 32-bit lengths (mz_ulong), so an input at or above 4 GiB
    // cannot be compressed by this codec. Report the bound as 0 for such an
    // input; compress() rejects it with a clear Error rather than truncating.
    if (input_size > std::numeric_limits<mz_ulong>::max()) {
        return 0;
    }
    return static_cast<usize>(mz_compressBound(static_cast<mz_ulong>(input_size)));
}

Result<std::vector<u8>> compress(const std::vector<u8>& input, PackCompression level) {
    const int mz_level = static_cast<int>(level);

    if (input.size() > std::numeric_limits<mz_ulong>::max()) {
        return Error("pack: payload is too large to compress (over 4 GiB)");
    }

    // miniz works in 32-bit lengths. The bound for a zero-length input is still
    // non-zero (the DEFLATE stream's own header), so the output buffer is always
    // large enough; the compressed length is what is returned.
    const mz_ulong bound = mz_compressBound(static_cast<mz_ulong>(input.size()));
    std::vector<u8> out(static_cast<usize>(bound));
    mz_ulong out_len = bound;
    const unsigned char* source = input.empty() ? nullptr : input.data();
    const int status = mz_compress2(out.data(), &out_len, source, static_cast<mz_ulong>(input.size()), mz_level);
    if (status != MZ_OK) {
        return Error("pack: DEFLATE compression failed (miniz status " + std::to_string(status) + ")");
    }
    out.resize(static_cast<usize>(out_len));
    return out;
}

Result<std::vector<u8>> decompress(const u8* input, usize input_size, usize uncompressed_size) {
    if (uncompressed_size == 0) {
        // A zero-length original is legal (an empty file): the stream decodes to
        // nothing and the result is an empty buffer.
        return std::vector<u8>{};
    }
    if (input == nullptr || input_size == 0) {
        return Error("pack: a compressed entry has no payload bytes");
    }
    if (uncompressed_size > std::numeric_limits<mz_ulong>::max()) {
        return Error("pack: entry is too large to decompress (over 4 GiB)");
    }

    std::vector<u8> out(uncompressed_size);
    mz_ulong out_len = static_cast<mz_ulong>(uncompressed_size);
    const int status = mz_uncompress(out.data(), &out_len, input, static_cast<mz_ulong>(input_size));
    if (status != MZ_OK) {
        return Error("pack: DEFLATE decompression failed (miniz status " + std::to_string(status) + ")");
    }
    // The index promised `uncompressed_size` bytes; a stream that decodes to a
    // different length is corrupt, and handing back a short or long buffer would
    // let a bad pack masquerade as a good one.
    if (out_len != static_cast<mz_ulong>(uncompressed_size)) {
        return Error("pack: decompressed size " + std::to_string(out_len) + " does not match the index (" +
                     std::to_string(uncompressed_size) + ")");
    }
    return out;
}

} // namespace fe::pack
