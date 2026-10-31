# ADR-0126: Compressed pack payloads

- **Status:** Accepted
- **Date:** 2026-10
- **Related:** [ADR-0030](0030-asset-pack-format.md),
  [ADR-0018](0018-multi-platform-shipping.md),
  [ADR-0065](0065-pack-pruning.md), [ADR-0071](0071-web-export-template.md)

## Context

[ADR-0030](0030-asset-pack-format.md) defined the pack as a container the runtime
mounts like a directory, and its "not yet done" list named the first polish item:
**payload compression**. The format stored every entry's bytes raw, so a pack was
the sum of its assets and nothing smaller. The shipping queue's `pack-compression`
item is the acceptance test: the writer compresses an entry, the reader
decompresses transparently, a round trip proves the bytes survive, and an
uncompressed pack still loads byte for byte.

The forces are the ones the pack format already answers, plus the ones a codec
adds:

- **The format is read by every target.** A pack written on a desktop build is
  read by the native player, the web runtime, and a mobile app. The codec must
  be the same everywhere and must not need a system library the wasm or mobile
  build cannot supply.
- **Compatibility is a rule, not a hope.** [ADR-0030](0030-asset-pack-format.md)
  rejects a wrong magic or a future version; a pack built before compression
  existed must still load, and a pack built with no compression must still be the
  same bytes, so a build system that treats an unchanged pack as up to date keeps
  working.
- **Dependencies are pinned and permissive.** The engine already vendors
  single-file libraries (glad, stb) and fetches the rest; a compression library
  must fit that policy — permissive, non-transitive, and cross-compilable to
  wasm.
- **A pack is built once and read many times.** The encoder runs at build time;
  the decoder runs at load time on the player. A slow encoder is a build-time
  cost, a slow decoder is a run-time one, and both must stay pure so they are
  testable without a filesystem.

## Decision

**Add a per-entry codec to the pack format, backed by a vendored DEFLATE
implementation (miniz), and compress a payload only when that makes it smaller.**

### The codec is per entry, and `None` is the identity

`PackCodec` is `None` (raw) or `Deflate`. The index records the codec and the
payload's original length per entry, so a pack compresses what benefits (a text
scene, a mesh, a procedural texture) and leaves what does not (a PNG, a
compressed WAV) raw, and the reader serves both through one path. `None` is the
identity codec: the payload bytes are the file bytes.

### Format version 2, written only when used

The on-disk index gains `codec u8 + uncompressed_size u64` per entry, and the
header's `format_version` becomes **2** *only when at least one entry is
compressed*. A pack with no compressed entry is written as version 1 and is
**byte for byte** the format [ADR-0030](0030-asset-pack-format.md) defined, so
the compatibility guarantee is exact rather than approximate. `parse()` accepts
both versions: a version-1 entry is stored raw and its original length is its
on-disk length.

### Compress only when it shrinks the payload

The writer compresses each non-empty payload and keeps the compressed form only
when it is strictly smaller than the raw bytes. An incompressible payload is
stored raw with the `None` codec, so **enabling compression never makes a pack
larger than the uncompressed one** — the property that makes the flag safe to
turn on for a whole tree.

### The reader decompresses transparently, and strictly

`PackEntry::data` is always the *decompressed* bytes, whatever the codec: the
codec is an on-disk detail the reader hides, so `find_path` / `find_guid` and
every loader above them read a stored and a compressed entry the same way. The
reader sizes its output buffer from the index's `uncompressed_size` and **rejects
a stream that does not decode to exactly that length**, so a truncated, corrupt,
or wrong-length payload is an Error — the same strictness
[ADR-0030](0030-asset-pack-format.md) applies to offsets and lengths.

### miniz, vendored

The codec is **miniz** (v11.3.2, MIT), vendored under `third_party/miniz/` like
glad and stb: one `.c` and one `.h`, no build system, no network at configure
time, and compiles to wasm. `fe::pack::compress` / `decompress` wrap it and name
no miniz type in their signature (`PackCompression.h` includes no miniz header),
so the public API does not leak the dependency. The archive is on the engine's
final link line and installed with the rest of the SDK payload.

`fork_pack` and `fork_export` gain `--compress none|fast|best`; `fast` is
miniz's default level, `best` its highest.

## Consequences

**Easier:**

- A shipped pack is smaller. A tree of text, meshes, and procedural data shrinks
  by the ratio DEFLATE achieves on it (the acceptance run turned a 180 KB pack
  into 1.6 KB); a tree of already-compressed assets is unchanged, because
  incompressible entries are stored raw.
- The win reaches every target at once: compression happens at build time and the
  decoder is the same vendored code on desktop, web, and mobile.
- The compatibility rule holds exactly: an uncompressed pack is the same bytes as
  before, an old pack parses as version 1, and the codec is opt-in.
- The whole codec is a pure function over byte vectors, so the round trip, the
  empty payload, the wrong-length rejection, the incompressible-stays-raw rule,
  the guid lookup through a compressed pack, and a full mount-and-load are all
  asserted without a filesystem.

**Harder / not yet done:**

- A pack is still read whole into memory; compression changes its size, not its
  streaming story. Streaming a large compressed entry is later work.
- Only DEFLATE is available. It is the one codec every target can decode with one
  vendored file; a stronger codec (LZMA, zstd) would be a larger dependency for a
  build-time-only win and is its own decision.
- The encoder is single-threaded. A large tree compresses serially; parallelising
  it is the performance pillar's work (ADR-0017) and does not change the format.

## Alternatives considered

- **Compress the whole payload region as one stream.** Rejected: it would make
  the index and the payload inseparable, so a single-entry read would decode the
  whole pack, and it would break the per-entry random access the mount relies on.
  Per-entry is the granularity the reader actually uses.
- **Always bump the format version to 2.** Rejected: it would change the bytes of
  an uncompressed pack, breaking the exact-compatibility guarantee and any build
  that treats an unchanged pack as up to date. The version follows the content.
- **Compress every entry unconditionally.** Rejected: an incompressible payload
  grows under DEFLATE (the stream has a header), so a pack of PNGs would get
  larger. "Only when smaller" is what makes the flag safe.
- **Link the system zlib.** Rejected: it is not present on every target (wasm,
  mobile), it is not pinned, and it makes the build depend on the host. The
  engine's policy is pinned, permissive, cross-compilable dependencies; a
  vendored single-file miniz fits and a system library does not.
- **zstd or LZMA.** Rejected for now: both are larger dependencies for a
  build-time-only win, and neither is a single file. DEFLATE is the codec every
  target can decode with one vendored file; a stronger one is a separate
  decision if a profile ever calls for it.
- **A best-effort reader that tolerates a wrong-length decode.** Rejected: a
  short or long buffer a loader then misreads is worse than a clear error, and
  the pack format's rule is to reject a malformed pack, not to guess.
