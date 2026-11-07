# ADR-0131: A pack is opened by its index, and a payload is read on demand

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0003](0003-error-handling.md), [ADR-0004](0004-resource-ownership-weak-cache.md),
  [ADR-0018](0018-multi-platform-shipping.md), [ADR-0030](0030-asset-pack-format.md),
  [ADR-0126](0126-pack-compression.md)

## Context

[ADR-0030](0030-asset-pack-format.md) made a pack the shipping form of an asset
tree and a `PackSource` the runtime mount, and it named streaming as later work:
"the whole pack is read into memory". [ADR-0126](0126-pack-compression.md) then
added a per-entry codec so a payload could be smaller on disk, but it kept the
whole-file read: `PackFile::parse` parses the index *and* materialises every
payload, and `PackSource::open` called it on the file's full bytes.

The consequence is that mounting a pack costs memory proportional to what the
game *ships*, not to what it *uses*. A large audio bank or a big texture set
inflates the resident set even when most of it is never played, and a payload
that is never touched is still read, decompressed, and held.

The pieces a streaming open needs already exist:

- The format separates an **index** (key, scheme, guid, on-disk span, codec,
  original length) from the **payload region**, and every offset is validated
  against the file on parse ([ADR-0030](0030-asset-pack-format.md)).
- A payload's on-disk span and codec are all that is needed to materialise it
  ([ADR-0126](0126-pack-compression.md)).
- `fe::fs` is the one place filesystem access lives, so a range read is one
  primitive beside `read_binary_file`.

## Decision

**Split a pack read into an index open and an on-demand payload read. `PackSource`
opens a pack by reading only its header and index, and materialises a single
entry's payload from its own on-disk span the first time that entry is read.**

### What changes

- **`fe::fs::read_binary_file_range(path, offset, size)`** and
  **`fe::fs::file_size(path)`** are added. A range past the end is an `Error`, not
  a short read: a caller asking for a fixed span (a header, an index) must never
  receive fewer bytes and treat them as complete.
- **`PackFile::parse_header`** reads the fixed header alone and reports where the
  index ends (`payload_offset`). It validates the header's offsets against the
  *true* container size, which the caller supplies when the buffer is only a
  prefix of the file, so an index-only read still rejects a malformed header.
- **`PackFile::parse_index`** gains an optional `container_size` used only to
  validate each entry's payload span, so a buffer holding the header and index
  but not the payload region is validated against the real file size. Passing
  zero keeps the whole-file behaviour.
- **`PackFile::decode_payload`** is the half of `read_payload` that does not need
  the whole container: given bytes already located at an entry's span, it decodes
  them (decompressing when the entry names a codec). `read_payload` is now a thin
  span-check over it, so `parse` and a streaming read share one decode path.
- **`PackSource::open`** reads the header, then the header and index together (one
  range read, since they are contiguous from the start of the file), and parses
  the index against the true container size. It holds the container path and
  reads a payload with `read_binary_file_range` on first use, decoding it through
  `decode_payload`. A single memo slot holds the last payload materialised, so a
  load that reads the same entry twice pays for it once; the source is read from
  one thread, exactly as its existing resolved-location slot already assumed.

### What does not change

- **The format is untouched.** No version bump: the header, index, and payload
  layout are what ADR-0030 and ADR-0126 defined. A streaming reader and a
  whole-file reader accept the same bytes.
- **The whole-file path stays.** `PackFile::parse` is index-then-every-payload as
  before, and a `PackSource` constructed from a parsed pack still answers from
  the entry's in-memory bytes. The HTTP fetch source, which already has the pack
  in memory, keeps using it unchanged.
- **The mount contract is identical.** `read_binary`/`read_text`/`exists`/
  `resolve_asset` behave the same; only where the bytes come from changes.

### Failure timing

A whole-file parse fails at open on a payload that will not decode. A streaming
open succeeds and fails only when the bad entry is read. That is the intended
trade: one corrupt payload is not a mount-time failure for every asset in the
pack, and the failure is still clean (an `Error`, never a short or wrong buffer).

## Consequences

**Easier:**

- Mounting a large pack costs the index, not the payload region. The resident set
  follows what the game loads, and an asset that is never played is never read.
- A big, rarely-used asset set (an audio bank) can ship in the same pack as the
  hot assets without inflating every run.

**Harder / accepted costs:**

- **Two open paths.** A pack can be wrapped whole (bytes in memory) or opened
  lazily (a container path). The source distinguishes them by an empty container
  path; the whole-file wrap still answers from the entry's own bytes.
- **A payload is read and decoded per first use.** A source that reloads every
  asset each frame would do more I/O than the whole-file read; the memo slot and
  the resource cache's weak-reference lifetime make that the load path's concern,
  not the source's. A general LRU over payloads is a measured optimisation if a
  profile ever shows it hot.
- **A corrupt payload fails later.** See failure timing above.

## Alternatives considered

- **Keep the whole-file read and add a separate streaming API.** Rejected: two
  readers of one format drift, and the whole-file read is exactly the code path
  that wastes the memory. One format, one decode path, two ways to reach it.
- **Memory-map the pack.** Rejected for now: a browser and a sandboxed mobile app
  do not offer `mmap`, which is the same reason the pack exists
  ([ADR-0018](0018-multi-platform-shipping.md)). A range read is portable and is
  what a mapped read would do anyway.
- **Cache every payload materialised, not just the last.** Rejected: the resource
  cache already holds what is live, and a payload cache in the source would pin
  bytes the manager has already released. One memo slot absorbs the re-read that
  actually happens (a reference resolved twice) without duplicating the manager's
  lifetime rule.
- **Read the index in a second open (header, then seek to index).** Rejected: the
  header and the index are contiguous from the start of the file, so one range
  read covers both; a second open is a wasted syscall for no benefit.
