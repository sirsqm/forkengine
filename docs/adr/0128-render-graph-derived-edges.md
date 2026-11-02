# ADR-0128: Render-graph edges derived from resource access

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0100](0100-render-graph-seam.md),
  [ADR-0110](0110-render-graph-pass-culling.md),
  [ADR-0127](0127-renderer3d-graph-passes.md),
  [ADR-0017](0017-performance-strategy.md),
  [ADR-0052](0052-render-extraction-seam.md)

## Context

[ADR-0100](0100-render-graph-seam.md) made pass order data and
[ADR-0110](0110-render-graph-pass-culling.md) gave a pass resource declarations
so the graph can drop a pass whose output nobody reads. Both left one thing
implicit: the *ordering* between two passes that touch the same resource.

Today a pass declares its order with `add_dependency` by hand. The resource
declarations the renderers already carry say more than that. `Renderer2D` writes
`world`'s `backbuffer`, has `ui` read it, and *also* declares `ui` after `world`;
`Renderer3D` writes `shadow`'s `shadow_map`, has `main` read it, and declares the
same edge by hand. In both cases the dependency the caller wrote down is already
implied by the access: a pass that writes a resource must run before a pass that
reads it, or the reader samples a frame that has not been drawn.

Writing that edge by hand is a place to be wrong. A caller who adds a pass that
reads an existing target, and forgets the matching `add_dependency`, gets a graph
that orders the pair by declaration order and happens to be right -- until it is
not. The graph holds the information needed to derive the edge; leaving it to the
caller is the same implicit-order problem [ADR-0100](0100-render-graph-seam.md)
set out to remove, one level down.

Constraints that shaped the decision:

- **Declared order is not overridden.** A caller may have a reason to run two
  passes in an order the raw access does not imply (a debug read of last frame's
  output, a deliberate double buffer). A derived edge must never silently reverse
  a statement the caller made.
- **The graph stays acyclic.** Two passes that write one resource and read it in
  a way that derives both directions would otherwise report a cycle that is an
  artifact of the derivation, not of the frame.
- **The graph stays a pure value** ([ADR-0042](0042-null-rhi-device.md)). The
  derivation is a function of the declarations, so it is testable headlessly.
- **Determinism.** The derived order must be the same every frame, or a benchmark
  and a visual test go flaky. "First declared writer" is the tie-break, not any
  hash or address order.

## Decision

`order()` **derives an ordering edge from resource access** as well as reading the
declared dependencies.

- For each resource, the **first live pass that writes it, in declaration order**,
  is its producer. Every other live pass that reads or writes that name runs after
  the producer. This covers write-then-read (a shadow map, a backbuffer) and
  write-then-write (two passes that compose into one target).
- A pass that both reads and writes one name produces **no self-edge**; it is its
  own producer and its own consumer, so the pair is already satisfied and a
  caller's declared edge for it is kept.
- A derived edge between two passes the caller has already ordered by a declared
  dependency is **dropped**: the declaration already states how they run, so the
  inference adds nothing and the caller's order governs. This is what lets a
  caller state an order the raw access would reverse -- a debug read of last
  frame's output, a deliberate double buffer -- without the graph reporting a
  false cycle. A cycle made only of derived edges is a real data cycle and is
  still reported, because the frame is genuinely unordered.
- Declared dependencies are unchanged. A dependency on a culled pass is still
  dropped, and the culling rule ([ADR-0110](0110-render-graph-pass-culling.md)) is
  untouched: this changes *order*, not liveness.

The derivation is a private step of `order()`, so `run()`, `plan()`, and
`allocate()` all see the derived order without changes -- a target's lifetime,
which the alias plan computes from the run order, now reflects the real
producer/consumer order even when the caller declared no edge.

## Consequences

**Easier:**

- A pass that reads a target an earlier pass writes is ordered correctly without
  an `add_dependency`. The edge cannot be forgotten, because it is a property of
  the declarations the renderer already makes for culling.
- The renderers' hand-written edges become redundant, not wrong. They can be
  removed as a follow-on; until then they agree with the derived edge.
- The derivation is a pure function of the declarations, so it is asserted
  headlessly: a derived order, an unrelated pass keeping its place, two writers
  ordered by declaration, a declared order that wins over a derived reversal, and
  a read-modify-write pass that is not ordered against itself.

**Harder / not yet done:**

- The graph still emits no GPU barriers or transitions; it orders *passes*, and
  the backend still handles resource state. A real barrier model needs the
  resource *object* and its before/after state, which is a larger change than a
  name.
- A resource written by two passes and read by a third is ordered
  producer, producer, reader by declaration, not by any finer analysis. That is
  enough for the frames the engine builds; a compositing case that needs the
  readers interleaved would need an explicit edge.
- Derived edges make some cycles reachable that hand-written edges did not, so a
  caller who writes two passes that each read what the other writes now gets a
  cycle report. That is the correct outcome -- the frame is unordered -- but it is
  a new failure a caller can hit.

## Alternatives considered

- **Keep edges hand-written.** The status quo. Rejected: it is the implicit-order
  problem [ADR-0100](0100-render-graph-seam.md) removed for pass *position*,
  reappearing for pass *dependency*, and the data to fix it is already there.
- **Order by resource access only, ignoring declared dependencies.** Simpler, but
  it removes the caller's ability to state an order the access does not imply, and
  it would make the existing renderers' edges meaningless. Declared edges are kept
  and derived edges defer to them.
- **Treat a derived conflict as an error.** Rejected: a caller who declares an
  order that the raw access reverses is expressing a real intent (double
  buffering, a debug read), and failing on it would make the derivation a
  straitjacket rather than a safety net.
- **Make the last writer, not the first, the producer.** Rejected: the first
  declared writer is the one the declaration order already treats as the
  producer, so using it keeps the derived order consistent with the tie-break the
  scheduler already uses, and keeps a caller's declaration order meaningful.
