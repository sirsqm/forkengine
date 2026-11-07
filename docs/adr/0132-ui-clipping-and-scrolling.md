# 0132 — UI clipping: a scissor rect on the RHI, carried through extraction

Date: 2026-11

Status: Accepted

## Context

[ADR-0086](0086-in-game-ui-layout-and-extraction.md) gave the engine in-game UI:
a `UiCanvas`/`UiElement` tree laid out by one pure solver and drawn as quads
through the 2D extraction seam. Its solver is deliberately small, and it named
scrolling and clipping as *not done*: "There is no wrapping, no flex-grow, no
grid, and no scroll view; those are named as not-done". The shipping queue's
`ui-scroll-and-masks` is that gap.

Two concrete defects follow from the gap:

- A list longer than its panel has nowhere to go. Every row is drawn, so a
  twenty-row menu paints over whatever is below its frame.
- A child drawn past its parent's bounds is not cut off. A menu overflows its
  frame, and a partially scrolled row draws on top of its neighbours.

The seam already knows how to express a clip — the layout solver computes every
element's rectangle in viewport pixels, and the draw step already walks the
resolved array in order — but nothing carries a "this draw is confined to here"
instruction down to the device, and the RHI has no such instruction at all.

## Decision

**Clipping is a property of a UI container, resolved by the layout solver, carried
per instance through extraction, and applied as a scissor rectangle on the RHI.**

### The data model

- **`UiElement::clip`** (bool) makes an element a clipping container: its
  children are confined to its box. **`UiElement::scroll`** (`Vec2`, design
  pixels) offsets those children; a positive `y` moves them up, as scrolling a
  list down does. A scroll is ignored unless `clip` is set, because an unclipped
  scroll has nothing to reveal.
- **`ui::Rect::intersect` and `ui::Rect::overlaps`** are the geometry the solver
  needs. `intersect` returns an empty rectangle for a non-overlapping pair, so
  `empty()` is the "did it overlap" test and no separate flag is needed.
- **`UiResolved` gains `clip` and `has_clip`**: the viewport-space rectangle the
  element is drawn inside, and whether any ancestor set one. `has_clip` false
  means "the whole viewport", so the common unclipped case carries no scissor
  work.

### The layout walk

`ui::layout` carries a `ClipContext` (a design-space rectangle plus an `active`
flag) down the walk:

- A clipping container narrows the context to the **intersection** of the
  ancestor region and its own box. Intersection is what makes nesting work
  without a special case: a clipped child inside a clipped parent is visible only
  where both boxes overlap.
- A child whose rectangle does not overlap the context is **culled with its whole
  subtree** — not emitted, so it neither draws nor is hit tested. A child that
  partly overlaps is **kept whole**, with the context rectangle recorded, and the
  draw step scissors it.
- A clipping container's children are shifted by `-scroll` inside the content
  box. The container itself does not move: a scroll moves content inside a
  window, not the window.
- The clip does not apply to the container's *own* fill, texture, or text — only
  to what is inside it.

`hit_test` consults the recorded clip as well as the element rectangle, so a
partly-scrolled button is hittable only in the part the scissor leaves visible.
What a user can click is exactly what a user can see.

### The RHI seam

- **`rhi::ScissorRect`** is a new value type: `x`, `y`, `width`, `height`, and an
  `enabled` flag, in framebuffer pixels with a **top-left origin and +Y down** —
  the same space `ui::Rect` uses, so a UI clip travels to the device without a
  flip the caller has to remember. `enabled` false means "no scissor".
- **`rhi::Device::set_scissor(const ScissorRect&)`** is a new pure virtual,
  beside the other state calls. A backend converts the origin to its own
  convention internally (OpenGL's scissor is bottom-left), so nothing above the
  RHI has to know it. `begin_frame` resets the scissor to disabled, and the 2D
  renderer disables it after its batches, so a clipped pass never leaks state
  into the next.

### Carrying the clip to the draw

- **`render::SpriteInstance` gains `clip` and `has_clip`.** The clip is carried
  per instance, not as a pass parameter, because a clipped panel and an unclipped
  element share one draw order: the scissor changes *between* instances, and the
  batching walk has to see it to split there.
- **`Renderer2D::Batch` gains a `scissor`.** Two instances of the same texture
  but different clips are separate batches, because the scissor is device state
  set between them. A world batch leaves it disabled.
- `Renderer2D::submit` sets the scissor per batch and disables it once at the
  end, so a clipped batch can never leave the scissor on for the next pass.

## Consequences

**Easier:**

- A list panel scrolls: `clip = true` and a `scroll` value is all a menu needs to
  keep its rows inside its frame.
- The clip is a pure function of the scene, so it is testable with no GPU, like
  every other layout concern, and it is drawn, hit tested, and culled from one
  computed rectangle.
- A game's UI and the engine's own panels share the scissor primitive, which any
  future pass (a viewport scissor, a minimap) can reuse.

**Harder / accepted costs:**

- **A new RHI call.** Every backend must implement `set_scissor`. The null device
  makes it a no-op; the OpenGL device flips the origin against the current
  framebuffer's height and clamps a below-the-edge rectangle to an empty box.
- **A batch split per clip change.** Two elements under different clips no longer
  batch together even when they share a texture. This is the correct trade: the
  alternative is a wrong scissor on one of them.
- **Culling is conservative.** A partly-visible element keeps its whole box and
  is scissored at draw, so it still costs a quad; only a wholly-outside element
  saves the work. This is the same conservative rule the world culling uses.

## Alternatives considered

- **Clip in the shader (a uniform rectangle, discard outside it).** Rejected: it
  needs a new shader path and a per-instance uniform or vertex attribute, and it
  changes the fragment cost for every draw. The scissor is fixed-function state
  the driver applies for free.
- **Clip by computing an intersected rectangle per element and drawing only that
  part.** Rejected: it would move the quad's UVs and position per element, which
  means the extraction would have to re-derive geometry it currently copies, and
  text would have to clip per glyph. A scissor cuts the rendered result and
  leaves the geometry alone.
- **A clip stack as a pass parameter rather than a per-instance field.** Rejected:
  the batch walk needs to know where the clip changes to split, and a per-pass
  value cannot express two clips in one pass.
- **Clamp the scroll to the content extent in the solver.** Rejected: the solver
  would need the content size, which it computes only after placing the children,
  and a caller that wants rubber-band bounds can clamp against the size it
  computed. Keeping the solver a pure function of the numbers an author typed is
  worth more than saving the caller one clamp.
- **A scroll view as a distinct component.** Rejected: clipping and scrolling are
  two fields on the container that already exists, so a scroll view would be a
  second container type with the same layout rules plus two fields.
