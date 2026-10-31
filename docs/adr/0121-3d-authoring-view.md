# ADR-0121: The 3D authoring view

- **Status:** Accepted
- **Date:** 2026-10
- **Related:** [ADR-0089](0089-editor-transform-editing.md),
  [ADR-0002](0002-hybrid-scene-ecs-architecture.md),
  [ADR-0018](0018-multi-platform-shipping.md)

## Context

The editor's viewport rendered through `Renderer2D` unconditionally, so a 3D
scene could be authored only by hand-editing a file: the 3D renderer, `Camera3D`,
and 3D physics all existed, but the editor could not show them.
[ADR-0089](0089-editor-transform-editing.md) recorded that gap explicitly —
"drag-to-move moves a node in the XY plane and a 3D authoring view is not yet
possible; that is a separate piece of work" — so this is that piece of work.

The 2D path already established the shape this has to follow. `Viewport` is a
headless type holding the parts of a 2D view that a screenshot would not make
obviously wrong — which node a click lands on, where a pan points, how "focus the
selection" frames a box — as pure functions of a camera and a scene, and
`ViewportPanel` is the thin toolkit projection of it. A 3D view has the same
class of bugs and needs the same testability, so it needs the same split.

## Decision

**Add a 3D path to the viewport, with the maths in a headless `Viewport3D` the
panel drives, and a single `ViewportMode` on the editor context selecting it.**

### The camera is an orbit rig, not a free camera

`Viewport3D::OrbitState` is a pivot, a yaw, a pitch, and a distance. This is the
authoring convention every 3D editor uses, and it is stored as those four values
rather than as a free-floating transform because that is what makes orbit, pan,
and dolly independent and reversible: a free camera that integrates an arbitrary
mouse delta drifts into an orientation no gesture can undo, and an author cannot
get back to axis-aligned without reopening the scene.

The rig drives a real `Camera3D` component on a node (`apply_to`), exactly as the
2D `Viewport` drives a `Camera2D`, so the renderer stays the only thing that
decides how a camera maps to the screen. `sync_from_camera` reads an existing
camera back into the rig, so opening a scene whose camera an author placed keeps
that view rather than snapping to a default.

### The mode is one source of truth

`EditorContext::ViewportMode` is `TwoD`, `ThreeD`, or `Auto`, and both the
toolbar and the panel read it. `Auto` selects 3D when the scene has a 3D camera
(`Renderer3D::find_active_camera`), so opening a 3D scene shows it without a
click. A second place deciding "is this 2D or 3D" is the bug this avoids.

### The 3D path owns a depth-buffered target

The 3D path renders into its own render target with a depth buffer; the 2D path
keeps the colour-only one, because a 2D scene has no depth to test and paying for
a depth attachment there would be waste. This is a per-mode property of the
target, not a global one.

### Gestures are the 2D set, adapted

Right-drag orbits, middle-drag pans, the wheel dollies, left-click picks,
left-drag moves the node in the camera-facing plane, `F` frames the selection and
`A` frames the scene, and the selection draws as a projected world-space box.
Every edit still routes through the command stack as the `SetTransform`
operation ([ADR-0089](0089-editor-transform-editing.md)) — the 3D view reuses
that editing model rather than adding a second one, which is why this record
changes no edit semantics.

### The maths is headless and tested

`Viewport3D` is a pure function of a camera and a scene: an orbit rig, ray/AABB
picking that picks the nearest node under the cursor, framing that fits a box to
the view, and world/screen projection for the selection box. Tests cover the
ray/AABB hit-test (a parallel miss, a ray pointing away, a start inside the box),
the screen ray through the view centre, the pick choosing the nearest of
overlapping boxes, the projection round-trip, the pitch and dolly clamps,
framing, the apply/sync round-trip, and the bounds walk — without a window, a
GPU, or a click.

## Consequences

- **A 3D game can be authored in the editor.** A 3D scene can be seen, orbited,
  picked, and placed, which is the precondition for authoring a 3D game that the
  roadmap named as the largest editor gap.
- **The 2D/3D decision is made once.** `ViewportMode` is read by both the toolbar
  and the panel, and `Auto` makes the common case (open a scene, see it) need no
  click.
- **The 2D path is untouched.** It keeps its colour-only target and its own
  gestures; the 3D path is a sibling, not a rewrite, so the 2D behaviour a game
  and its tests rely on cannot regress from this change.
- **A 3D view is testable without a GPU.** The parts most likely to be subtly
  wrong are in a headless type, so the tests run everywhere the unit suite does.
- **This record was written after the code landed.** The change shipped without
  one while [ADR-0089](0089-editor-transform-editing.md) pointed at "a separate
  piece of work" that had no record; this closes that reference and corrects the
  stale status in the roadmap. The decision text is drawn from the change itself,
  not invented afterwards.

## Alternatives considered

- **A free-flying camera.** Rejected: it drifts into orientations a gesture
  cannot undo, and an author needs to return to a known axis-aligned view. The
  orbit rig makes orbit, pan, and dolly independent and reversible.
- **A second editing model for 3D.** Rejected: every transform edit already goes
  through the command stack as a serializable, undoable operation
  ([ADR-0089](0089-editor-transform-editing.md)); the 3D view reuses it, so undo,
  redo, and replay keep working in 3D with no new code.
- **A single render target for both modes.** Rejected: a 2D scene has no depth to
  test, so a depth attachment there is waste; the target is per-mode.
- **Putting the rig maths in the panel.** Rejected for the same reason the 2D
  view split them: the pick, the orbit, and the framing are the parts a screenshot
  would not reveal as wrong, so they belong where a test can drive them.
