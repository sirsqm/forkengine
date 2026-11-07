# ADR-0145: Keyboard and gamepad focus navigation for UI

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0084](0084-in-game-ui.md),
  [ADR-0086](0086-in-game-ui-layout-and-extraction.md),
  [ADR-0075](0075-touch-input-actions.md)

## Context

The UI path has a pointer: `UiInputSystem` turns a mouse or a touch into
`hovered`, `pressed`, and `clicked` on the element under the cursor. That covers
a desktop and a phone, and nothing else. A console has no pointer, and a player on
a couch drives a menu with a d-pad or a stick; a keyboard-only player expects the
arrow keys to walk a menu and Enter to pick. Neither works today: a button is a
`UiElement` with a fill and a rectangle, and there is no notion of a *selection*.

The gap is not cosmetic. A game that ships to a TV or a pad is unplayable at its
first menu, and the in-game UI work (ADR-0084, ADR-0086) deliberately stopped at
the pointer because a focus model needs a decision the pointer path did not: how
the engine picks the next element.

Constraints that shaped the decision:

- **No extra authoring.** A menu already has a tree; layout already knows where
  every element is. A focus order should come from that geometry, not from a
  second list an author keeps in sync by hand.
- **The same seam as the pointer.** Both systems write the same `UiInteraction` on
  the same elements and call the one `ui::layout`, so a focus ring and a button's
  pixels cannot disagree -- the property ADR-0086 exists to protect.
- **A menu works with no action map.** A caller that declares no actions still
  needs a navigable menu; the arrow keys and Enter are the fallback.
- **Focus is per frame.** Like `hovered`, focus is a result of this frame's input,
  not authored state, so it is not serialized.

## Decision

Add `UiFocusSystem`, which reads a direction and a confirm from the frame's
actions (or the arrow keys and Enter/Space when no actions are bound) and writes
`focused` and `activated` onto the focused element's `UiInteraction`.

- `UiElement::focusable` (off by default) marks an element as a focus stop. A
  non-`interactable` element is never a stop, so a disabled button is skipped.
- Navigation is by geometry. A press of a direction picks the focus stop nearest
  the current one in that direction, scored so the cross-axis distance dominates
  and a press of "down" goes to the button directly below rather than a nearer one
  far to the side.
- A press is an edge and moves one stop; a *held* direction repeats on a fixed
  cadence, so an analog stick does not skip through a menu in a fraction of a
  second.
- Focus is applied to the first stop when nothing is focused or the focused
  element left the layout, so a menu that opens has a selection and a scene change
  does not leave focus dangling. A frame with no input at all clears the ring.

## Consequences

Easier: a menu is playable from a pad or a keyboard with no new authoring; the
focus order follows the layout an author already wrote; a menu system reads
`activated` exactly as it reads `clicked`, so a button works from every device
through one code path.

Harder: the system recomputes layout once more per frame (the input system and the
renderer already do; a caller that runs both in one frame pays for three layouts,
which the existing design already accepts as the price of the pixels and the hit
test agreeing). Focus is a single stop per scene rather than per menu, so a scene
with two menus must hide one (`visible = false`) or clear focus when switching, as
it already must for the pointer. The geometry rule is a nearest-neighbour choice,
which is right for a column or a grid and can surprise on a deliberately
irregular layout; such a layout would need an explicit order, which is not built.

## Alternatives considered

- **An authored focus order (a `next`/`prev` index per element).** Rejected: it is
  a second structure to keep in sync with the tree, and the tree already carries
  the geometry a player's expectation is based on.
- **Tree order rather than geometry.** Rejected: tree order does not match what a
  player sees when a menu is laid out with anchors, and a press of "down" would
  sometimes move up the screen.
- **A separate `focused`/`selected` component.** Rejected: focus is a per-frame
  input result exactly like `hovered`, and it belongs on the same `UiInteraction`
  so a button reads one struct.
- **Reading raw keys with no action seam.** Rejected: a pad, a remapped
  keyboard, and a touch region all have to reach the same navigation, which is
  what `ActionMap` is for; the arrow-key fallback is a convenience for the caller
  that declares none, not the model.
