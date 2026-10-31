# ADR-0120: The gamepad input seam

- **Status:** Accepted
- **Date:** 2026-10
- **Related:** [ADR-0041](0041-platform-contract-and-null-backend.md),
  [ADR-0075](0075-gameplay-actions-input-seam.md),
  [ADR-0018](0018-multi-platform-shipping.md)

## Context

[ADR-0041](0041-platform-contract-and-null-backend.md) records the platform
contract concern by concern and names gamepad as a deferred row of the input
group: "gamepad: — ... gamepad deferred". Touch landed later
([ADR-0075](0075-gameplay-actions-input-seam.md)) and is fed by the web backend,
so the input group is one device short of the contract it declares. The roadmap
carries the same row open ("Gamepad and suspend/resume remain"), and
[ADR-0075](0075-gameplay-actions-input-seam.md) anticipates this work directly:
"Adding a gamepad is a new binding kind and a row in the backend, not an edit to
every system."

This record is that row. It adds gamepad to `InputState` and `ActionMap`, and
feeds it from the desktop backend. It does not add suspend/resume, which is a
lifecycle concern with no device behind it and stays open.

Three properties of a gamepad shape the design, and none of them match the
keyboard or the touch path exactly.

**A gamepad is polled, not event-driven.** Every desktop API exposes it as a
snapshot: `glfwGetGamepadState` returns the current state of every button and
axis, and the caller reads it once per frame. Keyboard and mouse arrive as
callbacks; touch arrives as DOM events. A gamepad has no per-change event to
translate, so the backend reads the whole pad each frame and the engine derives
the press/release edges from the change between frames. That is a different feed
shape from every existing device, and pretending otherwise would mean the
backend fabricating events.

**A pad connects and disconnects at runtime.** A keyboard is present for the
life of the window; a gamepad is plugged in and pulled out while the game runs.
State has to follow the device: a pulled cable must not leave a button stuck
down, and a pad plugged back in must not inherit the buttons that were held when
it left.

**A stick is analog.** A key is down or up; a stick is a position in two
directions, and a trigger is a travel from 0 to 1. Gameplay that wants "move
left" from a stick wants a *direction past a threshold*, not a raw float, but
gameplay that wants "steer by how far the stick is pushed" wants the float. Both
have to be reachable.

## Decision

**Add gamepad to `InputState` as a polled, slot-keyed device, and add button and
axis bindings to `ActionMap` so a pad drives the same actions a key does.**

### The device model in `InputState`

- `GamepadButton` and `GamepadAxis` are positional enums: the four face buttons
  by position (`South`/`East`/`West`/`North`, not A/B/X/Y), the shoulders and
  their sticks, the d-pad, the menu buttons, and six axes (two sticks, two
  triggers). Positional naming is deliberate: the same physical button is "A" on
  an Xbox pad, "Cross" on a PlayStation pad, and "B" on a Nintendo pad, so the
  backend maps its native layout onto the engine's positions and gameplay asks
  for the position.
- A pad is keyed by a small **slot** index (`0..kMaxGamepads-1`), not a hardware
  id, because gameplay wants "the first pad", not a USB serial.
  `first_connected_gamepad()` is that query.
- The feed API is `on_gamepad_connected(slot, bool)`,
  `on_gamepad_button(slot, button, down)`, and `on_gamepad_axis(slot, axis,
  value)`. The backend re-reports the whole pad each frame; `on_gamepad_button`
  derives the press/release edges against the last reported value, and
  `begin_frame()` clears them exactly as it clears the key edges. This is
  correct for both a polling backend (the usual case) and an event-driven one.
- Axis values are clamped at the feed boundary: `-1..1` for a stick, `0..1` for
  a trigger. A backend reporting a raw ADC value cannot hand gameplay an
  out-of-range number.
- `on_gamepad_connected(slot, false)` raises the release edge for anything held
  and clears the slot, so a disconnect is observable as a release rather than a
  stuck button. A fresh connect starts from a clean state, so a reconnect does
  not inherit the previous pad's held buttons.
- `release_all()` (focus loss) clears held buttons and axes but keeps the slot
  connected, because losing focus does not unplug the pad. It raises the release
  edges for held buttons so the release is seen.

### The action layer

`ActionMap` gains two binding kinds:

- `bind_gamepad_button(action, button)` — a pad button drives an action, exactly
  as a key does.
- `bind_gamepad_axis(action, axis, sign, threshold)` — one *direction* of a
  stick or trigger drives an action. The sign (`+1`/`-1`) selects the direction
  and the threshold is the travel past which the action counts as held, so a
  single axis can drive two actions (`MoveLeft` on `-X` past 0.5, `MoveRight` on
  `+X` past 0.5) without either firing on a centred stick.

`update()` reads one slot: the map's configured slot, or the first connected pad
when unset. The slot is resolved at update time rather than bind time, so a pad
plugged in after the bindings were declared still drives them. With no pad
connected every gamepad query answers false/zero and the actions fall back to
the keyboard, mouse, and touch bindings unchanged. A map-wide deadzone
(`set_gamepad_deadzone`) suppresses rest noise; a binding's own threshold is
applied on top as the larger of the two.

The deadzone and threshold are compared on magnitude, so a centred stick's noise
cannot hold an action and an axis pushed the wrong way never fires the action
bound to the other direction.

### The backend

`GlfwWindow::poll_events()` calls a new `poll_gamepads()` after
`glfwPollEvents()`. It reads every slot GLFW exposes, and for each one that
`glfwJoystickIsGamepad` maps to the standard layout it feeds the whole pad
through `glfwGetGamepadState`. A raw joystick GLFW cannot map to the standard
layout is left unconnected rather than reported as a wrong set of buttons,
because a vendor's button numbering cannot be translated without a per-device
database. A slot that was connected and is no longer present is reported
disconnected.

The native-to-engine translation is two explicit `switch` tables
(`translate_gamepad_button`, `translate_gamepad_axis`) rather than a cast, so a
change in either numbering is a mismatch the compiler can see rather than a
silently swapped button. GLFW's layout happens to be the identity against the
engine's ordering, which is exactly why the cast would be a trap.

The null backend is unchanged: it feeds nothing, so a headless run has no pad,
which is correct.

## Consequences

- **The input group of the platform contract is complete.** Keyboard, mouse,
  touch, and gamepad all attach to `InputState` and all reach gameplay through
  `ActionMap`, so one gameplay system serves all four devices. The only deferred
  input concern left is nothing — the remaining deferred contract row is
  suspend/resume, a lifecycle concern.
- **A gamepad is a binding, not a system edit.** A game that already reads
  actions gets pad support by declaring bindings; no system learns a pad exists.
- **The edge semantics differ from every other device and are documented at the
  feed site.** A future backend (a console, a browser Gamepad API) has to poll
  and re-report the whole pad, not fabricate change events. The alternative —
  an event-driven gamepad feed — was rejected because it would require the
  backend to hold a previous snapshot and diff it, duplicating work the engine
  already does and getting the first frame after a connect wrong.
- **Analog and digital readings are both reachable.** `gamepad_axis()` gives the
  raw clamped value for analog steering; a binding gives the digital action. A
  game can use either or both.
- **A pad that GLFW cannot map is invisible.** This is a deliberate limit: the
  alternative is a per-device mapping database, which is not worth carrying
  before a player reports a pad that does not work. The `glfwJoystickIsGamepad`
  check is the seam where such a database would attach.
- **The tests are pure logic.** `InputState`'s gamepad tests feed `on_gamepad_*`
  directly and `ActionMap`'s drive `update()` over a hand-fed `InputState`, so
  the state machine and the binding evaluation are covered with no window, no
  GPU, and no physical pad — the same reason the touch tests can exist at all.

## Alternatives considered

- **A separate `Gamepad` interface.** Rejected: the pad is one more device on
  the existing `InputState` seam, exactly as touch was, and a second interface
  would be a second lifetime to thread through the runtime for no gain.
- **Hardware ids as the key.** Rejected: gameplay wants "the first pad", and a
  hardware id makes single-player code discover an id it does not care about.
  Slots give single-player the trivial query and local multiplayer a stable
  per-player index.
- **Reporting change events from the backend.** Rejected: every desktop API is a
  poll, so the backend would diff snapshots itself — work the engine does
  already, done in a place that cannot be tested without a real pad.
- **Vendor button names (A/B/X/Y).** Rejected: the engine would inherit one
  vendor's glyphs and mislead gameplay on every other pad. Positional names are
  what the backend can map reliably.
