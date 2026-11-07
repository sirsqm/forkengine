# ADR-0158: The listener's Doppler velocity is derived, not authored

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0034](0034-audio-attenuation-buses.md),
  [ADR-0097](0097-audio-listener-from-camera.md),
  [ADR-0112](0112-audio-directivity-and-doppler.md)

## Context

ADR-0112 added a Doppler shift with two velocities: the source's and the
listener's. It made the *source* velocity a finite difference of the node's
position, so a moving node shifts pitch with no authoring, and argued explicitly
against a stored velocity field for exactly that reason. The *listener* velocity
it left as "ordinary settable state" on `AudioSystem`
(`set_listener_velocity`), on the reasoning that "a game with a fast camera can
match the effect to its world".

Nothing ever set it. `AudioListenerSystem` — the one system that tracks the ear,
copying the active camera's world position onto the audio system every frame
(ADR-0097) — wrote the position and the orientation but not the velocity, and no
game or tool called the setter. So the listener velocity was permanently the zero
vector, and the Doppler effect's listener half was unreachable through the
engine's own systems: a fast-moving camera closing on a still siren heard no
shift at all, even though the mixer computes `(c - v_listener)` and would have.

This is the same shape of gap ADR-0097 fixed for the position: the state existed
and the mixer read it, but nothing in the engine ever moved it.

## Decision

`AudioListenerSystem` derives the ear's velocity from the camera's motion, the
same way `AudioSystem` derives a source's, and writes it to the audio system each
frame.

### The velocity is the position's finite difference

`set_velocity` keeps the previous frame's listener position and reports
`(position - previous_position) / delta_time`. The rules match the source path
exactly, so the two halves of the effect behave the same:

- The first frame the listener is tracked has no previous sample, so the velocity
  is zero and no shift is applied.
- A non-positive delta (a paused step) leaves the previous position in place and
  reports no velocity, so a pause is not read as a huge jump.
- When the scene has no camera — or a 3D camera with no owning node — the
  velocity is cleared and the baseline is dropped, so a sound is not shifted by
  motion that is no longer happening and a camera that reappears elsewhere does
  not report a teleport as velocity.

The velocity is written for both the 3D and the 2D camera paths: a 2D camera has
a position and moves, so it has a velocity even though it has no ear direction
for panning.

### It stays settable for a game that knows better

`AudioSystem::set_listener_velocity` is unchanged and still public. A game whose
camera is driven by physics, or whose unit scale is not the camera's world
units, can overwrite the derived value after the listener runs. The engine's
default is now correct without that, which is the point.

## Consequences

- A moving camera shifts the pitch of a spatial source with `doppler` set, with
  no authoring: the effect works end to end through the engine's own systems,
  where before only the source half did. A test mixes a still source against a
  moving listener and asserts the closing-speed factor, and the listener tests
  pin the derivation, the pause case, and the camera-lost case.
- The two velocities are derived the same way, so there is one rule to remember
  and one place a future change (a real velocity integration, say) would land.
- A camera that teleports registers one frame of a very large listener velocity,
  the same limitation the source path already has. A game that moves the camera
  by teleport can call the setter for that frame, which is why it stays public.

## Alternatives considered

- **Leave it to games to call `set_listener_velocity`.** Rejected: no game did,
  and the engine's own listener system is the natural owner of the ear's motion.
  A feature the engine ships but never wires is not shipped (the same reasoning
  as ADR-0097).
- **On by default at the system.** Rejected: the *shift* stays opt-in — a game
  sets `doppler` on its sources and `set_doppler_enabled(true)` — so deriving the
  velocity changes no existing mix. Only the state the effect reads is now
  populated; whether it is applied is still the game's choice.
- **A stored velocity field on the camera.** Rejected for the same reason
  ADR-0112 rejected it for the source: the camera node is what moves, so a stored
  velocity would be a second fact to keep in step with the transform. Differencing
  the position the listener already reads makes a moving camera shift pitch with
  no authoring.
