# ADR-0157: Positional audio in the shipped 3D slice

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0016](0016-audio-module.md),
  [ADR-0034](0034-audio-attenuation-buses.md),
  [ADR-0097](0097-audio-listener-from-camera.md),
  [ADR-0112](0112-audio-directivity-and-doppler.md),
  [ADR-0147](0147-second-vertical-slice-3d.md)

## Context

The audio module can attenuate a source by its distance to the listener
(ADR-0034), pan it by the listener's ear direction (ADR-0097), apply a
directivity cone and Doppler (ADR-0112), and route a voice to a named bus
(ADR-0034). None of that was exercised by a shipped game: the pickup cue in
`examples/collect3d` (Gem Rush) was a non-positional source on the master bus,
exactly as breakout's was, and the game never installed an
`AudioListenerSystem`, so the mixer's listener sat at the world origin.

The roadmap's M11 audio row names this gap:

> An audio mix that uses the 3D features: positional sources, directivity or
> Doppler, and a bus layout (M6) — *deferred:* the pickup cue is non-positional
> on the master bus, as in breakout.

A 3D game whose only sound is a flat, non-directional beep is not using the 3D
audio it already has, and "the game is worth shipping" (phase 4) is where a
slice is expected to exercise its subsystems end to end.

## Decision

The shipped 3D slice plays its pickup cue as a positional source on a group bus,
and the wiring that makes it audible from the camera is a tested function rather
than application-local code.

### The cue is positional and placed at the gem

`build_collect3d_scene` authors the pickup node's `AudioSource` with
`spatial = true`, `min_distance = 2`, `max_distance = 20`, and
`bus_name = "sfx"`. `CollectibleSystem::play_pickup_sound` now takes the collected
gem's world position and writes it onto the audio node's transform *before* it
calls `play()`, so a cue is attenuated by the distance from the listener to the
gem and panned to the side the gem was on. The node is a single reusable
one-shot, so moving it is how each pickup is heard from its own spot; a
non-positional cue ignores the position, so the placement is harmless when a
scene authors the source flat.

The falloff span (`2..20`) is chosen so a cue anywhere on the gem ring (radius
7) is still audible while the player can tell which side it came from: a cue at
the ring's near edge is at full gain, one across the arena is quiet but not
silent.

### The mix has a bus layout

The game's effects route to a `"sfx"` group bus (gain 1) created at startup. The
settings screen continues to drive the *master* bus, so the two are independent
layers: a future "mute effects" toggle does not have to touch the volume the
player set. This is the "bus layout" half of the roadmap row; it is a real
layout, not an implicit master-only mix.

### The listener follows the camera, and the wiring is one tested function

`install_collect3d_audio(schedule, audio)` is the single place the game wires its
audio: it creates the `"sfx"` bus and adds an `AudioListenerSystem` bound to the
audio system. `main.cpp` calls it, and it is compiled into `fork_engine_tests`
from the example's own sources, so a test exercises the wiring itself rather than
only its pieces.

The listener is added *before* the audio system (the caller adds the audio system
last), so the mix this frame uses the camera's position this frame rather than
one frame behind — the same ordering `fork_player` uses (ADR-0097). Without the
listener the positional cue would be attenuated against the world origin, which
is the failure this ADR exists to prevent.

## Consequences

- The shipped 3D game now sounds like a 3D game: a gem across the arena is
  quieter than one at the player's feet, and a gem to the left is heard on the
  left. The subsystem is exercised by the slice a player runs, not only by a unit
  test.
- The audio wiring is a function with a test, so a future edit that drops the
  listener, the bus, or the `spatial` flag fails at the source. A separate test
  loads the authored `collect3d.fescene` document (hand-edited, because a
  regeneration reshuffles node ids) and asserts the same audio fields, so the
  file path and the code path cannot drift.
- Directivity, Doppler, and the front/back head shadow remain opt-in and unused
  here: the cue is omnidirectional with no Doppler, and the roadmap row stays
  open for those rather than claiming them.

## Alternatives considered

- **Leave the cue non-positional and only add the listener.** Rejected: a
  non-positional source is never attenuated or panned, so the listener would
  change nothing audible and the roadmap row would still be open.
- **Wire the listener and bus inline in `main.cpp`.** Rejected: `main.cpp` is not
  compiled into the test binary, so the wiring would ship untested. Extracting it
  to the example's own sources (which the tests compile) makes the wiring itself
  provable, the same reason the systems live in `slice_systems.cpp`.
- **Route effects straight to the master.** Rejected: the roadmap row asks for a
  bus layout, and a named effects bus is what lets the volume setting and an
  effects mute be independent later.
