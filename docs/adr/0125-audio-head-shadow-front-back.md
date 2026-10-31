# ADR-0125: Audio head shadow for front/back cues

- **Status:** Accepted
- **Date:** 2026-10
- **Related:** [ADR-0034](0034-audio-attenuation-buses.md),
  [ADR-0097](0097-audio-listener-from-camera.md),
  [ADR-0099](0099-stereo-panning-from-listener-orientation.md),
  [ADR-0112](0112-audio-directivity-and-doppler.md),
  [ADR-0016](0016-audio-module.md)

## Context

[ADR-0112](0112-audio-directivity-and-doppler.md) closed the physical half of the
M6 audio refinement row: distance attenuation, buses, stereo panning, a
directivity cone, and Doppler. Its "not yet done" list left one item, and named
it precisely: a **head-related filter for front/back cues**. The M6 row calls it
out too ("A head-related filter for front/back cues remains"), and the shipping
queue's `audio-front-back` item is its acceptance test.

The gap is real and audible. Panning projects a source onto the listener's
*right* axis, so a source directly ahead and an identical one directly behind
both project to zero and both centre. A player cannot tell the two apart: the
sound is in the same place in the mix whether it comes from the direction they
are looking or from behind their head. Front/back is the other half of
localisation, and panning is structurally unable to supply it.

The physics that supplies it is the **head shadow**. A sound behind the head
reaches the ears by diffracting around it, which attenuates the high frequencies
more than the low ones; a sound in front reaches the ears unoccluded. The audible
cue is therefore a *timbre* change - a rear source is duller - not a level or a
pan change. That distinction decides the design: the cue must be a filter, not
another gain, and it must not double the distance attenuation by making a rear
source simply quieter.

The constraints are the ones ADR-0034 and ADR-0112 set, unchanged:

- **CI has no sound card.** The maths belongs in the pure mixer so it is
  testable through the null device; `AudioSystem` stays thin plumbing.
- **A scene authored before this sounds exactly as it did.** The cue is opt-in
  and defaults to off, so an existing mix is unchanged byte for byte.

## Decision

**Add a per-voice head-shadow low-pass to the pure mixer, keyed on the front/back
component of the listener-to-source direction, and let `AudioSystem` switch it on
once for a scene.**

### The cue: a low-pass mixed in by the rear component

`shadow_strength(voice, params)` returns how much head shadow a voice casts, in
`[0, 1]`: the component of the direction from the listener to the source along
the listener's **backward** axis (`-forward`). It is 0 for a source in front of
or level with the listener, 1 for one directly behind, and the cosine of the
angle between, so a source swung from the side toward the rear darkens
smoothly. It is 0 for a non-positional voice, for a listener with no facing, or
when the cue is disabled - each of which bypasses the filter.

The filter is a **one-pole low-pass**, `y[n] = y[n-1] + a * (x[n] - y[n-1])`,
with `a = 1 - exp(-2*pi*fc/fs)` (`head_shadow_coefficient`). The mixer resolves
`a` once per voice from `MixParams::head_shadow_cutoff_hz` (default 2000 Hz) and
applies it per ear, then mixes the filtered signal in by the shadow strength:

```
out = raw + strength * (filtered - raw)
```

so `strength = 0` is a **bit-for-bit bypass** (the raw sample, the filter state
untouched) and `strength = 1` is the fully filtered signal. The filter's own
state (`voice.shadow_state_left` / `shadow_state_right`) carries the tail across
calls, so a steady tone stays dark rather than re-attacking every frame. Because
the output is a *mix* of raw and filtered rather than only the filtered signal, a
source exactly behind keeps a real (if dull) signal and never goes silent: the
cue is a timbre change, not a level one, so it does not stack on top of distance
attenuation.

`AudioSystem` gains `set_front_back_enabled` (off by default) and
`set_head_shadow_cutoff`, and forwards both into `MixParams`. A game with a real
ear direction opts in once; a 2D scene has no facing, so it is unaffected whether
the flag is set or not.

## Consequences

**Easier:**

- A source behind the player is now distinguishable from one in front: the rear
  one is duller, the front one is unchanged. Front/back localisation is complete
  for the first-order model.
- The cue is a single number per voice plus one filter, and the whole of it is in
  the pure mixer, so `shadow_strength`, the coefficient curve, the front/rear
  difference, the side-is-unchanged guarantee, and the off-by-default bypass are
  all asserted through the null device.
- It is opt-in and additive: enabling it leaves a source level with the ear
  bit-for-bit unchanged, and a scene that never enables it mixes exactly as it
  did before the cue existed.

**Harder / not yet done:**

- It is still a first-order model, not a full HRTF. A true HRTF is a
  per-listener transfer function measured or modelled per direction; this is one
  low-pass keyed on elevation, which is the audible half of the front/back cue
  and the half a pure, headless mixer can prove. A full HRTF needs a filter bank
  and a measured dataset and remains its own record.
- Elevation above and below the listener is not distinguished: a source directly
  overhead has no front/back component and no shadow. That is the same limit the
  pan model has (an overhead source centres) and is the other axis a real HRTF
  would carry.
- The filter is a fixed cutoff scaled only by the shadow strength. A source with
  its own material or occlusion still has to fake it by lowering the system
  cutoff.

## Alternatives considered

- **A comb / notch filter keyed on the elevation angle**, as the queue item's
  "done when" text suggests. Rejected: a comb is the pinna's directional
  filtering, which needs a per-direction delay and is a filter bank, not one
  pole. A single one-pole low-pass is the head-shadow half of the same physics,
  is what a first-order model can honestly claim, and satisfies the acceptance
  test (front and behind differ measurably, the side is unchanged) without
  pretending to a measured transfer function.
- **Lowering a rear source's gain instead of filtering it.** Rejected: it would
  stack on distance attenuation and read as "the sound is farther away", not "the
  sound is behind me". The cue is a timbre change, so it must be a filter.
- **Fully replacing the sample with the filtered one at full shadow.** Rejected:
  a rear source would go silent, which is both wrong (a rear sound is audible)
  and a level change rather than a timbre change. Mixing the filtered signal in
  by the strength keeps a rear source audible and makes `strength = 0` an exact
  bypass.
- **Keying the cutoff on the shadow strength rather than mixing by it.**
  Rejected: sweeping a cutoff to and through zero to mean "no filter" is not
  continuous at the front (the coefficient jumps from 1 to the full-filter value
  the instant the source crosses level), and it re-attacks a filter with stale
  state. Mixing the filtered signal in by a strength that reaches exactly 0 at
  the front is continuous and leaves the bypass exact.
- **On by default.** Rejected for the same reason ADR-0034 and ADR-0112 rejected
  it: it would change every existing mix, and the compatibility rule is that a
  scene authored before a refinement sounds exactly as it did.
- **Putting the maths in `AudioSystem`.** Rejected: CI has no sound card, so
  anything in the system is untested on every build. The mixer is the single
  source of mixing truth, as ADR-0034 established.
