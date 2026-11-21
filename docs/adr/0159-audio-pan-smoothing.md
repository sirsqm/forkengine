# ADR-0159: Audio pan smoothing across the listener

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0016](0016-audio-module.md),
  [ADR-0034](0034-audio-attenuation-buses.md),
  [ADR-0097](0097-audio-listener-from-camera.md),
  [ADR-0099](0099-stereo-panning-from-listener-orientation.md),
  [ADR-0112](0112-audio-directivity-and-doppler.md),
  [ADR-0125](0125-audio-head-shadow-front-back.md)

## Context

[ADR-0099](0099-stereo-panning-from-listener-orientation.md) gave the mixer a
constant-power stereo pan computed from the listener's facing. Its "Deferred"
section named one consequence precisely: *"Smoothing a pan that crosses the
listener in a single frame."* The shipping queue's `audio-pan-smoothing` item is
that follow-up.

The pan is a pure function of the current geometry, so a source that passes the
listener — a bullet, a car, a thrown object — moves from `-1` to `+1` between two
frames. The per-ear gains are `cos` and `sin` of the pan angle, so the step is a
discontinuity in both channels: the left gain drops from 1 to 0 and the right
rises from 0 to 1 in one sample. That is a click, and a click is not a subtle
artifact — it is the loudest thing in the mix for its duration.

The constraints are the ones ADR-0034 and ADR-0099 set, unchanged:

- **CI has no sound card.** The maths belongs in the pure mixer so it is
  testable through the null device; `AudioSystem` stays thin plumbing.
- **A scene authored before this sounds exactly as it did.** The refinement is
  opt-in and defaults to off, so an existing mix is unchanged byte for byte.

## Decision

**Add a per-call one-pole smoothing step to the recorded pan, with the time
constant owned by `MixParams`, and let `AudioSystem` set it for a scene.**

### The pan is rate-limited toward its target, not recomputed in place

`voice_level` already recorded the pan it applied on `voice.pan`. It now reads
that value as the filter's state and advances it one step toward the freshly
computed pan:

```
pan += (target - pan) * (1 - exp(-dt / tau))
```

`smoothed_pan(current, target, elapsed, tau)` is that step, factored out as a
total, pure function so the tests assert it directly as well as through the
mixer. `dt` is the mixer call's duration, `frames / output_rate`; the mixer is
called once per frame, so one call is one step. A one-pole approach is chosen
over a fixed maximum step because its behaviour is frame-rate independent: the
same time constant gives the same physical easing at 30, 60, or 144 Hz, while a
per-frame clamp in pan units would ease faster on a faster display.

The recorded value is clamped to `[-1, 1]`, the pan law's domain, and the step is
the exact fixed point when `current == target`, so a settled pan does not drift by
a rounding error every frame. A zero time constant, or a call with no frames,
returns the target unchanged — that is the disabled path, and it leaves the
pre-smoothing mix bit for bit identical.

### The time constant belongs to the mix

`MixParams::pan_smoothing_seconds` defaults to **0**, which disables smoothing.
That keeps every existing scene byte-for-byte identical, including the existing
tests that assert a hard pan after one call. A game opts in once through
`AudioSystem::set_pan_smoothing_seconds`; every positional voice then eases with
the same rule, because the constant describes how the mix should behave, not how
one voice should.

The streamed path (`add_streamed_voice`) resolves the same step from the same
`frames / output_sample_rate`, so a streamed voice and a clip voice pan
identically.

## Consequences

**Easier:**

- A source that crosses the listener sweeps across the stereo field instead of
  snapping, which removes the click. The cue is a single number per voice plus
  one exponential, and the whole of it is in the pure mixer, so the step, the
  bound, the no-overshoot property, the settled fixed point, and the
  off-by-default identity are all asserted through the null device.
- It is additive: with the default constant the recorded pan is exactly the
  value ADR-0099 computed, so no existing mix or test changes.

**Harder / not yet done:**

- A fast crossing now lags the true pan by roughly the time constant. At 50 ms
  the source reaches the opposite ear a few frames after it passes the listener;
  that is the intended trade (a short lag is inaudible, a click is not), but a
  game that wants an instant pan must leave the constant at zero.
- The step is per mixer call, so a frame with an unusually long delta moves a
  larger fraction. That is correct for a time-based filter, but it means the
  smoothing is only as steady as the frame pacing.
- Only the pan is smoothed. Distance attenuation, the directivity cone, and the
  head shadow are still recomputed each frame; a source teleported across the
  world can still step its level. Those are separate refinements with their own
  curves.
- It is not a physical model: a real moving source has a continuously changing
  direction, and the smoothing is a presentation filter over a sampled one.

## Alternatives considered

- **A fixed maximum step per frame (a rate limit in pan units).** Rejected: the
  easing then depends on the frame rate, so the same game clicks at 30 Hz and
  glides at 144 Hz, and the constant means something different on every machine.
- **Smoothing in `AudioSystem` rather than the mixer.** Rejected: CI has no sound
  card, so anything in the system is untested on every build. The mixer is the
  single source of mixing truth, as ADR-0034 established.
- **On by default.** Rejected for the same reason ADR-0034, ADR-0099, and
  ADR-0125 rejected it: it would change every existing mix and every existing
  sample-by-sample test, and the compatibility rule is that a scene authored
  before a refinement sounds exactly as it did.
- **Smoothing the per-ear gains instead of the pan.** Rejected: the gains are a
  nonlinear (`cos`/`sin`) function of the pan, so easing them directly would not
  preserve the constant-power law during the sweep and would dip in loudness
  exactly where the pan is crossing the centre. Easing the pan and then applying
  the law keeps `left² + right² == 1` on every frame of the sweep.
- **A critically damped spring instead of a one-pole.** Rejected: a spring needs
  a velocity per voice (more state, and a second constant to tune) to buy
  overshoot that a pan does not want. A one-pole is monotone, never overshoots,
  and has the single time constant the acceptance test asks for.
