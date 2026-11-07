# ADR-0147: The second vertical slice, and the 3D integration test it is

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0036](0036-breakout-vertical-slice.md),
  [ADR-0008](0008-systems-and-frame-schedule.md),
  [ADR-0043](0043-physics3d-angular-dynamics.md),
  [ADR-0071](0071-web-first-2d-shipping-path.md),
  [ADR-0098](0098-persisted-settings-and-a-settings-screen.md)

## Context

The first vertical slice (ADR-0036) shipped breakout, and it did its job: it
proved the 2D subsystems work *together*, and every seam it stressed was a real
defect fixed in the engine. Since then the 3D rows have landed one by one —
skeletal animation and skinning, animation state machines and blend trees,
3D physics with angular dynamics and oriented colliders, positional audio,
instanced 3D draws, and the 3D frame on the render graph — each with its own
unit tests and none of them ever run together.

That is the same gap M2.5 closed for 2D, left open for 3D. A capability that has
only ever passed its own unit tests has not been proven to work with the others,
and the 3D systems are exactly the ones a game-shaped artifact has never
exercised at once. `examples/game3d_project` is the closest thing today and it is
deliberately a tech demo: a course to walk, props to shove, and no goal, score,
in-game UI, audio mix, or persistence.

The roadmap calls this out (`M11 — A second vertical slice: a 3D game, end to
end`) and repeats the project's guiding rule: a few genuinely working systems
beat a long list of half-finished features.

## Decision

Ship **one small, complete 3D game — Gem Rush (`examples/collect3d`) — as a
second integration test**, and treat its source as test code, exactly as ADR-0036
does for breakout.

Specifically:

1. **The game is an external consumer.** `examples/collect3d` is built against the
   engine's public headers only, with the engine linked as a library. If the
   public API is not self-sufficient to write a 3D game, that is a defect the game
   exposes.

2. **The tests compile the game's own sources.** `fork_engine_tests` pulls
   `slice_state.cpp`, `slice_scene.cpp`, and `slice_systems.cpp` in by path and
   steps them on an in-memory scene. The tests prove the shipped game, not a copy,
   and they need no window, GPU, or sound card — so they run in CI. The arena
   layout has one definition (`build_collect3d_scene`), shared by the application,
   the serialized scene file, and the tests, so the three cannot drift.

3. **The rules are pure.** `CollectState` and `Settings` have no engine dependency
   at all. Score, the gem count, the clock, win/lose, the screen flow, and the
   saved volume are plain C++ types, unit-tested by hand.

4. **The game ships for desktop and the web from one target.** The web build
   preloads the scene, the font, and the cue at the paths the game already reads,
   and uses a shell page for the `#canvas`, so the code never asks which platform
   it is on (ADR-0071). This is what makes "a 3D game ships" a demonstrated
   property rather than an assertion.

5. **The game is not a place to hide unfinished engine work.** Nothing in it is a
   placeholder presented as finished. Skeletal animation, positional audio, and
   streaming are documented as deferred and the game uses the simple path
   deliberately.

## Consequences

- The 3D subsystems are now exercised together, so a regression at a seam between
  them — a 3D physics call the renderer assumes, an input action a controller
  needs, a UI element the HUD fills — fails a test or the web acceptance check
  rather than being noticed a milestone later.
- There are now two integration tests with the same shape, which makes the
  pattern (pure rules, one layout definition, sources compiled into the suite)
  the project's convention rather than a one-off. A third slice, if one is ever
  wanted, has a template.
- The slice adds a third 3D `examples/` target to build and to keep green in the
  web job, which is a small, ongoing cost.
- The player is a rigid body with a controller, not a skinned mesh, so the
  animation rows are proven by their own tests but not by this artifact. That is
  the one 3D row the slice does not exercise end to end, and it is recorded as
  deferred rather than claimed.

## Alternatives considered

- **Extend `game3d_project` instead of a new example.** Rejected: the project is
  a scene-and-assets demo with no code, and the slice needs a rules class, systems
  and an application shell compiled into the test suite. Bolting those onto a demo
  would blur the two roles the same way ADR-0036 refused to blur a demo and an
  integration test.
- **Make the character a skinned mesh driven by a blend tree.** Rejected for this
  slice: it needs authored skeletal content, which is a larger art-and-content
  step than the seam-testing the slice exists to do. Recorded as deferred.
- **One larger game covering both 2D and 3D.** Rejected: a game that is half 2D
  and half 3D tests neither path cleanly, and the two slices stay small and
  legible by staying separate.
