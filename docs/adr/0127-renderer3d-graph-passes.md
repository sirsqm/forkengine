# ADR-0127: The 3D frame as a graph

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0100](0100-render-graph-seam.md),
  [ADR-0110](0110-render-graph-pass-culling.md),
  [ADR-0057](0057-environment-and-post-processing.md),
  [ADR-0017](0017-performance-strategy.md),
  [ADR-0052](0052-render-extraction-seam.md),
  [ADR-0042](0042-null-rhi-device.md)

## Context

[ADR-0100](0100-render-graph-seam.md) added the render-graph seam and drove
`Renderer2D`'s world and UI passes through it. It named the 3D path as the
follow-on it deliberately did not convert:

> `Renderer3D` is not yet driven by the graph. Its shadow/main/post sequence is
> still written as calls; converting it is a follow-on, and the graph is shaped so
> that conversion is a declaration change.

[ADR-0110](0110-render-graph-pass-culling.md) then added pass culling and named
the same gap as what stops the culling paying off on the 3D path:

> `Renderer3D`'s shadow/main/post sequence is still written as calls, so its
> passes are not yet declared to the graph and cannot yet be culled. Converting
> it is the follow-on that makes the culling pay off on the 3D path.

So the 3D frame was the last place pass order lived in the sequence of statements
in a `render()` body rather than in data. `Renderer3D::render` extracted, culled,
then called `draw_chain`, which bound the post-process target, drew the
background, called `draw` (the shadow pass then the main pass), and applied the
post chain -- correct, but implicit, and invisible to the profiler's per-pass
breakdown and to culling.

Constraints that shaped the decision:

- **Do not change the output.** The 3D frame is covered by pixel tests (a nearer
  mesh occludes a farther one, a point light lights a mesh, a background fills
  the frame), a shadow-caster count, and a draw-call count. The conversion must
  leave all of them exactly as they were, so it is a change of *shape*, not of
  behaviour.
- **Keep the direct `draw()` path working.** `draw()` takes a caller's extracted
  and culled instance array and draws it; tests and the benchmark call it
  directly. It must keep drawing, and it must keep drawing the same thing.
- **A graph callback sees only a pass name.** The passes need frame state resolved
  before the graph runs -- the lights, the batched instance stream, the post
  settings -- so that state has to be resolved up front and captured by the
  callbacks, not looked up inside them.

## Decision

`Renderer3D` declares its shadow, main, and post passes on a
`render::RenderGraph` and runs them through `run()`.

The frame's shared state is resolved once, before the graph runs, by a new
private `prepare_frame`: the gathered point and spot lights and their packed
arrays, the frame's lighting constants, and the batched instance stream the
shadow and main passes both draw. `prepare_frame` reports the resolved lighting
through a `FrameLighting` the callbacks share, so the main pass is lit by exactly
the constants the shadow pass used -- the same sharing the hand-ordered code got
by keeping one `FrameLighting` local.

The two draw steps become named private passes:

- `draw_shadow_pass` fits the light frustum to the visible instances, renders
  their depth into the shadow map, and restores the target it found bound. It is a
  no-op when `prepare_frame` resolved no directional light.
- `draw_main_pass` draws the camera's background gradient (when a camera is
  given), then the lit static and skinned geometry, into the currently bound
  target. It takes the camera as a pointer: the graph path passes the frame's
  camera, and the direct `draw()` path passes null and draws no background,
  exactly as before.

`draw_chain` declares `shadow`, `main`, and `post` with `main` after `shadow` and
`post` after `main`, and declares the flow (`shadow` writes `shadow_map`, `main`
reads it and writes `backbuffer`, `post` reads `backbuffer` and writes the marked
`swapchain`), the same shape `Renderer2D` uses. The declarations keep every pass
live, so the run order is the dependency order -- shadow, main, post -- and each
pass is timed as `pass/<name>`.

`render_frame` is the new public entry point; `render` is a thin wrapper around
it, so an existing caller is unchanged. The `post` callback grades the off-screen
image into the caller's target only when the chain is enabled, so a frame without
post-processing still declares and runs `post` (as a no-op) rather than changing
the graph's shape frame to frame.

The mesh-buffer sweep is not a pass: it is the frame's last step, after every
pass that draws from the cache, and runs from `draw_chain` after the graph.

## Consequences

**Easier:**

- The 3D frame's ordering is data, like the 2D frame's. Reordering the passes or
  inserting one is a declaration change rather than an edit to `draw_chain`.
- Every pass is timed as `pass/<name>`, so a frame report breaks the 3D frame into
  shadow, main, and post without per-call instrumentation.
- The 3D passes are declared to the culling rule, so a pass whose output nobody
  reads can be dropped as the post chain grows.
- The passes are named methods with explicit inputs, so a test can assert the run
  order from a profiler frame -- the same trick the graph's own test uses, here
  against a real renderer.

**Harder / not yet done:**

- The graph still does not own, allocate, or alias render targets; render-target
  ownership remains the open half of the render-graph row, and the off-screen
  post target is still owned by the renderer rather than the graph.
- The graph's shape is fixed at three passes. A frame that wants a different pass
  set still needs a code change; the declarations are rebuilt per call, so a
  caller's own pass is a change to `draw_chain`, not a parameter.

## Alternatives considered

- **Convert only the shadow and main passes, leaving post outside the graph.**
  Rejected: the post pass is where the off-screen target and the caller's target
  meet, and it is the pass a longer chain grows from. Leaving it outside would
  declare the part of the frame that is least likely to change and skip the part
  that is most likely to.
- **Declare only the two live passes, and add `post` conditionally.** Rejected:
  the run order -- and so the profiler's per-pass breakdown -- would change shape
  with a setting, which makes a benchmark and a test flaky. Declaring all three
  keeps the frame's shape stable, and the `post` callback is a no-op when the
  chain is off.
- **Pass the whole frame state through the graph instead of a shared
  `FrameLighting`.** Rejected: the graph is a scheduler of names, not a context
  object. Capturing the resolved state in the callback is the same lifetime the
  hand-ordered code had, with no new type in the graph's API.
- **Make `render_frame` the only entry point and delete `draw()`.** Rejected:
  `draw()` takes a caller's own extracted and culled array and is used by tests
  and the benchmark. It is now two calls (`prepare_frame`, then the two passes),
  so it stays a real path rather than a second copy of the frame.
