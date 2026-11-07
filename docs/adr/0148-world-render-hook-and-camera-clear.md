# ADR-0148: A world render hook, and the 3D camera's clear colour

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0086](0086-in-game-ui-layout-and-extraction.md),
  [ADR-0041](0041-platform-contract-and-null-backend.md),
  [ADR-0042](0042-null-rhi-device.md),
  [ADR-0100](0100-render-graph-seam.md),
  [ADR-0147](0147-second-vertical-slice-3d.md)

## Context

`Application`'s frame ran the 2D pass and the UI pass, and then called the
subclass's `on_render()` hook. The 3D renderer is not part of that frame: the
engine's own comment said so — "Application's frame draws 2D; 3D is the app's own
step" — and every 3D host (`examples/collect3d`, `tools/player`,
`examples/headless`) drew its world from `on_render()`.

That ordering was wrong, and the second vertical slice (ADR-0147) made it
visible. `on_render()` runs *after* the UI pass, so a 3D game that drew its world
there painted the arena over its own HUD: the labels were submitted, the draw
counts were non-zero, and the pixels were overwritten. A 3D game had to hand-roll
the whole frame to get its world under its UI, and none did.

A second defect sat beside it. `Application::render_frame()` chose the frame's
clear colour from the active `Camera2D` only:

```cpp
rhi::ClearColor clear{};
if (const Camera2D* camera = Renderer2D::find_active_camera(scene_); camera != nullptr) {
    clear = rhi::ClearColor{camera->clear_r, camera->clear_g, camera->clear_b, camera->clear_a};
}
```

`Camera3D` carries the same `clear_*` fields and a comment that they are "the
background colour the renderer clears to when this is the active camera", but
nothing read them. A scene whose only camera was a `Camera3D` therefore cleared
to black — and, because the gradient background is off by default, the whole
frame behind the geometry was black. The web game check called this "non-blank"
because the geometry drew, so the defect shipped past the check that exists to
catch exactly this.

## Decision

**Give the frame a world hook, and take the clear colour from whichever camera
the scene has.**

1. Add `Application::on_render_world(delta_time)`, called inside the render phase
   *before* the 2D and UI passes. A 3D game draws its scene there, so the 2D and
   UI passes layer over it and a HUD lands on top of the world. The default is
   empty, so a 2D game pays nothing.

   The frame's pass order is now explicit: clear → `on_render_world` (3D) → 2D
   pass → UI pass → `on_render()` (a game's own debug overlay, on top of
   everything).

2. Move the 3D draw in every host from `on_render()` to `on_render_world()`:
   `examples/collect3d`, `tools/player`, and `examples/headless`.

3. Extract the clear-colour choice to `Application::scene_clear_color(const
   Scene&)`, a static pure function: a 2D camera's colour when one exists, else
   the active 3D camera's colour, else black. A 2D game with a 3D backdrop keeps
   its 2D background; a 3D-only scene clears to its camera instead of to black.

## Consequences

- A 3D game's HUD is drawn over its world without the game doing anything
  special. `examples/collect3d`'s labels are now on top of the arena, which is
  what a player expects and what the slice could not deliver before.
- `Camera3D::clear_*` is live: the fields mean what their comment says, and a
  scene's background is a scene property rather than a hardcoded black.
- The selection is a pure function of the scene, so it is unit-tested with no
  graphics context; a graphics test proves the chosen colour reaches the
  framebuffer.
- The two behaviours are independent: the hook fixes layering, the clear colour
  fixes the background. A 2D-only game sees neither change.

## Alternatives considered

- **Draw the world in `on_render()` and move the UI pass after it.** This would
  fix the layering but invert the hook's documented meaning ("after the 2D
  pass") and break any host that used `on_render()` for a post-UI overlay. A
  dedicated hook states the order instead of relying on it.
- **Have `Application` call `Renderer3D` itself.** The runtime does not know
  whether a scene is 2D or 3D, and forcing a 3D pass on every 2D game would walk
  the scene and bind targets for nothing. The hook keeps the choice with the game
  and the cost at zero for a 2D game.
- **Default the 3D camera's clear to a visible sky and leave the selection
  alone.** That hides the dead fields rather than wiring them, and a game that
  wants a specific background still could not set one.
