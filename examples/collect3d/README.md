# Gem Rush — the M11 vertical slice (3D)

This is the engine's second complete game: a small, finished third-person
collect-the-gems game built entirely against Fork Engine's public API, exactly as
an external project would build one. It is the 3D counterpart of the
[breakout slice](../breakout/README.md) (M2.5), and it exists to answer the same
question in three dimensions: *do the 3D subsystems work together?* The 3D rows
each had their own tests, but they had never run together in one artifact — a
skinned/animated character, positional physics, the 3D renderer, a HUD, a menu,
and a save.

The game is deliberately not clever. It is the smallest thing that exercises the
whole 3D engine at once, and every gap it exposed was fixed in the engine rather
than papered over in the game.

## What it exercises

| Subsystem | How the game uses it |
| --- | --- |
| Scene graph and transforms | Every wall, gem, player, camera, light, and audio node is a `Node` |
| `Camera3D` | One camera frames the arena and follows the player |
| `DirectionalLight3D` | The sun that lights the arena |
| Resource manager + mesh loader | A unit cube loaded from `cube.obj` |
| `MeshRenderer3D` | Every visible box, drawn by scaling the shared unit cube |
| `Physics3D` | Static floor and wall colliders, a gravity-free player body |
| Built-in gameplay behaviours | `CharacterController3D`, `FollowCamera3D`, `SpinNode3D` |
| Audio | `WavLoader`, `AudioClip`, the pure mixer, `AudioSystem`, the real backend |
| Serialization | The arena round-trips through the JSON scene format |
| Input | WASD/arrows to move, Space to jump, P to pause |
| UI | A HUD, a title, a pause screen, a game-over screen, and a settings screen |
| Systems + `SceneSchedule` | The whole game is a fixed order of systems |

## How it is built

The code is split the same three ways as breakout, on purpose.

- **`slice_state.h/.cpp` — the rules.** Pure C++ with no engine dependency:
  score, the gem count, the clock, win/lose, the screen flow, and the player's
  saved settings. Because it has no dependency it is unit-tested directly, and
  the numbers are asserted by hand.
- **`slice_scene.h/.cpp` — the arena and the wiring.** `build_collect3d_scene`
  builds the canonical layout and `attach_collect3d_behaviours` binds the
  behaviours to their nodes. This is the single source of truth: the serialized
  scene file is a dump of exactly this, so the file path, the code-built fallback,
  and the tests cannot drift apart.
- **`slice_systems.h/.cpp` — the gameplay systems.** The only things that touch
  the scene; they read and write `CollectState`.
- **`main.cpp` — the application shell.** Loads or builds the scene, opens the
  audio device, drives the loop, rebuilds the arena when the rules ask (a restart
  or a fresh round), and logs a machine-checkable result.

The systems, in order (ADR-0008):

1. `UiInputSystem` — writes this frame's pointer interaction state first.
2. `MenuSystem` — decides the current screen before anything reads it.
3. `SettingsSystem` — applies and saves the volume.
4. `ScriptSystem` — runs the built-in behaviours (controller, follow, spin).
5. `AutoplaySystem` — inert unless autoplay is on; walks the player to a gem.
6. `Physics3DSystem` — steps the world.
7. `CollectibleSystem` — turns a player-vs-gem contact into a score and a cue.
8. `GoalSystem` — runs the clock and ends the round.
9. `HudSystem` — paints the score, the gems, and the time.
10. `AudioSystem` — added last by `main.cpp` (the engine owns it and the
    application adopts it), so a cue fired this frame is mixed this frame.

## Running it

```sh
cmake -S . -B build -DFE_BUILD_EXAMPLES=ON
cmake --build build --parallel
./build/bin/fork_collect3d
```

Controls: W/A/S/D or the arrow keys move, Space jumps, P pauses, Escape quits.
The game opens on a title screen; the same confirm press leaves it, and the
settings screen (reached from the title or a pause) changes the master volume.

Collect all eight gems before the clock runs out to win. The HUD shows the score,
the gems left, the time, and the best score from a previous run.

Headless (no window, autoplay walks the player, exits on win/lose or after N
frames):

```sh
./build/bin/fork_collect3d --headless 60000 --null-window
```

A run prints two lines a script can check:

```
info: collect3d: audio mixed 295200 frames, peak 0.4200
info: collect3d: score=1050 collected=8/8 result=win
```

`peak` is the largest absolute sample the mixer ever produced, so it is a real
"audio data was produced" signal even on a machine with no sound card.

Regenerate the authored scene from the code-built one:

```sh
./build/bin/fork_collect3d --null-window --save-scene examples/collect3d/scenes/collect3d.fescene
```

A save run gives itself a null RHI device, so it registers the font loader and
writes a real font reference for every canvas. Without that the font reference
would be empty — a device-free run has no font loader — and the regenerated file
would ship a HUD with no text on it. `tests/src/test_example_projects.cpp` fails
if a shipped scene's canvas has no font, so the mistake cannot land silently.

## Assets

The mesh, the font atlas, and the WAV cue are generated or hand-written, not
opaque binaries. `cube.obj` is a hand-written unit cube (one metre across,
centred on the origin) with six faces of four unique vertices each, so every face
carries one flat normal and a full 0..1 UV rectangle — the same convention
`Mesh::make_box` uses. The builder sizes every box by scaling the node, so this
one mesh draws the floor, the walls, the player, and the gems. The font atlas is
the engine's shared 5x7 pixel font (see `tools/gen_font_atlas.py`), and the WAV is
16-bit mono PCM at 44.1 kHz — a real file the `WavLoader` decodes.

## What is deferred

This slice is finished as a game, but these are explicitly later work, and the
game does not pretend to use them:

- **Skeletal animation.** The character is a rigid body with a controller, not a
  skinned mesh driven by a blend tree; that is a larger art-and-content step.
- **Positional audio.** The pickup cue is non-positional; wiring attenuation to
  the world transform is separate work.
- **Streaming / compressed codecs / audio buses.** As in breakout, the cue is
  fully decoded PCM on the master bus.
