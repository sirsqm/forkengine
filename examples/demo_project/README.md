# Demo project

A minimal Fork Engine project, used to try the editor without authoring
anything first. It has a project file, an asset root, and one scene with two
textured sprites.

```
fork.project.json      name, asset root, and startup scene
assets/textures/       the texture the scene references
scenes/main.fescene    the startup scene (a camera, a background and a hero sprite)
```

Open it with the editor shell:

```bash
./build/bin/fork_editor_shell --project examples/demo_project
```

`--project` accepts either a project directory (the file `fork.project.json`
inside it is loaded) or a project file directly. The scene's `tex:` reference is
resolved relative to the project directory, so the project opens correctly from
any working directory (see ADR-0114).

The scene carries a `Camera2D` whose reference box is the 1280x720 design
space the sprites are laid out in. Without it the 2D renderer falls back to a
raw pixel-space projection with the origin at the top-left, so a scene authored
around the origin is drawn off-centre and mostly clipped. A scene that uses
`Sprite2D` needs a camera for the same reason a scene that uses `MeshRenderer3D`
needs a `Camera3D`.

On a machine without a display, `--headless --frames N` opens the project, loads
the startup scene, and exits after N frames, which is enough to check that the
project loads:

```bash
./build/bin/fork_editor_shell --project examples/demo_project --headless
```

The scene stores its texture as a `tex:` reference. Once the asset is imported
into the project database it can be referenced by a stable `guid:` id instead,
so the reference survives the file being moved or renamed (see ADR-0020).
