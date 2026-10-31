# 0117 — A path-keyed reference resolves against the project, not the working directory

- **Status** — Accepted
- **Date** — 2026-10-30
- **Supersedes** — the follow-up named in [ADR-0108](0108-appimage-ships-the-editor.md)

## Context

A scene names its assets by a reference string. There are two forms:

- `guid:<uuid>` — a stable identity, resolved through the asset database (or a
  mounted pack) to the asset's *current* path. This already works from any
  working directory, because the database holds an absolute root and the pack
  carries its own index.
- `tex:<path>`, `obj:<path>`, or a bare `<path>` — a **path-keyed** reference,
  resolved by handing the path to the loader as written.

The path-keyed form was resolved against the **process working directory**. That
is correct only when the process happens to run from the directory the reference
was written relative to. Everywhere else it fails, and it fails *silently*: a
texture that does not load is a warning, and the scene renders untextured rather
than reporting an error.

The reference is in fact written relative to the **project directory**. This is
not an assumption; it is what the packer and the exporter already do. `fork_pack`
and `fork_export` take the project's directory as the pack root and key every
asset by its project-relative path (`assets/meshes/cube.obj`,
`scenes/main3d.fescene`), and the export manifest names the scene the same way.
The resource system was the one place that disagreed with the other three.

The cost was visible in two places:

- `examples/demo3d_project` and `examples/game3d_project` reference
  `obj:assets/meshes/cube.obj` and `tex:assets/textures/checker.png`. Opened from
  the repository root, the 3D demo viewport rendered nothing at all: ~99.5% flat
  clear colour. The scene was valid; the assets simply were not found.
- ADR-0108 shipped a copy of `assets/` into the AppImage so the demo scene's
  `tex:` reference would resolve from the AppDir root. That treated the symptom
  and left the underlying behaviour in place, and ADR-0108 named the real fix as
  a follow-up.

A related defect made this worse: the manager passed its *mounted source* to a
loader, and with no pack mounted that is null, so a loader fell back to reading
the filesystem itself. There was no single place that could apply a base to an
asset read, because half the reads did not go through the manager.

## Decision

`ResourceManager` gains a **reference base**: the directory a path-keyed
reference is resolved against. It is empty by default, which preserves the old
behaviour (resolve against the working directory) exactly, so a test, a tool, or
a caller that never sets it is unaffected.

- A loader always reads through the manager, which applies the base. The manager
  hands the loader a `LoaderSource` — itself — rather than the mounted pack, so
  there is one code path for "where do an asset's bytes come from", whether that
  is a pack or the filesystem.
- The base is applied to **loader** reads only, not to the manager's own
  `read_text`/`read_binary`. The scene reader calls those with a path it has
  already resolved; joining the base there would look for the scene inside the
  project a second time.
- An absolute path is never joined. The base is a prefix for relative references
  only.
- A mounted pack still wins. A pack's keys are its own, and a pack is already
  self-describing through its index, so the base is not consulted when a source
  is mounted.
- The `guid:` path is untouched. It was never working-directory dependent, and
  the fix must not disturb the mechanism that already survived a file move.

The editor sets the base to the project's directory when it opens a project
(`EditorContext::load_project`), and to the asset root in the `--project-root`
form, where there is no project file and therefore no project directory.

## Consequences

**Easier:**

- A project opens correctly from anywhere. `fork_editor_shell --project
  examples/demo3d_project` renders its meshes whether it is run from the
  repository root, from `/tmp`, or from the project directory itself.
- The reference forms now agree. The packer, the exporter, the manifest, and the
  resource system all treat a path-keyed reference as project-relative.
- The AppImage no longer needs the copy of `assets/` at the AppDir root for the
  demo scene to render; that copy becomes a convenience rather than a
  correctness requirement.
- A path-keyed reference and a `guid:` reference to the same asset resolve to
  the same file, which is the invariant the asset database was built to provide.

**Harder:**

- A loader may no longer read the filesystem directly. Every loader read must go
  through `read_bytes`/`read_text`, or it bypasses the base and reintroduces the
  bug for its own type. This is now a rule a new loader has to follow, and it is
  not enforced by the type system.
- There are two resolution mechanisms again — the base for path-keyed references
  and the database for `guid:` — where one would be simpler. They are kept
  separate deliberately: collapsing path-keyed references into the database would
  make an unimported asset unloadable, which is a state the editor supports.
- Changing the base does not invalidate the cache. The cache key is the
  reference as written, so a caller that repoints the base while holding loaded
  resources is responsible for reloading them; opening a project does exactly
  that by loading a fresh scene.

## Alternatives considered

**Resolve every path-keyed reference through the asset database.** Rejected: an
asset that has not been imported has no sidecar and no id, and the editor
supports authoring a scene against a file before importing it. Routing path
references through the database would break that state.

**Change the working directory to the project directory on open.** Rejected: it
is process-global state that a library should not own, it breaks a caller that
opens two projects, and it would make the base implicit in a way a test cannot
set or assert.

**Keep shipping the assets copy and leave the semantics alone.** Rejected: it
fixes the download and nothing else. The same failure reproduces for anyone
running the editor from a build directory, which is the common case during
development, and ADR-0108 had already named the correct fix.

**Require absolute paths in scene references.** Rejected: it makes a project
non-portable, which is the property the relative form exists to provide, and it
would break every existing scene.
