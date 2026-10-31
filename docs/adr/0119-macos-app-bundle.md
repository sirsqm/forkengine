# ADR-0119: A macOS app bundle and DMG, smoke-run from the mounted image

- **Status:** Accepted
- **Date:** 2026-10
- **Related:** [ADR-0041](0041-platform-contract-and-null-backend.md),
  [ADR-0102](0102-downloadable-release-builds.md),
  [ADR-0103](0103-linux-appimage.md),
  [ADR-0118](0118-windows-zip.md)

## Context

The queue item (`macos-app-bundle`) asks for "a `.app` bundle, then a DMG, with
the runtime's dependencies bundled", with the acceptance test that the artifact
is built and run on a macOS host. The release already produces a macOS
`-game.tar.gz` — a directory of loose binaries ([ADR-0102](0102-downloadable-release-builds.md)) —
which a developer can use but which is not what macOS runs as a program. macOS
runs an *application bundle*: a directory Finder treats as one double-clickable
item, that the Dock and Gatekeeper understand, and inside which an app's content
travels. The platform's distribution shape is a DMG holding that bundle.

Three things decide the design.

**The entry point cannot be the game binary.** macOS launches an app by executing
`Contents/MacOS/<CFBundleExecutable>` with the working directory at `/`, and the
game resolves its content (`examples/breakout/...`) by a relative path. A bundle
whose executable is the game binary directly would start with the wrong working
directory and load nothing. The executable has to be a small script that `cd`s
into `Contents/Resources` and then execs the game — the same shape the AppImage's
`AppRun` has ([ADR-0103](0103-linux-appimage.md)).

**Which libraries travel is a platform decision, and it is the opposite of
Linux's.** On Linux the C++ runtime (`libstdc++`) is the part an older machine
lacks, so the AppImage bundles it and leaves glibc and the graphics driver on the
host. On macOS the C++ runtime is `libc++` and the frameworks, which are part of
the OS: copying `libSystem` or `Cocoa` would fight the host's `dyld`, exactly as
bundling glibc does on Linux. What *can* need bundling on macOS is a third-party
dylib linked by an absolute path from the build host — a path that does not exist
on a player's machine. So the rule inverts: bundle nothing the OS provides, and
bundle-and-rewrite anything that resolves to a build-host path. Today the engine
and GLFW are static, so nothing is bundled; the step is kept because the failure
it prevents is silent.

**The artifact has to be run, not just built.** A `.app` that starts on the build
machine can fail on a player's with a `dyld` error naming a path only the builder
had, or with Gatekeeper refusing an unsigned bundle. Neither shows up in a build.

## Decision

**1. `packaging/package_macos.cmake` builds the bundle and the DMG.** It runs on
a macOS host (the tools it needs — `otool`, `install_name_tool`, `codesign`,
`hdiutil` — are macOS-only) and writes `Fork Engine Breakout.app` and
`fork-engine-<ver>-macos-<arch>.dmg` into `dist/`. It joins
`package_release.cmake` and `package_appimage.cmake` as a per-platform packaging
script, driven by the same release job.

**2. The bundle layout is the platform's.** `Contents/Info.plist` names the
bundle and its executable; `Contents/MacOS/fork-engine` is the entry point script
(executable bit set where it is written); `Contents/MacOS/` also holds the game
and the editor; `Contents/Resources/` holds the game's content, the demo project,
the editor's sample assets, the icon, and the license — at the paths the binaries
resolve by relative path, so the shipped binaries are the ones CI tests.

**3. The entry point `cd`s to `Contents/Resources` and dispatches.** No argument
runs the game (this is a game bundle); `--editor` opens the editor on the bundled
demo project; anything else is passed to the game, so the bundle is a drop-in for
`--null-window`, `--headless N`, and the other flags the smoke and the README use.

**4. Bundled libraries are copied by content and rewritten to `@rpath`.** `otool -L`
names every library a binary links; anything under `/usr/lib` or `/System` is
skipped (the OS provides it), anything already `@rpath`/`@executable_path` is
skipped (it is already relative), and anything else is copied into
`Contents/Frameworks` with `configure_file` (following a symlink, so the bundle
carries bytes) and its install name rewritten to `@rpath/<name>` in the copy and
in the binary.

**5. The bundle is ad-hoc signed.** `codesign --force --deep --sign -` gives it a
signature so Gatekeeper does not refuse it outright. A real release would sign
with a Developer ID and notarize; that needs a certificate the script does not
have, so it is left to the release process and recorded rather than pretended.

**6. The smoke runs the bundle from the mounted DMG.** After `hdiutil create`
(writing the UDZO image), the script asserts the UDIF `koly` trailer — a
truncated file fails here rather than at a downloader — then mounts the DMG,
runs `Contents/MacOS/fork-engine --help` from inside it, and where `FE_SMOKE_GAME`
is on plays the game with `--null-window`. The volume is detached even when a
check fails, so a failed run leaves nothing attached on the runner.

**7. The decisions are guarded by tests and wired into the release.** A new
`macos_packaging` guard test reads the script and asserts the bundle layout, the
content, the library rule (skip the OS's, rewrite the rest), the `koly` check,
the mounted-DMG smoke, and that the release workflow builds the DMG, collects
`*.dmg` when it assembles the published assets, and turns the play smoke on. The
release workflow gains the macOS packaging step and the flatten step gains
`-name '*.dmg'`.

## Consequences

- A macOS player downloads one `.dmg`, double-clicks it, and drags the bundle out
  (or runs it from the mounted volume); the bundle carries the game and the
  editor and runs on a machine that has never seen the engine.
- The macOS release job plays the bundle it built, from the mounted DMG, on every
  tag — so a `dyld` failure or a wrong working directory fails the release rather
  than the download.
- The engine and GLFW being static means the "bundle the libraries" step is a
  no-op today. It is not dead code: it is what keeps the artifact correct if a
  dynamic dependency is added, and the `macos_packaging` test asserts it is still
  there.
- The DMG is unsigned with a Developer ID and not notarized. A player on a
  machine with Gatekeeper's default settings may see the "unidentified developer"
  prompt and have to allow it explicitly. Signing and notarizing need an Apple
  Developer certificate and a network notarization step, which are outside this
  item; this is recorded as the known gap rather than hidden.
- Cross-compilation is still open: the bundle is built on a macOS runner, not
  from Linux. The queue's `cross-compilation` item covers that, and this script
  is the artifact it would produce.

## Alternatives considered

- **Ship only the tarball, as today.** Rejected: it is not the shape macOS runs.
  A `.app` is what Finder double-clicks and what Gatekeeper reasons about; the
  item asks for a bundle for that reason.
- **Make the game binary the bundle's executable.** Rejected: macOS launches it
  with the working directory at `/`, so the relative content paths resolve
  nowhere and the game starts empty. The entry-point script is what sets the
  directory, the same reason the AppImage has an `AppRun`.
- **Bundle the OS's libraries and frameworks, as Linux bundles `libstdc++`.**
  Rejected: `libc++` and the frameworks are the OS's, and a bundled copy fights
  the host's `dyld` — the same failure as bundling glibc on Linux. The rule is
  the inverse of Linux's because the platform's runtime boundary is.
- **Sign with a Developer ID and notarize in the packaging script.** Out of
  scope: it needs a certificate and a network notarization step that a packaging
  script should not carry, and the item is the bundle and the DMG. The script
  ad-hoc signs so the artifact is runnable, and the gap is recorded.
- **Skip the smoke and only build the DMG.** Rejected: the acceptance test is
  that the artifact runs on macOS, and a bundle that mounts but cannot start —
  a missing dylib, a wrong working directory — is exactly the failure a build
  alone does not catch.
