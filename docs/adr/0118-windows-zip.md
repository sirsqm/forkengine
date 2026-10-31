# ADR-0118: A self-contained Windows zip, played in CI with the null backend

- **Status:** Accepted
- **Date:** 2026-10
- **Related:** [ADR-0041](0041-platform-contract-and-null-backend.md),
  [ADR-0102](0102-downloadable-release-builds.md),
  [ADR-0103](0103-linux-appimage.md)

## Context

The queue item (`windows-zip`) asks for "a Windows artifact a player can unzip
and run", with the acceptance test "the zip builds in CI and the game launches
from it on a clean machine". The packaging already produces the zip: `windows-zip`
is one platform of `packaging/package_release.cmake`, which writes a `-game`,
`-sdk`, and `-runtime` archive per OS ([ADR-0102](0102-downloadable-release-builds.md)).
Three things are missing for the item to be *done* rather than *mostly done*.

**The zip may not run on a clean machine.** MSVC defaults to the DLL C runtime
(`/MD`), so the packaged `fork_breakout.exe` needs the Visual C++ redistributable
installed. A machine that has never seen the engine — the clean machine the
acceptance test names — may not have it, and the game then fails to start with a
missing `VCRUNTIME140.dll` while nothing in the archive is wrong. This is the
same class of bug as the AppImage's glibc floor
([ADR-0103](0103-linux-appimage.md)): the artifact works on the build machine and
fails on the downloader's.

**The zip is never exercised in CI.** The release workflow builds it, but that
workflow only runs on a tag, so a break in the Windows packaging path is invisible
until a release is cut. The Linux job packages and *plays* the artifact on every
push; Windows had no equivalent, so the one platform whose artifact is a zip was
the one nobody ran.

**Playing the game on a headless Windows runner needs no display.** The Linux
smoke plays the packaged game under Xvfb with Mesa's software driver, because the
game opens a real GL 3.3 window. A Windows CI runner has neither Xvfb nor a GPU
context, so the game could only be started, never played — which leaves the
interesting half (does the pack load, does the scene step, does the game run from
the archive) unproven.

## Decision

**1. The Windows build links the MSVC runtime statically.** `cmake/FESettings.cmake`
sets `CMAKE_MSVC_RUNTIME_LIBRARY` to `MultiThreaded$<$<CONFIG:Debug>:Debug>`
(`/MT`, `/MTd`), folding the CRT into the binaries so the zip is self-contained.
This is a `set()` in the settings module, not `add_compile_options()`, so it
carries on every target we own and not on third-party code. GLFW, which is
compiled into our binaries, is told to use the same runtime
(`USE_MSVC_RUNTIME_LIBRARY_DLL OFF` in `FEDependencies.cmake`): a target built
`/MT` that frees memory allocated by one built `/MD` is undefined behaviour, and
the mismatch is not a link error that would catch itself.

**2. The game can run with no display, on any platform.** `fork_breakout` gains a
`--null-window` flag that selects `WindowBackend::Null` ([ADR-0041](0041-platform-contract-and-null-backend.md)):
no OS window and no GL context, the render path drawing blank against the null
device. It is the same backend `fork_player --headless` already uses, exposed on
the game so the packaged binary itself can be started where there is no display.

**3. The packaging smoke can play the game with the null backend.**
`packaging/package_release.cmake` gains `FE_SMOKE_NULL_WINDOW`, which appends
`--null-window` to the game's `--headless` run. With it on, the smoke unpacks the
zip, runs the launcher (`run.bat`, through `cmd /c`, so the launcher a player
uses is what is exercised), and plays the game with no display — the whole
artifact on a headless runner.

**4. CI packages and plays the Windows zip on every push.** A Windows step in
`.github/workflows/ci.yml` runs the same packaging script the release runs, with
`FE_SMOKE_GAME=ON` and `FE_SMOKE_NULL_WINDOW=ON`, and asserts `SHA256SUMS` is
written. The release workflow's Windows job turns the game smoke on the same way,
so the artifact is played there too rather than only started.

**5. The decisions are guarded by tests.** `cmake/tests/test_release_workflow.cmake`
asserts the script selects the zip format for Windows, that it offers the
null-window smoke, that the settings module links the runtime statically, and
that GLFW is built with it — and reads `ci.yml` to assert the Windows packaging
step exists and asks for the null-window smoke. The failure each guards against
is silent: a zip that works on the build machine, or a step deleted without the
Windows artifact being played at all.

## Consequences

- The Windows zip is self-contained: a player unzips it and runs `run.bat` with
  no redistributable and no compiler. The static runtime is a few hundred KB
  larger than the DLL one and is the standard choice for a game that ships
  without an installer.
- The Windows artifact is exercised on every push, on the platform it ships to,
  so a break in the zip path fails CI rather than a release.
- The game's `--null-window` flag is a general facility: any packaged game can be
  started with no display, which is what a dedicated-server or a CI smoke of a
  shipped build wants, and it is the same null backend the rest of the engine
  already relies on.
- The Windows play smoke proves the pack loads, the scene steps, and the game
  runs from the unpacked zip. It does not prove the *renderer* draws on Windows —
  that needs a real GL context, which a headless runner has not got. The renderer
  is covered by the graphics tests (which skip without a context) and by the
  Linux Xvfb play smoke; this is recorded rather than hidden, and matches the
  Linux artifact's split between "runs" and "draws".
- macOS remains: its `.app`/DMG packaging is the next item, and it can reuse the
  `FE_SMOKE_NULL_WINDOW` path for its own headless check.

## Alternatives considered

- **Ship the VC++ redistributable beside the game.** Rejected: it is a second
  thing to install, it needs an elevated install or a per-app deployment, and the
  static runtime makes the question disappear for a game with no installer.
- **Bundle the runtime DLLs (`vcruntime140.dll`, `msvcp140.dll`) in the zip.**
  Rejected as more fragile than the static runtime: the list of DLLs to copy
  varies by MSVC version and by which C++ features are used, and getting it
  slightly wrong reproduces the exact failure it tries to avoid.
- **Give the Windows runner a software GL context (Mesa/llvmpipe) as Linux
  does.** Rejected: there is no Xvfb-equivalent for the Windows GL stack that is
  as simple to set up, and the null backend already proves everything except the
  draw — which the graphics tests cover where a context exists. Reaching for a
  full display stack on Windows CI would be more moving parts for less coverage
  than the null-backend play smoke.
- **Assert the zip exists without playing it.** Rejected: that is the state the
  item was in. The acceptance test is "the game launches from it", so the smoke
  launches it.
