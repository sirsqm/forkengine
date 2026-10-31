# ADR-0122: The MSVC runtime stays shared, and the zip bundles it

- **Status:** Accepted
- **Date:** 2026-10
- **Supersedes:** decision 1 of [ADR-0118](0118-windows-zip.md) (the static CRT)
- **Related:** [ADR-0061](0061-cpp-gameplay-modules-hot-reload.md),
  [ADR-0118](0118-windows-zip.md),
  [ADR-0102](0102-downloadable-release-builds.md)

## Context

[ADR-0118](0118-windows-zip.md) made the Windows zip self-contained by linking
the MSVC C runtime statically (`/MT`, `/MTd`) across the build. That decision was
made for the *game* archive, where it is correct in isolation: a static CRT means
a player unzips the game and runs it with no Visual C++ redistributable.

It broke the *engine*, and the break was not visible where the decision was made.

A gameplay module is a DLL built against the engine ([ADR-0061](0061-cpp-gameplay-modules-hot-reload.md)).
The engine's own header states the invariant: "A module and the engine exchange
`std::` types and the Script vtable across the binary boundary, so the C++ runtime
must be shared rather than statically duplicated into the module." A static CRT
gives the executable and each module their own copy of the runtime and their own
heap. A `std::string` or `std::function` allocated on one side and freed on the
other then frees a pointer the other heap does not own. Under the debug CRT
(`/MTd`, which CI builds) that trips the debug-heap assertion; on a headless
runner nothing dismisses the resulting dialog, so the test process hangs instead
of failing. The failure is a hang, not a red test, so it is easy to mistake for a
runner problem.

The regression was measurable: the last green Windows run before the change
passed `engine_tests` in about 11 seconds; every run after it hung past 20
minutes. The module-loading tests (`test_script_module`, `test_export`) had been
green on Windows for weeks under the shared runtime, and the only build-flag
change in the window was the static-CRT one.

## Decision

**1. The MSVC runtime stays shared (the `/MD` default).** `cmake/FESettings.cmake`
no longer sets `CMAKE_MSVC_RUNTIME_LIBRARY`, and `cmake/FEDependencies.cmake` no
longer forces GLFW onto the static runtime. The rule in `cmake/FEModules.cmake` --
the runtime is shared because a module and the host exchange `std::` objects --
is the binding constraint, and it outranks the zip's convenience.

**2. The zip is self-contained by bundling the redistributable runtime DLLs.**
`cmake/FEInstall.cmake` gains `fe_install_msvc_runtime()`, which installs the CRT
DLLs beside the binaries, and the packaging script copies them into the game
archive next to the game binary. A player still unzips and runs, with no
redistributable installed and no compiler.

The list is produced by CMake's `InstallRequiredSystemLibraries`, which asks the
toolchain for the exact DLLs this compiler needs and enables the UCRT for
app-local deployment. This answers the objection
[ADR-0118](0118-windows-zip.md) raised against bundling -- that a hand-maintained
list of `vcruntime140.dll`, `msvcp140.dll`, ... varies by MSVC version and is
fragile. The list is not hand-maintained; it is derived from the toolchain that
built the binaries, so it cannot drift from them.

## Consequences

- The module tests run on Windows again, and the invariant in `FEModules.cmake`
  is true of the build rather than merely documented. The failure mode this
  removes is a silent hang, so the change is worth more than the test it fixes.
- The Windows zip carries a handful of extra DLLs (`vcruntime140`, `msvcp140`,
  the UCRT set) instead of folding them into the binaries. The archive is a little
  larger and holds more files; it still needs nothing installed on the target.
- The guard test (`cmake/tests/test_release_workflow.cmake`) now asserts the
  *opposite* of what it asserted before: the runtime must stay shared, the
  install must carry the runtime, and the game archive must include the DLLs.
  A change that reintroduces the static CRT fails the suite with the reason.
- A Debug tree needs the debug CRT, which is not redistributable, so a Debug
  install is not a shipping configuration. This is unchanged from any Windows
  release build, which is `Release`.

## Alternatives considered

- **Keep the static CRT and give each module its own runtime too.** Rejected:
  the module/host `std::` exchange is the point of the module seam, and making
  both sides static does not give them a shared heap -- it gives each its own,
  which is the bug. Making the seam exchange only C types would remove the
  ability to register a behaviour by name and pass reflected properties, which is
  what the seam is for.
- **Bundle the runtime DLLs from a hand-written list.** Rejected, as in
  [ADR-0118](0118-windows-zip.md): the list varies by MSVC version and by which
  C++ features are used. `InstallRequiredSystemLibraries` derives it instead.
- **Link the game statically and the engine tests shared.** Rejected: two
  runtime configurations in one build is exactly the mismatch that is undefined
  behaviour rather than a link error, and it would leave the shipping path
  (which is what the tests stand in for) untested.
