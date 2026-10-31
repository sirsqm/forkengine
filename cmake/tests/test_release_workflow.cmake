# Test for the release workflow and the packaging script it drives, run by ctest
# in script mode.
#
# The release path is the one part of the project a contributor never runs: it
# fires on a version tag, so a mistake in it is invisible until a release is cut
# and the download is broken. Nothing in the desktop suite can observe it -- a
# green build says nothing about whether the workflow still packages every
# platform, still checksums what it publishes, or still refuses to publish a tag
# that disagrees with the engine version.
#
# A workflow is YAML, not code, so it is checked here the same way ci.yml is (see
# test_ci_web_job.cmake): by reading it as text and asserting the decisions it has
# to make. The packaging script is checked for the same reason: its contract is a
# set of inputs and a set of outputs, and the failure this guards against is a
# silent one -- an archive that is written uncompressed under a `.tar.gz` name, or
# a game archive that no longer carries the content the game loads.
#
# Run by tests/CMakeLists.txt:
#   cmake -DRELEASE_WORKFLOW=<path> -DPACKAGE_SCRIPT=<path> \
#         -DSETTINGS_SCRIPT=<path> -DINSTALL_SCRIPT=<path> \
#         -DCI_WORKFLOW=<path> -P cmake/tests/test_release_workflow.cmake

foreach(_required RELEASE_WORKFLOW PACKAGE_SCRIPT SETTINGS_SCRIPT INSTALL_SCRIPT CI_WORKFLOW)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "${_required} must be set (see the header of this file)")
  endif()
  if(NOT EXISTS "${${_required}}")
    message(FATAL_ERROR "${_required} does not exist: ${${_required}}")
  endif()
endforeach()

file(READ "${RELEASE_WORKFLOW}" _workflow)
string(REPLACE "\r\n" "\n" _workflow "${_workflow}")

# check: <label>|<literal substring the workflow must contain>
set(_checks
    "triggers on a version tag|tags:"
    "matches v-prefixed tags|\"v*\""
    "runs by hand too|workflow_dispatch:"
    "packages Linux|platform: linux"
    "packages Windows|platform: windows"
    "packages macOS|platform: macos"
    "uses the packaging script|packaging/package_release.cmake"
    "runs the packaged game as a smoke test|FE_SMOKE_PREFIX"
    "supplies a virtual display on Linux|xvfb-run"
    "checks the tag against the engine version|does not match the engine version"
    "uploads the artifacts|actions/upload-artifact@v4"
    "fails when no artifact is produced|if-no-files-found: error"
    "publishes only after packaging|needs: package"
    "publishes only for a tag|startsWith(github.ref, 'refs/tags/')"
    "creates the release|gh release create"
    "checksums the published assets|sha256sum"
)

# The Linux packaging job must not track `ubuntu-latest`. The AppImage's glibc
# floor is the build host's, and this is the job that publishes the one file a
# Linux player downloads; a runner-image rotation would silently raise that floor
# until older machines -- and the pinned `ubuntu:24.04` smoke container -- can no
# longer run the artifact. The pin is the whole defence, so assert it here.
if(_workflow MATCHES "os: ubuntu-latest")
  message(FATAL_ERROR
    "the release workflow builds on ubuntu-latest; the AppImage's glibc floor "
    "would follow the runner image. Pin the Linux job to ubuntu-24.04.")
endif()

foreach(_check IN LISTS _checks)
  string(REPLACE "|" ";" _parts "${_check}")
  list(GET _parts 0 _label)
  list(GET _parts 1 _needle)
  string(FIND "${_workflow}" "${_needle}" _at)
  if(_at EQUAL -1)
    message(FATAL_ERROR "release workflow is missing '${_label}' (looked for: ${_needle})")
  endif()
  message(STATUS "  ${_label}: ok")
endforeach()

# The publish job must be gated on every platform having packaged, or a release
# could appear with only some of its downloads. `needs: package` is the gate;
# a `continue-on-error` anywhere in the workflow would let a failed platform
# still publish.
if(_workflow MATCHES "continue-on-error")
  message(FATAL_ERROR "release workflow sets continue-on-error; a failed platform must fail the release")
endif()

# The workflow must be read-only by default, with the write permission granted
# only to the job that creates the release: a packaging step should never be able
# to write to the repository.
if(NOT _workflow MATCHES "permissions:\n  contents: read")
  message(FATAL_ERROR "release workflow does not default to read-only contents")
endif()
if(NOT _workflow MATCHES "contents: write")
  message(FATAL_ERROR "release workflow never grants contents: write; it cannot create a release")
endif()

# ---------------------------------------------------------------------------
# The packaging script.
# ---------------------------------------------------------------------------
file(READ "${PACKAGE_SCRIPT}" _package)

# The script decides compression from the platform, and the operation it uses has
# to be the compressing one for a `.tar.gz`. `cf` writes an uncompressed tar that
# `cmake -E tar xf` still reads by sniffing the format, so this mistake passes the
# smoke test and fails only for a downloader. Assert both halves are present.
if(NOT _package MATCHES "set\\(FE_TAR_OP \"czf\"\\)")
  message(FATAL_ERROR
    "the packaging script no longer uses the compressing tar operation for the "
    "tarball; a `.tar.gz` written with `cf` is a plain tar that `tar xzf` cannot read.")
endif()
if(NOT _package MATCHES "FE_TAR_OP")
  message(FATAL_ERROR "the packaging script never applies FE_TAR_OP to its tar calls")
endif()
if(NOT _package MATCHES "fe_assert_archive_format")
  message(FATAL_ERROR
    "the packaging script no longer asserts the archive format; the gzip-magic "
    "check is what makes an uncompressed `.tar.gz` fail loudly.")
endif()

# All three archives the release promises must be produced, and the game archive
# must carry the scene and the assets the game loads or it will not run from a
# download.
foreach(_needle "FE_GAME_ARCHIVE" "FE_SDK_ARCHIVE" "FE_RUNTIME_ARCHIVE" "SHA256SUMS")
  string(FIND "${_package}" "${_needle}" _at)
  if(_at EQUAL -1)
    message(FATAL_ERROR "the packaging script no longer produces ${_needle}")
  endif()
endforeach()

# The runtime archive is a prebuilt fork_player for `fork_export --player`, and
# the queue item is only satisfied if the export path actually accepts it. Assert
# the script checks that flow, not merely that it writes a file: it must invoke
# fork_export with --player against the unpacked runtime, or "a developer without
# the toolchain can export" is an assumption rather than a tested claim.
if(NOT _package MATCHES "--player")
  message(FATAL_ERROR
    "the packaging script no longer exports a game with the prebuilt runtime via "
    "--player; the runtime archive's whole purpose is that export flow.")
endif()
if(NOT _package MATCHES "fork_export accepted the prebuilt runtime")
  message(FATAL_ERROR
    "the packaging script no longer asserts that fork_export accepted the "
    "prebuilt runtime; the runtime smoke must check the export flow, not just "
    "that the archive carries a binary.")
endif()
# The runtime is staged from the installed tree, whose RPATH is relative, so it
# works after unpacking anywhere; the build tree's player has an absolute RPATH
# that would break the moment it was moved.
if(NOT _package MATCHES "stage-runtime-prefix")
  message(FATAL_ERROR
    "the runtime archive is no longer staged from an installed prefix; a player "
    "copied from the build tree carries an absolute RPATH and breaks when moved.")
endif()

# The game archive is staged by copying the example's content, so the script must
# still name the example and the subdirectories the game loads from. A rename of
# the example, or a dropped subdirectory, would ship an archive missing its scene
# or its assets, and the game would fail only at a player's first run.
foreach(_needle "examples/breakout" "foreach(_dir assets scenes)")
  string(FIND "${_package}" "${_needle}" _at)
  if(_at EQUAL -1)
    message(FATAL_ERROR "the packaging script no longer stages ${_needle}")
  endif()
endforeach()

# The launcher a player is told to run must be executable *in the archive*.
# `file(WRITE)` creates it 0644, the mode travels into the tarball, and a player
# who unpacks and runs the documented `./run.sh` gets "Permission denied". The
# smoke run does not catch this on its own if it invokes the binary directly, so
# assert the CHMOD is there.
if(NOT _package MATCHES "file\\(CHMOD \"\\$\\{FE_GAME_STAGE\\}/run\\.sh\"")
  message(FATAL_ERROR
    "the packaging script writes run.sh without making it executable; an "
    "unpacked tarball then fails on the documented './run.sh' with "
    "'Permission denied'.")
endif()
if(NOT _package MATCHES "FE_SMOKE_LAUNCH")
  message(FATAL_ERROR
    "the packaging smoke run no longer goes through the launcher; running the "
    "binary directly hides a launcher that ships non-executable.")
endif()

# The Windows artifact is a zip, and the game is played from it against the null
# backend so a headless runner can exercise the whole artifact (docs/SHIPPING_QUEUE.md,
# `windows-zip`). Both halves are decisions the script has to keep making: zip
# from the platform, and the null-backend flag reaching the game.
if(NOT _package MATCHES "set\\(FE_ARCHIVE_FORMAT \"zip\"\\)")
  message(FATAL_ERROR
    "the packaging script no longer selects the zip format for Windows; the "
    "artifact a player unzips would be a tarball.")
endif()
if(NOT _package MATCHES "FE_SMOKE_NULL_WINDOW")
  message(FATAL_ERROR
    "the packaging script no longer offers the null-window game smoke; the "
    "Windows artifact could then only be started, never played, on a runner with "
    "no GL context.")
endif()

# The Windows zip is self-contained, but *not* by linking the CRT statically: a
# gameplay module is a DLL that exchanges `std::` objects with the host, so the
# runtime must be shared (ADR-0061), and a static CRT hangs the module tests on
# Windows (ADR-0122). Self-containment comes from installing the redistributable
# runtime DLLs beside the binaries instead. Assert both halves, because each is a
# decision that a later change could quietly reverse: a static CRT reintroduces
# the hang, and dropping the runtime install reintroduces the clean-machine
# `VCRUNTIME140.dll` failure the zip exists to avoid.
file(READ "${SETTINGS_SCRIPT}" _settings)
if(_settings MATCHES "CMAKE_MSVC_RUNTIME_LIBRARY")
  message(FATAL_ERROR
    "the build links the MSVC runtime statically again; that gives a gameplay "
    "module and the host separate heaps, and the module tests hang on Windows "
    "(ADR-0122). Keep the runtime shared and bundle the redistributable DLLs.")
endif()
file(READ "${INSTALL_SCRIPT}" _install)
if(NOT _install MATCHES "fe_install_msvc_runtime")
  message(FATAL_ERROR
    "the install no longer carries the redistributable MSVC runtime; the Windows "
    "zip would then need the VC++ redistributable installed on a clean machine "
    "(ADR-0122).")
endif()
if(NOT _install MATCHES "InstallRequiredSystemLibraries")
  message(FATAL_ERROR
    "the MSVC runtime install no longer asks the toolchain which DLLs it needs; a "
    "hand-written list is the fragility ADR-0118 rejected.")
endif()
if(NOT _package MATCHES "FE_GAME_CRT_DLLS")
  message(FATAL_ERROR
    "the game archive no longer carries the MSVC runtime DLLs beside the game; "
    "the zip would fail to start on a machine with no VC++ redistributable "
    "(ADR-0122).")
endif()

# A downloadable build has to say which version it is (ADR-0094). The packaging
# smoke exports a real game with the prebuilt runtime, so it is where the
# manifest's version can be compared against the build tree's -- the end-to-end
# half of the queue item's "Done when". Assert the check is present, because a
# manifest that silently drops the version is exactly the failure it guards.
if(NOT _package MATCHES "engine_version")
  message(FATAL_ERROR
    "the packaging smoke no longer checks the export manifest's engine_version; a "
    "downloadable build must say which engine made it (ADR-0094).")
endif()

message(STATUS "release: every platform packages, the tag is checked, and the archives are compressed")

# ---------------------------------------------------------------------------
# The Windows zip is exercised on every push, not only at a tag.
#
# The release workflow produces the zip, but it only runs on a tag, so a break in
# the zip path would not be seen until a release was cut. The CI job packages the
# same artifact and plays it against the null backend, which is the only way to
# exercise the whole zip on a headless Windows runner (docs/SHIPPING_QUEUE.md,
# `windows-zip`). Assert the step exists and asks for the null-window smoke, or
# the Windows artifact is only ever started, never played.
# ---------------------------------------------------------------------------
file(READ "${CI_WORKFLOW}" _ci)
string(REPLACE "\r\n" "\n" _ci "${_ci}")
set(_ci_checks
    "packages the Windows zip in CI|package_release.cmake"
    "builds the Windows zip|-DFE_PLATFORM=windows"
    "plays the Windows game with no display|-DFE_SMOKE_NULL_WINDOW=ON"
    "turns the Windows game smoke on|-DFE_SMOKE_GAME=ON"
)
foreach(_check IN LISTS _ci_checks)
  string(REPLACE "|" ";" _parts "${_check}")
  list(GET _parts 0 _label)
  list(GET _parts 1 _needle)
  string(FIND "${_ci}" "${_needle}" _at)
  if(_at EQUAL -1)
    message(FATAL_ERROR "the CI workflow is missing '${_label}' (looked for: ${_needle})")
  endif()
endforeach()
message(STATUS "release: the Windows zip is packaged and played in CI")
