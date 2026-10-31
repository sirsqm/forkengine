# Test for the macOS app bundle and DMG packaging path, run by ctest in script
# mode.
#
# The bundle is built only on a macOS release runner, so a mistake in it is
# invisible to the desktop suite -- a green build on Linux says nothing about
# whether the artifact still carries the content the game loads, still runs the
# bundle from inside the DMG, or still refuses to bundle the OS's own libraries.
# Each of those failures produces a bundle that works on the build machine and
# fails on a player's, which is the worst kind of packaging bug: the person who
# downloaded it finds it.
#
# Like the other guards (test_appimage_packaging.cmake, test_release_workflow.cmake)
# this reads the script as text and asserts the decisions it has to make, because
# the alternative is running a macOS-only packaging step on every push.
#
# Run by tests/CMakeLists.txt:
#   cmake -DMACOS_SCRIPT=<path> -DRELEASE_WORKFLOW=<path>
#         -P cmake/tests/test_macos_packaging.cmake

foreach(_required MACOS_SCRIPT RELEASE_WORKFLOW)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "${_required} must be set (see the header of this file)")
  endif()
  if(NOT EXISTS "${${_required}}")
    message(FATAL_ERROR "${_required} does not exist: ${${_required}}")
  endif()
endforeach()

file(READ "${MACOS_SCRIPT}" _script)
string(REPLACE "\r\n" "\n" _script "${_script}")

# check: <label>|<literal substring the script must contain>
set(_checks
    # A bundle and a DMG cannot be assembled off macOS; failing early with a
    # clear message beats a "command not found" halfway through.
    "refuses to run off macOS|NOT CMAKE_HOST_APPLE"
    # The artifact is named from the version and the architecture.
    "names the DMG from version and arch|fork-engine-\${FE_VERSION}-macos-\${FE_ARCH}"
    # The bundle is what macOS runs; it is a directory with a Contents/ layout.
    "builds an app bundle|\${FE_APP_NAME}.app"
    "writes Info.plist|Contents/Info.plist"
    "names the executable in the plist|CFBundleExecutable"
    # The game and the content it loads by relative path.
    "packages the game|fork_breakout"
    "packages the editor|fork_editor_shell"
    "stages the example content|examples/breakout/\${_dir}"
    "stages the assets and the scene|foreach(_dir assets scenes)"
    "carries the license|LICENSE"
    # The entry point cd's to the content root, or the game resolves nothing.
    "writes the entry point|Contents/MacOS/fork-engine"
    "cds to the content root|cd \\\"\\\$HERE/../Resources\\\""
    # The library decision: the OS's own libraries and frameworks stay on the
    # host; a third-party dylib from the build host travels and is rewritten.
    "skips the system libraries|/usr/lib/"
    "skips the system frameworks|/System/"
    "rewrites an install name|install_name_tool"
    "uses @rpath for a bundled dylib|@rpath/"
    # The bundle is ad-hoc signed so Gatekeeper does not refuse it outright.
    "ad-hoc signs the bundle|codesign"
    # The DMG is the compressed read-only shape a player double-clicks.
    "writes a compressed disk image|-format UDZO"
    # The artifact is checked to be a disk image, not a truncated file.
    "asserts the UDIF trailer|koly"
    # The smoke runs the bundle from inside the mounted DMG, not the build tree,
    # and detaches the volume so a failed run leaves nothing attached.
    "mounts the DMG|hdiutil attach"
    "runs the bundle from the DMG|the app bundle did not run"
    "can play the bundle with no display|--null-window"
    "detaches the DMG|hdiutil detach"
)

foreach(_check IN LISTS _checks)
  string(REPLACE "|" ";" _parts "${_check}")
  list(GET _parts 0 _label)
  list(GET _parts 1 _needle)
  string(FIND "${_script}" "${_needle}" _at)
  if(_at EQUAL -1)
    message(FATAL_ERROR "the macOS script is missing '${_label}' (looked for: ${_needle})")
  endif()
  message(STATUS "  ${_label}: ok")
endforeach()

# A bundled dylib must be the *resolved contents*, not a symlink: copying a
# versioned dylib as a symlink leaves a dangling link and dyld falls back to a
# path that only existed on the builder. configure_file follows the symlink;
# file(COPY) does not.
set(_needle "configure_file(\"\${_path}\"")
string(FIND "${_script}" "${_needle}" _at)
if(_at EQUAL -1)
  message(FATAL_ERROR
    "the macOS script no longer copies a bundled dylib with configure_file. "
    "file(COPY) preserves a symlinked name as a dangling symlink, so dyld falls "
    "back to the build host's path and the bundle is a no-op.")
endif()

# The game must be playable with no display, so the smoke passes --null-window
# rather than depending on a window server a headless runner has not got.
if(NOT _script MATCHES "--null-window")
  message(FATAL_ERROR
    "the macOS smoke no longer plays the bundle against the null backend; a "
    "headless runner then cannot play the artifact it just built.")
endif()

# ---------------------------------------------------------------------------
# The wiring: the release must build the DMG, and must collect it when it
# assembles the published assets. A DMG that is built, uploaded as an artifact,
# and then never collected by the flatten step is not a download.
# ---------------------------------------------------------------------------
file(READ "${RELEASE_WORKFLOW}" _release)
string(REPLACE "\r\n" "\n" _release "${_release}")

if(NOT _release MATCHES "package_macos.cmake")
  message(FATAL_ERROR "the release workflow no longer builds the macOS bundle and DMG")
endif()
if(NOT _release MATCHES "name '\\*\\.dmg'")
  message(FATAL_ERROR
    "the release no longer collects '*.dmg' when it assembles the published "
    "assets, so the DMG would be built but never downloadable.")
endif()
# The macOS job must actually turn the play smoke on, or the bundle is only ever
# started and the interesting half (does the pack load, does the scene step) is
# never exercised on the platform it ships to.
if(NOT _release MATCHES "smoke_null_window: \"ON\"")
  message(FATAL_ERROR
    "the macOS release job no longer plays the packaged game with the null "
    "backend, so the bundle from the DMG is never actually played.")
endif()

message(STATUS "macos: the bundle and DMG carry the game, the OS libraries stay on the host, and the release builds it")
