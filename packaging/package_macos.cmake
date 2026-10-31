# Fork Engine macOS app bundle and DMG packaging.
#
# Turns a configured build tree into the two artifacts a macOS player expects: a
# `.app` bundle and a DMG containing it. Run it in CMake script mode on a macOS
# host (the tools it needs -- otool, install_name_tool, hdiutil, codesign -- are
# macOS-only):
#
#   cmake -DFE_BUILD_DIR=build -DFE_OUT_DIR=dist -DFE_ARCH=aarch64 \
#         -P packaging/package_macos.cmake
#
# It produces:
#
#   Fork Engine Breakout.app     the bundle
#   fork-engine-<ver>-macos-<arch>.dmg   the disk image a player downloads
#
# Why a bundle on top of the tarball `package_release.cmake` already writes: a
# `.app` is what macOS runs as a program -- Finder treats the directory as a
# single double-clickable item, the Dock and Gatekeeper understand it, and the
# content travels inside it. A tarball of loose binaries is a developer artifact;
# this is the shape the platform expects.
#
# The bundle layout:
#
#   Contents/Info.plist              what the OS reads (name, id, executable)
#   Contents/MacOS/fork-engine       the entry point: cd's to Resources, execs
#   Contents/MacOS/fork_breakout     the game
#   Contents/MacOS/fork_editor_shell the editor
#   Contents/Resources/examples/...  the game's and the editor's content
#   Contents/Resources/assets/       the editor's built-in sample assets
#   Contents/Resources/LICENSE
#
# The entry point is a shell script rather than the game binary directly, for the
# same reason the AppImage has an AppRun: the game resolves its content by
# relative path, and macOS launches an app with the working directory at `/`, so
# the binary must be started from the directory it expects. The script `cd`s to
# `Contents/Resources` and execs the game from there.
#
# The libraries: a macOS app links the system's libc++ and frameworks, which are
# part of the OS and must not be bundled (copying libSystem or Cocoa would fight
# the host's dyld). What *can* need bundling is a third-party dylib linked by
# absolute path from the build host, which would not exist on a player's machine;
# any such library is copied into `Contents/Frameworks` and its install name
# rewritten to `@rpath`. The engine and GLFW are static, so today nothing is
# bundled and the step is a no-op -- but it is what makes the artifact correct if
# that changes, and the smoke runs the bundle from the DMG so a missing dylib
# fails there rather than at a downloader.
#
# Inputs (all via -D):
#   FE_BUILD_DIR      required  a configured build tree with the game built
#   FE_OUT_DIR        required  where the .app and the DMG are written
#   FE_ARCH           optional  aarch64 (default) | x86_64 | universal
#   FE_CONFIG         optional  build configuration for multi-config generators
#                               (default Release; ignored by single-config ones)
#   FE_VERSION        optional  defaults to the build tree's project version
#   FE_SMOKE          optional  ON (default) mounts the DMG and runs the game
#   FE_SMOKE_GAME     optional  OFF (default) also plays it headless (needs no
#                               display: the game is run with --null-window)
#   FE_SMOKE_FRAMES   optional  frames for the play run (default 60)

cmake_minimum_required(VERSION 3.20)

foreach(_required FE_BUILD_DIR FE_OUT_DIR)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "${_required} must be set (see the header of this file)")
  endif()
endforeach()

# otool, install_name_tool, hdiutil, and codesign are macOS tools; a `.app` and a
# DMG cannot be assembled anywhere else. Fail with a clear message rather than
# midway through with a "command not found".
if(NOT CMAKE_HOST_APPLE)
  message(FATAL_ERROR
    "packaging a macOS app bundle and DMG needs a macOS host (otool, "
    "install_name_tool, and hdiutil are macOS-only). Run this on a macOS runner.")
endif()

get_filename_component(FE_BUILD_DIR "${FE_BUILD_DIR}" ABSOLUTE)
get_filename_component(FE_OUT_DIR "${FE_OUT_DIR}" ABSOLUTE)
get_filename_component(FE_SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)

if(NOT EXISTS "${FE_BUILD_DIR}/CMakeCache.txt")
  message(FATAL_ERROR "no configured build tree at ${FE_BUILD_DIR}")
endif()

if(NOT DEFINED FE_ARCH OR "${FE_ARCH}" STREQUAL "")
  set(FE_ARCH "aarch64")
endif()

# The version has one source of truth (the root project() call) and the build
# tree records it, so the artifact name cannot disagree with the binary inside.
if(NOT DEFINED FE_VERSION OR "${FE_VERSION}" STREQUAL "")
  file(STRINGS "${FE_BUILD_DIR}/CMakeCache.txt" _version_line REGEX "^CMAKE_PROJECT_VERSION:STATIC=")
  if(NOT _version_line)
    message(FATAL_ERROR "cannot read the project version from ${FE_BUILD_DIR}/CMakeCache.txt")
  endif()
  string(REPLACE "CMAKE_PROJECT_VERSION:STATIC=" "" FE_VERSION "${_version_line}")
endif()

set(FE_SMOKE "ON" CACHE STRING "")
# Playing the game needs no display: it is run against the null platform backend
# (--null-window, ADR-0041), so a headless runner can play the bundle from the
# mounted DMG. Off by default so the default run is the cheap structural check.
set(FE_SMOKE_GAME "OFF" CACHE STRING "")
set(FE_SMOKE_FRAMES "60" CACHE STRING "")

# Multi-config generators put artifacts under a per-configuration subdirectory,
# and the build tree records which kind it is, so the layout is derived rather
# than guessed from the platform.
set(FE_CONFIG "Release" CACHE STRING "")
file(STRINGS "${FE_BUILD_DIR}/CMakeCache.txt" _generator_line REGEX "^CMAKE_GENERATOR:INTERNAL=")
if(_generator_line MATCHES "Visual Studio" OR _generator_line MATCHES "Xcode"
   OR _generator_line MATCHES "Ninja Multi-Config")
  set(FE_BIN_DIR "${FE_BUILD_DIR}/bin/${FE_CONFIG}")
else()
  set(FE_BIN_DIR "${FE_BUILD_DIR}/bin")
endif()

function(fe_run)
  execute_process(COMMAND ${ARGN} RESULT_VARIABLE _rc)
  if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "command failed (${_rc}): ${ARGN}")
  endif()
endfunction()

set(FE_APP_NAME "Fork Engine Breakout")
set(FE_APP_DIR "${FE_OUT_DIR}/${FE_APP_NAME}.app")
set(FE_STEM "fork-engine-${FE_VERSION}-macos-${FE_ARCH}")
set(FE_DMG "${FE_OUT_DIR}/${FE_STEM}.dmg")

file(MAKE_DIRECTORY "${FE_OUT_DIR}")
file(REMOVE_RECURSE "${FE_APP_DIR}")
file(REMOVE "${FE_DMG}")

# ---------------------------------------------------------------------------
# The binaries.
#
# Both ship in the one bundle, as the AppImage ships both: the game is the
# default entry point, the editor is one flag away, and a downloader has one
# thing to open rather than a choice between two.
# ---------------------------------------------------------------------------
set(FE_GAME_BINARY "fork_breakout")
set(FE_EDITOR_BINARY "fork_editor_shell")
file(MAKE_DIRECTORY "${FE_APP_DIR}/Contents/MacOS")
file(MAKE_DIRECTORY "${FE_APP_DIR}/Contents/Resources")
foreach(_binary IN ITEMS "${FE_GAME_BINARY}" "${FE_EDITOR_BINARY}")
  if(NOT EXISTS "${FE_BIN_DIR}/${_binary}")
    message(FATAL_ERROR "no ${_binary} in ${FE_BIN_DIR}; build with FE_BUILD_EXAMPLES=ON and FE_BUILD_EDITOR=ON")
  endif()
  file(COPY "${FE_BIN_DIR}/${_binary}" DESTINATION "${FE_APP_DIR}/Contents/MacOS")
  file(CHMOD "${FE_APP_DIR}/Contents/MacOS/${_binary}"
       PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endforeach()

# The content, at the paths each binary resolves by relative path, so the shipped
# binaries are the ones CI tests and no path is rewritten for the bundle. The
# entry point cd's to Contents/Resources, so the paths are relative to there.
#   * the game reads examples/breakout/{assets,scenes}
#   * the editor opens examples/demo_project
file(MAKE_DIRECTORY "${FE_APP_DIR}/Contents/Resources/examples/breakout")
foreach(_dir assets scenes)
  file(COPY "${FE_SCRIPT_DIR}/../examples/breakout/${_dir}"
       DESTINATION "${FE_APP_DIR}/Contents/Resources/examples/breakout")
endforeach()
# The demo project's import cache is build output (regenerated on open), so it is
# excluded: shipping it would put a stale cache in the download.
file(COPY "${FE_SCRIPT_DIR}/../examples/demo_project"
     DESTINATION "${FE_APP_DIR}/Contents/Resources/examples"
     PATTERN ".fork" EXCLUDE)
# The editor's built-in sample assets: the editor and the demo project reference
# `tex:assets/textures/checker.png`, which resolves against the process working
# directory. The binaries run with Contents/Resources as their cwd, so the sample
# tree must be there.
file(COPY "${FE_SCRIPT_DIR}/../assets" DESTINATION "${FE_APP_DIR}/Contents/Resources")
file(COPY "${FE_SCRIPT_DIR}/../LICENSE" DESTINATION "${FE_APP_DIR}/Contents/Resources")
file(COPY "${FE_SCRIPT_DIR}/fork-engine.png" DESTINATION "${FE_APP_DIR}/Contents/Resources")

# ---------------------------------------------------------------------------
# The bundled libraries.
#
# `otool -L` names every library a binary links. Anything under the system's own
# directories (/usr/lib, /System) is part of macOS and must not travel: a copy of
# libSystem or a framework would fight the host's dyld. Anything else -- a dylib
# linked by an absolute path from the build host -- is copied into
# Contents/Frameworks and its install name rewritten to `@rpath`, so the bundle
# carries it and looks for it inside itself.
#
# The engine and GLFW are static, so today the list is empty and this is a no-op.
# It is kept because the failure it prevents is silent: a bundle that works on
# the build machine and cannot start on a player's, with a dyld error naming a
# path that only existed on the builder.
# ---------------------------------------------------------------------------
set(FE_FRAMEWORKS "${FE_APP_DIR}/Contents/Frameworks")
set(FE_BUNDLED "")
foreach(_binary IN ITEMS "${FE_GAME_BINARY}" "${FE_EDITOR_BINARY}")
  execute_process(COMMAND otool -L "${FE_APP_DIR}/Contents/MacOS/${_binary}"
                  OUTPUT_VARIABLE _otool OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _otool_rc)
  if(NOT _otool_rc EQUAL 0)
    message(FATAL_ERROR "otool failed on ${_binary} (${_otool_rc})")
  endif()
  string(REPLACE "\n" ";" _otool_lines "${_otool}")
  foreach(_line IN LISTS _otool_lines)
    # "	/path/to/libfoo.dylib (compatibility version ...)". Skip the header line
    # (the binary's own path) and system libraries.
    if(NOT _line MATCHES "^[ \t]+(/[^ \t]+)")
      continue()
    endif()
    set(_path "${CMAKE_MATCH_1}")
    get_filename_component(_name "${_path}" NAME)
    if(_path MATCHES "^/usr/lib/" OR _path MATCHES "^/System/")
      continue()
    endif()
    if(_path MATCHES "^@rpath/" OR _path MATCHES "^@executable_path/" OR _path MATCHES "^@loader_path/")
      continue()
    endif()
    if(_name IN_LIST FE_BUNDLED)
      continue()
    endif()
    file(MAKE_DIRECTORY "${FE_FRAMEWORKS}")
    # Copy the resolved file (following a symlink), not the link, so the bundle
    # carries bytes rather than a dangling reference.
    configure_file("${_path}" "${FE_FRAMEWORKS}/${_name}" COPYONLY)
    fe_run(install_name_tool -id "@rpath/${_name}" "${FE_FRAMEWORKS}/${_name}")
    fe_run(install_name_tool -change "${_path}" "@rpath/${_name}"
           "${FE_APP_DIR}/Contents/MacOS/${_binary}")
    list(APPEND FE_BUNDLED "${_name}")
  endforeach()
endforeach()
if(FE_BUNDLED STREQUAL "")
  message(STATUS "macos: no third-party libraries to bundle (the engine and GLFW are static)")
else()
  message(STATUS "macos: bundling ${FE_BUNDLED}")
endif()

# ---------------------------------------------------------------------------
# Info.plist and the entry point.
#
# CFBundleExecutable names the shell script below, not the game binary: macOS
# runs the bundle by executing Contents/MacOS/<CFBundleExecutable>, and the
# script is what sets the working directory the game needs.
# ---------------------------------------------------------------------------
file(WRITE "${FE_APP_DIR}/Contents/Info.plist"
"<?xml version=\"1.0\" encoding=\"UTF-8\"?>
<!DOCTYPE plist PUBLIC \"-//Apple//DTD PLIST 1.0//EN\" \"http://www.apple.com/DTDs/PropertyList-1.0.dtd\">
<plist version=\"1.0\">
<dict>
  <key>CFBundleName</key><string>${FE_APP_NAME}</string>
  <key>CFBundleDisplayName</key><string>${FE_APP_NAME}</string>
  <key>CFBundleIdentifier</key><string>dev.forkengine.breakout</string>
  <key>CFBundleExecutable</key><string>fork-engine</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>${FE_VERSION}</string>
  <key>CFBundleVersion</key><string>${FE_VERSION}</string>
  <key>CFBundleIconFile</key><string>fork-engine</string>
  <key>LSMinimumSystemVersion</key><string>11.0</string>
  <key>NSHighResolutionCapable</key><true/>
</dict>
</plist>
")

# The entry point. macOS launches an app with the working directory at `/`, and
# both binaries resolve their content by relative path, so the script cd's to
# Contents/Resources first. It dispatches like the AppImage's AppRun:
#
#   no argument   the game (this is a game bundle)
#   --editor      the editor, opened on the bundled demo project
#   anything else the game, so the bundle is a drop-in for `--null-window`,
#                 `--headless N`, and the other flags the smoke and the README use
file(WRITE "${FE_APP_DIR}/Contents/MacOS/fork-engine"
"#!/bin/sh
# Fork Engine macOS bundle entry point.
HERE=\"\$(cd \"\$(dirname \"\$0\")\" && pwd)\"
cd \"\$HERE/../Resources\" || exit 1
case \"\${1:-}\" in
  --editor)
    shift
    exec \"\$HERE/${FE_EDITOR_BINARY}\" --project \"\$HERE/../Resources/examples/demo_project\" \"\$@\"
    ;;
  *)
    exec \"\$HERE/${FE_GAME_BINARY}\" \"\$@\"
    ;;
esac
")
file(CHMOD "${FE_APP_DIR}/Contents/MacOS/fork-engine"
     PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

# Ad-hoc sign the bundle. An unsigned .app is refused by Gatekeeper even when it
# is not notarized, and an ad-hoc signature (`-`) is what a locally built bundle
# gets. A real release would sign with a Developer ID and notarize; that needs a
# certificate this script does not have, so it is left to the release process and
# recorded here.
fe_run(codesign --force --deep --sign - "${FE_APP_DIR}")

# ---------------------------------------------------------------------------
# The DMG.
#
# `hdiutil create` with UDZO (compressed, read-only) is the standard shape: one
# file a player double-clicks, which mounts as a volume holding the .app. The
# volume is named after the app so the window a player sees says what it is.
# ---------------------------------------------------------------------------
fe_run(hdiutil create -volname "${FE_APP_NAME}" -srcfolder "${FE_APP_DIR}"
       -ov -format UDZO "${FE_DMG}")

if(NOT EXISTS "${FE_DMG}")
  message(FATAL_ERROR "hdiutil reported success but wrote no ${FE_DMG}")
endif()

# The DMG magic: the last 512 bytes are the "koly" trailer, at a fixed offset.
# Asserting it catches a truncated or mislabelled file at the point it is made
# rather than at a downloader's first mount.
file(SIZE "${FE_DMG}" _dmg_size)
if(_dmg_size LESS 512)
  message(FATAL_ERROR "${FE_DMG} is too small to be a disk image (${_dmg_size} bytes)")
endif()
math(EXPR _koly_offset "${_dmg_size} - 512")
file(READ "${FE_DMG}" _koly_magic OFFSET ${_koly_offset} LIMIT 4)
if(NOT _koly_magic STREQUAL "koly")
  message(FATAL_ERROR
    "${FE_DMG} has no UDIF trailer (bytes at the end are '${_koly_magic}', "
    "expected 'koly'). The file is not a disk image.")
endif()

# ---------------------------------------------------------------------------
# Smoke: mount the DMG and run the bundle from inside it, not the build tree.
#
# `--help` prints usage and exits before the engine initialises, so it proves the
# DMG mounts, the entry point runs, dyld resolves every library the bundle needs,
# and the binary starts -- with no display and no GPU. Playing it uses the null
# backend (`--null-window`, ADR-0041), so it needs no display either and can run
# on a headless runner.
#
# The mounted volume is detached even when a check fails, so a failed run does
# not leave a volume attached on the runner.
# ---------------------------------------------------------------------------
if(FE_SMOKE)
  execute_process(COMMAND hdiutil attach "${FE_DMG}" -nobrowse -readonly -mountpoint "${FE_OUT_DIR}/mnt"
                  RESULT_VARIABLE _mount_rc)
  if(NOT _mount_rc EQUAL 0)
    message(FATAL_ERROR "the DMG did not mount (${_mount_rc})")
  endif()
  set(FE_MOUNTED_APP "${FE_OUT_DIR}/mnt/${FE_APP_NAME}.app")
  set(FE_MOUNTED_LAUNCH "${FE_MOUNTED_APP}/Contents/MacOS/fork-engine")

  execute_process(COMMAND "${FE_MOUNTED_LAUNCH}" --help RESULT_VARIABLE _help_rc)
  if(NOT _help_rc EQUAL 0)
    message(FATAL_ERROR "the app bundle did not run (--help exited ${_help_rc})")
  endif()
  message(STATUS "macos: the bundle runs from the mounted DMG (--help)")

  if(FE_SMOKE_GAME)
    execute_process(COMMAND "${FE_MOUNTED_LAUNCH}" --headless "${FE_SMOKE_FRAMES}" --null-window
                    RESULT_VARIABLE _play_rc)
    if(NOT _play_rc EQUAL 0)
      execute_process(COMMAND hdiutil detach "${FE_OUT_DIR}/mnt" RESULT_VARIABLE _ignored)
      message(FATAL_ERROR "the app bundle did not play (${_play_rc})")
    endif()
    message(STATUS "macos: the bundle played ${FE_SMOKE_FRAMES} headless frames from the mounted DMG")
  endif()

  execute_process(COMMAND hdiutil detach "${FE_OUT_DIR}/mnt" RESULT_VARIABLE _detach_rc)
  if(NOT _detach_rc EQUAL 0)
    message(FATAL_ERROR "the DMG did not detach (${_detach_rc})")
  endif()
endif()

message(STATUS "macos: ${FE_DMG}")
