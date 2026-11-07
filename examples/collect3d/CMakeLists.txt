# Fork Engine collect3d example — the M11 vertical slice (a finished 3D game).
#
# A complete, playable third-person collect-and-clear game built against the
# engine's public headers only, exactly as an external project would build it. It
# is the 3D counterpart of the M2.5 breakout slice: one artifact that exercises
# the scene graph, resources, physics3d, the built-in gameplay behaviours
# (character controller, follow camera), audio, serialization, input, the UI, and
# the systems layer together, with a goal, a score, a HUD, a menu, and a save.
#
# The rules (CollectState) and the gameplay systems are ordinary sources, not a
# separate library: the unit tests compile them straight from here by path, so the
# tests do not depend on FE_BUILD_EXAMPLES being on.

add_executable(fork_collect3d
  src/main.cpp
  src/slice_state.cpp
  src/slice_scene.cpp
  src/slice_systems.cpp
)
target_link_libraries(fork_collect3d PRIVATE FE::engine fe_warnings)
target_include_directories(fork_collect3d PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(fork_collect3d PRIVATE cxx_std_20)

# The game loads assets and its scene by relative path; run it from the
# repository root, exactly like the sandbox and breakout.
set_target_properties(fork_collect3d PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# --- Web target ---------------------------------------------------------------
#
# The slice ships for the browser as well as the desktop, which is the whole
# point of the engine-shipped gameplay behaviours (a browser has no dynamic
# loader). Two things are needed beyond a plain wasm link:
#
#   * the game reads its scene, font and cue by path, and a wasm build has no
#     filesystem, so those files are preloaded into the virtual one at exactly
#     the paths the game already uses;
#   * a page to host it. `--shell-file` supplies one that provides the #canvas
#     the backend renders into.
#
# The output keeps the target's name (`collect3d.html`/`collect3d.js`) rather
# than the generic `index`: the build directory holds more than one web game, so
# a shared `index` name would have the games overwrite each other's entry point
# and wasm module in `bin/`, and a full build would leave only the last one
# linked.
fe_platform_is_web(FE_COLLECT3D_WEB)
if(FE_COLLECT3D_WEB)
  target_link_options(
    fork_collect3d
    PRIVATE
      "-sMIN_WEBGL_VERSION=2"
      "-sMAX_WEBGL_VERSION=2"
      "-sALLOW_MEMORY_GROWTH=1"
      # The engine's startup path (scene load, reflection registration) overflows
      # Emscripten's 64 KiB default stack. See docs/adr/0073.
      "-sSTACK_SIZE=8388608"
      "-sENVIRONMENT=web"
      # main() installs the frame loop and returns to the page; the runtime must
      # not tear the page down when a run ends.
      "-sEXIT_RUNTIME=0"
      "--preload-file=${CMAKE_SOURCE_DIR}/examples/collect3d/assets@examples/collect3d/assets"
      "--preload-file=${CMAKE_SOURCE_DIR}/examples/collect3d/scenes@examples/collect3d/scenes"
      "--shell-file=${CMAKE_CURRENT_SOURCE_DIR}/web/shell.html"
  )
  # The toolchain sets CMAKE_EXECUTABLE_SUFFIX to ".js"; this target overrides it
  # so the shell page is used and the output is a `.html` entry point with a
  # matching `.js`/`.wasm` module, named after the target so the two web games in
  # `bin/` cannot collide.
  set_target_properties(fork_collect3d PROPERTIES OUTPUT_NAME "collect3d" SUFFIX ".html")
endif()
