# Fork Engine install rules.
#
# One place decides where an installed tree puts things, so the engine, the
# editor, and the tools cannot disagree. GNUInstallDirs supplies the platform's
# directories (bin, lib, include, share), which is what makes `cmake --install`
# produce a layout a packager can consume without per-platform special cases.
#
# What is installed: the engine library and its public headers, the editor
# shell, the shipping tools (fork_pack, fork_player, fork_export), the runtime
# shared libraries an installed binary needs, and the license/readme. What is
# deliberately not installed: the examples, the unit tests, and the benchmark -
# they are development artifacts, and a shipped tree should not carry them.
#
# There is no `find_package(ForkEngine)` config yet. An installed tree is
# runnable, not yet linkable from another CMake project; adding the export set
# and the config file is a separate, larger change (it has to name the fetched
# dependencies, which are not installed). This is recorded rather than
# half-done, so a consumer is not misled by a config that does not work.

include_guard(GLOBAL)

include(GNUInstallDirs)

# On Windows a shared library must sit beside the executable that loads it, so
# the DLL-search rule wins over the tidy `lib` layout. Everywhere else the
# library directory is `lib` (or `lib64`). This is the one place the choice is
# made.
if(WIN32)
  set(FE_INSTALL_LIBDIR "${CMAKE_INSTALL_BINDIR}")
else()
  set(FE_INSTALL_LIBDIR "${CMAKE_INSTALL_LIBDIR}")
endif()

# Installs `target`, putting each artifact where its platform expects it. A
# target that does not produce an artifact of a given kind simply does not
# install one, so a static library, a shared library, and an executable can all
# be passed here without a type check.
function(fe_install_target target)
  if(NOT FE_INSTALL)
    return()
  endif()

  install(
    TARGETS ${target}
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    LIBRARY DESTINATION "${FE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${FE_INSTALL_LIBDIR}"
  )

  # A relocated tree should run without LD_LIBRARY_PATH. $ORIGIN is the
  # executable's directory, so `../lib` finds the shared libraries beside it;
  # this is a no-op for the static engine but matters for anything shared.
  if(FE_INSTALL_RPATH AND NOT WIN32 AND NOT APPLE)
    set_target_properties(
      ${target}
      PROPERTIES INSTALL_RPATH "$ORIGIN/../${FE_INSTALL_LIBDIR}"
                 INSTALL_RPATH_USE_LINK_PATH TRUE
    )
  endif()
endfunction()

# Installs a header tree under the include root, preserving the tree. A caller
# passes the directory whose *contents* belong at the include root, so
# `engine/include/fe` installs as `include/fe/...`, not `include/fe/fe/...`.
function(fe_install_header_tree dir)
  if(NOT FE_INSTALL)
    return()
  endif()
  install(
    DIRECTORY "${dir}"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
  )
endfunction()

# Installs the redistributable MSVC runtime DLLs beside the executables, so an
# installed tree -- and the Windows zip staged from it -- runs on a machine with
# no Visual C++ redistributable installed (ADR-0122). This is the counterpart of
# leaving the runtime shared (`/MD`): the engine must not link the CRT
# statically, because a gameplay module is a DLL that exchanges `std::` objects
# with the host (ADR-0061), but the download still has to be self-contained.
#
# The list is not hand-written. `InstallRequiredSystemLibraries` asks the
# toolchain for the exact DLLs this compiler needs and which build type needs
# them, which is the fragility a hand-maintained list has and ADR-0118 rejected
# it for. The UCRT (`ucrtbase.dll` and its api-ms-win-crt-* forwarders) is
# included too, because app-local deployment needs it on a machine that has no
# Universal CRT, which is the same clean-machine case.
#
# It installs the *release* runtime, which is what a download is built from; a
# Debug tree needs the non-redistributable debug CRT, so it is not a shipping
# configuration and this deliberately does not try to make it one.
function(fe_install_msvc_runtime)
  if(NOT FE_INSTALL OR NOT MSVC)
    return()
  endif()
  set(CMAKE_INSTALL_UCRT_LIBRARIES TRUE)
  set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION "${CMAKE_INSTALL_BINDIR}")
  include(InstallRequiredSystemLibraries)
endfunction()
