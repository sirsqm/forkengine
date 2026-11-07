// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "slice_state.h"

#include "fe/core/Types.h"
#include "fe/resources/Resource.h"

#include <memory>

namespace fe {
class Scene;
class ScriptHost;
} // namespace fe

namespace collect3d {

// The mesh and the font the arena draws with. Both are optional: a headless test
// builds the same arena with no assets, which is why the builder takes them as
// data rather than loading them itself. The colliders and gameplay are identical
// either way; only what is drawn changes.
struct SceneAssets {
    // A unit cube (one metre across, centred on the origin) shared by every box
    // in the arena. The builder sizes a box by scaling the node, so one mesh draws
    // the floor, the walls, the player and the gems. Missing is not fatal: the
    // builder falls back to an in-code box of the same half extent, so a headless
    // run has real geometry with no file on disk.
    std::shared_ptr<fe::Resource> box;
    // The HUD and menu font: the glyph atlas and metrics behind every label.
    // Missing is not fatal -- the arena is built anyway and the text simply does
    // not draw, which is what a headless run wants.
    std::shared_ptr<fe::Resource> font;
};

// Builds the canonical arena into `scene`: the floor and four walls, the player
// (a mesh, a collider and a gravity-free rigid body), the ring of collectibles,
// the sun and the camera, the pickup cue, the HUD, and the menu screens. It does
// not clear `scene` first, so the caller decides whether to append or replace.
//
// This is the single source of truth for the layout. The authored scene file
// (scenes/collect3d.fescene) is a serialization of exactly this, and the tests
// build it in memory; that is what keeps the file path, the fallback path, and
// the tested path from drifting apart.
void build_collect3d_scene(fe::Scene& scene, const SceneAssets& assets = {});

// Attaches the gameplay behaviours the arena's nodes carry: the third-person
// character controller on the player, the follow camera on the camera, and a spin
// on each collectible so the ring is legible as something to pick up. The host is
// what binds a behaviour to its node, so this is the shipped wiring a scene file
// would otherwise name; a test that builds the arena in memory calls it to get
// the same live behaviour a player gets.
//
// Register the behaviours first (`fe::player::register_builtin_behaviours`) so
// the host can construct them by name.
void attach_collect3d_behaviours(fe::ScriptHost& host, fe::Scene& scene);

} // namespace collect3d
