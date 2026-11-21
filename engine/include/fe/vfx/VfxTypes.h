// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

namespace fe::reflect {

// Registers the visual-effects component types (ParticleEmitter3D) with the
// reflection registry.
//
// Kept separate from register_scene3d_types() for the same reason that one is
// separate from register_builtin_types(): the reflection layer must not have to
// depend on the vfx module, and a tool that builds a scene without an Engine
// calls this directly. The engine calls it during startup. Idempotent.
void register_vfx_types();

} // namespace fe::reflect
