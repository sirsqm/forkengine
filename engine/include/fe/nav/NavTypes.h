// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

namespace fe::reflect {

// Registers the nav module's component types (NavAgent2D, NavAgent3D) with the
// reflection registry.
//
// Kept separate from register_builtin_types() for the same reason
// register_physics_types() is: the reflection layer does not depend on the nav
// module. The engine calls it during startup; a tool or test that uses an agent
// without an Engine calls it directly. Idempotent, like the built-in
// registration.
void register_nav_types();

} // namespace fe::reflect
