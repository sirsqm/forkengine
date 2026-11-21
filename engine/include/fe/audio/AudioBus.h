// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Types.h"

#include <string>
#include <string_view>
#include <vector>

namespace fe::audio {

// A named mixing group with its own gain, mute, and solo.
//
// A bus is the mixing-console idea applied to the engine: instead of every voice
// summing straight into the master, a voice can route to a named bus, and the
// bus's gain scales every source on it at once. Pulling "Music" down to 0.3 is
// then one number, not an edit to every source; muting or soloing the bus is a
// switch that leaves that number intact.
//
// The value type is deliberately plain (a name, a gain, and two flags) so it is a pure
// value like AudioClip's format: the pure mixer reads a flat array of these and
// never allocates. The AudioSystem owns the table and matches sources to buses
// by name; the mixer only sees the resolved bus index on each voice, so the
// mix path stays deterministic and free of string lookups.
struct AudioBus {
    // The bus's name, matched against AudioSource::bus_name. The empty name is
    // reserved for the implicit master bus (see kMasterBusName).
    std::string name{};

    // Linear gain applied to every voice routed to this bus. 1 is unity, 0 is
    // silent. Not clamped: a scene may author a boost, exactly as it may on an
    // AudioSource.
    f32 gain = 1.0f;

    // Silences this bus without touching its authored `gain`, so a settings
    // menu's "Mute" toggle is a switch and unmuting restores exactly the level
    // the designer set. Off by default, so an existing bus mixes unchanged.
    bool mute = false;

    // Auditions this bus: while any bus is soloed, every bus that is not soloed
    // is silent, so a designer can hear one group in isolation. Independent of
    // `mute` and `gain` (a soloed bus still honours both), and off by default.
    bool solo = false;

    [[nodiscard]] bool is_master() const noexcept { return name.empty(); }
};

// The reserved name of the implicit master bus: every bus (and every source on
// no explicit bus) sums into it. A source that names this bus, or names no bus
// at all, is routed here.
inline constexpr std::string_view kMasterBusName = "";

} // namespace fe::audio
