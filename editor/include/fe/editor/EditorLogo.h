// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include <memory>

namespace fe {

namespace rhi {
class Device;
class Texture;
} // namespace rhi

namespace editor {

// Uploads the engine's own mark (the fork emblem) as a small GPU texture, for
// the editor to draw in its menu bar.
//
// The bytes are the same 1-bit coverage mask the window icon is expanded from
// (`fe::WindowIcon`, generated from `branding/forkengine-logo-white.svg` by
// tools/gen_engine_icon.py), so the mark in the menu bar is the mark in the
// title bar and the AppImage icon -- one geometry, three places. It is uploaded
// white and tinted at draw time with the scheme's text colour, so it reads on
// both the light and the dark theme without a second asset.
//
// Returns null when the device refuses the upload (a null RHI device, or a
// headless run); the editor then draws its name with no mark rather than
// failing, which is what a run with no graphics context should look like.
[[nodiscard]] std::shared_ptr<rhi::Texture> create_logo_texture(rhi::Device& device);

} // namespace editor
} // namespace fe
