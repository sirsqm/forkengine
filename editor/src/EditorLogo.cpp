// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/editor/EditorLogo.h"

#include "fe/platform/WindowIcon.h"
#include "fe/rhi/Device.h"
#include "fe/rhi/Resources.h"
#include "fe/rhi/Types.h"

namespace fe::editor {

std::shared_ptr<rhi::Texture> create_logo_texture(rhi::Device& device) {
    // The window icon is the mark already expanded to RGBA8 at a size the menu
    // bar scales from cleanly, so the editor reuses it rather than rasterising
    // the SVG a second way. White ink: the draw tints it per theme.
    const WindowIcon icon = WindowIcon::white();
    if (icon.empty()) {
        return nullptr;
    }

    rhi::TextureDesc desc;
    desc.name = "editor:logo";
    desc.width = icon.width();
    desc.height = icon.height();
    desc.format = rhi::PixelFormat::Rgba8;
    desc.min_filter = rhi::TextureFilter::Linear;
    desc.mag_filter = rhi::TextureFilter::Linear;
    desc.wrap_u = rhi::TextureWrap::ClampToEdge;
    desc.wrap_v = rhi::TextureWrap::ClampToEdge;
    desc.pixels = icon.pixels().data();

    auto created = device.create_texture(desc);
    if (!created) {
        return nullptr;
    }
    return created.value();
}

} // namespace fe::editor
