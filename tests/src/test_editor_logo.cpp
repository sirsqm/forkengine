// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// Tests for the editor's logo texture (the mark drawn at the head of the menu
// bar). The upload runs against the null RHI device, so the test proves the
// descriptor and the pixel handoff with no GL context; what the mark *looks*
// like is the window icon's geometry, already pinned by test_window_icon.

#include "fe/editor/EditorLogo.h"

#include "fe/platform/WindowIcon.h"
#include "fe/rhi/Device.h"
#include "fe/rhi/Resources.h"

#include <doctest/doctest.h>

#include <memory>

using fe::editor::create_logo_texture;
using fe::rhi::create_null_device;

TEST_CASE("the logo texture uploads the engine mark") {
    auto device = create_null_device();
    REQUIRE_FALSE(device.error());

    auto texture = create_logo_texture(*device.value());
    REQUIRE((texture != nullptr));
    // The mark is the same square the window icon expands to, so the menu bar
    // scales it from one clean source rather than a separate asset.
    CHECK(texture->width() == fe::WindowIcon::kEdge);
    CHECK(texture->height() == fe::WindowIcon::kEdge);
    CHECK(texture->name() == "editor:logo");
}
