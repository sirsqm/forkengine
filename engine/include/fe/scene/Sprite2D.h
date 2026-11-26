// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Math.h"
#include "fe/core/Types.h"
#include "fe/resources/Resource.h"
#include "fe/scene/Component.h"

#include <memory>

namespace fe {

// A textured quad drawn in the 2D pass.
//
// The component holds a reference to a texture resource. Holding the
// shared_ptr is what keeps the texture loaded: the ResourceManager caches
// weakly, so a sprite that owns the last reference is also the reason the
// texture is not released. This is the concrete expression of the resource
// ownership rule described in ResourceManager.
class Sprite2D final : public Component {
public:
    static constexpr const char* kTypeName = "Sprite2D";

    [[nodiscard]] const char* type_name() const noexcept override { return kTypeName; }

    // Size of the quad in world units. Zero means "use the texture size", which
    // is resolved by the renderer at draw time.
    Vec2 size{0.0f, 0.0f};

    // Tint multiplied with the sampled texel.
    Vec4 color{1.0f, 1.0f, 1.0f, 1.0f};

    // Normalised anchor: the point of the quad that sits on the node's origin.
    // (0, 0) is the bottom-left corner, (1, 1) the top-right, so a character
    // stands on its feet with (0.5, 0). Values outside [0, 1] are allowed and
    // push the quad further off the node. The default centres the quad, which is
    // the behaviour a sprite had before the pivot existed.
    Vec2 pivot{0.5f, 0.5f};

    // UV sub-rectangle (u0, v0, u1, v1) for atlas and sprite-sheet use.
    Vec4 uv{0.0f, 0.0f, 1.0f, 1.0f};

    // Mirror the sampled sub-rectangle about its own centre. A character walking
    // left can reuse the one texture drawn walking right, and an author does not
    // have to hand-write the reversed UV rect. Extraction swaps the U endpoints
    // (or the V ones), so the quad keeps its position and size and only the
    // sampling is mirrored; the defaults are false, so an existing sprite's
    // output is unchanged.
    bool flip_x = false;
    bool flip_y = false;

    // Render order within the 2D pass. Higher values draw on top.
    i32 layer = 0;

    // The texture to draw. Null is allowed (the sprite draws nothing) so a
    // scene can be authored before its assets exist.
    std::shared_ptr<Resource> texture;

    [[nodiscard]] bool has_texture() const noexcept { return texture != nullptr; }
};

} // namespace fe
