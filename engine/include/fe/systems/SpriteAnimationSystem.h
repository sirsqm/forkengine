// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/scene/SpriteAnimator.h"
#include "fe/systems/SceneSchedule.h"
#include "fe/systems/System.h"

#include <vector>

namespace fe {

class Node;
class Sprite2D;

// Advances SpriteAnimator state and writes the current cell's UVs onto the
// node's Sprite2D.
//
// This is the first concrete system, and it exists to prove the execution model
// end to end rather than to be a feature. It shows the shape every system
// follows: gather what it processes into its own flat list, advance it, and
// hand the result to the subsystem that consumes it (here, the renderer, via
// Sprite2D::uv). It does not call into the renderer and the renderer does not
// know animation exists.
//
// Ordering: add this system before anything that reads the resulting UVs in the
// same frame. The schedule runs systems in the order they were added, so the
// ordering is explicit at the call site.
class SpriteAnimationSystem final : public System {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "SpriteAnimationSystem"; }

    void update(const FrameContext& context, Scene& scene) override;

    // Cells advanced during the most recent update. Exposed for tests and for
    // the editor's debug view; it is not a gameplay-facing value.
    [[nodiscard]] usize last_advanced_count() const noexcept { return advanced_; }

    // One-shot animators that reached their last cell during the most recent
    // update, in scene order. A looping animator is never reported, and a
    // finished animator is reported on the update that reaches the end only,
    // not on later updates that leave it parked there. This is how a one-shot
    // (an attack, a death, a pickup) drives what happens next without the game
    // polling UVs. The vector is rebuilt at the top of every update, so a
    // caller that keeps it past the frame copies it.
    [[nodiscard]] usize finished_count() const noexcept { return finished_.size(); }
    [[nodiscard]] const std::vector<Node*>& finished_nodes() const noexcept { return finished_; }

private:
    // One entry per animator-bearing sprite, so the inner loop walks a
    // contiguous array instead of the scene's pointer graph.
    struct Entry {
        Node* node;
        SpriteAnimator* animator;
        Sprite2D* sprite;
        f32 elapsed;   // fractional-frame accumulator carried between updates
        bool finished; // a one-shot already reported at its last cell
    };

    std::vector<Entry> entries_;
    std::vector<Node*> finished_;
    usize advanced_ = 0;
};

} // namespace fe
