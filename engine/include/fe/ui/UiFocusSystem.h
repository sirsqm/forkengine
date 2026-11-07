// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/platform/InputActions.h"
#include "fe/systems/System.h"
#include "fe/ui/UiLayout.h"

#include <string_view>
#include <vector>

namespace fe::ui {

// Moves keyboard/gamepad focus between the focusable elements of a UI and
// confirms the focused one, so a menu works from a d-pad, a stick, or the arrow
// keys and not only from a pointer.
//
// This is the second half of the UI input seam. `UiInputSystem` turns a pointer
// into hover/press/click; this turns an action into `focused`/`activated`. Both
// write the same `UiInteraction` on the same elements, and both call the one
// `ui::layout`, so a button's focus ring and its pixels cannot disagree -- the
// same rule the pointer path follows (ADR-0086).
//
// Navigation is by geometry, not by an authored order. A press of "down" picks
// the focusable element below the current one (the nearest by a weighted
// distance that favours the requested direction), which is what a player expects
// from a menu laid out as a column or a grid, and it needs no extra authoring:
// the same tree that lays out and draws the buttons is the tree focus walks.
//
// What it does not do: decide what a confirmation means. It sets `activated` on
// the focused element and stops, exactly as `UiInputSystem` sets `clicked`. A
// menu system reads either and changes state, which is what keeps a button from
// being engine code.
//
// Ordering: add it after `UiInputSystem` (so both write the same frame's
// interactions) and before the systems that read them. It reads `FrameContext`
// actions, so a headless test drives it by binding actions on an `ActionMap` and
// calling `update`, the same seam every other input consumer is tested through.
class UiFocusSystem final : public System {
public:
    UiFocusSystem() = default;

    [[nodiscard]] std::string_view name() const noexcept override { return "ui/focus"; }

    void update(const FrameContext& context, Scene& scene) override;

    // The action handles to read. `kNone` (the default) means the direction is
    // unbound and the arrow-key fallback is used instead, so a caller that
    // declares no actions still gets a navigable menu. Confirm defaults to the
    // Enter/Space fallback.
    void set_navigation_actions(ActionMap::Id up, ActionMap::Id down, ActionMap::Id left, ActionMap::Id right) noexcept;
    void set_confirm_action(ActionMap::Id confirm) noexcept { confirm_ = confirm; }

    // The focused element's node, or nullptr when nothing is focused. Exposed so
    // a game can read the selection without walking the layout itself.
    [[nodiscard]] const Node* focused_node() const noexcept { return focused_node_; }

    // True when the frame activated the focused element. The same information is
    // on the element's `interaction.activated`, exposed here for a caller that
    // only wants the boolean.
    [[nodiscard]] bool activated_this_frame() const noexcept { return activated_this_frame_; }

    // Clears the focus and forgets the focused element. Call it when a menu is
    // torn down or the scene changes, so the next frame starts on the first
    // focusable element rather than on a stale one.
    void clear_focus() noexcept;

private:
    // The focusable elements of the layout, in draw order. A clipped-out element
    // never reaches the layout, so an element scrolled off a list is not a focus
    // stop, which is the behaviour a scrolling menu wants.
    [[nodiscard]] static std::vector<const UiResolved*> focusable(const UiLayoutResult& layout);

    // The index of `current` in `stops`, or -1.
    [[nodiscard]] static i32 index_of(const std::vector<const UiResolved*>& stops, const UiElement* current) noexcept;

    // The index of the stop nearest `origin` in direction `delta` (a unit-ish
    // vector), or -1 when there is none in that direction.
    [[nodiscard]] static i32 next_stop(const std::vector<const UiResolved*>& stops, const UiResolved& origin,
                                       const Vec2& delta) noexcept;

    // Applies the frame's focus to `layout`, writing `focused` and `activated`.
    void apply(const FrameContext& context, const UiLayoutResult& layout);

    ActionMap::Id up_ = ActionMap::kNone;
    ActionMap::Id down_ = ActionMap::kNone;
    ActionMap::Id left_ = ActionMap::kNone;
    ActionMap::Id right_ = ActionMap::kNone;
    ActionMap::Id confirm_ = ActionMap::kNone;

    UiLayoutResult layout_;
    // The element that holds focus across frames, by component identity (a scene
    // owns the element and outlives the frame, so the pointer is stable; the
    // layout array is rebuilt each frame).
    const UiElement* focused_ = nullptr;
    const Node* focused_node_ = nullptr;
    bool activated_this_frame_ = false;
    // The analog-repeat clock: how long a held direction has been down since it
    // last moved focus, and how long it must be held to move again.
    f32 repeat_timer_ = 0.0f;
    f32 repeat_interval_ = 0.25f;
};

} // namespace fe::ui
