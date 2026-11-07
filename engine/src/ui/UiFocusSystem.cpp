// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/ui/UiFocusSystem.h"

#include "fe/platform/InputState.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/ui/UiElement.h"

#include <algorithm>
#include <cmath>

namespace fe::ui {
namespace {

// The fallback direction keys when no navigation actions are bound, so a caller
// with no action map still gets a navigable menu. The arrow keys are what a
// player reaches for on a keyboard.
struct DirectionKeys {
    Key up = Key::Up;
    Key down = Key::Down;
    Key left = Key::Left;
    Key right = Key::Right;
};

// A press this frame for one direction, from the action when it is bound and
// from the fallback key otherwise. An action that is bound is authoritative: a
// caller that bound only "up" gets the action for "up" and the fallback key for
// the rest, which is the same partial-override rule ActionMap uses.
[[nodiscard]] bool pressed(const FrameContext& context, ActionMap::Id action, Key fallback) noexcept {
    if (action != ActionMap::kNone) {
        return context.actions != nullptr && context.actions->was_pressed(action);
    }
    return context.input != nullptr && context.input->was_key_pressed(fallback);
}

[[nodiscard]] bool held(const FrameContext& context, ActionMap::Id action, Key fallback) noexcept {
    if (action != ActionMap::kNone) {
        return context.actions != nullptr && context.actions->is_down(action);
    }
    return context.input != nullptr && context.input->is_key_down(fallback);
}

} // namespace

void UiFocusSystem::set_navigation_actions(ActionMap::Id up, ActionMap::Id down, ActionMap::Id left,
                                           ActionMap::Id right) noexcept {
    up_ = up;
    down_ = down;
    left_ = left;
    right_ = right;
}

void UiFocusSystem::clear_focus() noexcept {
    focused_ = nullptr;
    focused_node_ = nullptr;
    activated_this_frame_ = false;
}

std::vector<const UiResolved*> UiFocusSystem::focusable(const UiLayoutResult& layout) {
    std::vector<const UiResolved*> stops;
    for (const UiResolved& resolved : layout) {
        if (resolved.element == nullptr || !resolved.element->focusable || !resolved.element->interactable) {
            continue;
        }
        stops.push_back(&resolved);
    }
    return stops;
}

i32 UiFocusSystem::index_of(const std::vector<const UiResolved*>& stops, const UiElement* current) noexcept {
    for (usize i = 0; i < stops.size(); ++i) {
        if (stops[i]->element == current) {
            return static_cast<i32>(i);
        }
    }
    return -1;
}

i32 UiFocusSystem::next_stop(const std::vector<const UiResolved*>& stops, const UiResolved& origin,
                             const Vec2& delta) noexcept {
    const Vec2 centre{origin.rect.position.x + origin.rect.size.x * 0.5f,
                      origin.rect.position.y + origin.rect.size.y * 0.5f};
    i32 best = -1;
    f32 best_score = 0.0f;
    for (usize i = 0; i < stops.size(); ++i) {
        const UiResolved& candidate = *stops[i];
        if (candidate.element == origin.element) {
            continue;
        }
        const Vec2 other{candidate.rect.position.x + candidate.rect.size.x * 0.5f,
                         candidate.rect.position.y + candidate.rect.size.y * 0.5f};
        const Vec2 to{other.x - centre.x, other.y - centre.y};
        // The distance along the requested direction, and across it. A candidate
        // behind the origin (a negative projection) is not in that direction.
        const f32 along = to.x * delta.x + to.y * delta.y;
        if (along <= 0.0f) {
            continue;
        }
        const f32 across = std::fabs(to.x * delta.y - to.y * delta.x);
        // Prefer what is most in the requested direction and least off to the
        // side: the cross-axis distance dominates, so a press of "down" goes to
        // the button directly below and not to a nearer one far to the side. The
        // `along` term breaks ties so the closer of two aligned candidates wins.
        const f32 score = across * 4.0f + along;
        if (best < 0 || score < best_score) {
            best = static_cast<i32>(i);
            best_score = score;
        }
    }
    return best;
}

void UiFocusSystem::apply(const FrameContext& context, const UiLayoutResult& layout) {
    activated_this_frame_ = false;

    const std::vector<const UiResolved*> stops = focusable(layout);
    if (stops.empty()) {
        focused_ = nullptr;
        focused_node_ = nullptr;
        return;
    }

    i32 index = index_of(stops, focused_);
    if (index < 0) {
        // No focus yet, or the focused element left the layout: start on the
        // first stop, so a menu that opens has a selection and a scene change
        // does not leave focus dangling on a freed element.
        index = 0;
    }

    // The direction asked for this frame, if any. A key press is an edge and
    // moves one stop. A held direction (an analog stick, a held key) repeats on a
    // steady cadence rather than once per frame, so a menu is not skipped through
    // in a fraction of a second.
    const auto direction_pressed = [&]() -> Vec2 {
        if (pressed(context, down_, DirectionKeys{}.down)) {
            return Vec2{0.0f, 1.0f};
        }
        if (pressed(context, up_, DirectionKeys{}.up)) {
            return Vec2{0.0f, -1.0f};
        }
        if (pressed(context, right_, DirectionKeys{}.right)) {
            return Vec2{1.0f, 0.0f};
        }
        if (pressed(context, left_, DirectionKeys{}.left)) {
            return Vec2{-1.0f, 0.0f};
        }
        return Vec2{};
    };
    const auto direction_held = [&]() -> Vec2 {
        if (held(context, down_, DirectionKeys{}.down)) {
            return Vec2{0.0f, 1.0f};
        }
        if (held(context, up_, DirectionKeys{}.up)) {
            return Vec2{0.0f, -1.0f};
        }
        if (held(context, right_, DirectionKeys{}.right)) {
            return Vec2{1.0f, 0.0f};
        }
        if (held(context, left_, DirectionKeys{}.left)) {
            return Vec2{-1.0f, 0.0f};
        }
        return Vec2{};
    };

    Vec2 delta = direction_pressed();
    if (delta.x == 0.0f && delta.y == 0.0f) {
        if (const Vec2 hold = direction_held(); hold.x != 0.0f || hold.y != 0.0f) {
            repeat_timer_ += context.delta_time;
            if (repeat_timer_ >= repeat_interval_) {
                repeat_timer_ = 0.0f;
                delta = hold;
            }
        } else {
            repeat_timer_ = 0.0f;
        }
    } else {
        repeat_timer_ = 0.0f;
    }

    if (delta.x != 0.0f || delta.y != 0.0f) {
        const i32 next = next_stop(stops, *stops[static_cast<usize>(index)], delta);
        if (next >= 0) {
            index = next;
        }
    }

    const UiResolved* current = stops[static_cast<usize>(index)];
    focused_ = current->element;
    focused_node_ = current->node;

    // Confirm: the action when bound, the Enter/Space fallback otherwise, so a
    // menu with no action map is still confirmable.
    bool confirm = false;
    if (confirm_ != ActionMap::kNone) {
        confirm = context.actions != nullptr && context.actions->was_pressed(confirm_);
    } else if (context.input != nullptr) {
        confirm = context.input->was_key_pressed(Key::Enter) || context.input->was_key_pressed(Key::Space);
    }
    activated_this_frame_ = confirm;

    auto* element = const_cast<UiElement*>(current->element);
    element->interaction.focused = true;
    element->interaction.activated = confirm;
}

void UiFocusSystem::update(const FrameContext& context, Scene& scene) {
    focused_node_ = nullptr;
    activated_this_frame_ = false;

    // No input at all: clear every element's focus flags so a stale ring is not
    // drawn, and keep the remembered element so focus resumes where it was.
    if (context.input == nullptr && context.actions == nullptr) {
        scene.for_each_node([](Node& node) {
            if (auto* element = node.get_component<UiElement>(); element != nullptr) {
                element->interaction.focused = false;
                element->interaction.activated = false;
            }
        });
        return;
    }

    const u32 width = static_cast<u32>(std::max(context.viewport.x, 0.0f));
    const u32 height = static_cast<u32>(std::max(context.viewport.y, 0.0f));
    if (width == 0 || height == 0) {
        return;
    }

    // The same layout call the input system and the renderer make, so a focus
    // ring is drawn exactly where focus is.
    fe::ui::layout(scene, width, height, layout_);

    // Clear the previous frame's focus flags before writing this frame's, so an
    // element that lost focus does not keep the ring.
    scene.for_each_node([](Node& node) {
        if (auto* element = node.get_component<UiElement>(); element != nullptr) {
            element->interaction.focused = false;
            element->interaction.activated = false;
        }
    });

    apply(context, layout_);
}

} // namespace fe::ui
