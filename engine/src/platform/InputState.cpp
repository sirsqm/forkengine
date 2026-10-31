// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/platform/InputState.h"

#include "fe/core/Assert.h"

namespace fe {

void InputState::begin_frame() noexcept {
    keys_pressed_.fill(false);
    keys_released_.fill(false);
    buttons_pressed_.fill(false);
    buttons_released_.fill(false);
    for (Gamepad& pad : gamepads_) {
        pad.pressed.fill(false);
        pad.released.fill(false);
    }
    for (TouchPoint& touch : touches_) {
        // A slot whose contact ended last frame is freed here, after the end
        // edge has been observable for one frame.
        if (touch.ended) {
            touch = TouchPoint{};
            continue;
        }
        touch.began = false;
        touch.delta = Vec2{};
    }
    mouse_delta_ = Vec2{};
    scroll_delta_ = Vec2{};
}

void InputState::on_key(Key key, bool down) noexcept {
    FE_CHECK(key != Key::Unknown && key != Key::Count);
    const usize index = index_of(key);
    if (keys_down_[index] == down) {
        return;
    }
    keys_down_[index] = down;
    if (down) {
        keys_pressed_[index] = true;
    } else {
        keys_released_[index] = true;
    }
}

void InputState::on_mouse_button(MouseButton button, bool down) noexcept {
    FE_CHECK(button != MouseButton::Count);
    const usize index = index_of(button);
    if (buttons_down_[index] == down) {
        return;
    }
    buttons_down_[index] = down;
    if (down) {
        buttons_pressed_[index] = true;
    } else {
        buttons_released_[index] = true;
    }
}

void InputState::on_mouse_move(f32 x, f32 y) noexcept {
    const Vec2 previous = mouse_position_;
    mouse_position_ = Vec2{x, y};
    // The first motion event after a frame boundary contributes the delta from
    // the last known position; that is the value gameplay code expects.
    mouse_delta_ = mouse_position_ - previous;
}

void InputState::on_scroll(f32 dx, f32 dy) noexcept {
    scroll_delta_ = Vec2{scroll_delta_.x + dx, scroll_delta_.y + dy};
}

usize InputState::find_touch(u32 id) const noexcept {
    for (usize slot = 0; slot < kMaxTouchPoints; ++slot) {
        if (touches_[slot].active && touches_[slot].id == id) {
            return slot;
        }
    }
    return kMaxTouchPoints;
}

void InputState::on_touch(TouchPhase phase, u32 id, f32 x, f32 y) noexcept {
    FE_CHECK(phase == TouchPhase::Began || phase == TouchPhase::Moved || phase == TouchPhase::Ended);
    usize slot = find_touch(id);
    // A slot is reused only for a fresh contact, so a Began for an id we do not
    // track (or one that ended earlier this frame) starts a new contact.
    if (phase == TouchPhase::Began) {
        if (slot == kMaxTouchPoints) {
            for (usize candidate = 0; candidate < kMaxTouchPoints; ++candidate) {
                if (!touches_[candidate].active) {
                    slot = candidate;
                    break;
                }
            }
            if (slot == kMaxTouchPoints) {
                return; // More simultaneous contacts than we track; drop the extra.
            }
        }
        TouchPoint& fresh = touches_[slot];
        fresh = TouchPoint{};
        fresh.id = id;
        fresh.active = true;
        fresh.down = true;
        fresh.began = true;
        fresh.position = Vec2{x, y};
        return;
    }

    // A Moved/Ended for an id we never saw is ignored.
    if (slot == kMaxTouchPoints) {
        return;
    }
    TouchPoint& touch = touches_[slot];
    const Vec2 previous = touch.position;
    touch.position = Vec2{x, y};
    touch.delta =
        Vec2{touch.delta.x + (touch.position.x - previous.x), touch.delta.y + (touch.position.y - previous.y)};
    if (phase == TouchPhase::Ended) {
        touch.down = false;
        touch.ended = true;
    }
}

bool InputState::is_touch_down(u32 id) const noexcept {
    const usize slot = find_touch(id);
    return slot != kMaxTouchPoints && touches_[slot].down;
}

bool InputState::was_touch_began(u32 id) const noexcept {
    const usize slot = find_touch(id);
    return slot != kMaxTouchPoints && touches_[slot].began;
}

bool InputState::was_touch_ended(u32 id) const noexcept {
    const usize slot = find_touch(id);
    return slot != kMaxTouchPoints && touches_[slot].ended;
}

Vec2 InputState::touch_position(u32 id) const noexcept {
    const usize slot = find_touch(id);
    return slot != kMaxTouchPoints ? touches_[slot].position : Vec2{};
}

usize InputState::touch_count() const noexcept {
    usize count = 0;
    for (const TouchPoint& touch : touches_) {
        if (touch.active) {
            ++count;
        }
    }
    return count;
}

u32 InputState::touch_id_at(usize index) const noexcept {
    usize seen = 0;
    for (const TouchPoint& touch : touches_) {
        if (!touch.active) {
            continue;
        }
        if (seen == index) {
            return touch.id;
        }
        ++seen;
    }
    return kInvalidTouchId;
}

Vec2 InputState::touch_delta(u32 id) const noexcept {
    const usize slot = find_touch(id);
    return slot != kMaxTouchPoints ? touches_[slot].delta : Vec2{};
}

bool InputState::is_key_down(Key key) const noexcept {
    return keys_down_[index_of(key)];
}

bool InputState::was_key_pressed(Key key) const noexcept {
    return keys_pressed_[index_of(key)];
}

bool InputState::was_key_released(Key key) const noexcept {
    return keys_released_[index_of(key)];
}

bool InputState::is_mouse_down(MouseButton button) const noexcept {
    return buttons_down_[index_of(button)];
}

bool InputState::was_mouse_pressed(MouseButton button) const noexcept {
    return buttons_pressed_[index_of(button)];
}

bool InputState::was_mouse_released(MouseButton button) const noexcept {
    return buttons_released_[index_of(button)];
}

void InputState::on_gamepad_connected(usize slot, bool connected) noexcept {
    if (slot >= kMaxGamepads) {
        return;
    }
    Gamepad& pad = gamepads_[slot];
    if (connected) {
        // A fresh connect starts clean, so a pad plugged in while a button is
        // held does not inherit that button as "down".
        pad = Gamepad{};
        pad.connected = true;
        return;
    }
    if (pad.connected) {
        // Raise the released edge for anything held, so a consumer sees the
        // release rather than a button stuck down forever. The edges have to
        // survive the state reset below, so they are captured first.
        std::array<bool, kGamepadButtonCount> released{};
        for (usize i = 0; i < kGamepadButtonCount; ++i) {
            released[i] = pad.buttons[i];
        }
        pad = Gamepad{};
        pad.released = released;
    }
}

void InputState::on_gamepad_button(usize slot, GamepadButton button, bool down) noexcept {
    if (slot >= kMaxGamepads || button == GamepadButton::Count) {
        return;
    }
    Gamepad& pad = gamepads_[slot];
    // A button reported for a slot nobody marked connected is still recorded:
    // the backend may deliver a snapshot before it reports the connect, and
    // dropping the input would make the pad appear dead for a frame.
    const usize index = static_cast<usize>(button);
    if (pad.buttons[index] == down) {
        return;
    }
    pad.buttons[index] = down;
    if (down) {
        pad.pressed[index] = true;
    } else {
        pad.released[index] = true;
    }
}

void InputState::on_gamepad_axis(usize slot, GamepadAxis axis, f32 value) noexcept {
    if (slot >= kMaxGamepads || axis == GamepadAxis::Count) {
        return;
    }
    // A trigger is 0..1 from released to fully pressed; a stick is -1..1. The
    // clamp is here rather than at each query so every consumer sees the same
    // range regardless of what the backend reported.
    const bool trigger = axis == GamepadAxis::LeftTrigger || axis == GamepadAxis::RightTrigger;
    const f32 lo = trigger ? 0.0f : -1.0f;
    const f32 hi = 1.0f;
    gamepads_[slot].axes[static_cast<usize>(axis)] = value < lo ? lo : (value > hi ? hi : value);
}

bool InputState::is_gamepad_connected(usize slot) const noexcept {
    return slot < kMaxGamepads && gamepads_[slot].connected;
}

bool InputState::is_gamepad_button_down(usize slot, GamepadButton button) const noexcept {
    return slot < kMaxGamepads && button != GamepadButton::Count && gamepads_[slot].buttons[static_cast<usize>(button)];
}

bool InputState::was_gamepad_button_pressed(usize slot, GamepadButton button) const noexcept {
    return slot < kMaxGamepads && button != GamepadButton::Count && gamepads_[slot].pressed[static_cast<usize>(button)];
}

bool InputState::was_gamepad_button_released(usize slot, GamepadButton button) const noexcept {
    return slot < kMaxGamepads && button != GamepadButton::Count &&
           gamepads_[slot].released[static_cast<usize>(button)];
}

f32 InputState::gamepad_axis(usize slot, GamepadAxis axis) const noexcept {
    if (slot >= kMaxGamepads || axis == GamepadAxis::Count) {
        return 0.0f;
    }
    return gamepads_[slot].axes[static_cast<usize>(axis)];
}

usize InputState::first_connected_gamepad() const noexcept {
    for (usize slot = 0; slot < kMaxGamepads; ++slot) {
        if (gamepads_[slot].connected) {
            return slot;
        }
    }
    return kInvalidGamepad;
}

void InputState::release_all() noexcept {
    keys_down_.fill(false);
    buttons_down_.fill(false);
    for (Gamepad& pad : gamepads_) {
        // A focus loss does not disconnect the pad, so the slot stays present;
        // only the held state is cleared, and the buttons' own released edges
        // are raised so a consumer sees the release rather than a silent drop.
        for (usize i = 0; i < kGamepadButtonCount; ++i) {
            if (pad.buttons[i]) {
                pad.buttons[i] = false;
                pad.released[i] = true;
            }
        }
        pad.axes.fill(0.0f);
    }
    for (TouchPoint& touch : touches_) {
        touch.down = false;
    }
}

bool InputState::any_pressed() const noexcept {
    // A key or button edge, or a contact that began this frame. Only the edge
    // flags are consulted, so holding a key does not keep reporting a gesture:
    // the browser only honours the first real interaction anyway, and a caller
    // that acts on this every frame would otherwise do the work repeatedly.
    for (bool pressed : keys_pressed_) {
        if (pressed) {
            return true;
        }
    }
    for (bool pressed : buttons_pressed_) {
        if (pressed) {
            return true;
        }
    }
    for (const TouchPoint& touch : touches_) {
        if (touch.began) {
            return true;
        }
    }
    for (const Gamepad& pad : gamepads_) {
        for (bool pressed : pad.pressed) {
            if (pressed) {
                return true;
            }
        }
    }
    return false;
}

} // namespace fe
