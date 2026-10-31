// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include <doctest/doctest.h>

#include "fe/platform/InputState.h"

using namespace fe;

TEST_CASE("key down state persists until released") {
    InputState input;
    input.on_key(Key::W, true);

    CHECK(input.is_key_down(Key::W));
    CHECK(input.was_key_pressed(Key::W));
    CHECK_FALSE(input.was_key_released(Key::W));

    input.begin_frame();
    // The key is still held, but the press edge belongs to the previous frame.
    CHECK(input.is_key_down(Key::W));
    CHECK_FALSE(input.was_key_pressed(Key::W));
}

TEST_CASE("release produces a release edge for exactly one frame") {
    InputState input;
    input.on_key(Key::A, true);
    input.begin_frame();

    input.on_key(Key::A, false);
    CHECK(input.was_key_released(Key::A));
    CHECK_FALSE(input.is_key_down(Key::A));

    input.begin_frame();
    CHECK_FALSE(input.was_key_released(Key::A));
}

TEST_CASE("mouse buttons report down and press edges") {
    InputState input;
    input.on_mouse_button(MouseButton::Left, true);

    CHECK(input.is_mouse_down(MouseButton::Left));
    CHECK(input.was_mouse_pressed(MouseButton::Left));

    input.begin_frame();
    input.on_mouse_button(MouseButton::Left, false);
    CHECK(input.was_mouse_released(MouseButton::Left));
}

TEST_CASE("mouse movement accumulates a per-frame delta") {
    InputState input;
    input.on_mouse_move(10.0f, 20.0f);
    CHECK(input.mouse_position() == Vec2{10.0f, 20.0f});
    CHECK(input.mouse_delta() == Vec2{10.0f, 20.0f});

    input.begin_frame();
    CHECK(input.mouse_delta() == Vec2{0.0f, 0.0f});

    input.on_mouse_move(15.0f, 25.0f);
    CHECK(input.mouse_delta() == Vec2{5.0f, 5.0f});
}

TEST_CASE("scroll delta is reported per frame and then cleared") {
    InputState input;
    input.on_scroll(0.0f, 1.5f);
    CHECK(input.scroll_delta() == Vec2{0.0f, 1.5f});

    input.begin_frame();
    CHECK(input.scroll_delta() == Vec2{0.0f, 0.0f});
}

TEST_CASE("release_all clears held keys but preserves edge history") {
    InputState input;
    input.on_key(Key::D, true);
    CHECK(input.was_key_pressed(Key::D));

    input.release_all();
    CHECK_FALSE(input.is_key_down(Key::D));
    // The press edge from this frame is still observable, which is what lets
    // gameplay code notice a key that was pressed and the window then lost
    // focus.
    CHECK(input.was_key_pressed(Key::D));
}

TEST_CASE("a touch reports begin, update, and end keyed by id") {
    InputState input;
    input.on_touch(TouchPhase::Began, 7, 10.0f, 20.0f);
    CHECK(input.is_touch_down(7));
    CHECK(input.was_touch_began(7));
    CHECK_FALSE(input.was_touch_ended(7));
    CHECK(input.touch_position(7) == Vec2{10.0f, 20.0f});

    input.begin_frame();
    // The contact is still down, but the begin edge belongs to the previous
    // frame.
    CHECK(input.is_touch_down(7));
    CHECK_FALSE(input.was_touch_began(7));
    CHECK(input.touch_delta(7) == Vec2{0.0f, 0.0f});

    input.on_touch(TouchPhase::Moved, 7, 15.0f, 25.0f);
    CHECK(input.is_touch_down(7));
    CHECK(input.touch_position(7) == Vec2{15.0f, 25.0f});
    CHECK(input.touch_delta(7) == Vec2{5.0f, 5.0f});

    input.on_touch(TouchPhase::Ended, 7, 15.0f, 25.0f);
    CHECK_FALSE(input.is_touch_down(7));
    CHECK(input.was_touch_ended(7));

    input.begin_frame();
    // The end edge is gone and the id is no longer tracked.
    CHECK_FALSE(input.was_touch_ended(7));
    CHECK_FALSE(input.is_touch_down(7));
    CHECK(input.touch_position(7) == Vec2{0.0f, 0.0f});
}

TEST_CASE("touch ids are tracked independently") {
    InputState input;
    input.on_touch(TouchPhase::Began, 1, 0.0f, 0.0f);
    input.on_touch(TouchPhase::Began, 2, 100.0f, 100.0f);

    CHECK(input.is_touch_down(1));
    CHECK(input.is_touch_down(2));

    input.on_touch(TouchPhase::Ended, 1, 0.0f, 0.0f);
    CHECK_FALSE(input.is_touch_down(1));
    CHECK(input.was_touch_ended(1));
    // Ending one contact leaves the other untouched.
    CHECK(input.is_touch_down(2));
    CHECK_FALSE(input.was_touch_ended(2));
    CHECK(input.touch_position(2) == Vec2{100.0f, 100.0f});
}

TEST_CASE("begin_frame clears touch edge state") {
    InputState input;
    input.on_touch(TouchPhase::Began, 3, 1.0f, 2.0f);
    CHECK(input.was_touch_began(3));

    input.begin_frame();
    CHECK_FALSE(input.was_touch_began(3));
    CHECK_FALSE(input.was_touch_ended(3));
    CHECK(input.touch_delta(3) == Vec2{0.0f, 0.0f});
}

TEST_CASE("an unknown touch id reports nothing") {
    InputState input;
    // A Moved/Ended for an id that never began is ignored rather than starting
    // a phantom contact.
    input.on_touch(TouchPhase::Moved, 9, 5.0f, 5.0f);
    input.on_touch(TouchPhase::Ended, 9, 5.0f, 5.0f);
    CHECK_FALSE(input.is_touch_down(9));
    CHECK_FALSE(input.was_touch_ended(9));
    CHECK(input.touch_position(9) == Vec2{0.0f, 0.0f});
}

TEST_CASE("touches can be enumerated without knowing their ids") {
    InputState input;
    input.on_touch(TouchPhase::Began, 7, 10.0f, 20.0f);
    input.on_touch(TouchPhase::Began, 3, 30.0f, 40.0f);

    REQUIRE(input.touch_count() == 2);
    // The ids are the backend's, in slot order, so a caller finds contacts by
    // walking the count rather than guessing an id.
    const u32 first = input.touch_id_at(0);
    const u32 second = input.touch_id_at(1);
    CHECK(first != InputState::kInvalidTouchId);
    CHECK(second != InputState::kInvalidTouchId);
    CHECK(first != second);
    CHECK(input.is_touch_down(first));
    CHECK(input.is_touch_down(second));
    CHECK(input.touch_id_at(2) == InputState::kInvalidTouchId);
}

TEST_CASE("touch enumeration counts a contact until its end edge is consumed") {
    InputState input;
    input.on_touch(TouchPhase::Began, 1, 5.0f, 5.0f);
    input.on_touch(TouchPhase::Ended, 1, 5.0f, 5.0f);

    // The contact is still tracked this frame so its end edge is observable.
    CHECK(input.touch_count() == 1);
    CHECK_FALSE(input.is_touch_down(input.touch_id_at(0)));

    input.begin_frame();
    CHECK(input.touch_count() == 0);
}

TEST_CASE("touch enumeration is empty with no contacts") {
    InputState input;
    CHECK(input.touch_count() == 0);
    CHECK(input.touch_id_at(0) == InputState::kInvalidTouchId);
}

TEST_CASE("any_pressed reports a gesture for exactly one frame") {
    // The platform gesture signal: a browser only lets a page make sound after
    // one, and a caller acts on it once. Edge-based, so holding the key does not
    // keep reporting a gesture.
    InputState input;
    CHECK_FALSE(input.any_pressed());

    input.on_key(Key::Space, true);
    CHECK(input.any_pressed());

    // Next frame: the key is still down but the press edge is gone.
    input.begin_frame();
    CHECK_FALSE(input.any_pressed());
    CHECK(input.is_key_down(Key::Space));

    // A mouse button is a gesture too.
    input.on_mouse_button(MouseButton::Left, true);
    CHECK(input.any_pressed());

    // So is a new touch contact.
    input.begin_frame();
    CHECK_FALSE(input.any_pressed());
    input.on_touch(TouchPhase::Began, 0, 10.0f, 10.0f);
    CHECK(input.any_pressed());
}

TEST_CASE("a gamepad button reports down and press edges") {
    InputState input;
    input.on_gamepad_connected(0, true);
    input.on_gamepad_button(0, GamepadButton::South, true);

    CHECK(input.is_gamepad_connected(0));
    CHECK(input.is_gamepad_button_down(0, GamepadButton::South));
    CHECK(input.was_gamepad_button_pressed(0, GamepadButton::South));
    CHECK_FALSE(input.was_gamepad_button_released(0, GamepadButton::South));

    input.begin_frame();
    // Still held, but the press edge belonged to the previous frame.
    CHECK(input.is_gamepad_button_down(0, GamepadButton::South));
    CHECK_FALSE(input.was_gamepad_button_pressed(0, GamepadButton::South));

    input.on_gamepad_button(0, GamepadButton::South, false);
    CHECK(input.was_gamepad_button_released(0, GamepadButton::South));
    CHECK_FALSE(input.is_gamepad_button_down(0, GamepadButton::South));
}

TEST_CASE("gamepad slots are independent") {
    InputState input;
    input.on_gamepad_connected(0, true);
    input.on_gamepad_connected(1, true);
    input.on_gamepad_button(1, GamepadButton::Start, true);

    // Player one's pad is untouched by player two's press.
    CHECK_FALSE(input.is_gamepad_button_down(0, GamepadButton::Start));
    CHECK(input.is_gamepad_button_down(1, GamepadButton::Start));
}

TEST_CASE("an unconnected gamepad reports nothing") {
    InputState input;
    // A button fed for a slot that was never connected is still recorded (the
    // backend may deliver a snapshot before it reports the connect), but a slot
    // that is genuinely empty reads as no input.
    CHECK_FALSE(input.is_gamepad_connected(2));
    CHECK_FALSE(input.is_gamepad_button_down(2, GamepadButton::South));
    CHECK(input.gamepad_axis(2, GamepadAxis::LeftX) == 0.0f);
    CHECK(input.first_connected_gamepad() == InputState::kInvalidGamepad);
}

TEST_CASE("disconnecting a gamepad releases its held buttons") {
    InputState input;
    input.on_gamepad_connected(0, true);
    input.on_gamepad_button(0, GamepadButton::East, true);
    input.begin_frame();

    input.on_gamepad_connected(0, false);
    // The release edge is observable for one frame, so a consumer sees the
    // button come up rather than it being stuck down by a pulled cable.
    CHECK(input.was_gamepad_button_released(0, GamepadButton::East));
    CHECK_FALSE(input.is_gamepad_button_down(0, GamepadButton::East));
    CHECK_FALSE(input.is_gamepad_connected(0));

    input.begin_frame();
    CHECK_FALSE(input.was_gamepad_button_released(0, GamepadButton::East));
}

TEST_CASE("reconnecting a gamepad starts from a clean state") {
    InputState input;
    input.on_gamepad_connected(0, true);
    input.on_gamepad_button(0, GamepadButton::North, true);
    input.on_gamepad_connected(0, false);
    input.on_gamepad_connected(0, true);

    // A pad plugged back in does not inherit the button that was held when it
    // was unplugged.
    CHECK(input.is_gamepad_connected(0));
    CHECK_FALSE(input.is_gamepad_button_down(0, GamepadButton::North));
}

TEST_CASE("gamepad axes are clamped to their range") {
    InputState input;
    input.on_gamepad_connected(0, true);
    input.on_gamepad_axis(0, GamepadAxis::LeftX, 2.0f);
    input.on_gamepad_axis(0, GamepadAxis::LeftY, -3.0f);
    // A trigger is 0..1, so a negative raw value floors at 0 rather than -1.
    input.on_gamepad_axis(0, GamepadAxis::RightTrigger, -0.5f);

    CHECK(input.gamepad_axis(0, GamepadAxis::LeftX) == 1.0f);
    CHECK(input.gamepad_axis(0, GamepadAxis::LeftY) == -1.0f);
    CHECK(input.gamepad_axis(0, GamepadAxis::RightTrigger) == 0.0f);
}

TEST_CASE("first_connected_gamepad finds the lowest occupied slot") {
    InputState input;
    CHECK(input.first_connected_gamepad() == InputState::kInvalidGamepad);

    input.on_gamepad_connected(2, true);
    CHECK(input.first_connected_gamepad() == 2);

    input.on_gamepad_connected(1, true);
    CHECK(input.first_connected_gamepad() == 1);

    input.on_gamepad_connected(1, false);
    CHECK(input.first_connected_gamepad() == 2);
}

TEST_CASE("release_all clears held gamepad buttons but keeps the slot") {
    InputState input;
    input.on_gamepad_connected(0, true);
    input.on_gamepad_button(0, GamepadButton::LeftBumper, true);
    input.on_gamepad_axis(0, GamepadAxis::LeftX, 0.8f);

    input.release_all();
    // The pad is still plugged in; only the held input is dropped.
    CHECK(input.is_gamepad_connected(0));
    CHECK_FALSE(input.is_gamepad_button_down(0, GamepadButton::LeftBumper));
    CHECK(input.was_gamepad_button_released(0, GamepadButton::LeftBumper));
    CHECK(input.gamepad_axis(0, GamepadAxis::LeftX) == 0.0f);
}

TEST_CASE("a gamepad press is a user gesture") {
    InputState input;
    input.on_gamepad_connected(0, true);
    CHECK_FALSE(input.any_pressed());

    input.on_gamepad_button(0, GamepadButton::South, true);
    CHECK(input.any_pressed());

    // Holding it is not a new gesture.
    input.begin_frame();
    CHECK_FALSE(input.any_pressed());
    CHECK(input.is_gamepad_button_down(0, GamepadButton::South));
}

TEST_CASE("an out-of-range gamepad slot is ignored") {
    InputState input;
    // A slot at or above kMaxGamepads is a programming error the API absorbs
    // rather than reading past the array.
    input.on_gamepad_connected(kMaxGamepads, true);
    input.on_gamepad_button(kMaxGamepads, GamepadButton::South, true);
    input.on_gamepad_axis(kMaxGamepads, GamepadAxis::LeftX, 1.0f);

    CHECK_FALSE(input.is_gamepad_connected(kMaxGamepads));
    CHECK_FALSE(input.is_gamepad_button_down(kMaxGamepads, GamepadButton::South));
    CHECK(input.gamepad_axis(kMaxGamepads, GamepadAxis::LeftX) == 0.0f);
}
