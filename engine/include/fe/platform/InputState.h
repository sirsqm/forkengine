// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Math.h"
#include "fe/core/Types.h"

#include <array>

namespace fe {

// Physical key codes, independent of any windowing library. The values are
// stable within a major version and are the only key identity the rest of the
// engine sees; the platform backend translates native codes into these.
enum class Key : u16 {
    Unknown = 0,

    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    M,
    N,
    O,
    P,
    Q,
    R,
    S,
    T,
    U,
    V,
    W,
    X,
    Y,
    Z,

    Num0,
    Num1,
    Num2,
    Num3,
    Num4,
    Num5,
    Num6,
    Num7,
    Num8,
    Num9,

    Escape,
    Enter,
    Tab,
    Backspace,
    Insert,
    Delete,
    Right,
    Left,
    Down,
    Up,
    PageUp,
    PageDown,
    Home,
    End,
    Space,
    Minus,
    Equal,
    LeftBracket,
    RightBracket,
    Backslash,
    Semicolon,
    Apostrophe,
    Comma,
    Period,
    Slash,
    Grave,

    F1,
    F2,
    F3,
    F4,
    F5,
    F6,
    F7,
    F8,
    F9,
    F10,
    F11,
    F12,

    LeftShift,
    RightShift,
    LeftControl,
    RightControl,
    LeftAlt,
    RightAlt,

    Count,
};

enum class MouseButton : u8 {
    Left = 0,
    Right,
    Middle,
    Count,
};

// A touch point's lifecycle phase, translated from the backend's native event.
// A touch is keyed by an opaque id the backend assigns; ids are reused after a
// touch ends, so only the events between Began and Ended for one id describe
// one contact.
enum class TouchPhase : u8 {
    Began = 0,
    Moved,
    Ended,
};

// A gamepad's physical controls, in the layout every desktop backend can fill
// in: the four face buttons in the standard (Xbox) arrangement, the two
// shoulders and their sticks, the d-pad, and the menu buttons. The names are the
// positional ones the engine uses everywhere, not a vendor's glyphs, because the
// same physical button is "A" on an Xbox pad, "Cross" on a PlayStation pad, and
// "B" on a Nintendo pad - the backend maps its native layout onto this one, and
// gameplay asks for the position. `Count` bounds the fixed arrays.
enum class GamepadButton : u8 {
    South = 0, // Xbox A, PlayStation Cross, Nintendo B
    East,      // Xbox B, PlayStation Circle, Nintendo A
    West,      // Xbox X, PlayStation Square, Nintendo Y
    North,     // Xbox Y, PlayStation Triangle, Nintendo X
    LeftBumper,
    RightBumper,
    Back,
    Start,
    Guide,
    LeftStick,
    RightStick,
    DpadUp,
    DpadDown,
    DpadLeft,
    DpadRight,
    Count,
};

// A gamepad's analog axes. A stick is reported as a unit-range axis with a
// positive-up convention (`Y` is negative when the stick is pushed up), matching
// the way a screen-space Y grows downward so gameplay does not have to flip it;
// a trigger is reported 0..1 with 0 released.
enum class GamepadAxis : u8 {
    LeftX = 0,
    LeftY,
    RightX,
    RightY,
    LeftTrigger,
    RightTrigger,
    Count,
};

// The maximum number of gamepads tracked. GLFW exposes four slots; a backend
// that has more reuses the lowest free one.
inline constexpr usize kMaxGamepads = 4;

// The largest number of buttons and axes one pad reports, so a backend can size
// its native-to-engine translation table without a heap allocation. Both are
// above the standard layout's counts (15 buttons, 6 axes) with room for a
// vendor's extra controls.
inline constexpr usize kMaxGamepadButtons = 24;
inline constexpr usize kMaxGamepadAxes = 12;

// Per-frame input snapshot.
//
// The state is event-driven: the platform backend feeds raw events in, and the
// runtime calls begin_frame() once per iteration to advance the edge state.
// This keeps "was pressed this frame" queries cheap and deterministic, and
// keeps the whole subsystem testable without a window: tests call the on_*()
// methods directly.
class InputState {
public:
    static constexpr usize kKeyCount = static_cast<usize>(Key::Count);
    static constexpr usize kMouseButtonCount = static_cast<usize>(MouseButton::Count);
    static constexpr usize kGamepadButtonCount = static_cast<usize>(GamepadButton::Count);
    static constexpr usize kGamepadAxisCount = static_cast<usize>(GamepadAxis::Count);

    // The maximum number of simultaneous touch contacts tracked. A backend
    // reports more than this only by reusing an id, exactly as it would when a
    // contact ends and a new one begins.
    static constexpr usize kMaxTouchPoints = 10;

    // Advances the frame: current -> previous, edges and per-frame deltas
    // cleared. Call once at the start of each frame, before polling events.
    void begin_frame() noexcept;

    void on_key(Key key, bool down) noexcept;
    void on_mouse_button(MouseButton button, bool down) noexcept;
    void on_mouse_move(f32 x, f32 y) noexcept;
    void on_scroll(f32 dx, f32 dy) noexcept;

    // Feeds one touch event. `id` is the backend's contact id, stable across
    // Began/Moved/Ended for one contact. The position is in the same window
    // space as mouse_position(). The touch path mirrors the mouse path: down
    // state persists until the matching Ended, and the begin/end edges are
    // cleared by begin_frame().
    void on_touch(TouchPhase phase, u32 id, f32 x, f32 y) noexcept;

    // --- Gamepad -----------------------------------------------------------
    //
    // A gamepad is *polled*, not event-driven, because that is how every desktop
    // API exposes it: the backend reads the pad's current button/axis snapshot
    // once per frame and feeds it in, rather than receiving a stream of changes.
    // `begin_frame()` therefore does not clear gamepad state the way it clears
    // key edges; the backend re-reports the whole pad each frame and `on_gamepad_*`
    // derives the edges by comparing against the previous snapshot.
    //
    // A slot is keyed by a small index (0..kMaxGamepads-1), not by a hardware id,
    // because a pad is connected and disconnected at runtime and gameplay wants
    // "the first pad", not a USB serial. `on_gamepad_connected(slot, true)`
    // claims a slot and marks it present; `false` releases it and clears its
    // state, so a disconnected pad reads as no input rather than a stuck one.

    // Marks a slot present or absent. A newly-connected pad starts from a clean
    // state, so a reconnect does not inherit the previous pad's held buttons.
    void on_gamepad_connected(usize slot, bool connected) noexcept;

    // Feeds one button's state for a slot. Edge flags are derived here against
    // the previous frame's value, so a caller reports the current state every
    // frame and the press/release edges come out right.
    void on_gamepad_button(usize slot, GamepadButton button, bool down) noexcept;

    // Feeds one analog axis for a slot. The value is clamped to the axis's
    // range (see GamepadAxis), so a backend that reports a raw ADC value cannot
    // hand gameplay an out-of-range number.
    void on_gamepad_axis(usize slot, GamepadAxis axis, f32 value) noexcept;

    [[nodiscard]] bool is_gamepad_connected(usize slot) const noexcept;

    [[nodiscard]] bool is_gamepad_button_down(usize slot, GamepadButton button) const noexcept;
    [[nodiscard]] bool was_gamepad_button_pressed(usize slot, GamepadButton button) const noexcept;
    [[nodiscard]] bool was_gamepad_button_released(usize slot, GamepadButton button) const noexcept;

    // The axis's current value: -1..1 for a stick, 0..1 for a trigger. Zero for
    // a slot that is not connected, so a missing pad reads as centred.
    [[nodiscard]] f32 gamepad_axis(usize slot, GamepadAxis axis) const noexcept;

    // The first connected slot, or kInvalidGamepad when none is. This is the
    // query a single-player game wants: it does not care which USB port the pad
    // is in, only that one exists.
    [[nodiscard]] usize first_connected_gamepad() const noexcept;
    static constexpr usize kInvalidGamepad = static_cast<usize>(-1);

    [[nodiscard]] bool is_key_down(Key key) const noexcept;
    [[nodiscard]] bool was_key_pressed(Key key) const noexcept;
    [[nodiscard]] bool was_key_released(Key key) const noexcept;

    [[nodiscard]] bool is_mouse_down(MouseButton button) const noexcept;
    [[nodiscard]] bool was_mouse_pressed(MouseButton button) const noexcept;
    [[nodiscard]] bool was_mouse_released(MouseButton button) const noexcept;

    [[nodiscard]] Vec2 mouse_position() const noexcept { return mouse_position_; }
    [[nodiscard]] Vec2 mouse_delta() const noexcept { return mouse_delta_; }
    [[nodiscard]] Vec2 scroll_delta() const noexcept { return scroll_delta_; }

    // Touch queries, keyed by the backend's contact id. `is_touch_down` is true
    // from the Began event until the matching Ended event; `was_touch_began`
    // and `was_touch_ended` are edges that live for exactly one frame.
    // `touch_position` is the id's last known position (zero for an id that is
    // not down). `touch_delta` is the movement accumulated during the current
    // frame, cleared by begin_frame() like mouse_delta().
    [[nodiscard]] bool is_touch_down(u32 id) const noexcept;
    [[nodiscard]] bool was_touch_began(u32 id) const noexcept;
    [[nodiscard]] bool was_touch_ended(u32 id) const noexcept;
    [[nodiscard]] Vec2 touch_position(u32 id) const noexcept;
    [[nodiscard]] Vec2 touch_delta(u32 id) const noexcept;

    // Enumeration, for a caller that does not know the backend's ids and has to
    // find the contacts itself. An action map is exactly that caller: it maps a
    // contact's position to an action without caring which finger it is.
    // `touch_count()` counts tracked contacts (including one that ended this
    // frame, whose end edge is still observable); `touch_id_at()` returns the
    // id at an index below that count, or `kInvalidTouchId` when out of range.
    [[nodiscard]] usize touch_count() const noexcept;
    [[nodiscard]] u32 touch_id_at(usize index) const noexcept;

    // Returned by touch_id_at() for an index with no contact behind it. Never a
    // real id: a backend's first contact is 0, and ids only grow from there.
    static constexpr u32 kInvalidTouchId = 0xFFFFFFFFu;

    // Releases every key/button/touch without clearing edge history. Used when
    // the window loses focus so that a held key does not stick down.
    void release_all() noexcept;

    // Whether any key, mouse button, or touch contact began this frame. This is
    // a "user gesture" in the platform sense: the signal a browser requires
    // before it will let a page make sound (AudioDevice::needs_user_gesture).
    // Edge-based, so it is true for exactly the frame each input starts in.
    [[nodiscard]] bool any_pressed() const noexcept;

private:
    [[nodiscard]] static usize index_of(Key key) noexcept {
        const auto value = static_cast<usize>(key);
        return value < kKeyCount ? value : 0;
    }
    [[nodiscard]] static usize index_of(MouseButton button) noexcept {
        const auto value = static_cast<usize>(button);
        return value < kMouseButtonCount ? value : 0;
    }
    // Touch ids are arbitrary u32 values, so a fixed slot array tracks them:
    // `find_touch` returns the slot index of a tracked id, or kMaxTouchPoints
    // when the id is unknown.
    [[nodiscard]] usize find_touch(u32 id) const noexcept;

    std::array<bool, kKeyCount> keys_down_{};
    std::array<bool, kKeyCount> keys_pressed_{};
    std::array<bool, kKeyCount> keys_released_{};

    std::array<bool, kMouseButtonCount> buttons_down_{};
    std::array<bool, kMouseButtonCount> buttons_pressed_{};
    std::array<bool, kMouseButtonCount> buttons_released_{};

    // One slot per tracked contact. `active` means the slot is occupied by a
    // known id (it stays occupied for the frame in which the contact ends, so
    // the end edge can be read, and is freed at the next begin_frame()); `down`
    // means the contact is currently touching.
    struct TouchPoint {
        u32 id{};
        bool active{};
        bool down{};
        bool began{};
        bool ended{};
        Vec2 position{};
        Vec2 delta{};
    };
    std::array<TouchPoint, kMaxTouchPoints> touches_{};

    // One slot per tracked gamepad. The button edges are derived at feed time by
    // comparing against the last reported value, and cleared by begin_frame()
    // exactly like the key edges - so they are correct whether a backend
    // re-reports the whole pad every frame (the usual desktop case) or reports
    // only the changes (an event-driven one).
    struct Gamepad {
        bool connected{};
        std::array<bool, kGamepadButtonCount> buttons{};
        std::array<bool, kGamepadButtonCount> pressed{};
        std::array<bool, kGamepadButtonCount> released{};
        std::array<f32, kGamepadAxisCount> axes{};
    };
    std::array<Gamepad, kMaxGamepads> gamepads_{};

    Vec2 mouse_position_{};
    Vec2 mouse_delta_{};
    Vec2 scroll_delta_{};
};

} // namespace fe
