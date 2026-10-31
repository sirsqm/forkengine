// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "GlfwWindow.h"

#include "fe/core/Log.h"

// GLFW is an implementation detail and never appears in a public header.
#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>

#include <mutex>
#include <string>

namespace fe {
namespace {

std::mutex g_init_mutex;
u32 g_window_count = 0;

// Translates a GLFW key code into the engine's Key enum. GLFW's key codes for
// letters, digits, and function keys are already contiguous ASCII-ish values,
// but we map explicitly so a future GLFW change cannot silently shift them.
[[nodiscard]] Key translate_key(int glfw_key) noexcept {
    switch (glfw_key) {
    case GLFW_KEY_A:
        return Key::A;
    case GLFW_KEY_B:
        return Key::B;
    case GLFW_KEY_C:
        return Key::C;
    case GLFW_KEY_D:
        return Key::D;
    case GLFW_KEY_E:
        return Key::E;
    case GLFW_KEY_F:
        return Key::F;
    case GLFW_KEY_G:
        return Key::G;
    case GLFW_KEY_H:
        return Key::H;
    case GLFW_KEY_I:
        return Key::I;
    case GLFW_KEY_J:
        return Key::J;
    case GLFW_KEY_K:
        return Key::K;
    case GLFW_KEY_L:
        return Key::L;
    case GLFW_KEY_M:
        return Key::M;
    case GLFW_KEY_N:
        return Key::N;
    case GLFW_KEY_O:
        return Key::O;
    case GLFW_KEY_P:
        return Key::P;
    case GLFW_KEY_Q:
        return Key::Q;
    case GLFW_KEY_R:
        return Key::R;
    case GLFW_KEY_S:
        return Key::S;
    case GLFW_KEY_T:
        return Key::T;
    case GLFW_KEY_U:
        return Key::U;
    case GLFW_KEY_V:
        return Key::V;
    case GLFW_KEY_W:
        return Key::W;
    case GLFW_KEY_X:
        return Key::X;
    case GLFW_KEY_Y:
        return Key::Y;
    case GLFW_KEY_Z:
        return Key::Z;
    case GLFW_KEY_0:
        return Key::Num0;
    case GLFW_KEY_1:
        return Key::Num1;
    case GLFW_KEY_2:
        return Key::Num2;
    case GLFW_KEY_3:
        return Key::Num3;
    case GLFW_KEY_4:
        return Key::Num4;
    case GLFW_KEY_5:
        return Key::Num5;
    case GLFW_KEY_6:
        return Key::Num6;
    case GLFW_KEY_7:
        return Key::Num7;
    case GLFW_KEY_8:
        return Key::Num8;
    case GLFW_KEY_9:
        return Key::Num9;
    case GLFW_KEY_ESCAPE:
        return Key::Escape;
    case GLFW_KEY_ENTER:
        return Key::Enter;
    case GLFW_KEY_TAB:
        return Key::Tab;
    case GLFW_KEY_BACKSPACE:
        return Key::Backspace;
    case GLFW_KEY_INSERT:
        return Key::Insert;
    case GLFW_KEY_DELETE:
        return Key::Delete;
    case GLFW_KEY_RIGHT:
        return Key::Right;
    case GLFW_KEY_LEFT:
        return Key::Left;
    case GLFW_KEY_DOWN:
        return Key::Down;
    case GLFW_KEY_UP:
        return Key::Up;
    case GLFW_KEY_PAGE_UP:
        return Key::PageUp;
    case GLFW_KEY_PAGE_DOWN:
        return Key::PageDown;
    case GLFW_KEY_HOME:
        return Key::Home;
    case GLFW_KEY_END:
        return Key::End;
    case GLFW_KEY_SPACE:
        return Key::Space;
    case GLFW_KEY_MINUS:
        return Key::Minus;
    case GLFW_KEY_EQUAL:
        return Key::Equal;
    case GLFW_KEY_LEFT_BRACKET:
        return Key::LeftBracket;
    case GLFW_KEY_RIGHT_BRACKET:
        return Key::RightBracket;
    case GLFW_KEY_BACKSLASH:
        return Key::Backslash;
    case GLFW_KEY_SEMICOLON:
        return Key::Semicolon;
    case GLFW_KEY_APOSTROPHE:
        return Key::Apostrophe;
    case GLFW_KEY_COMMA:
        return Key::Comma;
    case GLFW_KEY_PERIOD:
        return Key::Period;
    case GLFW_KEY_SLASH:
        return Key::Slash;
    case GLFW_KEY_GRAVE_ACCENT:
        return Key::Grave;
    case GLFW_KEY_F1:
        return Key::F1;
    case GLFW_KEY_F2:
        return Key::F2;
    case GLFW_KEY_F3:
        return Key::F3;
    case GLFW_KEY_F4:
        return Key::F4;
    case GLFW_KEY_F5:
        return Key::F5;
    case GLFW_KEY_F6:
        return Key::F6;
    case GLFW_KEY_F7:
        return Key::F7;
    case GLFW_KEY_F8:
        return Key::F8;
    case GLFW_KEY_F9:
        return Key::F9;
    case GLFW_KEY_F10:
        return Key::F10;
    case GLFW_KEY_F11:
        return Key::F11;
    case GLFW_KEY_F12:
        return Key::F12;
    case GLFW_KEY_LEFT_SHIFT:
        return Key::LeftShift;
    case GLFW_KEY_RIGHT_SHIFT:
        return Key::RightShift;
    case GLFW_KEY_LEFT_CONTROL:
        return Key::LeftControl;
    case GLFW_KEY_RIGHT_CONTROL:
        return Key::RightControl;
    case GLFW_KEY_LEFT_ALT:
        return Key::LeftAlt;
    case GLFW_KEY_RIGHT_ALT:
        return Key::RightAlt;
    default:
        return Key::Unknown;
    }
}

[[nodiscard]] MouseButton translate_mouse_button(int glfw_button) noexcept {
    switch (glfw_button) {
    case GLFW_MOUSE_BUTTON_LEFT:
        return MouseButton::Left;
    case GLFW_MOUSE_BUTTON_RIGHT:
        return MouseButton::Right;
    case GLFW_MOUSE_BUTTON_MIDDLE:
        return MouseButton::Middle;
    default:
        return MouseButton::Count;
    }
}

// The GLFW callbacks receive the engine Window through the user pointer. This
// is how the platform layer keeps its event handling out of global state.
[[nodiscard]] Window* window_from(GLFWwindow* handle) noexcept {
    return static_cast<Window*>(glfwGetWindowUserPointer(handle));
}

// Maps an engine GamepadButton index onto GLFW's standard gamepad layout.
// GLFW's `GLFW_GAMEPAD_BUTTON_*` values are a contiguous 0..14 block in the
// same order the engine enum uses, so the mapping is the identity - but it is
// written out explicitly so a change in either numbering is a compile-time
// mismatch rather than a silently swapped button. Returns -1 for an index
// GLFW's layout does not carry.
[[nodiscard]] int translate_gamepad_button(usize engine_index) noexcept {
    switch (static_cast<GamepadButton>(engine_index)) {
    case GamepadButton::South:
        return GLFW_GAMEPAD_BUTTON_A;
    case GamepadButton::East:
        return GLFW_GAMEPAD_BUTTON_B;
    case GamepadButton::West:
        return GLFW_GAMEPAD_BUTTON_X;
    case GamepadButton::North:
        return GLFW_GAMEPAD_BUTTON_Y;
    case GamepadButton::LeftBumper:
        return GLFW_GAMEPAD_BUTTON_LEFT_BUMPER;
    case GamepadButton::RightBumper:
        return GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER;
    case GamepadButton::Back:
        return GLFW_GAMEPAD_BUTTON_BACK;
    case GamepadButton::Start:
        return GLFW_GAMEPAD_BUTTON_START;
    case GamepadButton::Guide:
        return GLFW_GAMEPAD_BUTTON_GUIDE;
    case GamepadButton::LeftStick:
        return GLFW_GAMEPAD_BUTTON_LEFT_THUMB;
    case GamepadButton::RightStick:
        return GLFW_GAMEPAD_BUTTON_RIGHT_THUMB;
    case GamepadButton::DpadUp:
        return GLFW_GAMEPAD_BUTTON_DPAD_UP;
    case GamepadButton::DpadDown:
        return GLFW_GAMEPAD_BUTTON_DPAD_DOWN;
    case GamepadButton::DpadLeft:
        return GLFW_GAMEPAD_BUTTON_DPAD_LEFT;
    case GamepadButton::DpadRight:
        return GLFW_GAMEPAD_BUTTON_DPAD_RIGHT;
    default:
        return -1;
    }
}

// Maps an engine GamepadAxis index onto GLFW's standard gamepad layout. Written
// out for the same reason as the button table. Returns -1 for an index GLFW's
// layout does not carry.
[[nodiscard]] int translate_gamepad_axis(usize engine_index) noexcept {
    switch (static_cast<GamepadAxis>(engine_index)) {
    case GamepadAxis::LeftX:
        return GLFW_GAMEPAD_AXIS_LEFT_X;
    case GamepadAxis::LeftY:
        return GLFW_GAMEPAD_AXIS_LEFT_Y;
    case GamepadAxis::RightX:
        return GLFW_GAMEPAD_AXIS_RIGHT_X;
    case GamepadAxis::RightY:
        return GLFW_GAMEPAD_AXIS_RIGHT_Y;
    case GamepadAxis::LeftTrigger:
        return GLFW_GAMEPAD_AXIS_LEFT_TRIGGER;
    case GamepadAxis::RightTrigger:
        return GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER;
    default:
        return -1;
    }
}

void key_callback(GLFWwindow* handle, int key, int /*scancode*/, int action, int /*mods*/) {
    Window* window = window_from(handle);
    if (window == nullptr) {
        return;
    }
    const Key translated = translate_key(key);
    if (translated == Key::Unknown) {
        return;
    }
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        window->input().on_key(translated, true);
    } else if (action == GLFW_RELEASE) {
        window->input().on_key(translated, false);
    }
}

void mouse_button_callback(GLFWwindow* handle, int button, int action, int /*mods*/) {
    Window* window = window_from(handle);
    if (window == nullptr) {
        return;
    }
    const MouseButton translated = translate_mouse_button(button);
    if (translated == MouseButton::Count) {
        return;
    }
    window->input().on_mouse_button(translated, action == GLFW_PRESS);
}

void cursor_position_callback(GLFWwindow* handle, double x, double y) {
    Window* window = window_from(handle);
    if (window == nullptr) {
        return;
    }
    window->input().on_mouse_move(static_cast<f32>(x), static_cast<f32>(y));
}

void scroll_callback(GLFWwindow* handle, double dx, double dy) {
    Window* window = window_from(handle);
    if (window == nullptr) {
        return;
    }
    window->input().on_scroll(static_cast<f32>(dx), static_cast<f32>(dy));
}

void error_callback(int code, const char* description) {
    FE_LOG_ERROR("glfw error %d: %s", code, description);
}

} // namespace

GlfwWindow::GlfwWindow() : Window("GLFW") {}

GlfwWindow::~GlfwWindow() {
    auto* handle = static_cast<GLFWwindow*>(native_handle());
    if (handle == nullptr) {
        return;
    }
    glfwDestroyWindow(handle);
    set_native_handle(nullptr);

    std::lock_guard lock(g_init_mutex);
    if (g_window_count > 0) {
        --g_window_count;
        if (g_window_count == 0) {
            glfwTerminate();
        }
    }
}

Result<std::unique_ptr<Window>> GlfwWindow::create(const WindowDesc& desc) {
    {
        std::lock_guard lock(g_init_mutex);
        if (g_window_count == 0) {
            glfwSetErrorCallback(&error_callback);
            if (glfwInit() != GLFW_TRUE) {
                return Error("glfwInit failed: no display or platform backend available");
            }
        }
        ++g_window_count;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, static_cast<int>(desc.gl_major));
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, static_cast<int>(desc.gl_minor));
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, desc.resizable ? GLFW_TRUE : GLFW_FALSE);
    glfwWindowHint(GLFW_VISIBLE, desc.visible ? GLFW_TRUE : GLFW_FALSE);

    GLFWwindow* handle = glfwCreateWindow(static_cast<int>(desc.width), static_cast<int>(desc.height),
                                          desc.title.c_str(), nullptr, nullptr);
    if (handle == nullptr) {
        std::lock_guard lock(g_init_mutex);
        --g_window_count;
        if (g_window_count == 0) {
            glfwTerminate();
        }
        return Error("glfwCreateWindow failed: could not create an OpenGL " + std::to_string(desc.gl_major) + "." +
                     std::to_string(desc.gl_minor) + " core context");
    }

    auto window = std::unique_ptr<GlfwWindow>(new GlfwWindow());
    window->set_native_handle(handle);
    glfwSetWindowUserPointer(handle, window.get());

    glfwSetKeyCallback(handle, &key_callback);
    glfwSetMouseButtonCallback(handle, &mouse_button_callback);
    glfwSetCursorPosCallback(handle, &cursor_position_callback);
    glfwSetScrollCallback(handle, &scroll_callback);

    glfwMakeContextCurrent(handle);
    glfwSwapInterval(desc.vsync ? 1 : 0);

    // The icon is set after the window exists (GLFW has no hint for it). An
    // empty descriptor icon leaves the OS default.
    window->set_icon(desc.icon);

    // Seed the mouse position so the first delta is not measured from (0, 0).
    double cursor_x = 0.0;
    double cursor_y = 0.0;
    glfwGetCursorPos(handle, &cursor_x, &cursor_y);
    window->input().on_mouse_move(static_cast<f32>(cursor_x), static_cast<f32>(cursor_y));
    window->input().begin_frame();

    return std::unique_ptr<Window>(window.release());
}

void GlfwWindow::poll_events() {
    glfwPollEvents();
    poll_gamepads();
}

// Reads every gamepad GLFW knows about and feeds the engine's slots.
//
// GLFW exposes a gamepad as a *polled snapshot* rather than an event stream -
// `glfwGetGamepadState` returns the current state of every button and axis - so
// this runs once per frame from poll_events() and reports the whole pad. The
// engine derives the press/release edges from the change between frames.
//
// Only the standard-layout pads GLFW maps (`glfwJoystickIsGamepad`) are read: a
// raw joystick's button numbering is vendor-specific and cannot be mapped to the
// engine's positional layout without a per-device database. A pad GLFW cannot
// map is left unconnected rather than reported as a wrong set of buttons.
//
// Disconnection is handled by the same pass: a slot that was connected and whose
// `glfwJoystickIsGamepad` is now false is reported disconnected, so a pad pulled
// out mid-game releases its buttons instead of leaving one stuck down.
void GlfwWindow::poll_gamepads() {
    InputState& state_in = input();
    for (usize slot = 0; slot < kMaxGamepads; ++slot) {
        const int jid = static_cast<int>(slot);
        const bool present = glfwJoystickPresent(jid) == GLFW_TRUE && glfwJoystickIsGamepad(jid) == GLFW_TRUE;
        if (!present) {
            if (state_in.is_gamepad_connected(slot)) {
                state_in.on_gamepad_connected(slot, false);
            }
            continue;
        }
        if (!state_in.is_gamepad_connected(slot)) {
            state_in.on_gamepad_connected(slot, true);
        }
        GLFWgamepadstate state{};
        if (glfwGetGamepadState(jid, &state) != GLFW_TRUE) {
            continue;
        }
        for (usize i = 0; i < InputState::kGamepadButtonCount; ++i) {
            const int glfw_button = translate_gamepad_button(i);
            if (glfw_button < 0) {
                continue;
            }
            state_in.on_gamepad_button(slot, static_cast<GamepadButton>(i), state.buttons[glfw_button] == GLFW_PRESS);
        }
        for (usize i = 0; i < InputState::kGamepadAxisCount; ++i) {
            const int glfw_axis = translate_gamepad_axis(i);
            if (glfw_axis < 0) {
                continue;
            }
            state_in.on_gamepad_axis(slot, static_cast<GamepadAxis>(i), state.axes[glfw_axis]);
        }
    }
}

void GlfwWindow::swap_buffers() {
    glfwSwapBuffers(static_cast<GLFWwindow*>(native_handle()));
}

bool GlfwWindow::should_close() const {
    return glfwWindowShouldClose(static_cast<GLFWwindow*>(native_handle())) == GLFW_TRUE;
}

void GlfwWindow::request_close() {
    glfwSetWindowShouldClose(static_cast<GLFWwindow*>(native_handle()), GLFW_TRUE);
}

u32 GlfwWindow::width() const noexcept {
    int w = 0;
    int h = 0;
    glfwGetWindowSize(static_cast<GLFWwindow*>(native_handle()), &w, &h);
    return w > 0 ? static_cast<u32>(w) : 0U;
}

u32 GlfwWindow::height() const noexcept {
    int w = 0;
    int h = 0;
    glfwGetWindowSize(static_cast<GLFWwindow*>(native_handle()), &w, &h);
    return h > 0 ? static_cast<u32>(h) : 0U;
}

u32 GlfwWindow::framebuffer_width() const noexcept {
    int w = 0;
    int h = 0;
    glfwGetFramebufferSize(static_cast<GLFWwindow*>(native_handle()), &w, &h);
    return w > 0 ? static_cast<u32>(w) : 0U;
}

u32 GlfwWindow::framebuffer_height() const noexcept {
    int w = 0;
    int h = 0;
    glfwGetFramebufferSize(static_cast<GLFWwindow*>(native_handle()), &w, &h);
    return h > 0 ? static_cast<u32>(h) : 0U;
}

void GlfwWindow::set_title(const std::string& title) {
    glfwSetWindowTitle(static_cast<GLFWwindow*>(native_handle()), title.c_str());
}

void GlfwWindow::set_vsync(bool enabled) {
    glfwSwapInterval(enabled ? 1 : 0);
}

void GlfwWindow::set_icon(const WindowIcon& icon) {
    if (icon.empty()) {
        return;
    }
    // GLFW takes the icon as 32-bit RGBA with the top row first, which is the
    // layout WindowIcon already uses, so the pixels are handed over directly.
    // GLFW copies them, so the icon need not outlive the call.
    const GLFWimage image{static_cast<int>(icon.width()), static_cast<int>(icon.height()),
                          const_cast<unsigned char*>(icon.pixels().data())};
    glfwSetWindowIcon(static_cast<GLFWwindow*>(native_handle()), 1, &image);
}

void* GlfwWindow::get_proc_address(const char* name) const {
    return reinterpret_cast<void*>(glfwGetProcAddress(name));
}

} // namespace fe
