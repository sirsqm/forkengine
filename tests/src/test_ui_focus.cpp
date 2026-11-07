// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
//
// Tests for keyboard/gamepad focus navigation of a UI (ADR-0145).
//
// Focus is written by `UiFocusSystem` and read off each element's
// `interaction`. The tests drive an `InputState` and an `ActionMap` directly,
// which is the same seam every other input consumer is tested through, and assert
// on which element carries `focused`/`activated` after a frame.

#include <doctest/doctest.h>

#include "fe/platform/InputActions.h"
#include "fe/platform/InputState.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/systems/FrameContext.h"
#include "fe/ui/UiElement.h"
#include "fe/ui/UiFocusSystem.h"
#include "fe/ui/UiLayout.h"
#include "fe/ui/UiTypes.h"

#include <string>

using namespace fe;
using namespace fe::ui;

namespace {

Node* add_canvas(Scene& scene) {
    Node* node = scene.create_node("Canvas");
    auto* canvas = node->add_component<UiCanvas>();
    canvas->design_width = 1280.0f;
    canvas->design_height = 720.0f;
    return node;
}

// A focusable button at a fixed design position, so the geometry focus navigates
// by is the geometry the test placed.
Node* add_button(Node* parent, std::string name, Vec2 offset, Vec2 size = {200.0f, 40.0f}) {
    Node* node = parent->create_child<Node>(std::move(name));
    auto* element = node->add_component<UiElement>();
    element->width = UiSize::fixed(size.x);
    element->height = UiSize::fixed(size.y);
    element->anchor = UiAnchor::TopLeft;
    element->offset = offset;
    element->focusable = true;
    return node;
}

[[nodiscard]] FrameContext make_context(InputState& input, Vec2 viewport = {1280.0f, 720.0f}) {
    FrameContext context;
    context.input = &input;
    context.viewport = viewport;
    context.delta_time = 1.0f / 60.0f;
    return context;
}

[[nodiscard]] bool focused(const Node* node) {
    const auto* element = node->get_component<UiElement>();
    return element != nullptr && element->interaction.focused;
}

[[nodiscard]] bool activated(const Node* node) {
    const auto* element = node->get_component<UiElement>();
    return element != nullptr && element->interaction.activated;
}

} // namespace

TEST_CASE("the first focusable element takes focus when nothing is focused") {
    Scene scene;
    Node* canvas = add_canvas(scene);
    Node* first = add_button(canvas, "One", {0.0f, 0.0f});
    add_button(canvas, "Two", {0.0f, 60.0f});

    InputState input;
    input.begin_frame();
    FrameContext context = make_context(input);

    UiFocusSystem system;
    system.update(context, scene);

    CHECK(focused(first));
    CHECK(system.focused_node() == first);
}

TEST_CASE("a down press moves focus to the element below") {
    Scene scene;
    Node* canvas = add_canvas(scene);
    add_button(canvas, "One", {0.0f, 0.0f});
    Node* second = add_button(canvas, "Two", {0.0f, 60.0f});

    InputState input;
    UiFocusSystem system;

    input.begin_frame();
    FrameContext context = make_context(input);
    system.update(context, scene);

    // Press Down: an edge, so focus moves one stop.
    input.begin_frame();
    input.on_key(Key::Down, true);
    system.update(context, scene);

    CHECK(focused(second));
    CHECK(system.focused_node() == second);
}

TEST_CASE("focus follows geometry, not tree order") {
    Scene scene;
    Node* canvas = add_canvas(scene);
    // "Top" is added first but is *below* "Bottom" on screen, so a press of Down
    // must go by position, not by the order the nodes were created.
    Node* lower = add_button(canvas, "Lower", {0.0f, 120.0f});
    add_button(canvas, "Upper", {0.0f, 0.0f});

    InputState input;
    UiFocusSystem system;

    // Start on the first-created element ("Lower"), then press Up: the element
    // above it ("Upper") is the one by geometry.
    input.begin_frame();
    FrameContext context = make_context(input);
    system.update(context, scene);
    CHECK(focused(lower));

    input.begin_frame();
    input.on_key(Key::Up, true);
    system.update(context, scene);

    CHECK(focused(scene.find_node_by_name("Upper")));
}

TEST_CASE("a press in a direction with nothing there keeps focus") {
    Scene scene;
    Node* canvas = add_canvas(scene);
    Node* only = add_button(canvas, "Only", {0.0f, 0.0f});

    InputState input;
    UiFocusSystem system;
    input.begin_frame();
    FrameContext context = make_context(input);
    system.update(context, scene);

    input.begin_frame();
    input.on_key(Key::Down, true);
    system.update(context, scene);

    CHECK(focused(only));
}

TEST_CASE("a non-focusable element is not a focus stop") {
    Scene scene;
    Node* canvas = add_canvas(scene);
    Node* label = add_button(canvas, "Label", {0.0f, 0.0f});
    label->get_component<UiElement>()->focusable = false;
    Node* button = add_button(canvas, "Button", {0.0f, 60.0f});

    InputState input;
    input.begin_frame();
    FrameContext context = make_context(input);

    UiFocusSystem system;
    system.update(context, scene);

    CHECK(focused(button));
    CHECK_FALSE(focused(label));
}

TEST_CASE("a disabled element is skipped by focus navigation") {
    Scene scene;
    Node* canvas = add_canvas(scene);
    Node* enabled = add_button(canvas, "Enabled", {0.0f, 0.0f});
    Node* disabled = add_button(canvas, "Disabled", {0.0f, 60.0f});
    disabled->get_component<UiElement>()->interactable = false;

    InputState input;
    input.begin_frame();
    FrameContext context = make_context(input);

    UiFocusSystem system;
    system.update(context, scene);
    CHECK(focused(enabled));

    input.begin_frame();
    input.on_key(Key::Down, true);
    system.update(context, scene);
    // The only element below is disabled, so focus stays where it was.
    CHECK(focused(enabled));
    CHECK_FALSE(focused(disabled));
}

TEST_CASE("confirm activates the focused element") {
    Scene scene;
    Node* canvas = add_canvas(scene);
    Node* first = add_button(canvas, "One", {0.0f, 0.0f});

    InputState input;
    input.begin_frame();
    FrameContext context = make_context(input);

    UiFocusSystem system;
    system.update(context, scene);

    input.begin_frame();
    input.on_key(Key::Enter, true);
    system.update(context, scene);

    CHECK(activated(first));
    CHECK(system.activated_this_frame());
}

TEST_CASE("focus is only reported on the frame it moves") {
    Scene scene;
    Node* canvas = add_canvas(scene);
    Node* first = add_button(canvas, "One", {0.0f, 0.0f});

    InputState input;
    UiFocusSystem system;
    input.begin_frame();
    FrameContext context = make_context(input);
    system.update(context, scene);

    // A frame with no input: focus stays, but nothing is activated.
    input.begin_frame();
    system.update(context, scene);
    CHECK(focused(first));
    CHECK_FALSE(activated(first));
    CHECK_FALSE(system.activated_this_frame());
}

TEST_CASE("focus moves with a bound navigation action") {
    Scene scene;
    Node* canvas = add_canvas(scene);
    add_button(canvas, "One", {0.0f, 0.0f});
    Node* second = add_button(canvas, "Two", {0.0f, 60.0f});

    ActionMap actions;
    const ActionMap::Id down = actions.add("MenuDown");
    actions.bind_key(down, Key::S);

    InputState input;
    input.begin_frame();
    FrameContext context = make_context(input);
    context.actions = &actions;
    actions.update(input, Vec2{1280.0f, 720.0f});

    UiFocusSystem system;
    system.set_navigation_actions(ActionMap::kNone, down, ActionMap::kNone, ActionMap::kNone);
    system.update(context, scene);

    input.begin_frame();
    input.on_key(Key::S, true);
    actions.update(input, Vec2{1280.0f, 720.0f});
    system.update(context, scene);

    CHECK(focused(second));
}

TEST_CASE("a held direction repeats focus on a steady cadence") {
    Scene scene;
    Node* canvas = add_canvas(scene);
    add_button(canvas, "One", {0.0f, 0.0f});
    add_button(canvas, "Two", {0.0f, 60.0f});
    Node* third = add_button(canvas, "Three", {0.0f, 120.0f});

    InputState input;
    UiFocusSystem system;

    input.begin_frame();
    FrameContext context = make_context(input);
    system.update(context, scene);

    // Hold Down across several frames. The first frame is the press edge; the
    // later frames repeat once the cadence elapses, so by the time enough frames
    // have run focus has walked down the whole column.
    input.begin_frame();
    input.on_key(Key::Down, true);
    system.update(context, scene);
    for (int i = 0; i < 30; ++i) {
        input.begin_frame();
        input.on_key(Key::Down, true);
        system.update(context, scene);
    }

    CHECK(focused(third));
}

TEST_CASE("clearing focus restarts on the first stop") {
    Scene scene;
    Node* canvas = add_canvas(scene);
    Node* first = add_button(canvas, "One", {0.0f, 0.0f});
    add_button(canvas, "Two", {0.0f, 60.0f});

    InputState input;
    UiFocusSystem system;
    input.begin_frame();
    FrameContext context = make_context(input);
    system.update(context, scene);

    input.begin_frame();
    input.on_key(Key::Down, true);
    system.update(context, scene);
    CHECK_FALSE(focused(first));

    system.clear_focus();
    input.begin_frame();
    system.update(context, scene);
    CHECK(focused(first));
}

TEST_CASE("a frame with no input clears the focus ring") {
    Scene scene;
    Node* canvas = add_canvas(scene);
    Node* first = add_button(canvas, "One", {0.0f, 0.0f});

    InputState input;
    UiFocusSystem system;
    input.begin_frame();
    FrameContext context = make_context(input);
    system.update(context, scene);
    CHECK(focused(first));

    // A frame with neither input nor actions: the ring is cleared and no element
    // reports focus, so a torn-down window does not leave a stale highlight.
    FrameContext empty;
    system.update(empty, scene);
    CHECK_FALSE(focused(first));
}
