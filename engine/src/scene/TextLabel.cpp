// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/scene/TextLabel.h"

#include "fe/resources/Font.h"

#include <algorithm>

namespace fe {

f32 TextLabel::line_width(std::string_view line) const noexcept {
    const auto* as_font = dynamic_cast<const Font*>(font.get());
    if (as_font == nullptr) {
        return 0.0f;
    }
    // The atlas is laid out at a nominal pixel size and `size` scales it, so the
    // measured advance is scaled the same way the quads are. Without a nominal
    // size in the metrics the scale is the identity, which is what a font
    // generated at one pixel size wants.
    // The tracking sits between glyphs, so a line of N glyphs has N - 1 gaps.
    const f32 gaps = line.size() > 1 ? static_cast<f32>(line.size() - 1) : 0.0f;
    return as_font->measure(line) * size + tracking * gaps;
}

f32 TextLabel::measured_width() const noexcept {
    // Each `\n` ends a line, and the label is as wide as its widest one. A label
    // with no newline runs the loop once, so its answer is the single line's.
    f32 widest = 0.0f;
    std::string_view rest{text};
    for (;;) {
        const usize break_at = rest.find('\n');
        widest = std::max(widest, line_width(rest.substr(0, break_at)));
        if (break_at == std::string_view::npos) {
            return widest;
        }
        rest.remove_prefix(break_at + 1);
    }
}

} // namespace fe
