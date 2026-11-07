// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "slice_state.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>

namespace collect3d {

namespace {

fe::f32 clampf(fe::f32 value, fe::f32 low, fe::f32 high) noexcept {
    return std::min(std::max(value, low), high);
}

// Parses one "key=value" line's value as a float. A value that is not a finite
// number is rejected rather than clamped, because clamping a NaN would be
// meaningless and clamping an infinity would silently pick an end.
bool parse_float(std::string_view text, fe::f32& out) noexcept {
    if (text.empty()) {
        return false;
    }
    fe::f32 value = 0.0f;
    const auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (ec != std::errc{} || ptr != text.data() + text.size() || !std::isfinite(value)) {
        return false;
    }
    out = value;
    return true;
}

} // namespace

void Settings::set_volume(fe::f32 value) noexcept {
    volume = clampf(value, config::kMinVolume, config::kMaxVolume);
}

void Settings::adjust_volume(int steps) noexcept {
    set_volume(volume + config::kVolumeStep * static_cast<fe::f32>(steps));
}

std::string serialize_settings(const Settings& settings) {
    // One "key=value" line per field, in a fixed order, with a fixed precision so
    // a save round-trips through text exactly.
    char buffer[64];
    std::string text;
    std::snprintf(buffer, sizeof(buffer), "volume=%.3f\n", static_cast<double>(settings.volume));
    text += buffer;
    return text;
}

bool parse_settings(std::string_view text, Settings& out) noexcept {
    bool read_any = false;
    std::size_t pos = 0;
    while (pos < text.size()) {
        const std::size_t newline = text.find('\n', pos);
        const std::size_t end = newline == std::string_view::npos ? text.size() : newline;
        const std::string_view line = text.substr(pos, end - pos);
        pos = end + 1;

        const std::size_t eq = line.find('=');
        if (eq == std::string_view::npos) {
            continue; // not a key=value line: skip it rather than fail
        }
        const std::string_view key = line.substr(0, eq);
        const std::string_view value_text = line.substr(eq + 1);

        fe::f32 value = 0.0f;
        if (!parse_float(value_text, value)) {
            continue;
        }
        if (key == "volume") {
            out.set_volume(value);
            read_any = true;
        }
        // An unknown key is ignored, not fatal: a save from a newer build that
        // added a setting must still load here.
    }
    return read_any;
}

CollectState::CollectState(int collectible_count, fe::f32 time_limit) noexcept
    : collectibles_total_(std::max(collectible_count, 0)), collectibles_remaining_(std::max(collectible_count, 0)),
      time_limit_(std::max(time_limit, 0.0f)), time_left_(std::max(time_limit, 0.0f)) {}

const char* CollectState::result_name() const noexcept {
    switch (result_) {
    case Result::Won:
        return "win";
    case Result::Lost:
        return "lose";
    case Result::Playing:
        break;
    }
    return "playing";
}

const char* CollectState::screen_name() const noexcept {
    switch (screen_) {
    case Screen::Title:
        return "title";
    case Screen::Playing:
        return "playing";
    case Screen::Paused:
        return "paused";
    case Screen::Settings:
        return "settings";
    case Screen::GameOver:
        return "gameover";
    }
    return "playing";
}

int CollectState::seconds_left() const noexcept {
    // Rounded up, so a clock reading "0" means "expired" and a run in its last
    // fractional second still shows "1" rather than "0" while time remains.
    return static_cast<int>(std::ceil(time_left_));
}

void CollectState::start_game() noexcept {
    // A fresh arena and the clock running. No scene rebuild is requested: the
    // caller has already built (or is about to build) a fresh board, so starting
    // from it has nothing to rebuild.
    reset();
    screen_ = Screen::Playing;
}

void CollectState::return_to_title() noexcept {
    // A fresh board behind the title, so the title never shows a half-collected
    // arena from the game that just ended. The rebuild is requested because the
    // caller owns the scene and the collected gems must reappear.
    reset();
    screen_ = Screen::Title;
    request_restart();
}

void CollectState::show_title() noexcept {
    // Puts the game on the title without rebuilding: used once by the application
    // before its first frame, when the scene it just built is already fresh.
    screen_ = Screen::Title;
}

void CollectState::pause() noexcept {
    if (screen_ == Screen::Playing) {
        screen_ = Screen::Paused;
    }
}

void CollectState::resume() noexcept {
    if (screen_ == Screen::Paused) {
        screen_ = Screen::Playing;
    }
}

void CollectState::open_settings() noexcept {
    if (screen_ == Screen::Title || screen_ == Screen::Paused) {
        settings_return_ = screen_;
        screen_ = Screen::Settings;
    }
}

void CollectState::close_settings() noexcept {
    if (screen_ == Screen::Settings) {
        screen_ = settings_return_;
    }
}

void CollectState::sync_screen() noexcept {
    if (result_ != Result::Playing) {
        screen_ = Screen::GameOver;
    }
}

bool CollectState::collect_one() noexcept {
    if (result_ != Result::Playing || collectibles_remaining_ <= 0) {
        return false;
    }
    --collectibles_remaining_;
    ++collected_;
    score_ += config::kPointsPerCollectible;
    if (collectibles_remaining_ == 0) {
        score_ += config::kGoalBonus;
        set_over(Result::Won);
    }
    return true;
}

void CollectState::tick(fe::f32 delta) noexcept {
    if (result_ != Result::Playing || screen_ != Screen::Playing) {
        return;
    }
    if (delta <= 0.0f) {
        return;
    }
    time_left_ = std::max(time_left_ - delta, 0.0f);
    if (time_left_ <= 0.0f) {
        set_over(Result::Lost);
    }
}

void CollectState::set_best_score(int value) noexcept {
    // A loaded save may be lower than a record this process already set (a save
    // from an older run, or a corrupt file); the record only ever rises, so a
    // lower value is ignored rather than trusted. A negative value is not a score.
    if (value < 0) {
        return;
    }
    if (!has_best_score_ || value > best_score_) {
        best_score_ = value;
        has_best_score_ = true;
    }
}

bool CollectState::record_best_score() noexcept {
    if (score_ > best_score_) {
        best_score_ = score_;
        has_best_score_ = true;
        return true;
    }
    // A first game that ends on zero still has a best score of zero, which is
    // worth knowing (the HUD stops leaving the line blank) even though the number
    // did not move.
    if (!has_best_score_) {
        best_score_ = score_;
        has_best_score_ = true;
        return true;
    }
    return false;
}

void CollectState::reset() noexcept {
    collectibles_remaining_ = collectibles_total_;
    collected_ = 0;
    score_ = 0;
    time_left_ = time_limit_;
    result_ = Result::Playing;
    restart_requested_ = false;
}

bool CollectState::consume_restart() noexcept {
    const bool requested = restart_requested_;
    restart_requested_ = false;
    return requested;
}

} // namespace collect3d
