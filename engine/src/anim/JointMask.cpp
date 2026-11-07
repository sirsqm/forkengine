// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/anim/JointMask.h"

#include <nlohmann/json.hpp>

#include <algorithm>

namespace fe::anim {

void JointMask::add(std::string_view joint_name) {
    if (joint_name.empty()) {
        return;
    }
    const std::string name{joint_name};
    if (std::find(names_.begin(), names_.end(), name) == names_.end()) {
        names_.push_back(name);
    }
}

usize JointMask::resolve(const Skeleton& skeleton) {
    indices_.clear();
    resolved_ = true;
    for (const std::string& name : names_) {
        const i32 index = skeleton.find_joint(name);
        if (index < 0) {
            continue;
        }
        indices_.push_back(static_cast<usize>(index));
    }
    std::sort(indices_.begin(), indices_.end());
    indices_.erase(std::unique(indices_.begin(), indices_.end()), indices_.end());
    return indices_.size();
}

bool JointMask::contains(usize joint_index) const noexcept {
    if (names_.empty()) {
        return true; // an empty mask restricts nothing
    }
    if (!resolved_) {
        return true; // unresolved: the unmasked default, never a silent no-op
    }
    return std::binary_search(indices_.begin(), indices_.end(), joint_index);
}

std::string JointMask::to_json() const {
    nlohmann::json document;
    document["joints"] = names_;
    return document.dump();
}

bool JointMask::from_json(std::string_view text) {
    nlohmann::json document;
    try {
        document = nlohmann::json::parse(text);
    } catch (const nlohmann::json::exception&) {
        return false;
    }
    if (!document.is_object()) {
        return false;
    }
    names_.clear();
    indices_.clear();
    resolved_ = false;
    if (document.contains("joints") && document.at("joints").is_array()) {
        for (const auto& entry : document.at("joints")) {
            if (entry.is_string()) {
                add(entry.get<std::string>());
            }
        }
    }
    return true;
}

} // namespace fe::anim
