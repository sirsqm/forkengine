// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/anim/Skeleton.h"
#include "fe/core/Types.h"

#include <string>
#include <string_view>
#include <vector>

namespace fe::anim {

// The set of joints an animation layer applies to.
//
// A locomotion blend that drives the whole body is wrong for an additive layer
// that should move only the upper body: a hit reaction should not swing the
// legs, and a breathing idle should not fight the run cycle. A mask names the
// joints a layer is allowed to write, and every joint it does not name keeps
// whatever the base pose held.
//
// The names are the skeleton's joint names, resolved to indices against a
// skeleton once (`resolve`) so the per-frame step tests an index rather than
// searching a string list. A mask that names nothing means "every joint", which
// is the unmasked default, so a layer that wants no restriction carries no mask.
class JointMask {
public:
    // Adds a joint by name. A duplicate name is ignored.
    void add(std::string_view joint_name);

    // True when the mask names no joints, i.e. it restricts nothing.
    [[nodiscard]] bool empty() const noexcept { return names_.empty(); }

    // How many joint names the mask carries.
    [[nodiscard]] usize size() const noexcept { return names_.size(); }

    [[nodiscard]] const std::vector<std::string>& names() const noexcept { return names_; }

    // Resolves the names against `skeleton`, keeping the indices that exist, and
    // returns how many resolved. A name with no joint in the skeleton is dropped
    // from the resolved set (and reported by the count), so a mask authored
    // against a different rig restricts what it can rather than nothing or
    // everything. Calling again with another skeleton re-resolves from the names.
    usize resolve(const Skeleton& skeleton);

    // True when the resolved mask contains `joint_index`. An unresolved mask (one
    // that has never been `resolve`d) contains every joint, so a caller that
    // forgets to resolve gets the unmasked behaviour rather than a silent
    // no-op. An empty mask likewise contains every joint.
    [[nodiscard]] bool contains(usize joint_index) const noexcept;

    // True once `resolve` has run against a skeleton.
    [[nodiscard]] bool is_resolved() const noexcept { return resolved_; }

    [[nodiscard]] std::string to_json() const;
    bool from_json(std::string_view text);

private:
    std::vector<std::string> names_;
    // The resolved joint indices, sorted. Empty until `resolve` runs; an empty
    // resolved set after a resolve means "no joint matched", not "all joints".
    std::vector<usize> indices_;
    bool resolved_ = false;
};

} // namespace fe::anim
