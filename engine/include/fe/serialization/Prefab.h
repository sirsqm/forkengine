// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Error.h"
#include "fe/core/Types.h"

#include <string>
#include <string_view>

namespace fe {

class Node;
class Scene;
class ResourceManager;

namespace serialization {

class ScriptBinding;

// A reusable node subtree: a definition captured once and instantiated many
// times.
//
// The scene row left prefab instancing open. Without it a repeated object (a
// bullet, a tile's decoration) is a full node subtree authored again for every
// copy, so a scene with many copies costs memory and load time proportional to
// the count. A Prefab captures one subtree as its definition and produces
// independent instances from it on demand.
//
// The definition is the serialized subtree (the same document the scene
// serializer writes, one root), so a prefab is a scene fragment and the code
// that reads it is the code that already reads scenes -- no second format and no
// second parser. Capture and instantiate both go through the reflection-driven
// serializer, so a component that persists is one that instantiates, with no
// per-type work here.
//
// Instances are independent: instantiation assigns every node and component a
// fresh id and rewrites the intra-prefab references that name them, so two
// instances share the definition but not their state. Editing one instance
// leaves the other unchanged. See docs/adr/0133-prefab-instancing.md.
class Prefab {
public:
    Prefab() = default;

    // Captures `node` and its subtree as a prefab definition. `name` labels the
    // prefab (it is not the node's name; the instance keeps the authored name).
    [[nodiscard]] static Result<Prefab> capture(std::string name, const Node& node,
                                                const ResourceManager* resources = nullptr,
                                                const ScriptBinding* scripts = nullptr);

    // Rebuilds a prefab from a previously serialized definition, as written by
    // `document()`. The document is validated lazily, at instantiation, so a
    // hand-authored prefab file can be loaded without a scene to instantiate it
    // into.
    [[nodiscard]] static Prefab from_document(std::string name, std::string document);

    [[nodiscard]] const std::string& name() const noexcept { return name_; }
    [[nodiscard]] const std::string& document() const noexcept { return document_; }

    // True when the prefab holds a definition (a captured or loaded one).
    [[nodiscard]] bool is_valid() const noexcept { return !document_.empty(); }

    // How many instances this prefab has produced. The instances share this one
    // definition; the count is the observable side of that sharing.
    [[nodiscard]] usize instance_count() const noexcept { return instance_count_; }

    // Appends a fresh instance of the subtree to `scene` and returns its root.
    //
    // Every node and component in the instance gets a new id, and any reference
    // inside the prefab that names one of them is rewritten to match, so the
    // instance is self-consistent and independent of the definition and of every
    // other instance. On failure the scene is left as it was.
    //
    // Gameplay scripts bound to the prefab are not carried into the instance
    // yet; a prefab captures and instantiates the node subtree and its
    // components. See docs/adr/0133-prefab-instancing.md.
    [[nodiscard]] Result<Node*> instantiate(Scene& scene, ResourceManager* resources = nullptr);

private:
    std::string name_;
    std::string document_;
    usize instance_count_ = 0;
};

} // namespace serialization
} // namespace fe
