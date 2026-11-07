// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/serialization/Prefab.h"

#include "fe/reflect/Registry.h"
#include "fe/scene/Node.h"
#include "fe/scene/Scene.h"
#include "fe/serialization/SceneSerializer.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <unordered_map>
#include <utility>

namespace fe::serialization {
namespace {

using json = nlohmann::json;

// Instances are independent because their identities are. Before a definition is
// read into a scene, every node and component id it names is replaced with a
// fresh one, and every reference that named an old id is rewritten to the new
// one, so the instance neither collides with the definition nor shares state
// with another instance.

void collect_ids(const json& node_json, std::unordered_map<Uuid, Uuid>& remap) {
    Uuid id;
    if (node_json.contains("id") && Uuid::try_parse(node_json.at("id").get<std::string>(), id)) {
        remap.emplace(id, Uuid::generate());
    }
    if (node_json.contains("components")) {
        for (const auto& component : node_json.at("components")) {
            Uuid component_id;
            if (component.contains("id") && Uuid::try_parse(component.at("id").get<std::string>(), component_id)) {
                remap.emplace(component_id, Uuid::generate());
            }
        }
    }
    if (node_json.contains("children")) {
        for (const auto& child : node_json.at("children")) {
            collect_ids(child, remap);
        }
    }
}

// Rewrites a JSON string that names a remapped id. A string that is not a
// prefab id (an asset key, a plain string) is left alone.
void rewrite_reference(json& value, const std::unordered_map<Uuid, Uuid>& remap) {
    if (!value.is_string()) {
        return;
    }
    Uuid id;
    if (!Uuid::try_parse(value.get<std::string>(), id)) {
        return;
    }
    if (const auto it = remap.find(id); it != remap.end()) {
        value = it->second.to_string();
    }
}

void rewrite_property(json& value, const reflect::Property& property, const std::unordered_map<Uuid, Uuid>& remap) {
    using reflect::ValueKind;
    switch (property.kind()) {
    case ValueKind::Uuid:
        rewrite_reference(value, remap);
        break;
    case ValueKind::Group:
        if (property.array_length > 0) {
            // A vector group is an array aligned to its members.
            if (value.is_array()) {
                const usize count = std::min<usize>(property.members.size(), value.size());
                for (usize i = 0; i < count; ++i) {
                    rewrite_property(value[i], property.members[i], remap);
                }
            }
        } else if (value.is_object()) {
            for (const auto& member : property.members) {
                if (value.contains(member.name)) {
                    rewrite_property(value[member.name], member, remap);
                }
            }
        }
        break;
    default:
        break;
    }
}

void rewrite_node(json& node_json, const std::unordered_map<Uuid, Uuid>& remap) {
    if (node_json.contains("id")) {
        rewrite_reference(node_json["id"], remap);
    }
    if (node_json.contains("components")) {
        for (auto& component : node_json["components"]) {
            if (component.contains("id")) {
                rewrite_reference(component["id"], remap);
            }
            // Only a reflected Uuid property is a node/component reference. A
            // resource reference is a string too, but it names an asset guid,
            // which the instance shares with the definition and must not rewrite.
            const reflect::TypeDesc* type = reflect::Registry::instance().find(component.value("type", std::string{}));
            if (type == nullptr) {
                continue;
            }
            for (const auto& property : type->properties) {
                if (component.contains(property.name)) {
                    rewrite_property(component[property.name], property, remap);
                }
            }
        }
    }
    if (node_json.contains("children")) {
        for (auto& child : node_json["children"]) {
            rewrite_node(child, remap);
        }
    }
}

} // namespace

Result<Prefab> Prefab::capture(std::string name, const Node& node, const ResourceManager* resources,
                               const ScriptBinding* scripts) {
    auto text = scene::subtree_to_json(node, resources, scripts);
    if (!text) {
        return text.error();
    }
    Prefab prefab;
    prefab.name_ = std::move(name);
    prefab.document_ = std::move(text.value());
    return prefab;
}

Prefab Prefab::from_document(std::string name, std::string document) {
    Prefab prefab;
    prefab.name_ = std::move(name);
    prefab.document_ = std::move(document);
    return prefab;
}

Result<Node*> Prefab::instantiate(Scene& scene, ResourceManager* resources) {
    if (!is_valid()) {
        return Error("prefab '" + name_ + "' has no definition to instantiate");
    }

    json document;
    try {
        document = json::parse(document_);
    } catch (const json::parse_error& e) {
        return Error("prefab '" + name_ + "' parse error: " + std::string(e.what()));
    }

    if (!document.contains("roots") || !document.at("roots").is_array()) {
        return Error("prefab '" + name_ + "' has no roots");
    }

    std::unordered_map<Uuid, Uuid> remap;
    for (const auto& root : document["roots"]) {
        collect_ids(root, remap);
    }
    for (auto& root : document["roots"]) {
        rewrite_node(root, remap);
    }

    const usize roots_before = scene.root_count();
    if (Error error = scene::subtree_from_json(document.dump(), scene, resources, nullptr); error) {
        // Leave the scene as it was: drop any root the failed read appended.
        while (scene.root_count() > roots_before) {
            (void)scene.remove_root(scene.roots().back().get());
        }
        return error;
    }
    if (scene.root_count() <= roots_before) {
        return Error("prefab '" + name_ + "' produced no instance root");
    }

    Node* root = scene.roots()[roots_before].get();
    ++instance_count_;
    return root;
}

} // namespace fe::serialization
