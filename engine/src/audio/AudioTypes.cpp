// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/audio/AudioTypes.h"

#include "fe/audio/AudioSource.h"
#include "fe/audio/WavLoader.h"
#include "fe/reflect/PropertyBuilders.h"
#include "fe/reflect/Registry.h"

#include <memory>
#include <string>

// The audio component registrations, in the same style as PhysicsTypes.cpp: each
// block describes a component's properties once, and the serializer and the
// inspector read those descriptions rather than naming the type. This is what
// makes an AudioSource serialize and appear in the inspector with no per-type
// code in either.

namespace fe::reflect {
namespace {

// An int-backed property for an enum member of `Class`, rejecting a value
// outside [0, count). The audio components' enums are stored as `u8`, so the
// serializer sees a plain integer and an out-of-range document value is refused
// rather than cast.
template <typename Class, typename Enum>
[[nodiscard]] Property make_enum_property(std::string name, Enum Class::* field, i64 default_value, i64 count) {
    Property property;
    property.name = std::move(name);
    property.value_kind = ValueKind::Int;
    property.getter = [field](const void* object) -> Value {
        return static_cast<i64>(static_cast<const Class*>(object)->*field);
    };
    property.setter = [field, count](void* object, const Value& value) -> bool {
        const auto* as_int = std::get_if<i64>(&value);
        if (as_int == nullptr || *as_int < 0 || *as_int >= count) {
            return false;
        }
        static_cast<Class*>(object)->*field = static_cast<Enum>(*as_int);
        return true;
    };
    property.default_value = default_value;
    return property;
}

void register_audio_source(Registry& registry) {
    audio::AudioSource defaults;

    TypeDesc desc;
    desc.name = audio::AudioSource::kTypeName;
    desc.factory = [] { return std::unique_ptr<Component>(std::make_unique<audio::AudioSource>()); };

    // The clip reference stores the manager cache key (the "wav:"-schemed path)
    // rather than the resource's own path, so a reload resolves the decoded
    // clip through the ordinary ResourceManager path - exactly how Sprite2D
    // stores a texture key.
    desc.properties.push_back(make_resource_ref<audio::AudioSource, std::shared_ptr<Resource>>(
        "clip", &audio::AudioSource::clip, "AudioClip",
        [](const std::shared_ptr<Resource>& clip) -> std::string {
            return clip != nullptr ? audio::WavLoader::reference_key(clip->path()) : std::string{};
        },
        [](std::shared_ptr<Resource>& slot, std::shared_ptr<Resource> loaded) { slot = std::move(loaded); }));

    desc.properties.push_back(
        make_scalar<audio::AudioSource, f32>("gain", &audio::AudioSource::gain, ValueKind::Float, &defaults));
    desc.properties.push_back(
        make_scalar<audio::AudioSource, f32>("pitch", &audio::AudioSource::pitch, ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<audio::AudioSource, f32>("start_offset", &audio::AudioSource::start_offset,
                                                                   ValueKind::Float, &defaults));
    desc.properties.push_back(
        make_scalar<audio::AudioSource, bool>("loop", &audio::AudioSource::loop, ValueKind::Bool, &defaults));
    desc.properties.push_back(
        make_scalar<audio::AudioSource, bool>("stream", &audio::AudioSource::stream, ValueKind::Bool, &defaults));
    desc.properties.push_back(make_scalar<audio::AudioSource, bool>("play_on_start", &audio::AudioSource::play_on_start,
                                                                    ValueKind::Bool, &defaults));
    desc.properties.push_back(
        make_scalar<audio::AudioSource, bool>("spatial", &audio::AudioSource::spatial, ValueKind::Bool, &defaults));
    desc.properties.push_back(make_scalar<audio::AudioSource, f32>("min_distance", &audio::AudioSource::min_distance,
                                                                   ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<audio::AudioSource, f32>("max_distance", &audio::AudioSource::max_distance,
                                                                   ValueKind::Float, &defaults));
    // The falloff's shape and rate. The model is an enum stored as an integer so
    // a document carries 0/1/2 and an unknown value is refused; the default 0 is
    // the inverse-distance curve, so a document written before this refinement
    // reads back as it played.
    desc.properties.push_back(make_enum_property<audio::AudioSource, audio::DistanceModel>(
        "distance_model", &audio::AudioSource::distance_model, 0, 3));
    desc.properties.push_back(
        make_scalar<audio::AudioSource, f32>("rolloff", &audio::AudioSource::rolloff, ValueKind::Float, &defaults));
    // The directivity cone. The axis is the node's own forward, so it is aimed by
    // rotating the node and has no property of its own; the three numbers below
    // describe the cone's shape. The defaults make the source omnidirectional.
    desc.properties.push_back(make_scalar<audio::AudioSource, f32>(
        "cone_inner_degrees", &audio::AudioSource::cone_inner_degrees, ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<audio::AudioSource, f32>(
        "cone_outer_degrees", &audio::AudioSource::cone_outer_degrees, ValueKind::Float, &defaults));
    desc.properties.push_back(make_scalar<audio::AudioSource, f32>(
        "cone_outer_gain", &audio::AudioSource::cone_outer_gain, ValueKind::Float, &defaults));
    desc.properties.push_back(
        make_scalar<audio::AudioSource, bool>("doppler", &audio::AudioSource::doppler, ValueKind::Bool, &defaults));
    // The bus name is a plain string, so a scene persists which mixing group the
    // source routes to. An empty string is the master bus and is the default.
    desc.properties.push_back(make_scalar<audio::AudioSource, std::string>("bus_name", &audio::AudioSource::bus_name,
                                                                           ValueKind::String, &defaults));

    registry.register_type(std::move(desc));
}

} // namespace

void register_audio_types() {
    Registry& registry = Registry::instance();
    register_audio_source(registry);
}

} // namespace fe::reflect
