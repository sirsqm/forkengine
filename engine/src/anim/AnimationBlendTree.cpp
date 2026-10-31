// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/anim/AnimationBlendTree.h"

#include "fe/anim/Skeleton.h"
#include "fe/resources/ResourceManager.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>

namespace fe::anim {
namespace {

[[nodiscard]] f32 clamp01(f32 value) noexcept {
    return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

// The weight of the *second* sample for a parameter, given the two thresholds.
//
// The two thresholds name the interval the blend spans: at `a` the first clip is
// full and at `b` the second is. The parameter's position between them is the
// weight, clamped, so a parameter past either end pins the nearer clip rather
// than extrapolating. A degenerate interval (a == b) is a hard cut at that value.
[[nodiscard]] f32 weight_between(f32 parameter, f32 a, f32 b) noexcept {
    const f32 span = b - a;
    if (std::fabs(span) < 1e-6f) {
        return parameter >= b ? 1.0f : 0.0f;
    }
    return clamp01((parameter - a) / span);
}

} // namespace

f32 AnimationBlendTree::blend_weight() const noexcept {
    if (!use_parameter) {
        return clamp01(weight);
    }
    if (samples.size() < 2) {
        return 0.0f;
    }
    // The two thresholds may be authored in either order; normalise so the
    // weight always runs from the first sample (0) to the second (1).
    const f32 first = samples[0].threshold;
    const f32 second = samples[1].threshold;
    return weight_between(parameter, first, second);
}

void AnimationBlendTree::set_parameter(f32 value) noexcept {
    parameter = value;
    if (use_parameter) {
        weight = blend_weight();
    }
}

usize AnimationBlendTree::resolve_clips(ResourceManager& manager) {
    usize resolved = 0;
    auto resolve = [&](std::shared_ptr<AnimationClip>& slot, const std::string& key) {
        if (slot != nullptr || key.empty()) {
            return;
        }
        auto loaded = manager.load(key);
        if (!loaded) {
            return; // left in place and retried; the asset may not be mounted yet
        }
        auto clip = std::dynamic_pointer_cast<AnimationClip>(loaded.value());
        if (clip == nullptr) {
            return;
        }
        slot = std::move(clip);
        ++resolved;
    };

    for (BlendSample& sample : samples) {
        resolve(sample.clip, sample.clip_key);
    }
    resolve(additive_clip, additive_clip_key);
    return resolved;
}

void sample_clip_pose(const Skeleton& skeleton, const AnimationClip* clip, f32 time, LoopMode mode, SkeletonPose& out) {
    out.reset_to_rest(skeleton);
    if (clip == nullptr) {
        return;
    }
    const f32 duration = clip->duration();
    if (duration <= 0.0f) {
        return;
    }
    clip->sample(AnimationClip::wrap_time(time, duration, mode), out);
}

void sample_additive_pose(const Skeleton& skeleton, const AnimationClip& clip, f32 time, LoopMode mode,
                          SkeletonPose& out) {
    // Identity, not rest: a joint the additive clip does not animate must be a
    // no-op when it is composed, and identity is the no-op for the composition
    // below. Resizing leaves every joint at the identity, which is exactly the
    // surface an additive delta is sampled over.
    out.resize(skeleton.joint_count());
    for (JointPose& joint : out.joints()) {
        joint = JointPose::identity();
    }
    const f32 duration = clip.duration();
    if (duration <= 0.0f) {
        return;
    }
    clip.sample(AnimationClip::wrap_time(time, duration, mode), out);
}

void compose_additive(SkeletonPose& base, const SkeletonPose& delta, f32 alpha) {
    const f32 weight = clamp01(alpha);
    if (weight <= 0.0f) {
        return;
    }
    const usize count = std::min(base.joint_count(), delta.joint_count());
    for (usize i = 0; i < count; ++i) {
        const JointPose& add = delta.at(i);
        JointPose& target = base.at(i);
        target.translation =
            Vec3{target.translation.x + add.translation.x * weight, target.translation.y + add.translation.y * weight,
                 target.translation.z + add.translation.z * weight};
        // Rotation composes: the delta is applied in the joint's local frame, and
        // the weight scales it on the sphere from the identity. A weighted sum of
        // quaternions is not a rotation, so the delta is slerped from identity by
        // the weight rather than scaled componentwise.
        const Quat scaled_delta = Quat::slerp(Quat::identity(), add.rotation, weight);
        target.rotation = (target.rotation * scaled_delta).normalized();
        target.scale = Vec3{target.scale.x * (1.0f + (add.scale.x - 1.0f) * weight),
                            target.scale.y * (1.0f + (add.scale.y - 1.0f) * weight),
                            target.scale.z * (1.0f + (add.scale.z - 1.0f) * weight)};
    }
}

std::string AnimationBlendTree::to_json() const {
    nlohmann::json document;
    document["parameter"] = parameter;
    document["use_parameter"] = use_parameter;
    document["weight"] = weight;
    document["drive_animator"] = drive_animator;
    document["additive_weight"] = additive_weight;
    document["additive_time"] = additive_time;
    document["additive_speed"] = additive_speed;

    nlohmann::json samples_json = nlohmann::json::array();
    for (const BlendSample& sample : samples) {
        std::string key = sample.clip_key;
        if (key.empty() && sample.clip != nullptr) {
            key = sample.clip->path();
        }
        samples_json.push_back(nlohmann::json{
            {"name", sample.name},
            {"clip", key},
            {"threshold", sample.threshold},
            {"other_threshold", sample.other_threshold},
            {"speed", sample.speed},
        });
    }
    document["samples"] = std::move(samples_json);

    std::string additive_key = additive_clip_key;
    if (additive_key.empty() && additive_clip != nullptr) {
        additive_key = additive_clip->path();
    }
    document["additive_clip"] = additive_key;

    return document.dump();
}

bool AnimationBlendTree::from_json(std::string_view text) {
    nlohmann::json document;
    try {
        document = nlohmann::json::parse(text);
    } catch (const nlohmann::json::exception&) {
        return false;
    }
    if (!document.is_object()) {
        return false;
    }

    std::vector<BlendSample> new_samples;
    if (document.contains("samples") && document.at("samples").is_array()) {
        for (const auto& entry : document.at("samples")) {
            if (!entry.is_object()) {
                continue;
            }
            BlendSample sample;
            sample.name = entry.value("name", std::string{});
            sample.clip_key = entry.value("clip", std::string{});
            sample.threshold = entry.value("threshold", 0.0f);
            sample.other_threshold = entry.value("other_threshold", 1.0f);
            sample.speed = entry.value("speed", 1.0f);
            new_samples.push_back(std::move(sample));
        }
    }

    samples = std::move(new_samples);
    parameter = document.value("parameter", 0.0f);
    use_parameter = document.value("use_parameter", true);
    weight = document.value("weight", 0.0f);
    drive_animator = document.value("drive_animator", true);
    additive_clip_key = document.value("additive_clip", std::string{});
    additive_weight = document.value("additive_weight", 0.0f);
    additive_time = document.value("additive_time", 0.0f);
    additive_speed = document.value("additive_speed", 1.0f);

    // The clips are resolved by the system from their keys; a document load is
    // pure and has no ResourceManager.
    additive_clip = nullptr;
    for (BlendSample& sample : samples) {
        sample.clip = nullptr;
    }
    return true;
}

} // namespace fe::anim
