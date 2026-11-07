// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/anim/AnimationBlendSpace2D.h"

#include "fe/resources/ResourceManager.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace fe::anim {
namespace {

constexpr f32 kEpsilon = 1e-5f;

[[nodiscard]] f32 clamp01(f32 value) noexcept {
    return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

// A triangle over three sample indices.
struct Triangle {
    usize a = 0;
    usize b = 0;
    usize c = 0;
};

// Twice the signed area of (a, b, c). Positive when the three points turn
// counter-clockwise, negative when they turn clockwise, zero when collinear.
[[nodiscard]] f32 cross_z(const Vec2& a, const Vec2& b, const Vec2& c) noexcept {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// True when `p` is strictly inside the circumcircle of the counter-clockwise
// triangle (a, b, c). The standard determinant test.
[[nodiscard]] bool in_circumcircle(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& p) noexcept {
    const f32 ax = a.x - p.x;
    const f32 ay = a.y - p.y;
    const f32 bx = b.x - p.x;
    const f32 by = b.y - p.y;
    const f32 cx = c.x - p.x;
    const f32 cy = c.y - p.y;
    const f32 det = (ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay) +
                    (cx * cx + cy * cy) * (ax * by - bx * ay);
    return det > 0.0f;
}

// The Delaunay triangulation of the sample positions, over sample indices.
//
// Bowyer-Watson: start from a super-triangle that encloses every sample, insert
// the samples one at a time, and after each insertion delete the triangles whose
// circumcircle contains the new point and re-triangulate the resulting cavity.
// The super-triangle's triangles are then dropped. The sample count is tiny (a
// locomotion grid is a handful of points), so the quadratic algorithm is not
// worth replacing with an incremental one.
[[nodiscard]] std::vector<Triangle> triangulate(const std::vector<BlendPoint>& samples) {
    std::vector<Triangle> result;
    if (samples.size() < 3) {
        return result;
    }

    // A super-triangle large enough to contain every sample: the bounding box of
    // the samples, scaled well past the points so none lies on its edge.
    Vec2 lo = samples[0].position;
    Vec2 hi = samples[0].position;
    for (const BlendPoint& sample : samples) {
        lo.x = std::min(lo.x, sample.position.x);
        lo.y = std::min(lo.y, sample.position.y);
        hi.x = std::max(hi.x, sample.position.x);
        hi.y = std::max(hi.y, sample.position.y);
    }
    const Vec2 mid{(lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f};
    const f32 span = std::max(hi.x - lo.x, hi.y - lo.y);
    const f32 radius = (span > 0.0f ? span : 1.0f) * 16.0f;

    // The super vertices are appended after the real samples, at fixed indices,
    // so a triangle referring to one can be recognised and dropped.
    const usize super_a = samples.size();
    const usize super_b = super_a + 1;
    const usize super_c = super_a + 2;
    std::vector<Vec2> points;
    points.reserve(samples.size() + 3);
    for (const BlendPoint& sample : samples) {
        points.push_back(sample.position);
    }
    points.push_back(Vec2{mid.x - radius, mid.y - radius});
    points.push_back(Vec2{mid.x + radius, mid.y - radius});
    points.push_back(Vec2{mid.x, mid.y + radius});

    // Emit a triangle in counter-clockwise order, so the circumcircle test has a
    // consistent sign, and drop a degenerate one.
    const auto push_ccw = [&](std::vector<Triangle>& into, usize a, usize b, usize c) {
        const f32 area = cross_z(points[a], points[b], points[c]);
        if (std::abs(area) <= kEpsilon) {
            return;
        }
        if (area > 0.0f) {
            into.push_back(Triangle{a, b, c});
        } else {
            into.push_back(Triangle{a, c, b});
        }
    };

    std::vector<Triangle> triangles;
    push_ccw(triangles, super_a, super_b, super_c);

    for (usize i = 0; i < samples.size(); ++i) {
        const Vec2 p = points[i];
        std::vector<Triangle> kept;
        std::vector<std::pair<usize, usize>> cavity_edges;
        for (const Triangle& tri : triangles) {
            if (!in_circumcircle(points[tri.a], points[tri.b], points[tri.c], p)) {
                kept.push_back(tri);
                continue;
            }
            // An edge of the cavity is one not shared with another removed
            // triangle; adding both orientations and cancelling the pairs finds
            // the boundary without a separate adjacency structure.
            const std::pair<usize, usize> edges[3] = {{tri.a, tri.b}, {tri.b, tri.c}, {tri.c, tri.a}};
            for (const auto& edge : edges) {
                const auto reverse = std::make_pair(edge.second, edge.first);
                const auto it = std::find(cavity_edges.begin(), cavity_edges.end(), reverse);
                if (it != cavity_edges.end()) {
                    cavity_edges.erase(it);
                } else {
                    cavity_edges.push_back(edge);
                }
            }
        }
        triangles = std::move(kept);
        for (const auto& edge : cavity_edges) {
            push_ccw(triangles, edge.first, edge.second, i);
        }
    }

    for (const Triangle& tri : triangles) {
        if (tri.a >= samples.size() || tri.b >= samples.size() || tri.c >= samples.size()) {
            continue; // a triangle that still touches the super-triangle
        }
        result.push_back(tri);
    }
    return result;
}

// The barycentric weights of `p` in the triangle (a, b, c). `out` receives the
// three weights; returns false when the triangle is degenerate.
[[nodiscard]] bool barycentric(const Vec2& p, const Vec2& a, const Vec2& b, const Vec2& c, f32 out[3]) noexcept {
    const f32 denom = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
    if (std::abs(denom) <= kEpsilon) {
        return false;
    }
    out[0] = ((b.y - c.y) * (p.x - c.x) + (c.x - b.x) * (p.y - c.y)) / denom;
    out[1] = ((c.y - a.y) * (p.x - c.x) + (a.x - c.x) * (p.y - c.y)) / denom;
    out[2] = 1.0f - out[0] - out[1];
    return true;
}

} // namespace

usize AnimationBlendSpace2D::weights(BlendWeight* out, usize capacity) const noexcept {
    if (out == nullptr || capacity == 0 || samples.empty()) {
        return 0;
    }
    if (samples.size() == 1) {
        out[0] = BlendWeight{0, 1.0f};
        return 1;
    }
    if (samples.size() == 2) {
        // Blend along the segment between the two samples: the projection of the
        // parameter onto the segment, clamped, is the second sample's weight.
        const Vec2 a = samples[0].position;
        const Vec2 b = samples[1].position;
        const Vec2 ab = b - a;
        const f32 len_sq = ab.length_squared();
        f32 t = 0.0f;
        if (len_sq > kEpsilon) {
            t = clamp01((parameter - a).dot(ab) / len_sq);
        }
        if (t <= kEpsilon) {
            out[0] = BlendWeight{0, 1.0f};
            return 1;
        }
        if (t >= 1.0f - kEpsilon) {
            out[0] = BlendWeight{1, 1.0f};
            return 1;
        }
        out[0] = BlendWeight{0, 1.0f - t};
        out[1] = BlendWeight{1, t};
        return 2;
    }

    const std::vector<Triangle> triangles = triangulate(samples);
    for (const Triangle& tri : triangles) {
        f32 b[3] = {0.0f, 0.0f, 0.0f};
        if (!barycentric(parameter, samples[tri.a].position, samples[tri.b].position, samples[tri.c].position, b)) {
            continue;
        }
        // A small negative weight is the point sitting on an edge, which belongs
        // to the triangle; a clearly negative one means the point is outside it.
        if (b[0] < -kEpsilon || b[1] < -kEpsilon || b[2] < -kEpsilon) {
            continue;
        }
        const usize indices[3] = {tri.a, tri.b, tri.c};
        usize count = 0;
        for (usize k = 0; k < 3 && count < capacity; ++k) {
            const f32 weight = clamp01(b[k]);
            if (weight <= kEpsilon) {
                continue; // a vertex the point does not use; keep the output compact
            }
            out[count++] = BlendWeight{indices[k], weight};
        }
        if (count == 0 && capacity >= 1) {
            // Every weight rounded to zero: fall back to the nearest vertex.
            out[0] = BlendWeight{tri.a, 1.0f};
            count = 1;
        }
        return count;
    }

    // The parameter is outside every triangle: the grid does not cover the whole
    // plane, so clamp to the nearest sample at full weight rather than
    // extrapolating past the authored clips.
    usize nearest = 0;
    f32 best = std::numeric_limits<f32>::max();
    for (usize i = 0; i < samples.size(); ++i) {
        const f32 d = (samples[i].position - parameter).length_squared();
        if (d < best) {
            best = d;
            nearest = i;
        }
    }
    out[0] = BlendWeight{nearest, 1.0f};
    return 1;
}

usize AnimationBlendSpace2D::resolve_clips(ResourceManager& manager) {
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

    for (BlendPoint& sample : samples) {
        resolve(sample.clip, sample.clip_key);
    }
    resolve(additive_clip, additive_clip_key);
    for (AdditiveLayer& layer : additive_layers) {
        resolve(layer.clip, layer.clip_key);
    }
    return resolved;
}

std::string AnimationBlendSpace2D::to_json() const {
    nlohmann::json document;
    document["parameter"] = {parameter.x, parameter.y};
    document["drive_animator"] = drive_animator;
    document["additive_weight"] = additive_weight;
    document["additive_time"] = additive_time;
    document["additive_speed"] = additive_speed;
    document["additive_mask"] = additive_mask.names();

    nlohmann::json layers_json = nlohmann::json::array();
    for (const AdditiveLayer& layer : additive_layers) {
        std::string key = layer.clip_key;
        if (key.empty() && layer.clip != nullptr) {
            key = layer.clip->path();
        }
        layers_json.push_back(nlohmann::json{
            {"clip", key},
            {"weight", layer.weight},
            {"time", layer.time},
            {"speed", layer.speed},
            {"mask", layer.mask.names()},
        });
    }
    document["additive_layers"] = std::move(layers_json);

    nlohmann::json samples_json = nlohmann::json::array();
    for (const BlendPoint& sample : samples) {
        std::string key = sample.clip_key;
        if (key.empty() && sample.clip != nullptr) {
            key = sample.clip->path();
        }
        samples_json.push_back(nlohmann::json{
            {"name", sample.name},
            {"clip", key},
            {"position", {sample.position.x, sample.position.y}},
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

bool AnimationBlendSpace2D::from_json(std::string_view text) {
    nlohmann::json document;
    try {
        document = nlohmann::json::parse(text);
    } catch (const nlohmann::json::exception&) {
        return false;
    }
    if (!document.is_object()) {
        return false;
    }

    std::vector<BlendPoint> new_samples;
    if (document.contains("samples") && document.at("samples").is_array()) {
        for (const auto& entry : document.at("samples")) {
            if (!entry.is_object()) {
                continue;
            }
            BlendPoint sample;
            sample.name = entry.value("name", std::string{});
            sample.clip_key = entry.value("clip", std::string{});
            sample.speed = entry.value("speed", 1.0f);
            if (entry.contains("position") && entry.at("position").is_array() && entry.at("position").size() == 2) {
                sample.position = Vec2{entry.at("position")[0].get<f32>(), entry.at("position")[1].get<f32>()};
            }
            new_samples.push_back(std::move(sample));
        }
    }

    samples = std::move(new_samples);
    if (document.contains("parameter") && document.at("parameter").is_array() && document.at("parameter").size() == 2) {
        parameter = Vec2{document.at("parameter")[0].get<f32>(), document.at("parameter")[1].get<f32>()};
    } else {
        parameter = Vec2{};
    }
    drive_animator = document.value("drive_animator", true);
    additive_clip_key = document.value("additive_clip", std::string{});
    additive_weight = document.value("additive_weight", 0.0f);
    additive_time = document.value("additive_time", 0.0f);
    additive_speed = document.value("additive_speed", 1.0f);

    additive_mask = JointMask{};
    if (document.contains("additive_mask") && document.at("additive_mask").is_array()) {
        for (const auto& entry : document.at("additive_mask")) {
            if (entry.is_string()) {
                additive_mask.add(entry.get<std::string>());
            }
        }
    }

    additive_layers.clear();
    if (document.contains("additive_layers") && document.at("additive_layers").is_array()) {
        for (const auto& entry : document.at("additive_layers")) {
            if (!entry.is_object()) {
                continue;
            }
            AdditiveLayer layer;
            layer.clip_key = entry.value("clip", std::string{});
            layer.weight = entry.value("weight", 0.0f);
            layer.time = entry.value("time", 0.0f);
            layer.speed = entry.value("speed", 1.0f);
            if (entry.contains("mask") && entry.at("mask").is_array()) {
                for (const auto& name : entry.at("mask")) {
                    if (name.is_string()) {
                        layer.mask.add(name.get<std::string>());
                    }
                }
            }
            additive_layers.push_back(std::move(layer));
        }
    }
    // The clips are resolved by the system from their keys; a document load is
    // pure and has no ResourceManager.
    additive_clip = nullptr;
    for (BlendPoint& sample : samples) {
        sample.clip = nullptr;
    }
    return true;
}

} // namespace fe::anim
