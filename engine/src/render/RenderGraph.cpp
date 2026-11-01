// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#include "fe/render/RenderGraph.h"

#include "fe/core/Profiler.h"
#include "fe/rhi/Device.h"

#include <algorithm>

namespace fe::render {

namespace {

// A dependency edge, resolved to indices once so ordering does not compare
// strings per pass.
struct Edge {
    u32 pass = 0;
    u32 after = 0;
};

// True when two target descriptions allocate the same physical shape, so one
// allocation can back either. The name is deliberately excluded: it identifies
// the target, not the storage.
bool same_shape(const rhi::RenderTargetDesc& a, const rhi::RenderTargetDesc& b) noexcept {
    return a.width == b.width && a.height == b.height && a.color == b.color && a.depth == b.depth &&
           a.sample_depth == b.sample_depth;
}

} // namespace

u32 RenderGraph::find(std::string_view name) const noexcept {
    for (u32 i = 0; i < passes_.size(); ++i) {
        if (passes_[i].name == name) {
            return i;
        }
    }
    return kInvalid;
}

Result<void> RenderGraph::add_pass(std::string_view name) {
    if (name.empty()) {
        return Error{"render graph: a pass name must not be empty"};
    }
    if (find(name) != kInvalid) {
        return Error{"render graph: duplicate pass name '" + std::string{name} + "'"};
    }
    passes_.push_back(Pass{std::string{name}, {}, {}, {}});
    return {};
}

Result<void> RenderGraph::add_dependency(std::string_view pass, std::string_view dependency) {
    const u32 pass_index = find(pass);
    if (pass_index == kInvalid) {
        return Error{"render graph: unknown pass '" + std::string{pass} + "'"};
    }
    if (find(dependency) == kInvalid) {
        return Error{"render graph: unknown dependency '" + std::string{dependency} + "'"};
    }
    if (pass == dependency) {
        return Error{"render graph: pass '" + std::string{pass} + "' cannot depend on itself"};
    }
    passes_[pass_index].after.push_back(std::string{dependency});
    return {};
}

Result<void> RenderGraph::add_read(std::string_view pass, std::string_view resource) {
    const u32 pass_index = find(pass);
    if (pass_index == kInvalid) {
        return Error{"render graph: unknown pass '" + std::string{pass} + "'"};
    }
    if (resource.empty()) {
        return Error{"render graph: a resource name must not be empty"};
    }
    std::vector<std::string>& reads = passes_[pass_index].reads;
    if (std::find(reads.begin(), reads.end(), resource) == reads.end()) {
        reads.emplace_back(resource);
    }
    return {};
}

Result<void> RenderGraph::add_write(std::string_view pass, std::string_view resource) {
    const u32 pass_index = find(pass);
    if (pass_index == kInvalid) {
        return Error{"render graph: unknown pass '" + std::string{pass} + "'"};
    }
    if (resource.empty()) {
        return Error{"render graph: a resource name must not be empty"};
    }
    std::vector<std::string>& writes = passes_[pass_index].writes;
    if (std::find(writes.begin(), writes.end(), resource) == writes.end()) {
        writes.emplace_back(resource);
    }
    return {};
}

Result<void> RenderGraph::mark_output(std::string_view resource) {
    if (resource.empty()) {
        return Error{"render graph: a resource name must not be empty"};
    }
    if (!is_output(resource)) {
        outputs_.emplace_back(resource);
    }
    return {};
}

bool RenderGraph::is_output(std::string_view resource) const noexcept {
    return std::find(outputs_.begin(), outputs_.end(), resource) != outputs_.end();
}

std::vector<u32> RenderGraph::live_pass_indices() const {
    const usize count = passes_.size();
    // `live[i]` starts true and is cleared by culling. A pass is culled when it
    // declares writes (so it is a producer, not a presenter) and none of those
    // writes is read by a still-live pass or marked as an output. Iterating to a
    // fixed point is required because culling a consumer makes its producer's
    // write dead too.
    std::vector<bool> live(count, true);
    bool changed = true;
    while (changed) {
        changed = false;
        for (usize i = 0; i < count; ++i) {
            if (!live[i]) {
                continue;
            }
            const Pass& pass = passes_[i];
            // A pass that writes nothing is assumed to have an external effect
            // (it presents) and is never culled.
            if (pass.writes.empty()) {
                continue;
            }
            bool output_live = false;
            for (const std::string& written : pass.writes) {
                if (is_output(written)) {
                    output_live = true;
                    break;
                }
                for (usize j = 0; j < count; ++j) {
                    if (j == i || !live[j]) {
                        continue;
                    }
                    const Pass& consumer = passes_[j];
                    if (std::find(consumer.reads.begin(), consumer.reads.end(), written) != consumer.reads.end()) {
                        output_live = true;
                        break;
                    }
                }
                if (output_live) {
                    break;
                }
            }
            if (!output_live) {
                live[i] = false;
                changed = true;
            }
        }
    }

    std::vector<u32> indices;
    for (usize i = 0; i < count; ++i) {
        if (live[i]) {
            indices.push_back(static_cast<u32>(i));
        }
    }
    return indices;
}

std::vector<std::string> RenderGraph::culled() const {
    const std::vector<u32> live = live_pass_indices();
    std::vector<std::string> dropped;
    for (usize i = 0; i < passes_.size(); ++i) {
        if (std::find(live.begin(), live.end(), static_cast<u32>(i)) == live.end()) {
            dropped.push_back(passes_[i].name);
        }
    }
    return dropped;
}

Result<std::string_view> RenderGraph::pass_name(usize index) const {
    if (index >= passes_.size()) {
        return Error{"render graph: pass index out of range"};
    }
    return std::string_view{passes_[index].name};
}

void RenderGraph::clear() noexcept {
    passes_.clear();
    outputs_.clear();
    targets_.clear();
}

Result<std::vector<std::string>> RenderGraph::order() const {
    // Only live passes are ordered and run. A pass the culling rule drops is
    // absent from the result entirely, and a dependency on it is dropped with
    // it: the dependent was written against a producer whose output nobody reads.
    const std::vector<u32> live = live_pass_indices();
    const usize count = live.size();
    std::vector<Edge> edges;
    edges.reserve(count);
    // remaining[k] is how many of live pass k's dependencies have not yet been
    // emitted. `position_of` maps a pass index to its slot in the live list, so
    // an edge can be resolved to a live slot or recognised as dangling.
    std::vector<u32> remaining(count, 0);
    std::vector<u32> position_of(passes_.size(), kInvalid);
    for (usize k = 0; k < count; ++k) {
        position_of[live[k]] = static_cast<u32>(k);
    }
    for (usize k = 0; k < count; ++k) {
        for (const std::string& dependency : passes_[live[k]].after) {
            const u32 after = find(dependency);
            // A dependency can only be unknown if a caller mutated the pass list
            // directly; add_dependency validates it, so this is a guard, not a
            // reachable path.
            if (after == kInvalid) {
                return Error{"render graph: pass '" + passes_[live[k]].name + "' depends on unknown '" + dependency +
                             "'"};
            }
            // A dependency on a culled pass is dropped rather than an error.
            const u32 after_slot = position_of[after];
            if (after_slot == kInvalid) {
                continue;
            }
            edges.push_back(Edge{static_cast<u32>(k), after_slot});
            ++remaining[k];
        }
    }

    std::vector<std::string> result;
    result.reserve(count);
    std::vector<bool> emitted(count, false);

    // Kahn's algorithm, choosing the lowest unemitted live slot each round.
    // Picking by position rather than by any traversal order is what makes the
    // output stable and declaration-order preserving: a pass with no unmet
    // dependency runs as early as its position allows, so an unconstrained graph
    // comes out in exactly the order it was declared.
    for (usize step = 0; step < count; ++step) {
        usize next = count;
        for (usize i = 0; i < count; ++i) {
            if (!emitted[i] && remaining[i] == 0) {
                next = i;
                break;
            }
        }
        if (next == count) {
            // No pass is ready, so a cycle holds the rest. Name them: that is
            // what a caller needs to find the loop.
            std::string stuck;
            for (usize i = 0; i < count; ++i) {
                if (!emitted[i]) {
                    if (!stuck.empty()) {
                        stuck += ", ";
                    }
                    stuck += passes_[live[i]].name;
                }
            }
            return Error{"render graph: dependency cycle among: " + stuck};
        }
        emitted[next] = true;
        result.push_back(passes_[live[next]].name);
        for (const Edge& edge : edges) {
            if (edge.after == next) {
                --remaining[edge.pass];
            }
        }
    }

    return result;
}

Result<void> RenderGraph::add_target(const rhi::RenderTargetDesc& desc) {
    if (desc.name.empty()) {
        return Error{"render graph: a target name must not be empty"};
    }
    if (desc.width == 0 || desc.height == 0) {
        return Error{"render graph: target '" + desc.name + "' must have non-zero dimensions"};
    }
    if (!desc.color && !(desc.depth && desc.sample_depth)) {
        return Error{"render graph: target '" + desc.name +
                     "' has no attachment; set color, or depth with sample_depth"};
    }
    for (const Target& target : targets_) {
        if (target.desc.name == desc.name) {
            return Error{"render graph: duplicate target name '" + desc.name + "'"};
        }
    }
    targets_.push_back(Target{desc, nullptr});
    return {};
}

Result<RenderGraph::TargetPlan> RenderGraph::plan() const {
    auto ordered = order();
    if (!ordered) {
        return ordered.error();
    }
    // The run order resolved to pass indices, so a target's first and last use
    // are a min and a max over live passes rather than a second graph walk.
    std::vector<u32> run;
    run.reserve(ordered.value().size());
    for (const std::string& name : ordered.value()) {
        run.push_back(find(name));
    }

    const u32 never = TargetPlan::kNoSlot;
    std::vector<u32> first_use(targets_.size(), never);
    std::vector<u32> last_use(targets_.size(), never);
    for (u32 i = 0; i < targets_.size(); ++i) {
        const std::string& name = targets_[i].desc.name;
        for (u32 p = 0; p < run.size(); ++p) {
            const Pass& pass = passes_[run[p]];
            const bool touches = std::find(pass.writes.begin(), pass.writes.end(), name) != pass.writes.end() ||
                                 std::find(pass.reads.begin(), pass.reads.end(), name) != pass.reads.end();
            if (touches) {
                if (first_use[i] == never) {
                    first_use[i] = p;
                }
                last_use[i] = p;
            }
        }
    }

    TargetPlan result;
    result.slots.assign(targets_.size(), never);
    // A target's slot is the first one that fits: no shape mismatch, and no
    // already-assigned target whose lifetime overlaps this one's. Declaring the
    // targets in the order they would be created keeps the assignment stable.
    for (u32 i = 0; i < targets_.size(); ++i) {
        if (first_use[i] == never) {
            continue; // no live pass touches it; it gets no storage.
        }
        for (u32 slot = 0; slot < result.slot_count; ++slot) {
            bool fits = true;
            for (u32 j = 0; j < i; ++j) {
                if (result.slots[j] != slot) {
                    continue;
                }
                if (!same_shape(targets_[i].desc, targets_[j].desc)) {
                    fits = false;
                    break;
                }
                // Half-open intervals: one target's last use on the same pass as
                // another's first use still overlaps, so they cannot share.
                const bool disjoint = last_use[j] < first_use[i] || last_use[i] < first_use[j];
                if (!disjoint) {
                    fits = false;
                    break;
                }
            }
            if (fits) {
                result.slots[i] = slot;
                break;
            }
        }
        if (result.slots[i] == never) {
            result.slots[i] = result.slot_count++;
        }
    }
    return result;
}

Error RenderGraph::allocate(rhi::Device& device) {
    release();
    auto planned = plan();
    if (!planned) {
        return planned.error();
    }
    std::vector<rhi::RenderTargetRef> slots(planned.value().slot_count);
    for (u32 i = 0; i < targets_.size(); ++i) {
        const u32 slot = planned.value().slots[i];
        if (slot == TargetPlan::kNoSlot) {
            continue;
        }
        if (slots[slot] == nullptr) {
            Result<rhi::RenderTargetRef> created = device.create_render_target(targets_[i].desc);
            if (!created) {
                return created.error();
            }
            slots[slot] = created.value();
        }
        // Targets that alias a slot share the one allocation; a caller reading a
        // target's name sees the first description declared for that slot, which
        // is why aliasing requires identical shapes.
        targets_[i].resource = slots[slot];
    }
    return Error::none();
}

rhi::RenderTarget* RenderGraph::target(std::string_view resource) const noexcept {
    for (const Target& target : targets_) {
        if (target.desc.name == resource) {
            return target.resource.get();
        }
    }
    return nullptr;
}

void RenderGraph::release() noexcept {
    for (Target& target : targets_) {
        target.resource.reset();
    }
}

Error RenderGraph::run(const std::function<Error(std::string_view)>& run_pass) const {
    auto ordered = order();
    if (!ordered) {
        return ordered.error();
    }
    for (const std::string& pass : ordered.value()) {
#if FE_PROFILING
        // The scope is named per pass so a FrameReport attributes time to each
        // one. `scope_name` must outlive the scope object's construction, which
        // it does: it lives to the end of the iteration.
        const std::string scope_name = "pass/" + pass;
        ProfileScope scope(scope_name);
#endif
        if (Error error = run_pass(pass); error) {
            return error;
        }
    }
    return Error::none();
}

} // namespace fe::render
