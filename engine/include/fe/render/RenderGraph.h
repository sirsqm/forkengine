// Copyright (c) 2026 sqm
// SPDX-License-Identifier: MIT
#pragma once

#include "fe/core/Error.h"
#include "fe/core/Types.h"
#include "fe/rhi/Resources.h"
#include "fe/rhi/Types.h"

#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace fe::rhi {
class Device;
} // namespace fe::rhi

namespace fe::render {

// The render-graph seam (ADR-0100).
//
// A frame's passes used to be ordered by hand in code: `Renderer2D` draws the
// world, then the UI; `Renderer3D` draws the shadow pass, then the main pass,
// then the post-processing chain. The order was correct but implicit -- it lived
// in the sequence of calls in a `render()` body, so nothing could reorder a
// pass, drop one whose output nobody reads, or time one as a unit.
//
// This type makes the order data. A caller declares passes and the dependencies
// between them; `order()` returns a run order that respects every dependency, and
// `run()` invokes each pass in that order, bracketing each in a profiler scope so
// a pass shows up in a `FrameReport` as `pass/<name>`.
//
// Scope of this cut: it orders passes, culls passes whose output nobody reads,
// and runs them. It does *not* own render targets or manage resource lifetimes
// -- those need a real resource description, and are the follow-on that makes
// the graph an allocator rather than a scheduler/culler. What it establishes is
// the seam: the moment pass order and pass resources are data, reordering,
// culling, and per-pass profiling become changes to that data rather than edits
// to the renderers' structure (ADR-0017).
//
// Culling (ADR-0110). A pass may declare the resources it reads and writes. A
// pass is *live* when it writes a resource that some other live pass reads, or
// writes a resource marked as an output (the swapchain, a readback, anything the
// frame is observed through); a pass that declares no writes is treated as live
// unconditionally, because a pass with no declared output is assumed to have an
// external effect (it presents). Culling is iterative: dropping a pass can make
// another pass's write dead, so it repeats until a fixed point. A pass with no
// resource declarations at all is therefore never culled, which is what keeps
// every existing graph -- `Renderer2D`'s world/ui pair, for instance -- running
// byte for byte as before.
//
// The graph is a pure value: it holds no GPU state and touches no device, so it
// is fully testable headlessly. That is the same split the extraction seam
// (ADR-0052) uses, and for the same reason -- the decision can be asserted
// without a context.
class RenderGraph {
public:
    // One pass: a name, the names of passes it must run after, and the resources
    // it reads and writes (empty for a pass that declares none).
    struct Pass {
        std::string name;
        std::vector<std::string> after;
        std::vector<std::string> reads;
        std::vector<std::string> writes;
    };

    RenderGraph() = default;

    // Declares a pass. Fails if the name is empty or already declared -- two
    // passes with one name is ambiguous, so it is refused rather than guessed at.
    [[nodiscard]] Result<void> add_pass(std::string_view name);

    // Declares that `pass` must run after `dependency`. Fails if either name is
    // unknown. A dependency may be declared before either pass is added only
    // through `add_pass`'s `after` list; this method requires both to exist.
    [[nodiscard]] Result<void> add_dependency(std::string_view pass, std::string_view dependency);

    // Declares that `pass` reads / writes `resource`. A resource is just a name:
    // the graph never allocates one, it only reasons about which pass produces
    // it and which consume it. Fails if the pass is unknown or the resource name
    // is empty. Declaring the same read or write twice is a no-op, so a caller
    // that builds a graph incrementally need not deduplicate.
    [[nodiscard]] Result<void> add_read(std::string_view pass, std::string_view resource);
    [[nodiscard]] Result<void> add_write(std::string_view pass, std::string_view resource);

    // Marks `resource` as an output: something outside the graph observes it
    // (the swapchain image, a readback buffer), so a pass that writes it is never
    // culled. Fails if the name is empty. Idempotent.
    [[nodiscard]] Result<void> mark_output(std::string_view resource);

    // A pass's declared name, in declaration order. Fails if out of range.
    [[nodiscard]] Result<std::string_view> pass_name(usize index) const;

    // The number of declared passes.
    [[nodiscard]] usize pass_count() const noexcept { return passes_.size(); }

    // True when no passes are declared.
    [[nodiscard]] bool empty() const noexcept { return passes_.empty(); }

    // Drops every pass, dependency, resource declaration, and output mark.
    void clear() noexcept;

    // The run order: every *live* pass follows the passes it declared. Ties are
    // broken by declaration order, so the result is stable -- two graphs with the
    // same passes and the same dependencies order them identically, and a pass
    // with no constraints keeps the position it was added in. A dependency on a
    // culled pass is dropped, not an error: the dependent was written against a
    // pass that turned out to have no live output. Fails if the dependencies
    // among the live passes form a cycle, naming the passes left unresolved.
    [[nodiscard]] Result<std::vector<std::string>> order() const;

    // The names of the passes the culling rule drops, in declaration order. A
    // pass is culled when it declares writes and none of them is live (read by
    // another live pass, or marked as an output). Exposed so a test can assert
    // what was dropped without reading the run order.
    [[nodiscard]] std::vector<std::string> culled() const;

    // --- Render-target ownership (ADR-0100) --------------------------------
    //
    // A graph may own the render targets its passes draw into, rather than
    // receiving them from the caller. A target is declared by a `RenderTargetDesc`
    // whose `name` is the resource name a pass writes to it, so a pass's writes
    // and its target are the same name. Targets are allocated from their
    // descriptions on `allocate()`, which needs a device (the null device works
    // headlessly), and are owned by the graph until `release()`.
    //
    // Targets are aliased: two targets whose lifetimes do not overlap -- one's
    // last use precedes the other's first -- may share one physical allocation,
    // because nothing observes both at once. A target's lifetime is the span, in
    // run order, from the first live pass that writes it to the last live pass
    // that reads or writes it. Aliasing only ever combines targets whose
    // descriptions are identical, since a slot is allocated once from one
    // description. A target no live pass touches is dead and gets no slot.
    [[nodiscard]] Result<void> add_target(const rhi::RenderTargetDesc& desc);

    // Re-describes an existing target, so a resize rebuilds its allocation at
    // the new size. The description is validated exactly as `add_target`'s is,
    // and the name must match a declared target (it identifies the target, so
    // re-describing cannot rename it). Only the *declaration* changes: a target
    // already allocated keeps its old size until `allocate()` runs again, which
    // re-runs the whole plan so aliasing and slots follow the new shapes.
    [[nodiscard]] Result<void> set_target_desc(const rhi::RenderTargetDesc& desc);

    // The number of declared targets.
    [[nodiscard]] usize target_count() const noexcept { return targets_.size(); }

    // The physical slot each declared target is assigned, in declaration order,
    // and the number of distinct slots. Two targets with the same slot alias one
    // allocation; `kNoSlot` marks a target no live pass touches. Fails if the
    // dependencies among live passes form a cycle, as `order()` does.
    struct TargetPlan {
        std::vector<u32> slots;
        u32 slot_count = 0;
        static constexpr u32 kNoSlot = 0xFFFFFFFFu;
    };
    [[nodiscard]] Result<TargetPlan> plan() const;

    // Allocates one render target per slot in `plan()`, through `device`, and
    // keeps them alive. Fails on a cycle or on a device refusal. Calling it again
    // releases what was allocated first, so it is safe to re-run after the graph
    // changes.
    [[nodiscard]] Error allocate(rhi::Device& device);

    // The target allocated for `resource`, or nullptr when none is (an unknown
    // name, a dead target, or before `allocate()`).
    [[nodiscard]] rhi::RenderTarget* target(std::string_view resource) const noexcept;

    // Drops every allocated target. The declarations are kept, so a later
    // `allocate()` rebuilds them.
    void release() noexcept;

    // `order()`, then `run_pass` for each pass in turn. Stops at the first pass
    // that returns an Error and propagates it. Each pass runs inside a profiler
    // scope named `pass/<name>` (a no-op when no profiler frame is active), so
    // the frame report attributes time per pass without the caller instrumenting
    // each one.
    [[nodiscard]] Error run(const std::function<Error(std::string_view)>& run_pass) const;

private:
    // Finds a pass index by name, or kInvalid when absent.
    [[nodiscard]] u32 find(std::string_view name) const noexcept;

    // True when `resource` is observed outside the graph.
    [[nodiscard]] bool is_output(std::string_view resource) const noexcept;

    // The indices of the passes that survive culling, in declaration order.
    [[nodiscard]] std::vector<u32> live_pass_indices() const;

    // A declared render target: its description, plus the allocation `allocate()`
    // makes for its slot (null until then, and shared with the other targets that
    // alias the same slot).
    struct Target {
        rhi::RenderTargetDesc desc;
        rhi::RenderTargetRef resource;
    };

    static constexpr u32 kInvalid = 0xFFFFFFFFu;

    std::vector<Pass> passes_;
    std::vector<std::string> outputs_;
    std::vector<Target> targets_;
};

} // namespace fe::render
