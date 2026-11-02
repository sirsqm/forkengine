# ADR-0129: Per-pass timings read back from the render graph

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0100](0100-render-graph-seam.md),
  [ADR-0022](0022-profiler-and-frame-timing.md),
  [ADR-0128](0128-render-graph-derived-edges.md),
  [ADR-0017](0017-performance-strategy.md)

## Context

[ADR-0100](0100-render-graph-seam.md) made the frame's passes data and had
`run()` bracket each pass in a profiler scope named `pass/<name>`
([ADR-0022](0022-profiler-and-frame-timing.md)), so a pass shows up in a
`FrameReport`. That report is a whole-frame snapshot: it aggregates every scope
by name and sorts by cost. Reading one pass's time out of it means scanning the
report's aggregates and matching the `pass/<name>` string, which every caller who
wants "which pass dominates this frame" would otherwise write for itself.

The information is already recorded -- the scope exists, the duration is in the
frame sample -- but the graph exposes no way to read it back keyed by pass name.
That is the gap: the seam produces the data and cannot hand it out.

Constraints that shaped the decision:

- **No new recording.** The pass scopes already exist; the readback must not add
  a second timing path or change what the profiler records.
- **No second frame boundary.** `run()` is called inside a frame the caller owns
  (`Application` brackets `frame/update` and `frame/render`); the graph must not
  open or commit a frame of its own.
- **The graph stays a pure value** ([ADR-0042](0042-null-rhi-device.md)). The
  readback is a query over the profiler's current frame, so it stays testable
  headlessly and touches no device.
- **Honest when there is nothing to report.** A run outside a profiler frame
  records no scopes, so the readback is empty rather than zeroes that look like
  measurements.

## Decision

`run()` becomes non-`const` and records the last run's per-pass times, which
`last_pass_timings()` exposes.

- `last_pass_timings()` returns `PassTiming{name, duration_ms}` for each pass that
  ran, **in run order**, keyed by pass name.
- Each pass's time is its `pass/<name>` scope, read from the profiler frame that
  was **in progress** during the run. `run()` executes inside the caller's frame
  and the frame is committed only after `run()` returns, so the frame still being
  recorded is the one holding this run's samples -- not the most recently
  *completed* frame, which is the previous one.
- The readback is a query, not a record: it walks the in-progress frame's scope
  samples once and copies the matching durations. It adds no scope, no counter,
  and no allocation on the profiling hot path.
- **Empty when there is no frame.** With no current profiler, or no frame in
  progress, nothing was recorded, so the list is left empty rather than filled
  with zeroes.
- `clear()` drops the recorded timings along with the passes, so a reused graph
  does not report a stale frame's times.
- `run()` is non-`const` because it now writes the timings; it was already
  logically a mutation of the graph's per-run state. A failed pass still records
  the timings of the passes that ran before it, so a partial frame is not lost.

## Consequences

**Easier:**

- A caller reads which pass dominates the frame as `graph.last_pass_timings()`,
  in run order, without parsing a `FrameReport` or knowing the `pass/<name>`
  naming convention.
- The timings come from the same samples a `FrameReport` is built from, so the
  two never disagree: there is one recording path.
- The readback is asserted headlessly: a run under a real profiler frame reads a
  named pass's time back, the order follows the run order rather than declaration
  order, a run outside a frame reports nothing, and `clear()` drops the timings.

**Harder / not yet done:**

- The timings are the *last* run's, held on the graph. Two graphs, or a caller
  that wants a history, still reads the `FrameReport` for anything but the most
  recent frame.
- The readback matches a scope by its interned name per pass, which is a string
  compare per pass per frame. That is a handful of passes, not a hot loop; a
  name handle cached at declaration time is the change if it ever matters.
- `run()` is no longer usable on a `const` graph. That is honest -- it records
  state -- but a caller that held the graph const must drop the `const`.

## Alternatives considered

- **Return a `FrameReport` from `run()`.** Rejected: the report is the whole
  frame, and the graph does not own the frame -- the caller does. Handing back a
  frame report would claim a boundary the graph does not control, and it would
  drop the counters and scopes the caller's own code recorded outside the passes.
- **Have the caller read `pass/<name>` from the frame report itself.** The status
  quo, and the thing this removes: every caller re-implements the naming
  convention and the aggregate scan, and the convention becomes load-bearing
  outside the graph.
- **Read the most recent *completed* frame.** Rejected: during `run()` that is
  the previous frame, so the readback would report the wrong frame's times -- a
  subtle, frame-late bug. The in-progress frame is the one this run wrote into.
- **Record the timings on the graph only when profiling is compiled in
  (`FE_PROFILING`).** This is what happens, but the API stays present either way:
  with profiling off, the scopes do not exist, so the list is empty, which is the
  same contract as a run outside a frame. A caller need not be compiled
  differently to read it.
