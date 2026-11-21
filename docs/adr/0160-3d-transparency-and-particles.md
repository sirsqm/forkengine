# ADR-0160: 3D transparency and particle billboards

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0010](0010-3d-rendering-foundation.md),
  [ADR-0014](0014-directional-shadow-mapping.md),
  [ADR-0053](0053-spatial-bvh.md),
  [ADR-0101](0101-instanced-3d-draws.md),
  [ADR-0104](0104-skeletal-animation.md),
  [ADR-0150](0150-per-object-shadow-casting.md)

## Context

`Renderer3D` had one draw state for every mesh: blending off, depth test on,
depth write on, and instances batched in extraction order. `BlendMode::Alpha`
existed in the RHI and the OpenGL backend, and `Material::base_color` had an
alpha channel, but nothing consumed either, so an alpha below one drew an opaque
surface.

That is two gaps, not one:

1. **A translucent surface had no pass.** Blending it in the opaque pass would
   make the result depend on draw order, and with instancing the order is the
   order the scene graph was walked. A glass pane in front of a wall would show
   the wall or not depending on which node was created first.
2. **Nothing in the engine emitted an effect.** There was no particle system, so
   a game had no way to draw a spark, smoke, or a pickup burst without authoring
   one mesh per particle per frame.

The constraints are the ones the 3D renderer already works under:

- **The extraction seam stays the only place the scene is read.**
  `render::extract_meshes` produces plain `MeshInstance` values; the renderer
  batches them. A new drawable has to arrive the same way, or the parallel
  extraction path and the cull have to be taught about a second traversal.
- **A scene authored before this renders exactly as it did.** Transparency is
  opt-in through the material's alpha, and a scene with no alpha below one and no
  emitter must produce the same draw list and the same pixels.
- **CI has no GPU on the pure-logic job.** The simulation must be testable
  headlessly, and the pixels only in the graphics job.

## Decision

### A translucent instance is resolved at extraction and drawn in a sorted pass

`render::MeshInstance` gained `transparent`, set in `resolve_mesh` from
`resolve_surface().material.base_color.w < 1`. Extraction is already the place
the material is resolved for the batch key, so this reads the same resolution
rather than introducing a second one; the alternative — asking each instance for
its alpha again while batching — would resolve every material twice per frame.

`Renderer3D` then does three things with that flag:

- **Partitions and sorts.** `sort_for_transparency` stable-partitions the visible
  instances into the opaque run followed by the translucent run, and stable-sorts
  the translucent run farthest-first by squared distance from the camera to the
  instance's world-bounds centre. A stable partition leaves extraction order
  untouched inside each group, so a scene with nothing translucent keeps exactly
  the array it had before this pass existed. The sort runs after the cull, whose
  cached BVH keys are built from extraction order, so the cache is not
  invalidated by the reorder.
- **Keeps the two groups in separate batches.** `transparent` joined the batch
  merge key, so a run never mixes an opaque and a translucent instance: the two
  need different state, and the shadow pass draws or skips a whole batch.
- **Draws the translucent batches after every opaque one**, static and skinned,
  with `BlendMode::Alpha` and depth writing off, depth testing on. Testing keeps
  an opaque wall in front of a translucent surface; not writing depth keeps a
  farther translucent surface from being rejected by a nearer one in the same
  sorted run. The lit shader is rebound for the pass because the skinned pass in
  between binds its own.

Depth writing is a new RHI call, `Device::set_depth_write`. The RHI had a depth
*test* toggle and no way to separate the two, and a transparent pass needs
exactly that separation. The null device accepts it as a no-op.

A translucent instance is also excluded from the shadow pass, both from the fit
bounds and from the draw: it would write an opaque shadow for something the light
passes through.

### Particles are a component, a system, and a billboard pass

- **`ParticleEmitter3D`** (new `vfx` module) carries the authored parameters —
  rate, one-shot burst, lifetime range, speed and its spread, cone half-angle, a
  local emission axis, gravity, start/end size, start/end colour, a cap, and a
  seed — plus the runtime array `particles` and the emitter's random state. The
  runtime fields are not reflected, so a saved scene stores parameters and not
  the particles alive at the save.
- **`ParticleSystem`** is the execution half, in the shape `Animator` +
  `SkeletonAnimationSystem` established: the component owns the data, the system
  advances it, and the renderer only reads the result. It spawns this frame's
  particles, integrates every live one (gravity, velocity, age), drops the
  expired, and clamps `delta_time` to a quarter second so a debugger break cannot
  teleport a burst. Its random stream is a per-emitter xorshift32 seeded from
  `seed`, advanced only by the system, so the same seed, parameters, transforms,
  and time steps produce identical particles — which is what makes the headless
  assertions meaningful.
- **`render::extract_particles`** turns each live particle into a
  `ParticleInstance` with its size and colour interpolated by age, and
  `cull_particles` drops a billboard whose centre is outside the frustum. It
  follows `extract_meshes`/`cull_meshes` exactly, so the seam keeps one shape.
- **The renderer draws them last**, in the transparent state, through a shader
  that expands a shared unit quad into a camera-facing square of the particle's
  diameter along the camera's right and up axes. The fragment stage turns the
  square into a soft circle, because a hard-edged quad reads as a square however
  it is blended. The billboards are sorted farthest-first like the translucent
  meshes, and `last_particles()` reports how many drew.

## Consequences

**Easier:**

- A material's alpha now means what it says, and the pass it selects is the one
  the frame needs: opaque first, translucent back to front, over the top. The
  ordering is a property of the renderer, not of the order nodes were created.
- An effect is authorable data plus one system. A game adds
  `schedule.add_system<ParticleSystem>()` and a `ParticleEmitter3D` on a node; the
  burst is a number, not a mesh per frame.
- The particle simulation is pure and headless: emission rate, burst-once,
  ageing out, gravity, node-aimed emission, determinism, extraction
  interpolation, culling, and the serialization boundary are all asserted in
  `fork_engine_tests`. The pixels are asserted in `fork_graphics_tests`: a
  translucent surface blends over what is behind it (with an opaque control that
  hides it), a billboard draws its colour, and a billboard behind an opaque mesh
  is hidden by it.

**Harder / not yet done:**

- **Sorting is per instance, not per triangle.** Two translucent surfaces that
  interpenetrate are ordered by their bounds' centres and can still blend
  wrongly where they cross. That is the standard trade for a first cut; a
  per-triangle sort is a different order of work.
- **The distance key is the bounds centre**, which is wrong for a long object
  whose centre is far but whose near end is close. It is cheap and needs no
  per-instance projection, and it is right whenever two instances' bounds do not
  overlap in depth.
- **A translucent mesh is drawn double-sided**, because the renderer never culls
  and the RHI has no cull state. A closed translucent object therefore blends its
  unlit back faces through its front ones and looks darker than a single surface
  would. `Material::double_sided` is still not consulted by any backend.
- **Skinned meshes are opaque.** A translucent material on a `SkinnedMeshRenderer3D`
  still draws in the opaque skinned pass; the transparent pass handles static
  meshes and billboards only.
- **Billboards are camera-facing, not velocity-aligned or stretched**, and a
  particle has no texture, rotation, or sub-frame motion. A trail or a stretched
  spark needs more per-particle state.
- **Particles are culled by their centre**, so a large particle straddling the
  frame edge can disappear a frame early.

## Alternatives considered

- **Blend translucent instances in place in the opaque pass.** Rejected: the
  result would depend on extraction order, so the same scene would look different
  after a node was re-created or the extraction became parallel, and a farther
  surface drawn later would vanish behind a nearer one that wrote depth.
- **Sort the whole instance array by distance.** Rejected: it would reorder the
  opaque run for no benefit and break the batch runs that make instancing pay,
  and it would change the draw list of every existing scene.
- **Draw transparent meshes without depth testing.** Rejected: a translucent
  surface behind an opaque wall would then draw over the wall, which is a worse
  artifact than a slightly wrong sort between two translucent surfaces.
- **Simulate particles in the renderer.** Rejected: the renderer would own
  gameplay state, a headless test could not step it, and a paused editor could
  not freeze it without a renderer-specific flag. The component-and-system split
  matches the animation path the engine already uses.
- **A CPU mesh per particle.** Rejected: it puts the simulation on the vertex
  path (one buffer per particle per frame) and throws away instancing. A billboard
  shader is one draw for the whole emitter.
- **Randomness from `rand()`.** Rejected: its sequence differs between C
  libraries, so the determinism test could not assert positions, and two builds
  of the same game would show different sparks.
- **A texture per particle.** Deferred, not rejected: a soft radial falloff in
  the shader needs no asset and no texture bind, which is the right first cut; a
  textured particle is additive later and does not change the interface.
