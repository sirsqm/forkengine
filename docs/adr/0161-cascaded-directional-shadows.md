# ADR-0161: Cascaded directional shadows

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0014](0014-directional-shadow-mapping.md),
  [ADR-0053](0053-spatial-bvh.md),
  [ADR-0101](0101-instanced-3d-draws.md),
  [ADR-0150](0150-per-object-shadow-casting.md),
  [ADR-0162](0162-view-matrix-rows.md)

## Context

A directional light's shadow map was one 1024x1024 depth texture fitted to the
world bounds of every caster ([ADR-0014](0014-directional-shadow-mapping.md)),
with a per-object `casts_shadow` flag deciding who shapes the fit
([ADR-0150](0150-per-object-shadow-casting.md)). That gives one texel density for
the whole scene: a character two units from the camera and a tower thirty units
away are resolved by the same map, so the near shadow carries the stair-step of
a map sized for the far one, and enlarging the map trades memory for resolution
everywhere at once.

The constraints were the ones the 3D path already works under:

- **One depth texture and a hand-written comparison.** The shader samples the
  map with `texture()` and compares by hand (compare mode NONE), so the same
  source compiles for desktop GL and WebGL2 (GLSL ES 3.00). A cascade scheme has
  to fit inside that: no sampler array, no compare sampler.
- **The null device is a first-class backend.** `Renderer3D` must keep running
  its passes against it, so the cascade count has to be a constant the CPU side
  owns, not something discovered from the device.
- **The extraction seam stays the only place the scene is read.** Casters arrive
  as `render::MeshInstance` values with world bounds; the fit reads those bounds
  and nothing else.
- **A shadow test must fail when the thing it tests is removed.** The existing
  shadow tests sample one pixel under one occluder, which a single-cascade
  renderer also passes, so they cannot distinguish cascaded from not.

## Decision

The directional shadow map is **two cascades sharing one atlas**, laid out side
by side, so the pass switches viewport rather than render target and the shader
samples one texture.

- **The band** the cascades divide is a view-depth range: it starts at the later
  of the camera's near plane and the nearest caster, and ends at the farthest
  caster. Nothing nearer than the near plane is drawn, and nothing farther than
  the farthest caster can cast into the view. Starting at the nearest caster
  alone would be tighter, but a caster whose bounding box straddles the camera
  plane — a ground plane extending behind the camera — reports a nearest depth of
  zero and would hand the near cascade almost no world to cover.
- **The split** is the practical split scheme (Zhang et al.): a blend of the
  logarithmic and uniform partitions with `lambda = 0.6`, so the near band
  follows how perspective spends texels on distance without collapsing to the
  logarithmic scheme's very short near band.
- **Each cascade's light box** is fitted to the eight corners of its slice of the
  camera frustum, which is where the density comes from: the near box covers a
  fraction of the far box's world extent at the same texel count. The
  projection's *depth* range is extended to cover every caster, so a caster
  outside the slice still shadows into it instead of being clipped out of the map
  at the band edge.
- **The shader** turns a world position into a view depth (`u_camera_forward`,
  `u_cascade_split`), picks the cascade, projects with that cascade's matrix,
  remaps x into the cascade's half of the atlas, and clamps its 3x3 taps to the
  tile so a filter at the seam cannot read the other cascade's depths. Outside a
  cascade's box the fragment is lit, which is also what keeps a scene with no
  directional light unchanged: the matrices are identity and the map is white.
- **The math is a seam of its own.** `fe/renderer3d/ShadowCascades.h` holds the
  partition, the frustum-slice corners, the caster depth range and the per-cascade
  fit as pure functions, so the arithmetic is tested with no GPU on the
  pure-logic job. `Renderer3D` reports `last_shadow_cascades()` and
  `last_cascade_split()` so a graphics test can prove *which* cascade shaded a
  sample instead of assuming.

## Consequences

- A near shadow resolves at roughly twice the texel density of a single map of
  the same size, and the shadow pass costs one draw per caster per cascade.
- A scene whose camera near plane is far closer than its geometry spends the near
  band on empty space. That is the standard trade-off: the band starts at the
  near plane because nothing nearer is drawn, and an author who wants the near
  cascade to carry close detail sets a near plane near the closest geometry.
- There is no blending between cascades, so a shadow crossing the split can show
  a resolution step. Nothing in the engine's scenes is long enough for it to read
  as an artefact yet; blending is a later addition, not a gap in this one.
- The cascade count is a constant (`kShadowCascades`), so raising it to four is a
  shader array size and an atlas width, not a redesign. Two is what the current
  scenes justify.
- The fit needs the inverse view-projection, which is what surfaced the view
  matrix bug in [ADR-0162](0162-view-matrix-rows.md): the containment test failed
  because the frustum corners were being unprojected through a transposed camera.

## Alternatives considered

- **One larger map (2048 or 4096).** Memory and fill cost scale with the whole
  scene, and the near shadow still shares its density with the far geometry.
- **One render target per cascade.** A second framebuffer, a bind per cascade, and
  a second texture unit; the atlas keeps one target, one clear, and one binding.
- **Four cascades.** More uniform state, a wider atlas, and two more passes for a
  scene extent the engine's games do not have yet. The count is a constant so it
  can grow when a scene needs it.
- **Splitting from the camera's near and far planes only.** Simpler, and what
  most engines do, but it wastes the near band whenever the near plane is much
  closer than the geometry — which is the engine's default (0.1) against scenes
  measured in single-digit units.
- **A sampler array of cascades.** Cleaner in the shader, but a dynamic index into
  a sampler array is not portable to GLSL ES 3.00, which is a shipping target.
