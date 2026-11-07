# ADR-0150: A per-object shadow-casting flag

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0014](0014-directional-shadow-mapping.md),
  [ADR-0101](0101-instanced-3d-draws.md),
  [ADR-0104](0104-skeletal-animation.md)

## Context

ADR-0014 added a single-cascade directional shadow map and deliberately left out
a per-object shadow flag, recording it as the first thing a user would want for
a large ground plane and deferring it "until a concrete need appears". The need
has appeared. The map covers the whole scene in one cascade, fitted to the world
bounds of the geometry that draws, so a large ground plane does two bad things:
its own shadow is never seen (the sun is above it, so it only ever shadows
itself), and its extent stretches the one 1024×1024 map over the whole world,
starving the objects whose shadows *are* seen of resolution. A floor plane is
exactly the mesh a user wants to exclude, and there is no way to exclude it.

The forces at play:

- **The flag must be opt-out, and default to casting.** Every scene and every
  test was authored when every mesh cast. A default of "cast" keeps those
  byte-identical; a default of "do not cast" would silently darken the world.
- **The renderer batches.** Instanced draws (ADR-0101) group consecutive
  instances that share a mesh, textures, and shading state into one call. If the
  shadow pass is to draw a whole batch or skip it, a caster and a non-caster must
  never share a batch, so the flag has to be part of the batch key.
- **Two renderer components cast.** A static `MeshRenderer3D` and a skinned
  `SkinnedMeshRenderer3D` (ADR-0104) each run a shadow path. A flag on only one
  would make the knob inconsistent between a prop and a character.
- **The fit reads the bounds.** The shadow map's orthographic box is fitted to
  the union of the instances' world AABBs. If a non-caster is left in that
  union, the fit still stretches over it and the resolution win is lost even
  though the draw was skipped. Excluding it from the fit is the half of the
  feature that actually helps the case the flag exists for.

## Decision

**Add a `casts_shadow` bool to `MeshRenderer3D` and `SkinnedMeshRenderer3D`,
default true, and honour it in all three places the shadow pass touches an
instance: the fit, the static draw, and the skinned draw.**

### The flag is component state, reflected and serialized

`casts_shadow` is a plain `bool` member on each renderer component, registered
with the ordinary scalar property builder beside `specular`, so the inspector
shows it and the serializer round-trips it with no bespoke path. It defaults to
`true`, so a document authored before the flag — which has no key — reads back
the default and casts as it always did.

### A non-caster is excluded from the fit, not only from the draw

The shadow pass builds the light's orthographic box from the world AABB union of
the *casting* instances. A non-casting mesh is skipped when the union is built,
so a large ground plane neither casts nor widens the map. A scene whose only
mesh does not cast therefore fits a degenerate (near-point) box; that is
harmless because the pass then draws nothing into it, and the main pass is
unaffected.

### The flag is part of the batch key

`Batch` carries `casts_shadow`, and it is compared with the mesh, textures, and
shading state when consecutive instances are merged. A caster and a non-caster
of the same mesh therefore split into two batches even when everything else
matches. The main pass draws every batch; the shadow pass skips a batch whose
flag is false. The skinned pass is per-instance (one draw each) and skips a
non-casting character directly.

Receiving shadows is unchanged. The main lit shader samples the shadow map for
every fragment it draws, whatever the flag says, so a non-casting ground plane
still receives the shadows the props cast onto it — which is the point: a floor
should be shadowed by what stands on it, not shadow itself.

## Consequences

**Easier.** A large ground plane can be authored to receive shadows without
casting them, which both removes a caster whose shadow is never seen and frees
the single shadow map to cover only the geometry whose shadows matter. The flag
reads the same on a static prop and a character, and it round-trips through the
existing reflection and serialization path with no new machinery.

**Harder / not yet done.** A `receives_shadow` flag is still deferred: with one
material path, a mesh that is drawn is lit and receives, so the flag would have
nothing to switch off until a second path (an unlit or shadow-free material)
exists. Per-object flags do not fix the underlying resolution limit — one
cascade still shares one map — so cascaded shadow maps remain the change that
buys resolution everywhere, and this flag is the cheap, local alternative that
answers the ground-plane case now.

## Alternatives considered

- **Default `casts_shadow` to false.** Rejected: it silently changes every
  existing scene, and the common case (a prop that casts) is the one that would
  have to opt in.
- **Leave the fit alone and only skip the draw.** Rejected: it fixes the draw
  cost but not the resolution loss, which is the half that makes a large ground
  plane a problem. The fit has to see only casters for the feature to help.
- **A `receives_shadow` flag in the same change.** Rejected: receiving is
  already correct and there is no second material path for the flag to switch
  to, so it would add component surface and serialization for a knob with no
  effect yet — the same reasoning ADR-0014 used to defer the pair.
- **A per-mesh flag on the resource rather than the component.** Rejected: the
  same mesh can be a casting prop in one place and a non-casting floor in
  another, so the flag belongs to the placement, not the asset.
