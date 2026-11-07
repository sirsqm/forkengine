# ADR-0133: Prefab instancing: a subtree captured once, instantiated many

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0045](0045-component-identity.md),
  [ADR-0047](0047-local-operation-model.md),
  [ADR-0019](0019-collaborative-editing.md),
  [ADR-0062](0062-tilemaps.md)

## Context

The scene row left prefab instancing open. A repeated object — a bullet, a tile's
decoration, a wall segment — is authored as a full node subtree each time it
appears, so a scene with many copies carries the whole subtree's nodes,
components, and ids once per copy. Memory and load time grow with the copy count
even though the copies are identical by construction.

The engine already has the two pieces a prefab needs, and neither is used for
this yet:

- The scene serializer writes and reads a whole node subtree through reflection
  (`SceneSerializer`), and it names no component type: it asks the reflection
  registry for a type by name and applies the properties the descriptor lists.
  A subtree is therefore already a serializable value with no per-type code.
- Identity is stable and explicit. Every node and component has a `Uuid`
  ([ADR-0045](0045-component-identity.md)), references name their targets by
  that id rather than by pointer or index, and the serializer already round-trips
  ids. So an instance can be made independent by giving it fresh ids and
  rewriting the references that pointed at the old ones.

What was missing was the operation that ties them together: capture a subtree as
a definition, and produce a fresh, self-contained instance of it on demand.

## Decision

**1. A prefab is a scene fragment, not a second format.** `Prefab::capture`
serializes the chosen subtree with the scene serializer's own writer, and
`Prefab::instantiate` reads it back with the same reader, appended to a live
scene rather than replacing it. The definition is the exact document shape a
scene file uses (one root), so there is one parser and one set of property
mappings, and a component that persists is a component that instantiates with no
per-type work in the prefab code.

**2. Instances are independent because their identities are.** Before the
definition is read into the scene, every node and component id it names is
replaced with a fresh `Uuid`, and every reference inside the definition that
named an old id is rewritten to the new one. Two instances therefore share the
definition but not their state: moving one leaves the other and the definition
unchanged, and a joint inside the prefab points at the instance's own bodies, not
the definition's.

**3. Only reflected `Uuid` properties are rewritten.** The remap walks the
reflected properties of each component and rewrites a value only when its kind is
`Uuid` (a body reference such as a joint's `body_a`). A resource reference is a
string too, but it names an asset guid that every instance shares with the
definition, so it is left alone. A property kind that names something the prefab
does not contain (a node outside the subtree) is not rewritten, because the remap
holds only ids collected from the definition.

**4. Instantiation appends and is all-or-nothing.** The reader appends to the
scene, so an instance joins a live scene without disturbing its existing roots.
If the read fails, the roots it had appended are removed again, so a failed
instantiation leaves the scene as it was.

## Consequences

Easier:

- A repeated object is authored once. Many instances cost the nodes and
  components they actually carry, and the definition is a single document.
- A prefab is a file. Because it is the scene document shape, a prefab can be
  saved, loaded, diffed, and hand-authored with the tools the scene format
  already has.
- A new component type is instantiable with no prefab-side edit: it already
  registers with reflection for persistence.

Harder, or deferred:

- The remap relies on the reflection model being complete. A component that
  stores a node reference outside a reflected `Uuid` property would keep the
  definition's id in an instance; the model has no such case today, and adding
  one is a signal to reflect it.
- Gameplay scripts bound to a prefab are not carried into an instance yet.
  Scripts are bound through `ScriptBinding` after a scene is built
  ([ADR-0063](0063-reflection-backed-script-properties.md)); instancing a prefab's scripts
  is a follow-up, and the instance is a plain unscripted subtree for now.
- A shared definition edited in place does not push the change to existing
  instances. Instances are copies, not live links, which matches the row's
  "editing one instance leaves the other unchanged" requirement; a live-link mode
  is a separate decision.

## Alternatives considered

**A parallel prefab format and loader.** Rejected: it would duplicate the
property mappings and drift from the scene format, and every new component would
need a second registration. Reusing the scene document is one format and no
second parser.

**Instances as live references to a shared definition (the "prefab link" model
some editors use).** Rejected for this row: it requires the definition to be
addressable and edited in place, and the row asks specifically that editing one
instance leave another unchanged. Independent copies are the smaller, correct
first step; a link mode can be layered on the same document later.

**Copy the subtree in memory rather than through the serializer.** Rejected: the
serializer is the one place that already knows how to copy every reflected
property, including resource references and nested groups. A hand-written memory
copy would be a second, per-type implementation that drifts.
