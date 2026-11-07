# ADR-0156: Navigation and AI

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0002](0002-hybrid-scene-ecs-architecture.md),
  [ADR-0008](0008-systems-and-frame-schedule.md),
  [ADR-0031](0031-physics3d-and-backend-seam.md),
  [ADR-0062](0062-tilemaps.md)

## Context

The engine can render, simulate, animate, mix audio, draw UI, and run gameplay
modules. It cannot answer the one question every non-trivial game asks: *where
can this thing walk, and how does it get there?* Both shipped vertical slices
(breakout, Gem Rush) move the player directly from input and have no enemies,
because there is no navigation layer to move anything else. An agent that
patrols a route or chases the player is the difference between a tech demo and a
game, and the roadmap's "the game is worth shipping" phase has no way to build
one.

Two shapes of the problem exist, and the engine already has both:

- A 2D level is a `Tilemap` (ADR-0062). It already knows which cells are solid:
  `Tilemap::is_solid`, the single rule the physics collision build reads.
- A 3D level is a set of static `Collider3D` shapes on the scene graph
  (ADR-0031). There is no grid, only boxes and spheres in world space.

The trap to avoid is a *second* description of the level. A hand-authored navmesh
is an asset that can drift from the geometry it is supposed to describe, and
keeping the two in sync is a tool, a build step, and a class of bug. The level
already exists; navigation should be *derived* from it, exactly as tile collision
is derived from the tilemap rather than from a parallel collider grid.

## Decision

A new `nav` module holds navigation and steering, in both dimensions, built on
the level the game already authored.

### The navigation grid is derived, not authored

`NavGrid2D` is a uniform grid of walkable cells. It is built from a `Tilemap`
(and, optionally, the scene's `Collider2D` obstacles) by `build_nav_grid_2d`:
a cell is walkable when the tile under it is not solid and no collider blocks
it. This is the same solidity rule the physics build reads, so the ground an
agent walks on and the ground a body collides with are one fact, not two.

`NavGrid3D` is a uniform voxel grid built from the scene's static 3D colliders by
`build_nav_grid_3d`: a cell is walkable when it does not intersect any collider
box or sphere. A grid over boxes is the honest first cut; a polygon navmesh is
the refinement if the grid's memory or its diagonal cost ever matters.

Both grids are runtime data owned by the system that built them, not components:
a grid is a function of the scene, cheap to rebuild when the scene changes, and
would be a poor thing to serialize (it is derivable, and a stale copy is a bug).

### Pathfinding is A* over the grid

`find_path_2d` runs A* over the 8-connected grid with a Euclidean heuristic and
*no corner cutting*: a diagonal step is allowed only when both orthogonal
neighbours are walkable, so an agent cannot slip between two diagonal walls. The
3D counterpart is 26-connected with a Euclidean heuristic. Both return the
walkable cell centres from start to goal, or an error when no route exists.

### Paths are smoothed by line of sight

A grid path stair-steps. `smooth_path_2d` / `smooth_path_3d` walk the raw path and
keep the farthest waypoint each step can still see in a straight line
(`line_of_sight_2d`, a supercover ray over the grid), so the result hugs corners
and the agent walks diagonals instead of zig-zags. This is the grid analogue of
the funnel algorithm, and it needs no polygon mesh to run.

### Steering turns a path into a velocity

`steering` is a set of pure functions over a desired velocity: `seek`, `flee`,
`arrive` (which eases in so an agent does not overshoot its goal), `wander`
(a persistent random heading, not per-frame noise), `separation` and `avoidance`
(which push an agent away from its neighbours and from the walls ahead). Each
returns a velocity contribution; a caller combines them by weight. They are
pure, so they are tested without a scene.

### An agent is a component; a system advances it

`NavAgent2D` and `NavAgent3D` are components: an agent names the grid it walks
(by node, the tilemap host or a named 3D root), a goal, a speed, an
acceleration, and the steering weights. `NavigationSystem2D` and
`NavigationSystem3D` own the grids, gather the agents, plan a path when a goal
changes, and advance each agent along it, publishing a desired velocity. The
agent's node is moved by the system directly (a kinematic mover, the same shape
`CharacterController3D` uses), so navigation works with or without a rigid body;
a game that wants physics can read the published velocity instead.

The steering weights are reflected component fields, so an agent round-trips
through the serializer and shows in the inspector like any other component.

## Consequences

- A game gets patrolling and chasing enemies from the level it already has: the
  nav grid is derived from the tilemap or the colliders, so there is nothing new
  to author and nothing to keep in sync.
- Both dimensions are covered by one module with one shape, so a 2D and a 3D
  game use the same concepts (grid, A*, smoothing, agent, system).
- A grid is an approximation: diagonal cost and clearance are cell-quantised, and
  a voxel grid is memory-hungry at fine resolutions. A polygon navmesh is the
  refinement, not the starting point, and the grid is behind the same seam a
  navmesh could later implement.
- The agent moves its node directly by default. That is the simple, predictable
  choice for a kinematic character and it is what the shipped example uses; a
  physics-driven agent reads the published velocity and is a caller's choice, not
  a second code path in the system.

## Alternatives considered

- **A hand-authored navmesh asset.** Rejected: it is a second description of the
  level that can drift from the geometry, and it needs a bake tool and a file
  format before a single agent moves. Deriving from the tilemap and the colliders
  gets an agent walking in one step and cannot disagree with the level.
- **Navigation inside the physics module.** Rejected: navigation is a *consumer*
  of geometry, not a simulation of it. Putting it in `physics2d`/`physics3d`
  would make the solver depend on a grid it never uses and would duplicate the
  module in both dimensions. A separate `nav` module depends downwards on
  `scene` and reads colliders through their public shape queries.
- **Steering baked into the path follower.** Rejected: a steering behaviour is a
  pure function of positions and velocities, and keeping it separate lets a game
  compose its own (flocking, a formation, a scripted dodge) from the same
  primitives the built-in agent uses.
