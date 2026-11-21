# ADR-0162: The view matrix's basis vectors are rows

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0010](0010-3d-rendering-foundation.md),
  [ADR-0161](0161-cascaded-directional-shadows.md)

## Context

`look_at` built the view matrix by writing the three basis vectors into its
*columns*:

```cpp
r.at(0, 0) = right.x;
r.at(1, 0) = right.y;
r.at(2, 0) = right.z; // ... and the same for true_up and -forward
```

Everything else in the engine reads the other way. `Mat4::at(row, col)` indexes
`m[col * 4 + row]`, `transform_point` computes `row . p`, `operator*` is the
standard product, and the OpenGL backend uploads `m[]` with `glUniformMatrix4fv(…,
GL_FALSE, …)`, so a shader's `mat4 * vec4` uses the same rows. Written into the
columns, `look_at` returned the *transpose* of the view matrix it meant to
return.

The bug was invisible for a long time because a camera with no rotation has an
identity basis, and the transpose of the identity is the identity. Every 2D
camera, and every 3D test that looked straight down an axis, was correct. A
pitched camera got the inverse of its rotation: a camera tilted 45 degrees down
aimed 45 degrees up, and the frame was mirrored about the horizon.

It was found by the cascade fit in [ADR-0161](0161-cascaded-directional-shadows.md).
That fit unprojects the camera frustum's corners through the inverse
view-projection, and its first test — "each cascade's light box contains its
slice of the frustum" — failed with `ndc.y = 1.50`. The same check run as a
direct question ("a floor point three units towards the camera must project below
the centre") showed the point at the top of the screen.

## Decision

`look_at` writes each basis vector as a **row**, with the translation in column
three:

```cpp
r.at(0, 0) = right.x;
r.at(0, 1) = right.y;
r.at(0, 2) = right.z;
r.at(0, 3) = -right.dot(eye);
// true_up on row 1, -forward on row 2
```

That is the one convention the rest of the engine already uses, so the view
matrix, `transform_point`, the matrix product, and the shader upload now agree
without a transpose anywhere.

## Consequences

- A pitched 3D camera aims where its transform says it does. The 3D demo and the
  playable game change, correctly: the horizon moves to where the camera points.
- The graphics tests that sample the centre of a symmetric scene are unaffected,
  because the look-at target still projects to the centre. The tests that would
  have caught it did not exist until the cascade fit needed the inverse.
- A test that reads a pixel has to know where the geometry lands, so the cascade
  test now asserts the corrected mapping: a floor point towards the camera is at
  the *bottom* of the frame.

## Alternatives considered

- **Transpose on upload instead.** `glUniformMatrix4fv(…, GL_TRUE, …)` would make
  the column-written basis correct in the shader, but `perspective()` is already
  written row-major, so the projection would then be transposed, and the CPU's
  own `transform_point` would still disagree.
- **Keep both conventions and document which callers get which.** A matrix whose
  meaning depends on who reads it is the defect, not the fix.
- **Leave it and build the cascade fit on the transposed matrix.** The fit would
  then describe a frustum the renderer does not draw, which is a second bug
  layered on the first.
