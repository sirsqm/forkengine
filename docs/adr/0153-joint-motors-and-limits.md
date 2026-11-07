# ADR-0153: Joint motors and prismatic translation limits

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0134](0134-extended-joint-set.md),
  [ADR-0135](0135-joint-reactions-and-breaking.md),
  [ADR-0021](0021-physics2d-joints.md)

## Context

[ADR-0134](0134-extended-joint-set.md) extended the joint set with a fixed
(weld) joint and a prismatic (slider) joint, and gave the distance joint length
limits. It left three gaps open, recorded in its "not yet done" list: a prismatic
joint slid freely along its whole axis with no travel stops, no joint could be
driven (a motor), and no joint could break under load. A piston with end caps, an
elevator that climbs at a set speed, a powered drawer, and a windmill were all
unauthorable.

[ADR-0135](0135-joint-reactions-and-breaking.md) closed the third gap: every
joint reports the force it carries and snaps past an authored threshold. The
motors and the prismatic limits landed in the same period and were never written
down, so this record covers them and closes the drift between the code and the
docs. The code shipped and is tested; this ADR is the missing decision record,
not a plan.

Constraints that shaped the design, inherited from ADR-0134:

- **The frozen default.** A joint with no motor and no limits must step
  bit-for-bit as it did. Every new field defaults to zero/off, so a scene that
  sets nothing is unchanged.
- **The load is a force, not a step-dependent impulse.** A motor authored as a
  maximum torque (or force) in newton-metres (or newtons) must mean the same at
  60 Hz and 120 Hz, so the authored value is multiplied by the step to become the
  per-step impulse clamp.
- **One seam, two worlds.** A motor is one design expressed in both solvers; the
  2D and 3D row builders line up, with the 3D revolute motor naming an axis
  because a 3D revolute joint leaves rotation about every axis free.
- **A joint is a component with a stable identity.** The motor and limit fields
  are component fields, registered with reflection and round-tripped by the
  serializer, the same rule every joint follows.

## Decision

### Motors

A motor drives a joint's free degree of freedom towards a target speed, with a
maximum effort it may spend. It is a clamping constraint row, so it adds no new
solving mechanism -- the shared apply helper already holds a clamping row's
accumulated impulse inside its bounds.

- **Revolute motor.** `motor_speed` (radians per second) is the relative angular
  velocity the motor drives B towards; `motor_max_torque` (newton-metres) is the
  largest torque it may apply. In 2D the pin's free degree of freedom is the one
  relative angle. In 3D the joint leaves rotation about every axis free, so the
  motor names the axis it turns with `motor_axis` (body A's local frame, rotated
  to world when the row is built). A wheel, a windmill, or a driven arm is this
  motor on a revolute joint.
- **Prismatic linear motor.** `motor_speed` (world units per second) is the
  relative speed along the slide axis; `motor_max_force` (newtons) is the largest
  force the motor may apply. The row acts along the joint's own slide axis, so a
  piston extends, an elevator climbs, a drawer is powered.
- **The effort cap is a force.** The system builds `JointMotor`/`JointMotor3D`
  with `max_impulse = motor_max_* * delta_time`, so a motor too weak to hold a
  load slips rather than winning, and the authored number is frame-rate
  independent. `max_impulse == 0` disables the motor, so a joint with no motor
  behaves exactly as before.
- **A motor's reaction is part of the joint's reaction.** The motor row is fed to
  `row_reactions` alongside the joint's other rows, so the load a motor carries
  counts toward the joint's reported force and its break threshold (ADR-0135).

### Prismatic translation limits

`enable_limits` (off by default) turns the slider into a limited one that travels
between `lower_translation` and `upper_translation` (B's anchor relative to A's
along the axis; positive means B is ahead), like a drawer's stops or a piston's
end caps.

- The two bounds are two extra rows in the prismatic joint's row set (the 2D
  joint grows from two rows to four; the 3D joint from five to seven). Each row
  is a limit, not a target: it is a one-sided row that only stops motion past its
  bound, so inside the range it is an all-zero no-op the apply helpers already
  treat as slack.
- The bound is compared against the current translation along the axis at build
  time, so the row is a function of the pose and stores no state between steps.

### Shared across the two worlds

- Each new field is a plain component field with reflection, so it round-trips
  and shows in the inspector; a default-constructed joint serializes and behaves
  exactly as before.
- Each system builds the motor row after the joint's positional rows and applies
  it through the same `apply_row` helper, so the sign convention is written once.

## Consequences

**Easier:**

- A driven degree of freedom is authorable: a wheel that spins at a set rate, a
  piston that extends at a set speed, an elevator that climbs, a powered door.
- A motor that cannot hold its load slips rather than winning, because the cap is
  a real maximum effort, which is what a designer expects from a "weak" motor.
- A prismatic joint can be a bounded slider: a drawer with stops, a piston with
  end caps, a lift that cannot overrun.
- The default is unchanged, so every existing scene steps as before and every
  new field is opt-in.

**Harder / not yet done:**

- A motor is a speed target by default; a *position* (servo) target landed
  later, in both 2D and 3D, as the same row with a spring bias and stiffness and
  damping gains, so a joint can hold a pose against a load
  ([ADR-0155](0155-joint-position-motors.md)).
- A revolute joint's angular limits landed later in both 2D and 3D (a hinge that
  stops at 90 degrees), reusing the same one-sided bound machinery on the
  relative-angle row; the 3D hinge names the axis it limits
  ([ADR-0154](0154-revolute-angular-limits.md)).
- The 2D and 3D motor rows are separate functions. They share the design and the
  tests, not the implementation, for the same reason the joint rows differ.

## Alternatives considered

- **An infinite motor (no effort cap).** Rejected: a motor that always wins can
  drag an arbitrarily heavy load and breaks the feel of a weak actuator. The cap
  is what makes "a motor too weak to hold this" expressible.
- **Author the cap as a per-step impulse.** Rejected: it would depend on the
  step size, so the same motor would hold a load at 60 Hz and slip at 120 Hz.
  The cap is authored as a force/torque and converted with the step.
- **A position target instead of a speed target.** Deferred, not rejected: a
  speed motor is the smaller, more general primitive and the one a wheel or a
  piston wants; a position target can be layered on it later.
- **Fold the motor and limits into ADR-0134.** Rejected: ADR-0134 is a complete
  record of the extended joint set as it stood, and its "not yet done" list is
  the honest state at that time. A later record closes the gap and points back,
  which is how the ADR set stays a history rather than a single moving document.
