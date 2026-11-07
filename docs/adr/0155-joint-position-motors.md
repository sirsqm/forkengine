# ADR-0155: Joint position (servo) motors

- **Status:** Accepted
- **Date:** 2026-11
- **Related:** [ADR-0153](0153-joint-motors-and-limits.md),
  [ADR-0154](0154-revolute-angular-limits.md),
  [ADR-0134](0134-extended-joint-set.md)

## Context

Joint motors landed as speed motors ([ADR-0153](0153-joint-motors-and-limits.md)):
a revolute motor drives the relative angular velocity towards `target_speed` and
a prismatic motor drives the slide speed towards a target, each clamped to a
maximum impulse so a weak motor slips instead of winning. That is exactly a wheel
or a piston, and it is enough for a windmill or a conveyor.

It is not enough for a *servo*. A robot arm that holds a pose, a door held ajar,
a limb posed for an animation, or a turret tracking a bearing all need the joint
to drive a **position**, not a rate: given a target angle, apply whatever torque
it takes (up to a maximum) to reach and hold it against gravity or a load. A
speed motor cannot express that -- with no target rate it does nothing, and it
has no notion of where the joint is.

ADR-0154 named this as the one remaining refinement: "a position/spring motor
target remains." This record closes it, in both 2D and 3D, for both motorised
joints: the revolute (angular) joint and the prismatic (linear) joint.

## Decision

A joint motor gains a position target. The same row a speed motor builds is
reused; only the bias changes from a target rate to a spring, so no new solver
machinery is introduced.

### The row

`JointMotor` (2D) and `JointMotor3D` gain four fields:

- `use_target` (off by default) selects position mode. A motor with no target is
  the speed motor it was, so an existing scene is unchanged.
- `target_position` is the coordinate to hold -- the relative angle `b - a` in
  radians (about the named axis in 3D) for a revolute joint, or the signed slide
  `b - a` along the axis for a prismatic one.
- `stiffness` and `damping` are the gains, with the same meaning as the distance
  joint's spring: `stiffness` is a restoring velocity per unit of error and
  `damping` is a fraction in `[0, 1]` of the relative velocity the row opposes.
  The defaults (0/0) are a motor that does nothing until the gains are authored.

Each joint type gets a position builder beside its speed builder:
`build_revolute_position_motor_row` and `build_prismatic_position_motor_row`, in
both 2D and 3D. Each builds the same row the speed motor builds -- a purely
angular row (`angular_a = -axis` / `angular_b = +axis`) for the revolute, a
linear row along the slide axis for the prismatic -- and sets its bias to
`stiffness * error + damping * row_velocity`. Because the row's impulse is
`-eff * (row_velocity + bias)`, a positive error drives a negative row velocity,
so the coordinate moves back towards the target -- the same sign convention the
distance spring uses. For the revolute joint the error is the shortest signed
difference `current - target`, so a target across the +/-pi seam is approached
the short way. The impulse is clamped to `max_impulse` as before, so a servo too
weak to hold its pose slips rather than winning.

The row is built with the joint's current coordinate, which the system passes
in. For a revolute joint, in 2D that is `b.rotation - a.rotation`; in 3D it is
the rotation vector of B relative to A projected onto the world motor axis --
the same quantity the angular limits measure
([ADR-0154](0154-revolute-angular-limits.md)). For a prismatic joint it is the
signed slide `prismatic_slide_coordinate` returns, the same coordinate the
translation limits measure. So a servo and a limit on the same joint always
agree on the coordinate.

### The component

`RevoluteJoint2D`/`RevoluteJoint3D` and `PrismaticJoint2D`/`PrismaticJoint3D`
each gain `motor_use_target` plus a target and two gains -- `motor_target_angle`,
`motor_stiffness`, `motor_damping` for the revolute joints and
`motor_target_translation` for the prismatic ones -- registered for reflection
and serialization beside the existing motor fields. The system selects the
position row when `motor_use_target` is set and the speed row otherwise, so the
two modes share the joint, the axis, and the force/torque bound.

## Consequences

- A joint can hold a pose or a position: a servo arm against gravity, a door
  ajar, a posed limb, a turret tracking a bearing, a lift that holds a floor, a
  powered drawer that stays shut. The gains give a spring feel when soft and a
  stiff hold when high, all within the authored maximum torque or force.
- No new solver machinery: the position motor is the speed motor's row with a
  spring bias, so the clamping, the reaction reporting, and the breaking
  threshold all apply unchanged.
- Off by default, so every existing scene mixes exactly as it did.
- The 3D revolute servo drives one axis, like the 3D motor and the 3D limits; a
  joint that must hold a full orientation uses a fixed joint, not a servo.

## Alternatives considered

- **A separate servo joint type.** Rejected: it would duplicate the pin's rows,
  its limits, and its motor clamp for one different bias, and a joint could no
  longer switch between a speed motor and a servo without being replaced.
- **Drive the position with a direct positional correction** (the
  `position_factor` path a rigid row uses). Rejected: that path removes the whole
  error at a fixed rate regardless of load, so it cannot be a *spring* and cannot
  give way under a torque it cannot hold; the velocity bias with an impulse clamp
  is what makes the maximum torque meaningful.
- **A speed motor with an external controller.** Rejected: it pushes the servo
  loop into every caller and still cannot hold a pose at rest, which is the case
  a servo exists for.
