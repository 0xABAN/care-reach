# SO-101 kinematics on the robot Pi

The arm uses the official SO-101 `so101_new_calib.urdf` model from
`TheRobotStudio/SO-ARM100` commit `a758567c3978dfeefe282ede0500085a48fe8f78`.
The Pi caches `Simulation/SO101/` at
`~/.cache/care-reach/so101-model/`; the URDF references meshes in that folder.
Its Python environment has `placo==0.9.25` and LeRobot source pinned to
`d3793b7e36522321c1c455f0dc4826db3793dceb`. The model files are cached,
not committed into this project. To reconstruct that cache, sparse-clone the
SO-ARM100 commit and copy `Simulation/SO101/` with its meshes to that path.

`so101_ik.py` wraps LeRobot's `RobotKinematics` for the five arm joints and
the `gripper_frame_link` tool frame. It reads the arm's actual calibration and
converts each raw STS3215 tick to model degrees as
`(tick - (range_min + range_max)/2) * 360/4095`. The inverse calculation
rounds to the nearest raw tick. The five midpoints differ: at the robot's
2000-tick Home, the model angles are not all zero. Servo drive modes must be
zero. The gripper remains a raw tick and is not an IK joint; its URDF joint
does not encode LeRobot's gripper-percent mapping.

The named tool point is the URDF's `gripper_frame_link` by default. A caller
can select another frame with `tool_frame_name` if that frame exists in the
model. Its XYZ coordinates are in the URDF base frame, not camera coordinates.
This is the nominal model point; the real point between the gripper jaws has
not been measured.

`SO101Kinematics.solve_xyz(start_ticks, target_xyz_m)` solves one nearby
**absolute** XYZ request in that base frame. `translate(start_ticks,
delta_xyz_m)` remains available for relative steps. Both use position-only IK
by default (`orientation_weight=0`), seeded from the supplied encoder ticks.
They round the solution to real encoder ticks, compute FK again at those
rounded ticks, and reject a model residual over 3 mm. A single request is
limited to 30 mm of Cartesian travel and 250 ticks per arm joint. Invalid
numbers, calibration ranges, unsafe joint targets, and large solution jumps
are rejected. The gripper tick is preserved.

`plan_xyz(start_ticks, target_xyz_m)` divides a straight model-space line
into nearby candidate waypoints. Its default total-path limit is 150 mm,
with at most 12 waypoints. Each waypoint passes the same limits as
`solve_xyz`. An optional `validate_waypoint(previous_ticks, next_ticks)`
callback checks one leg; `validate_plan(start_ticks, waypoint_ticks)` can
check the complete candidate path. Apply the movement policy to the complete
path, because a path that moves motor 2 first and other joints later is still
disallowed in motor-2-isolated mode. For example, after planning:

```python
from arm_control.movement_policy import validate_movement_path

validate_movement_path(
    mode, plan.start_ticks, [step.target_ticks for step in plan.waypoints]
)
```

The returned `XYZPlan` holds the requested XYZ, each candidate joint pose,
and the final **model-predicted** XYZ. These calls have no hardware access.
They do not check collisions or the swept joint path between waypoints, and
they cannot tell whether the physical gripper reached the point.

Use the published URDF and existing LeRobot calibration as the first FK
estimate. Measuring every link is unnecessary for that first pass. If we
need to know the real XYZ error, we still need at least a few independent
observations of the gripper point at varied poses, such as a calibrated
fixed camera looking at a marker or a small fixture with known points.
Encoder-based FK alone checks the model against itself. Keep observations
used to tune any zero or tool-frame offset separate from those used to check
the result.

Before any planned waypoint is sent to hardware, a live controller must
validate the full motion path and monitor firmware limits, load, status,
voltage, torque, and progress. Only one process may own the URT-2 bus. Stop
the dashboard service before a direct-serial test and restart it afterward.
Neither operation releases torque.

For Cartesian trials, use `MeasuredPoseRunner.move(..., tolerance=15,
trim_deadband=True, hold_target_on_success=True)` and read the encoders again
after the goal settles. The opt-in trim sends short bounded goals beyond small
remaining errors to overcome servo deadband; the exact endpoint then remains
the holding goal. A runner stop or fault still holds measured position. Check
the measured FK result after every leg, since a solved model pose is not proof
that the servos reached it.

Near Home, the earlier 0.01 orientation weight made a 20 mm Z request use
large, opposing elbow and wrist changes; position-only IK produced a much
shorter joint move. Position-only IK does not hold the tool angle, so use an
explicit nonzero orientation weight for tasks that require one. Feedback
corrections also need a regularized or waypoint-aware solver: repeated Z
corrections in the first live trial did not converge below about 8 mm and
eventually reached the joint-jump guard. No automatic Cartesian correction
loop is enabled.

## Live validation, 2026-10-24

Eight nearby paired Cartesian steps were run on the assembled Pi-connected
arm with 15-tick deadband trimming and `orientation_weight=0`. All eight
completed without a servo fault; torque stayed enabled. Predicted endpoints
were within 0.06–0.50 mm of each requested target in the URDF model. Encoder
positions read after settling imply these actual **model-space** translations:

| Request | Encoder-based FK translation (mm) |
| --- | --- |
| X −20, then +20 | (−18.9, +2.1, −6.7), then (+19.8, −3.9, −6.9) |
| Y +30, then −30 | (+0.3, +30.7, −4.5), then (−0.0, −31.4, −9.5) |
| Z +20, then −20 | (+1.6, +0.2, +13.1), then (+0.6, −0.1, −25.6) |
| Diagonal ±15 on each axis | (+16.0, +18.9, +9.4), then (−14.7, −17.6, −19.7) |

This distinguishes solver accuracy from actuator tracking: a valid IK target
does not mean the encoders reached that joint target, and encoder-based FK is
not an external measurement of the tool. The arm returned to Home afterward
with all joints within 9 ticks, goals at 2000, torque enabled, and no faults.
The dashboard service was restored. More aggressive automatic corrections
should wait for a better-conditioned solver and an observed tool pose.

This is *local* IK from a measured pose, not a collision-aware path planner.
The official URDF reports self-collisions even in neutral, and its base
collision geometry is incomplete. We have not measured the real tool's
Cartesian position or the camera-to-tool transform. Physical zero/direction
and link geometry must be checked against external measurements before using
absolute targets near people, tables, or the wheelchair. The dashboard's
Home and Fold remain recorded tick poses and do not depend on IK.
