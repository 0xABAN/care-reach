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

For a small Cartesian translation, the adapter starts from measured encoder
ticks, computes FK, shifts only the tool position, and asks LeRobot/PlaCo for
IK with position weight 1 and orientation weight 0.01. It then converts the
result back to ticks and rejects nonfinite answers, model residual over 3 mm,
steps longer than 30 mm, any arm joint jump over 250 ticks, or a goal outside
the arm's saved soft bounds. It never silently clips a solution. Run each
accepted joint target through the live measured-motion controller as well;
that controller checks firmware limits, load, fault, voltage, torque, and
progress while moving. Only one process may own the URT-2 bus. Stop the
dashboard service before a direct-serial test and restart it afterward.
Neither operation releases torque.

For Cartesian trials, use `MeasuredPoseRunner.move(..., tolerance=15,
trim_deadband=True, hold_target_on_success=True)` and read the encoders again
after the goal settles. The opt-in trim sends short bounded goals beyond small
remaining errors to overcome servo deadband; the exact endpoint then remains
the holding goal. A runner stop or fault still holds measured position. Check
the measured FK result after every leg, since a solved model pose is not proof
that the servos reached it.

This is *local* IK from a measured pose, not a collision-aware path planner.
The official URDF reports self-collisions even in neutral, and its base
collision geometry is incomplete. We have not measured the real tool's
Cartesian position or the camera-to-tool transform. Physical zero/direction
and link geometry must be checked against external measurements before using
absolute targets near people, tables, or the wheelchair. The dashboard's
Home and Fold remain recorded tick poses and do not depend on IK.
