# SO-101 pose terminology

All tuples below are raw STS3215 encoder ticks in motor order: shoulder pan (1), shoulder lift (2), elbow flex (3), wrist flex (4), wrist roll (5), gripper (6). They belong to this assembled arm and its `care_reach_follower` calibration; they are not transferable to another arm without measurement.

| Term | Raw ticks | Meaning |
| --- | --- | --- |
| **Home** | `(2000, 2000, 2000, 2000, 2000, 2000)` | The powered test pose and start/end pose for the dance and fold planner. It is a convention, not a guarantee that every joint is at its geometric center. |
| **Folded** | `(2040, 893, 3064, 2888, 2416, 1470)` | Lance physically folded the torque-off arm on 2026-10-24; two encoder reads agreed. Stored in [`folded_pose.json`](../folded_pose.json). |
| **Parked** | Folded endpoint | An active `park.py --execute` move from Home to the recorded Folded pose. The command keeps torque enabled, including when it exits. |
| **Coiled** | No separate validated tuple | Informal term for a compact resting pose and the intent to protect the arm before eventual torque release. Treat Folded as the measured candidate, not as proof that torque release is safe in every setup. |

## Motion rules

- Check the current measured pose, clear workspace, torque, voltage, fault flags, and saved limits before motion. A pose file records an endpoint, not the current arm position.
- Only one process opens the URT-2 serial port. Stop the Pi dashboard service before `home.py`, `park.py`, `dance_demo.py`, or another controller; start it again afterward.
- `park.py` permits the small measured exception required for Folded: motor 2 is 10 ticks below its saved panel lower bound, and motor 3 is 77 ticks above its saved upper bound. That exception is confined to the validated fold path. It does not change the general soft bounds, LeRobot calibration, or firmware limits.
- None of these scripts automatically disables torque. Before any future torque-release function is added, validate a pose that remains mechanically supported after the wheelchair mount, then test its hold and release behavior under that mounting.
- The jaw's measured contact was 1317 ticks, but ordinary position-mode motion uses a provisional 1340-tick lower stop. Motor 5 encountered resistance near 3358 ticks, so ordinary motion stops at or below 3300.

The panel's saved bounds and speed scales live in [`joint_settings.json`](../joint_settings.json). The LeRobot calibration backup is [`config/care_reach_follower.json`](../config/care_reach_follower.json); the active Pi copy lives in the LeRobot calibration cache. If a joint is rebuilt or recalibrated, review both records and revalidate the pose path before replay.
