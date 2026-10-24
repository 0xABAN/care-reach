# SO-101 pose terminology

All tuples below are raw STS3215 encoder ticks in motor order: shoulder pan (1), shoulder lift (2), elbow flex (3), wrist flex (4), wrist roll (5), gripper (6). They belong to this assembled arm and its `care_reach_follower` calibration; they are not transferable to another arm without measurement.

| Term | Raw ticks | Meaning |
| --- | --- | --- |
| **Home** | `(2000, 1056, 3065, 2027, 387, 1993)` | Lance physically set this pose and recorded raw ticks in [`home_pose.json`](../home_pose.json). Torque was enabled at capture. |
| **Folded** | `(2040, 893, 3064, 2888, 2416, 1470)` | Lance physically folded the torque-off arm on 2026-10-24; two encoder reads agreed. Stored in [`folded_pose.json`](../folded_pose.json). |
| **Parked** | Folded endpoint | An active `park.py --execute` move from Home to the recorded Folded pose. The command keeps torque enabled, including when it exits. |
| **Coiled** | No separate validated tuple | Informal term for a compact resting pose and the intent to protect the arm before eventual torque release. Treat Folded as the measured candidate, not as proof that torque release is safe in every setup. |

## Motion rules

- Check the current measured pose, clear workspace, torque, voltage, fault flags, and saved limits before motion. A pose file records an endpoint, not the current arm position.
- Only one process opens the URT-2 serial port. Stop the Pi dashboard service before `home.py`, `park.py`, `dance_demo.py`, or another controller; start it again afterward.
- The dashboard's Home and Fold buttons move all six joints together through the panel's bus owner. Home targets the saved raw ticks; Fold targets the recorded pose, using its narrow motor 2 exception. Both start from the measured position inside the validated envelope, including Fold to Home. Before and during motion, the controller checks bounds, firmware limits, torque, status, voltage, and progress. It keeps each commanded goal within 80 ticks of its encoder reading. On success it holds the exact target; cancel, fault, or service restart holds the measured position. Torque stays on.
- The dashboard's Dance button uses the same bus owner and full-size `dance_demo.py` plan. It moves to recorded Home, then to the ordinary-bounds choreography center `(2000,)*6`, plays each synchronized frame at up to 20 Hz, and returns to recorded Home. The button shows progress and cancels the active dance when pressed again. Competing jog and pose commands stay disabled; cancellation or a fault holds the measured pose with torque on. The revised path has not had a live dance test.
- Live dashboard checks on 2026-10-24: one complete full-size run finished near the former `(2000,)*6` choreography center with no faults. Two other runs stopped on a transient motor 6 status `1`, which the [ST-series memory table](https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation) identifies as a voltage alarm. Later readings returned to 5.1–5.4 V and status 0, so the brief low point was not captured. The software keeps the immediate fault stop. Inspect the 5 V/4 A arm supply and the daisy-chain power connections before relying on repeated dances; [six 7.4 V servos](https://docs.waveshare.com/ST3215_Servo) can draw more than that supply during simultaneous acceleration. The dashboard now labels this status as a voltage alarm.
- `park.py` permits the small measured exception required for Folded: motor 2 is 10 ticks below its saved panel lower bound. Motor 3's ordinary upper bound is 3065, exactly the recorded Home tick. Trim commands never exceed it.
- None of these scripts automatically disables torque. Before any future torque-release function is added, validate a pose that remains mechanically supported after the wheelchair mount, then test its hold and release behavior under that mounting.
- The jaw's measured contact was 1317 ticks, but ordinary position-mode motion uses a provisional 1340-tick lower stop. Motor 5 encountered resistance near 3358 ticks, so ordinary motion stops at or below 3300.

The panel's saved bounds and speed scales live in [`joint_settings.json`](../joint_settings.json). The LeRobot calibration backup is [`config/care_reach_follower.json`](../config/care_reach_follower.json); the active Pi copy lives in the LeRobot calibration cache. If a joint is rebuilt or recalibrated, review both records and revalidate the pose path before replay.
