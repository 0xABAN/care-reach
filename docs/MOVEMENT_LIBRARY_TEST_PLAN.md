# Movement library validation

Use this checklist after reviewing the `arm_control` changes and before putting them in the Pi's running checkout. The offline tests do not prove that the assembled arm will track a goal under load. Record the results and the controller commit used for each live run.

## 1. Prepare the arm

- Have one person at the arm and keep the workspace clear. Support the arm if a stop could let it shift or if its present pose is uncertain.
- Confirm that the current pose is known and that the recorded Home and Fold endpoints still match this assembled arm. Review `home_pose.json`, `folded_pose.json`, `joint_settings.json`, and the active LeRobot calibration before motion.
- Run one controller at a time. The dashboard owns the bus while it is running. Stop it before a direct `home.py`, `park.py`, or library call, and restart it afterward. A busy-lock error is a failed test; do not work around it by opening the serial port directly.
- Keep the prior Pi checkout commit available for rollback. Do not switch the running checkout while the dashboard is active.

## 2. Read-only baseline

With the dashboard running, open `/api/state` and `/api/diagnostics` at its Pi address. On the Pi, both reads can be made without moving the arm:

```bash
curl -fsS http://127.0.0.1:8765/api/state
curl -fsS http://127.0.0.1:8765/api/diagnostics
```

Save the returned state and note all six motor IDs, measured and goal ticks, torque state, operating mode, voltage, and status. Pass only if IDs 1 through 6 respond, positions are plausible for the visible pose, the motors expected to hold are torque-enabled, and there are no active faults or low-voltage readings. Stop here if a read fails or the reported pose disagrees with the arm in front of you.

## 3. First live movements

1. Set the dashboard jog scale to **1×**. Pick a joint with visible clearance and saved room in both directions. Hold Forward briefly, release, then repeat with Backward. Watch the joint until it has stopped. Pass if both moves stay within the displayed limits, release does not cause a reversal or continued jog, torque remains on, and no fault appears. Save the before, commanded-goal, and settled measured ticks.
2. From a clear, supported pose inside the validated envelope, run one reviewed **small six-joint target** through `MeasuredPoseRunner`. Choose each of the six targets 20 to 30 ticks from that joint's read-only measured position, inside both saved and firmware limits. Have the person at the arm approve every joint's direction before execution. Use a short goal lead and a 15-tick completion tolerance, so every target starts beyond the tolerance and each joint has to move for the trial to pass. The dashboard must be stopped for this direct call. There is no generic six-joint target button in the dashboard, so prepare and review the library caller and its exact six ticks before this step. Pass if each joint moves in its intended direction, the move completes within its configured tolerance and timeout, and all six still report torque on with no fault. Return to the starting pose before increasing travel.
3. With the path clear, use the dashboard's **Home** control, then **Fold**. These buttons use their built-in motion profile; the 1× jog setting does not slow them down. Check the measured endpoint against each control's completion tolerance and confirm the exact endpoint goal remains set. Stop the sequence at the first fault, out-of-bounds reading, unexpected direction, or visible contact.

Do not run the full Dance or the high-amplitude wrist shake until the smaller checks pass and the power and wrist travel remain healthy under load.

## 4. Stop and fault behavior

Use fake-bus tests to verify a lost jog heartbeat, a servo fault, low voltage, and a stalled joint. Each case must end the active command, retain torque, and produce an error record with the same motion token. During live trials, check the same behavior if a fault occurs naturally. Do not deliberately force a joint against a stop or lower the supply voltage to create a fault.

## 5. Timing and logs

Read `~/.local/state/care-reach/telemetry.jsonl` after each dashboard move, for example with `tail -n 100 ~/.local/state/care-reach/telemetry.jsonl`. Match `motion_started` to `motion_stopped` or `motion_completed` by `token`; an error must also carry that token. Confirm the UTC `time` values are ordered and the timing windows contain counts and nonnegative millisecond values. Check `first_command_ms`, `bus_read_ms`, `bus_write_ms`, `compute_ms`, `cycle_ms`, and `schedule_lag_ms`; a completed pose should include `settle_ms`, while a stopped move should include `motion_elapsed_ms` and `stop_command_ms`.

`bus_write_ms` measures how long the serial call takes, not when the servo physically starts or finishes moving. Compare command and measured tick samples to discuss response and tracking. Save the log excerpt with the commit and test result so later runs can be compared.

## If a step fails

Stop further motion. Record the last state, fault/status values, and matching telemetry token. Keep torque behavior under observation and support the arm as needed. To roll back code, stop the dashboard, restore the previously recorded Pi checkout commit using the deployment procedure, then restart the dashboard and repeat only the read-only baseline. Do not resume movement until the cause is understood.
