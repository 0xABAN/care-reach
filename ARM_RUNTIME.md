# Third Hand SO-101 motor runtime

This repository controls Lance's assembled six-motor SO-101 follower. The headless Raspberry Pi 5 (8 GB) is connected to the URT-2 board and owns the servo bus. A Mac mini is reachable from the Pi over Tailscale for later model inference.

## Connect and open the dashboard

```bash
ssh robot@pi.local
```

The `robot` account uses SSH keys only, with no password, and has passwordless `sudo`. The Pi also appears on Tailscale as `care-reach-robot-pi`. From any Pi shell, `ssh mac` reaches the Mac mini as `lancestreuber` through Tailscale SSH. Pi users share that Mac account, so give Pi shell access only to trusted teammates.

The dashboard runs from `~/worktrees/care-reach/arm-runtime-main-20261025`; the separate voice service runs from `~/worktrees/care-reach/voice-runtime`. The older `pi-runtime` checkout supplies the arm's Python virtual environment. Check `systemctl --user cat care-reach-dashboard.service care-reach-pi-voice.service` to confirm the live paths. Teammates should use [separate branches and worktrees on the Pi](docs/PI_BRANCHES.md) so they do not switch a live checkout. The URT-2's stable port is `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078563-if00`.

The dashboard runs headless on the Pi and binds `0.0.0.0:8765`. Install it with `bash deploy/pi/install-dashboard.sh` from the Pi checkout. Open [`http://pi.local:8765`](http://pi.local:8765) from the same local network. For a direct cable to a computer, run `bash deploy/pi/setup-direct-ethernet.sh` once on the Pi. Anyone who can reach the panel can control the arm; it does not ask for a login. On this Mac, `bash deploy/mac/dashboard-tunnel.sh` opens [the dashboard](http://127.0.0.1:8765) through Tailscale SSH without Ethernet; its `stop` argument closes the tunnel. The panel provides held Forward/Backward jog, 1×/2×/4×/8× rates, go-to-tick, live servo readings, and Home / Retract, Extend, and full-size Dance buttons. A faulted servo's jog is disabled while healthy joints remain available; group poses pause until every joint is healthy. Shared voltage alarms still stop motion. Saved soft bounds and speed scales persist in tracked [`joint_settings.json`](joint_settings.json); the panel shows the bounds read-only, so edit that file directly to change them.

Only **one process** may own the serial bus. Before a standalone dance or other CLI motion, stop the panel on the Pi with `systemctl --user stop care-reach-dashboard.service`; restart it afterward with `systemctl --user start care-reach-dashboard.service`. The panel's Home / Retract, Extend, and Dance buttons use that same bus owner. Home targets [`home_pose.json`](home_pose.json); Extend targets the physically confirmed upright ticks in [`extended_pose.json`](extended_pose.json). Home↔Extend uses synchronized motion with at most 55 ticks of commanded lead. Dance moves to Home, enters the ordinary-bounds choreography at `(2000,)*6`, plays the synchronized plan, and returns Home. Each move monitors bounds, torque, faults, voltage, load, progress, and timeout. A completed move keeps its exact pose as the holding goal; cancel or fault holds the measured position. Torque stays on. Press the active button again to cancel. The old Fold command is unavailable because the wrist camera blocks that pose.

The Pi records timestamped motor positions, goals, voltage, status, load, temperature, motion commands, fault transitions, and errors in `~/.local/state/care-reach/telemetry.jsonl` (20 MB × 6 rotating files). It samples about once per second at rest and up to ten times per second while moving; status and voltage-threshold changes are recorded immediately when observed. Current readings are added when a fault or low-voltage transition is observed. Use `tail -n 30 ~/.local/state/care-reach/telemetry.jsonl` on the Pi, or expand **Recent power and servo events** in the dashboard. A previously reported voltage alarm on an idle, unrelated servo does not block a healthy joint after voltage has recovered; a new alarm during motion or a confirmed voltage below 4.8 V still stops the move. The controller does not resume a stopped move automatically.

When a closing gripper jog ends normally or loses its heartbeat, the controller keeps a closing goal no more than 30 ticks inside its measured position, rather than replacing it with the measured position and letting the jaws rebound. A closing go-to-tick retains its target when it reaches it. A servo fault still ends the move with a measured-position hold, and the gripper should not be used to hold a critical load until its force and power delivery have been tested with that object.

## Movement library

The importable `arm_control/` package owns motor communication and movement. Its modules have separate jobs:

- `arm.py` runs dashboard jogs, go-to-tick, Home, Extend, Dance, and taught moves with the existing safety checks.
- `runtime.py` (`ArmRuntime`), `pose_runner.py` (`MeasuredPoseRunner`), `home.py`, and `shake.py` (`WristShake`) run the direct movement routines.
- `bus.py` builds the six STS3215 motors and manages exclusive bus ownership.

`server.py` now handles HTTP and the camera feed. It re-exports `Arm`, `find_port`, and tuning constants for existing callers. `motion_core.py` remains an offline planner with no hardware access.

Each real connection holds a nonblocking lock at `~/.local/state/care-reach/arm-bus.lock` until it disconnects. A standalone program gets a busy error if the dashboard already owns the bus. Fake buses used in tests skip the default lock; a caller that explicitly supplies a `BusLock` still uses it.

Timing uses `time.perf_counter_ns`, so changes to the wall clock do not affect durations. Dashboard logs still have UTC timestamps. Records for one motion share a `token`; stop, completion, and error records include timing measured so far. Long motions also log a `timing_sample` about every five seconds. Duration windows report `count`, `mean_ms`, and `max_ms`:

| Field | Meaning |
| --- | --- |
| `first_command_ms` | From motion start to submission of its first goal write. This does not measure when the motor starts moving. |
| `bus_read_ms` | Time spent completing serial register reads. |
| `bus_write_ms` | Time for a serial write call to return. Addressed writes may wait for an acknowledgement; sync writes do not. This is not physical actuator latency. |
| `compute_ms` | Python controller time per control cycle outside measured serial calls, including safety checks and telemetry. |
| `cycle_ms` | Total elapsed time spent executing a control cycle. |
| `schedule_lag_ms` | How late a cycle starts past its scheduled time. |
| `settle_ms` | Time from starting a pose move until encoder readings reach the target tolerance; present on completed poses. |
| `motion_elapsed_ms` | Time from start to stop for a stopped motion. |
| `stop_command_ms` | Time spent reading feedback and issuing the final holding goal during stop. |

Direct runners return bus and compute timing windows without persistent telemetry. The Home, Dance, and wrist trial commands print a short summary after a completed run.

`home.py` and `shake_it_crazy_style.py` are command-line wrappers around the library. `dance_demo.py` keeps its offline planner and calls a library runner for `--execute`; `park.py` only plans the historical Fold pose offline because the wrist camera blocks execution. Existing imports of `arm_runtime.py` and `measured_pose_runner.py` still work. Stop the dashboard before a standalone hardware run so only one program controls the arm.

The USB camera's [live feed](docs/CAMERA.md) appears in the dashboard's **Camera** tab at the same dashboard address. The camera service stays on Pi localhost; the dashboard relays the feed to anyone who can reach the panel.

## Pose names

| Name | Meaning |
| --- | --- |
| **Home / Retract** | `(2000, 1056, 3065, 2027, 387, 1993)` raw ticks on motors 1–6, saved in [`home_pose.json`](home_pose.json). |
| **Extend** | `(2000, 1903, 1126, 1988, 387, 1993)` raw ticks, upright with the wrist camera attached, saved in [`extended_pose.json`](extended_pose.json). |
| **Folded** | Historical `(2040, 893, 3064, 2888, 2416, 1470)` ticks in [`folded_pose.json`](folded_pose.json). The camera obstructs this pose; dashboard and voice commands reject it, and `park.py --execute` refuses. |
| **Parked / Coiled** | Historical compact-rest terminology. No camera-safe torque-release target is validated. |

See [pose definitions and limits](docs/POSES.md). The camera-mounted controller has no automatic torque-release procedure.

The Pi voice tester acts after the exact wake phrase **“Hey Arm”** followed by a command in the same utterance. The wake phrase can appear after other speech in Cartesia's final transcript. The Pi sends only the words after it to Jev (`typesafe/jev-1.14`) through OpenRouter; it never sends pre-wake speech for a command decision. Transcription starts paused on every service launch; enable it with the dashboard toggle for that session. Once listening is on, no separate motion enable is required. “Retract” and “go home” select Home; “extend” selects the upright pose; “dance” selects the saved choreography. “Open the hand” and “close the hand” use the bounded gripper move. The retired Fold pose is not a voice command.

## Motion

`motion_core.py` plans bounded synchronized trajectories. `arm_control/runtime.py` (still importable as `arm_runtime`) checks measured position, torque, mode, fault status, voltage, load, software limits, and firmware limits, and leaves torque enabled on exit. `arm_control/pose_runner.py` (still importable as `measured_pose_runner`) moves from an in-bounds measured pose with bounded position lead. Motor order is 1 shoulder pan, 2 shoulder lift, 3 elbow flex, 4 wrist flex, 5 wrist roll, 6 gripper.

[`so101_ik.py`](so101_ik.py) translates this arm's calibrated encoder ticks to the official SO-101 model's joint degrees and provides guarded local FK/IK. See [kinematics setup and live validation](docs/KINEMATICS.md). Cartesian targets are still experimental; the dashboard's Home and Extend use saved tick poses and a guarded synchronized path.

The original full-scale free-space dance ran on the Pi on 2026-10-24: 1,182 frames over 59.4 seconds, ending at `(1994, 2015, 2003, 2009, 2001, 2006)`, near the former choreography center. The transition to the newly recorded Home has only been checked offline. To preview its path without opening the bus:

```bash
cd ~/worktrees/care-reach/pi-runtime
PYTHONPATH=~/worktrees/care-reach/lerobot-runtime-src .venv/bin/python dance_demo.py --dry-run --scale 1
```

After stopping the dashboard and verifying a clear, fault-free arm near Home, run:

```bash
PYTHONPATH=~/worktrees/care-reach/lerobot-runtime-src .venv/bin/python dance_demo.py --execute --scale 1 \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078563-if00
```

The saved soft bounds intersect [`joint_settings.json`](joint_settings.json), LeRobot calibration, and provisional stops of wrist roll at 3300 ticks and gripper close at 1340 ticks. Motor 3's upper bound is 3065 ticks, the recorded Home position. `park.py` retains the old Fold plan for offline inspection, but `--execute` refuses before opening the bus. Wrist roll resisted movement near 3358 ticks, so its full-turn sweep is unverified.

The active calibration lives in `~/.cache/huggingface/lerobot/calibration/robots/so_follower/care_reach_follower.json` on the Pi. A versioned recovery copy is [`config/care_reach_follower.json`](config/care_reach_follower.json). The Pi uses a motor-only LeRobot 0.6.2 source pinned to commit `d3793b7e36522321c1c455f0dc4826db3793dceb` plus our tracked [import patch](config/lerobot_motor_only_utils.patch); see [runtime setup](docs/PI_RUNTIME.md).

## Models and tests

The downloaded public SO-101 dataset and ACT checkpoint are research inputs, not validated live policies here. Converted source actions exceed this arm's saved limits in 18.47% of frames, and all 128 source episodes contain an out-of-limit action. The checkpoint also requires calibrated top and wrist cameras. See [findings](research/open_so101_motion/FINDINGS.md) and [local feasibility](research/open_so101_motion/LOCAL_FEASIBILITY.md). Large model and dataset files stay outside Git.

From the Pi checkout, run the offline tests with:

```bash
PYTHONPATH=~/worktrees/care-reach/lerobot-runtime-src .venv/bin/python -m unittest discover -s tests -q
```
