# Care Reach SO-101 follower

This repository controls Lance's assembled six-motor SO-101 follower. The headless Raspberry Pi 5 (8 GB) is connected to the URT-2 board and owns the servo bus. A Mac mini is reachable from the Pi over Tailscale for later model inference.

## Connect and open the dashboard

```bash
ssh robot@pi.local
```

The `robot` account uses SSH keys only, with no password, and has passwordless `sudo`. The Pi also appears on Tailscale as `care-reach-robot-pi`. From any Pi shell, `ssh mac` reaches the Mac mini as `lancestreuber` through Tailscale SSH. Pi users share that Mac account, so give Pi shell access only to trusted teammates.

The running dashboard uses the Pi checkout at `~/worktrees/care-reach/pi-runtime` on **`lance/robot-pi`**. Teammates should use [separate branches and worktrees on the Pi](docs/PI_BRANCHES.md) so they do not switch the live checkout. The URT-2's stable port is `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078563-if00`.

The dashboard runs headless on the Pi and binds `0.0.0.0:8765`. Install it with `bash deploy/pi/install-dashboard.sh` from the Pi checkout. Open [`http://pi.local:8765`](http://pi.local:8765) from the same local network. For a direct cable to a computer, run `bash deploy/pi/setup-direct-ethernet.sh` once on the Pi. Anyone who can reach the panel can control the arm; it does not ask for a login. On this Mac, `bash deploy/mac/dashboard-tunnel.sh` opens [the dashboard](http://127.0.0.1:8765) through Tailscale SSH without Ethernet; its `stop` argument closes the tunnel. The panel provides held Forward/Backward jog, 1×/2×/4×/8× rates, go-to-tick, live servo readings, and Home, Fold, and full-size Dance buttons. A faulted servo's jog is disabled while healthy joints remain available; group poses pause until every joint is healthy. Shared voltage alarms still stop motion. Saved soft bounds and speed scales persist in tracked [`joint_settings.json`](joint_settings.json); the panel shows the bounds read-only, so edit that file directly to change them.

Only **one process** may own the serial bus. Before a standalone dance, park, or other CLI motion, stop the panel on the Pi with `systemctl --user stop care-reach-dashboard.service`; restart it afterward with `systemctl --user start care-reach-dashboard.service`. The panel's Home, Fold, and Dance buttons use that same bus owner. Home targets the recorded ticks in [`home_pose.json`](home_pose.json), and Fold targets the recorded folded pose with its narrow motor 2 exception. Dance moves to recorded Home, enters the ordinary-bounds choreography at `(2000,)*6`, plays the synchronized plan, and returns to recorded Home. Each move monitors bounds, torque, faults, voltage, load, progress, and timeout, with commanded lead at or below 80 ticks per joint. A completed move keeps its exact pose as the holding goal; cancel or fault holds the measured position. Torque stays on. Press the active button again to cancel.

The Pi records timestamped motor positions, goals, voltage, status, load, temperature, motion commands, fault transitions, and errors in `~/.local/state/care-reach/telemetry.jsonl` (20 MB × 6 rotating files). It samples about once per second at rest and up to ten times per second while moving; status and voltage-threshold changes are recorded immediately when observed. Current readings are added when a fault or low-voltage transition is observed. Use `tail -n 30 ~/.local/state/care-reach/telemetry.jsonl` on the Pi, or expand **Recent power and servo events** in the dashboard. A previously reported voltage alarm on an idle, unrelated servo does not block a healthy joint after voltage has recovered; a new alarm during motion or a confirmed voltage below 4.8 V still stops the move. The controller does not resume a stopped move automatically.

When a closing gripper jog ends normally or loses its heartbeat, the controller keeps a closing goal no more than 30 ticks inside its measured position, rather than replacing it with the measured position and letting the jaws rebound. A closing go-to-tick retains its target when it reaches it. A servo fault still ends the move with a measured-position hold, and the gripper should not be used to hold a critical load until its force and power delivery have been tested with that object.

The USB camera's [live feed](docs/CAMERA.md) appears in the dashboard's **Camera** tab at the same dashboard address. The camera service stays on Pi localhost; the dashboard relays the feed to anyone who can reach the panel.

## Pose names

| Name | Meaning |
| --- | --- |
| **Home** | `(2000, 1056, 3065, 2027, 387, 1993)` raw ticks on motors 1–6, saved in [`home_pose.json`](home_pose.json). |
| **Folded** | `(2040, 893, 3064, 2888, 2416, 1470)` raw ticks, manually set by Lance and recorded twice in [`folded_pose.json`](folded_pose.json). |
| **Parked** | A commanded move to Folded using `park.py`; torque stays on afterward. |
| **Coiled** | Informal name for a resting/folded arm. No distinct numeric Coiled target has been validated. |

See [pose definitions and limits](docs/POSES.md). Folded was captured with torque off, but the current controller has no automatic torque-release procedure. A wheelchair mount may change whether a pose can safely rest without torque.

## Motion

`motion_core.py` plans bounded synchronized trajectories. `arm_runtime.py` checks measured position, torque, mode, fault status, voltage, load, software limits, and firmware limits, and leaves torque enabled on exit. `measured_pose_runner.py` moves from an in-bounds measured pose with bounded position lead. Motor order is 1 shoulder pan, 2 shoulder lift, 3 elbow flex, 4 wrist flex, 5 wrist roll, 6 gripper.

[`so101_ik.py`](so101_ik.py) translates this arm's calibrated encoder ticks to the official SO-101 model's joint degrees and provides guarded local FK/IK. See [kinematics setup and live validation](docs/KINEMATICS.md). Cartesian targets are still experimental; the dashboard's Home and Fold use validated tick poses.

`pick_up.py "blue box" --execute` picks up a named object with the wrist camera: Grounding DINO on the GPU pod finds it, the arm nudges until the jaws are over it, then lowers, grips until contact, and lifts it back to the survey height with the dashboard's 30-tick squeeze. Without `--execute` it only looks. The [`pick`](pick) package holds the camera, detector, centering, and grasp code; run `pick_setup.py` once first. See [picking up objects](docs/PICK.md).

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

The saved soft bounds intersect [`joint_settings.json`](joint_settings.json), LeRobot calibration, and provisional stops of wrist roll at 3300 ticks and gripper close at 1340 ticks. Motor 3's upper bound is 3065 ticks, the recorded Home position. `park.py` permits only the small measured extension needed for motor 2 at Folded; it does not widen ordinary jog limits. Wrist roll resisted movement near 3358 ticks, so its full-turn sweep is unverified.

The active calibration lives in `~/.cache/huggingface/lerobot/calibration/robots/so_follower/care_reach_follower.json` on the Pi. A versioned recovery copy is [`config/care_reach_follower.json`](config/care_reach_follower.json). The Pi uses a motor-only LeRobot 0.6.2 source pinned to commit `d3793b7e36522321c1c455f0dc4826db3793dceb` plus our tracked [import patch](config/lerobot_motor_only_utils.patch); see [runtime setup](docs/PI_RUNTIME.md).

## Models and tests

The downloaded public SO-101 dataset and ACT checkpoint are research inputs, not validated live policies here. Converted source actions exceed this arm's saved limits in 18.47% of frames, and all 128 source episodes contain an out-of-limit action. The checkpoint also requires calibrated top and wrist cameras. See [findings](research/open_so101_motion/FINDINGS.md) and [local feasibility](research/open_so101_motion/LOCAL_FEASIBILITY.md). Large model and dataset files stay outside Git.

From the Pi checkout, run the offline tests with:

```bash
PYTHONPATH=~/worktrees/care-reach/lerobot-runtime-src .venv/bin/python -m unittest discover -s tests -q
```
