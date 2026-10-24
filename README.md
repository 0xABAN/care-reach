# Care Reach SO-101 follower

This repository controls Lance's assembled six-motor SO-101 follower. The headless Raspberry Pi 5 (8 GB) is connected to the URT-2 board and owns the servo bus. A Mac mini is reachable from the Pi over Tailscale for later model inference.

## Connect and open the dashboard

```bash
ssh robot@pi.local
```

The `robot` account uses SSH keys only, with no password, and has passwordless `sudo`. The Pi also appears on Tailscale as `care-reach-robot-pi`. From any Pi shell, `ssh mac` reaches the Mac mini as `lancestreuber` through Tailscale SSH. Pi users share that Mac account, so give Pi shell access only to trusted teammates.

The running dashboard uses the Pi checkout at `~/worktrees/care-reach/pi-runtime` on **`lance/robot-pi`**. Teammates should use [separate branches and worktrees on the Pi](docs/PI_BRANCHES.md) so they do not switch the live checkout. The URT-2's stable port is `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078563-if00`.

The dashboard runs headless on the Pi and binds `0.0.0.0:8765`. Install it with `bash deploy/pi/install-dashboard.sh` from the Pi checkout. Open [`http://pi.local:8765`](http://pi.local:8765) from the same local network. For a direct cable to a computer, run `bash deploy/pi/setup-direct-ethernet.sh` once on the Pi. Anyone who can reach the panel can control the arm; it does not ask for a login. On this Mac, `bash deploy/mac/dashboard-tunnel.sh` opens [the dashboard](http://127.0.0.1:8765) through Tailscale SSH without Ethernet; its `stop` argument closes the tunnel. The panel provides held Forward/Backward jog, 1×/2×/4×/8× rates, go-to-tick, live servo readings, and Home, Fold, and full-size Dance buttons. Saved soft bounds and speed scales persist in tracked [`joint_settings.json`](joint_settings.json); the panel shows the bounds read-only, so edit that file directly to change them.

Only **one process** may own the serial bus. Before a standalone dance, park, or other CLI motion, stop the panel on the Pi with `systemctl --user stop care-reach-dashboard.service`; restart it afterward with `systemctl --user start care-reach-dashboard.service`. The panel's Home, Fold, and Dance buttons use that same bus owner. Home targets `(2000,)*6` ticks and Fold targets the recorded folded pose with its narrow recorded exceptions for motors 2 and 3. Dance moves to Home first, plays the full-scale synchronized plan, and settles at Home. Each move monitors bounds, torque, faults, voltage, load, progress, and timeout, with commanded lead at or below 80 ticks per joint. A completed move keeps its exact pose as the holding goal; cancel or fault holds the measured position. Torque stays on. Press the active button again to cancel.

## Pose names

| Name | Meaning |
| --- | --- |
| **Home** | `(2000, 2000, 2000, 2000, 2000, 2000)` raw ticks on motors 1–6: the powered test and dance start/end pose. |
| **Folded** | `(2040, 893, 3064, 2888, 2416, 1470)` raw ticks, manually set by Lance and recorded twice in [`folded_pose.json`](folded_pose.json). |
| **Parked** | A commanded move to Folded using `park.py`; torque stays on afterward. |
| **Coiled** | Informal name for a resting/folded arm. No distinct numeric Coiled target has been validated. |

See [pose definitions and limits](docs/POSES.md). Folded was captured with torque off, but the current controller has no automatic torque-release procedure. A wheelchair mount may change whether a pose can safely rest without torque.

## Motion

`motion_core.py` plans bounded synchronized trajectories. `arm_runtime.py` checks measured position, torque, mode, fault status, voltage, load, software limits, and firmware limits, and leaves torque enabled on exit. `measured_pose_runner.py` moves from an in-bounds measured pose with bounded position lead. Motor order is 1 shoulder pan, 2 shoulder lift, 3 elbow flex, 4 wrist flex, 5 wrist roll, 6 gripper.

[`so101_ik.py`](so101_ik.py) translates this arm's calibrated encoder ticks to the official SO-101 model's joint degrees and provides guarded local FK/IK. See [kinematics setup and live validation](docs/KINEMATICS.md). Cartesian targets are still experimental; the dashboard's Home and Fold use validated tick poses.

The full-scale free-space dance ran on the Pi on 2026-10-24: 1,182 frames over 59.4 seconds, ending at `(1994, 2015, 2003, 2009, 2001, 2006)`. To preview its path without opening the bus:

```bash
cd ~/worktrees/care-reach/pi-runtime
PYTHONPATH=~/worktrees/care-reach/lerobot-runtime-src .venv/bin/python dance_demo.py --dry-run --scale 1
```

After stopping the dashboard and verifying a clear, fault-free arm near Home, run:

```bash
PYTHONPATH=~/worktrees/care-reach/lerobot-runtime-src .venv/bin/python dance_demo.py --execute --scale 1 \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078563-if00
```

The saved soft bounds intersect [`joint_settings.json`](joint_settings.json), LeRobot calibration, and provisional stops of wrist roll at 3300 ticks and gripper close at 1340 ticks. `park.py` permits only the small measured extension needed for Folded; it does not widen ordinary jog limits. Wrist roll resisted movement near 3358 ticks, so its full-turn sweep is unverified.

The active calibration lives in `~/.cache/huggingface/lerobot/calibration/robots/so_follower/care_reach_follower.json` on the Pi. A versioned recovery copy is [`config/care_reach_follower.json`](config/care_reach_follower.json). The Pi uses a motor-only LeRobot 0.6.2 source pinned to commit `d3793b7e36522321c1c455f0dc4826db3793dceb` plus our tracked [import patch](config/lerobot_motor_only_utils.patch); see [runtime setup](docs/PI_RUNTIME.md).

## Models and tests

The downloaded public SO-101 dataset and ACT checkpoint are research inputs, not validated live policies here. Converted source actions exceed this arm's saved limits in 18.47% of frames, and all 128 source episodes contain an out-of-limit action. The checkpoint also requires calibrated top and wrist cameras. See [findings](research/open_so101_motion/FINDINGS.md) and [local feasibility](research/open_so101_motion/LOCAL_FEASIBILITY.md). Large model and dataset files stay outside Git.

From the Pi checkout, run the offline tests with:

```bash
PYTHONPATH=~/worktrees/care-reach/lerobot-runtime-src .venv/bin/python -m unittest discover -s tests -q
```
