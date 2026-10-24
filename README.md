# Care Reach SO-101 follower

This repository controls Lance's assembled six-motor SO-101 follower. The headless Raspberry Pi 5 (8 GB) is connected to the URT-2 board and owns the servo bus. A Mac mini is reachable from the Pi over Tailscale for later model inference.

## Connect and open the dashboard

```bash
ssh robot@pi.local
```

The `robot` account uses SSH keys only, with no password, and has passwordless `sudo`. The Pi also appears on Tailscale as `care-reach-robot-pi`. From any Pi shell, `ssh mac` reaches the Mac mini as `lancestreuber` through Tailscale SSH. Pi users share that Mac account, so give Pi shell access only to trusted teammates.

The Pi has one Git working checkout at `~/worktrees/care-reach/pi-runtime`, on the dedicated **`lance/robot-pi`** branch. Commit and push this arm's future changes only on that branch. The URT-2's stable port is `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078563-if00`.

The dashboard runs on the Pi and listens only on its loopback address. Install and start its headless user service with `bash deploy/pi/install-dashboard.sh` from the Pi checkout. On this Mac, run `bash deploy/mac/dashboard-tunnel.sh` to open [the dashboard](http://127.0.0.1:8765); `bash deploy/mac/dashboard-tunnel.sh stop` closes that tunnel. An authorized teammate can also make an SSH port forward to the Pi's `127.0.0.1:8765`. The dashboard provides held Forward/Backward jog, 1×/2×/4×/8× rates, go-to-tick, live servo readings, and Save lower/Save upper controls. Limits and scales persist in tracked [`config/joint_settings.json`](config/joint_settings.json).

Only **one process** may own the serial bus. Before a standalone dance, park, or other CLI motion, stop the panel on the Pi with `systemctl --user stop care-reach-dashboard.service`; restart it afterward with `systemctl --user start care-reach-dashboard.service`. Closing a jog or panel does not release torque.

## Pose names

| Name | Meaning |
| --- | --- |
| **Home** | `(2000, 2000, 2000, 2000, 2000, 2000)` raw ticks on motors 1–6: the powered test and dance start/end pose. |
| **Folded** | `(2040, 893, 3064, 2888, 2416, 1470)` raw ticks, manually set by Lance and recorded twice in [`config/folded_pose.json`](config/folded_pose.json). |
| **Parked** | A commanded move to Folded using `care_reach.routines.park`; torque stays on afterward. |
| **Coiled** | Informal name for a resting/folded arm. No distinct numeric Coiled target has been validated. |

See [pose definitions and limits](docs/POSES.md). Folded was captured with torque off, but the current controller has no automatic torque-release procedure. A wheelchair mount may change whether a pose can safely rest without torque.

## Code layout

The `care_reach` package is organized by responsibility, from pure planning outward to the programs that own the servo bus:

| Module | Responsibility |
| --- | --- |
| `care_reach/robot.py` | Facts about this arm: joint order, robot id, and where its record files live. |
| `care_reach/motion/` | Safety bounds (`bounds.py`) and blended trajectory planning (`trajectory.py`). Never touches hardware. |
| `care_reach/policy.py` | Converts and splices model (VLA) actions into validated tick poses. Not yet wired to a live policy. |
| `care_reach/hardware/` | Guarded controllers that open the bus: trajectory playback from Home (`arm_runtime.py`) and a measured-feedback pose move (`measured_pose_runner.py`). |
| `care_reach/routines/` | Command-line motions: `home`, `park`, and `dance`. Each plans offline and opens the bus only with `--execute` (home always moves). |
| `care_reach/dashboard/` | The jog panel: bus-owning controller (`panel.py`) and loopback HTTP server (`server.py`). |
| `config/` | This arm's joint settings, folded pose, calibration backup, and LeRobot patch. |

Run every command from the checkout root with `python -m`, so Python finds the package.

## Motion

The planner builds bounded, synchronized trajectories. Playback checks measured position, torque, mode, fault status, voltage, load, software limits, and firmware limits, and leaves torque enabled on exit. The measured pose runner moves from an in-bounds measured pose with bounded position lead. Motor order is 1 shoulder pan, 2 shoulder lift, 3 elbow flex, 4 wrist flex, 5 wrist roll, 6 gripper.

The full-scale free-space dance ran on the Pi on 2026-10-24: 1,182 frames over 59.4 seconds, ending at `(1994, 2015, 2003, 2009, 2001, 2006)`. To preview its path without opening the bus:

```bash
cd ~/worktrees/care-reach/pi-runtime
PYTHONPATH=~/worktrees/care-reach/lerobot-runtime-src .venv/bin/python -m care_reach.routines.dance --dry-run --scale 1
```

After stopping the dashboard and verifying a clear, fault-free arm near Home, run:

```bash
PYTHONPATH=~/worktrees/care-reach/lerobot-runtime-src .venv/bin/python -m care_reach.routines.dance --execute --scale 1 \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078563-if00
```

The saved soft bounds intersect [`config/joint_settings.json`](config/joint_settings.json), LeRobot calibration, and provisional stops of wrist roll at 3300 ticks and gripper close at 1340 ticks. The park routine permits only the small measured extension needed for Folded; it does not widen ordinary jog limits. Wrist roll resisted movement near 3358 ticks, so its full-turn sweep is unverified.

The active calibration lives in `~/.cache/huggingface/lerobot/calibration/robots/so_follower/care_reach_follower.json` on the Pi. A versioned recovery copy is [`config/care_reach_follower.json`](config/care_reach_follower.json). The Pi uses a motor-only LeRobot 0.6.2 source pinned to commit `d3793b7e36522321c1c455f0dc4826db3793dceb` plus our tracked [import patch](config/lerobot_motor_only_utils.patch); see [runtime setup](docs/PI_RUNTIME.md).

## Models and tests

The downloaded public SO-101 dataset and ACT checkpoint are research inputs, not validated live policies here. Converted source actions exceed this arm's saved limits in 18.47% of frames, and all 128 source episodes contain an out-of-limit action. The checkpoint also requires calibrated top and wrist cameras. See [findings](research/open_so101_motion/FINDINGS.md) and [local feasibility](research/open_so101_motion/LOCAL_FEASIBILITY.md). Large model and dataset files stay outside Git.

From the Pi checkout, run the offline tests with:

```bash
PYTHONPATH=~/worktrees/care-reach/lerobot-runtime-src .venv/bin/python -m unittest discover -s tests -t . -q
```
