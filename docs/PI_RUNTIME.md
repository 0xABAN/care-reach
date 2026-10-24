# Headless Pi motor runtime

The tested robot host is a Raspberry Pi 5 with 8 GB RAM, Raspberry Pi OS, Python 3.13, and login `robot@pi.local`. The single care-reach Git checkout is `~/worktrees/care-reach/pi-runtime` on branch `lance/robot-pi`. The URT-2 appears at `/dev/ttyACM0`; use its stable `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078563-if00` path in service and motion commands.

## Pinned motor dependency

The tested Pi environment uses LeRobot 0.6.2 source at upstream commit `d3793b7e36522321c1c455f0dc4826db3793dceb`. The Pi currently has a 16 MB copy of that commit's `src/lerobot` package under `~/worktrees/care-reach/lerobot-runtime-src/lerobot`. Only `lerobot/utils/__init__.py` differs: the tracked [`config/lerobot_motor_only_utils.patch`](../config/lerobot_motor_only_utils.patch) removes eager PyTorch imports that the motor bus does not use. The original file is saved next to the Pi shim as `__init__.py.upstream`. This is a motor-only runtime; it is not a substitute for a full LeRobot install for training or VLA inference.

For a clean reconstruction, fetch the pinned upstream source, apply the tracked patch to that source, then copy its `src/lerobot` directory into `~/worktrees/care-reach/lerobot-runtime-src/lerobot`. `git apply --check config/lerobot_motor_only_utils.patch` passes against that upstream commit. Keep the original source and patch provenance; do not copy a macOS virtual environment to the Pi. The tested Pi virtual environment is `~/worktrees/care-reach/pi-runtime/.venv` with `draccus==0.11.6`, `numpy==2.3.5`, `pyserial==3.5`, `rustypot==1.13.1`, and `placo==0.9.25` plus their dependencies. PlaCo and the pinned SO-101 URDF are used by the optional [kinematics adapter](KINEMATICS.md); the dashboard motor runtime does not require them.

The active LeRobot calibration file is `~/.cache/huggingface/lerobot/calibration/robots/so_follower/care_reach_follower.json`. Restore it from [`config/care_reach_follower.json`](../config/care_reach_follower.json) only when the Pi copy is missing or intentionally being reset. The tracked [`joint_settings.json`](../joint_settings.json) and [`folded_pose.json`](../folded_pose.json) belong in the checkout root.

## Dashboard service

From the Pi checkout, `bash deploy/pi/install-dashboard.sh` installs a systemd user service and enables linger so it runs on a headless boot. The service uses the stable serial path and binds `0.0.0.0:8765`. Open `http://pi.local:8765` from the same local network, or use the Pi's IPv4 address. A direct cable needs a link-local address on the Pi; `bash deploy/pi/setup-direct-ethernet.sh` creates a persistent NetworkManager profile for `eth0` without changing Wi-Fi or Tailscale. This has been applied to the robot Pi. The server accepts `localhost`, `pi`, `pi.local`, or its receiving IP address as the Host, and requires any browser Origin to match. The panel has no login, so network access grants robot control. It requires the motor source, virtual environment, calibration, settings, and pose file above. `bash deploy/mac/dashboard-tunnel.sh` opens an SSH tunnel through the Pi's Tailscale name at `http://127.0.0.1:8765`; the script's `stop` argument closes it.

Before opening the servo bus from another program, stop the dashboard with `systemctl --user stop care-reach-dashboard.service` on the Pi. Start it afterward with `systemctl --user start care-reach-dashboard.service`. No two programs should open the URT-2 at once. The dashboard and motion runtime preserve torque when they stop; they do not provide an automatic torque-release command.

The public dataset and checkpoint described in `research/open_so101_motion/` are not deployed on this Pi. Model inference can run on the Mac mini via the Pi's Tailscale connection after cameras and a task policy are validated.

## Live handoff check (2026-10-24)

The Pi completed the full-scale six-joint dance in 1,182 command frames over 59.4 seconds. It ended near Home with torque enabled and no servo faults. The headless dashboard service is enabled and active. Through the Mac SSH tunnel, the panel and `/api/state` returned HTTP 200. A panel command moved motor 6 toward 1900 ticks (measured 1914), then toward 2000 (measured 1989). A held jog moved motor 5 from 2001 to 2068 ticks and back near 2022. All six motors still reported torque enabled and no faults after these checks.

The Pi can push to GitHub from this checkout, and `ssh mac` works from Pi shell accounts through Tailscale. Teammates need an authorized SSH key to enter the Pi; anyone with Pi shell access can use its shared Mac login and GitHub credential.

## Dashboard release check (2026-10-24)

The first live trace found the jog goal 44 ticks ahead of motor 5 when the button was released. The old controller replaced that goal with the measured position, and the joint then moved backward after coasting forward. The revised controller keeps a bounded goal in the jog direction. With the final 50-tick stopping allowance, live 8× forward and backward jogs on motor 5 settled within 2 ticks of their goals without the earlier rebound. Short 8× jogs on the loaded motor 2 also showed no post-release reversal. Torque stayed enabled and no servo faults appeared. Direct HTTP on `pi.local` and the Pi's Wi-Fi IPv4 address, plus the dashboard through Tailscale SSH, returned HTTP 200. The direct Ethernet address was verified while the cable was connected; HTTP over that cable was not retested after it was disconnected.
