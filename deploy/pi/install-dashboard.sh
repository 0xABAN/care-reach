#!/usr/bin/env bash
set -euo pipefail

if [[ $(id -un) != robot ]]; then
  echo 'Run this installer as the robot user on the robot Pi.' >&2
  exit 1
fi

repo=/home/robot/worktrees/care-reach/pi-runtime
source_dir=/home/robot/worktrees/care-reach/lerobot-runtime-src
port=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078563-if00
calibration=/home/robot/.cache/huggingface/lerobot/calibration/robots/so_follower/care_reach_follower.json
unit_name=care-reach-dashboard.service
unit_source="$repo/deploy/pi/$unit_name"
unit_destination="$HOME/.config/systemd/user/$unit_name"

for path in "$repo/care_reach/dashboard/server.py" "$repo/config/joint_settings.json" "$repo/config/folded_pose.json" "$repo/.venv/bin/python" "$calibration" "$unit_source" "$port"; do
  if [[ ! -e $path ]]; then
    echo "Required dashboard dependency missing: $path" >&2
    exit 1
  fi
done
if [[ ! -d $source_dir/lerobot/motors && ! -d $source_dir/src/lerobot/motors ]]; then
  echo "LeRobot motor source missing under $source_dir" >&2
  exit 1
fi

mkdir -p "$HOME/.config/systemd/user"
install -m 0644 "$unit_source" "$unit_destination"
sudo -n loginctl enable-linger robot
systemctl --user daemon-reload
systemctl --user enable --now "$unit_name"
systemctl --user --no-pager --full status "$unit_name"
echo 'Dashboard runs on the Pi at http://127.0.0.1:8765.'
echo 'Stop the service before running another program that opens the URT-2:'
echo "  systemctl --user stop $unit_name"
