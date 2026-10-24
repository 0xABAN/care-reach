#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
unit_source="$repo/deploy/pi/care-reach-camera.service"
unit_target="$HOME/.config/systemd/user/care-reach-camera.service"
camera=/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0

command -v ustreamer >/dev/null || { echo 'Install ustreamer first: sudo apt-get install ustreamer' >&2; exit 1; }
[[ -e $camera ]] || { echo "Camera not found: $camera" >&2; exit 1; }
mkdir -p "$(dirname "$unit_target")"
cp "$unit_source" "$unit_target"
systemctl --user daemon-reload
systemctl --user enable --now care-reach-camera.service
echo 'Camera feed running on Pi localhost: http://127.0.0.1:8766'
