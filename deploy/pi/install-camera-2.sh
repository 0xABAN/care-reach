#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
unit_source="$repo/deploy/pi/care-reach-camera-2.service"
unit_target="$HOME/.config/systemd/user/care-reach-camera-2.service"
camera=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._w200_SN0001-video-index0

command -v ustreamer >/dev/null || { echo 'Install ustreamer first: sudo apt-get install ustreamer' >&2; exit 1; }
mkdir -p "$(dirname "$unit_target")"
cp "$unit_source" "$unit_target"
systemctl --user daemon-reload
systemctl --user enable care-reach-camera-2.service
if [[ ! -e $camera ]]; then
  echo "Second camera is absent ($camera); the enabled service will retry after it appears." >&2
fi
if ! systemctl --user start care-reach-camera-2.service; then
  echo 'Second camera service will retry when the camera is available; inspect with systemctl --user status care-reach-camera-2.service.' >&2
fi
echo 'Second camera feed: http://127.0.0.1:8768 (Pi localhost)'
