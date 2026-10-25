#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
unit_source="$repo/deploy/pi/care-reach-ultrasonic.service"
unit_target="$HOME/.config/systemd/user/care-reach-ultrasonic.service"

/usr/bin/python3 -c 'import serial' || {
  echo 'Install pyserial first: sudo apt-get install python3-serial' >&2
  exit 1
}
mkdir -p "$(dirname "$unit_target")"
sed "s|@REPO@|$repo|g" "$unit_source" > "$unit_target"
systemctl --user daemon-reload
systemctl --user enable --now care-reach-ultrasonic.service
echo 'Ultrasonic state: http://127.0.0.1:8767/state (Pi localhost)'
