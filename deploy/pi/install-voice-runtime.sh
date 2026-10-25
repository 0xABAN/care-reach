#!/usr/bin/env bash
set -euo pipefail

if [[ $(id -un) != robot ]]; then
  echo 'Run this installer as robot on the Pi.' >&2
  exit 1
fi

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ $repo != /home/robot/worktrees/care-reach/voice-runtime ]]; then
  echo "Expected the dedicated voice-runtime worktree, got $repo" >&2
  exit 1
fi

unit_dir=$HOME/.config/systemd/user
voice_env=$HOME/.config/care-reach/voice.env
backup=$HOME/.local/state/care-reach/unit-backups/$(date -u +%Y%m%dT%H%M%SZ)-$$
[[ -r $voice_env && -f $repo/voice/pi_voice.py ]] || {
  echo 'Voice configuration or source is missing.' >&2
  exit 1
}
if [[ ! -x $repo/.venv/bin/python ]]; then
  python3 -m venv "$repo/.venv"
  "$repo/.venv/bin/python" -m pip install -r "$repo/voice/requirements.txt"
fi
"$repo/.venv/bin/python" -c 'import cartesia, websockets'

was_listening=false
if state=$(curl -fsS --max-time 2 http://127.0.0.1:8770/state 2>/dev/null); then
  was_listening=$(python3 -c 'import json,sys; print("true" if json.load(sys.stdin).get("listening") else "false")' <<< "$state")
fi

mkdir -p "$unit_dir" "$backup"
chmod 700 "$backup"
for name in care-reach-pi-voice.service care-reach-voice-capture-watchdog.service care-reach-voice-capture-watchdog.timer; do
  if [[ -e $unit_dir/$name ]]; then cp -p "$unit_dir/$name" "$backup/$name"; fi
  install -m 0644 "$repo/deploy/pi/$name" "$unit_dir/$name"
done
if [[ -d $unit_dir/care-reach-pi-voice.service.d ]]; then
  mv "$unit_dir/care-reach-pi-voice.service.d" "$backup/old-voice-dropins"
fi

systemctl --user daemon-reload
systemctl --user enable care-reach-pi-voice.service care-reach-voice-capture-watchdog.timer
systemctl --user restart care-reach-pi-voice.service
for _ in {1..20}; do
  if curl -fsS --max-time 1 http://127.0.0.1:8770/state >/dev/null 2>&1; then break; fi
  sleep 0.5
done
curl -fsS --max-time 3 http://127.0.0.1:8770/state >/dev/null
if [[ $was_listening == true ]]; then
  curl -fsS --max-time 5 -H 'Content-Type: application/json' \
    -d '{"enabled":true}' http://127.0.0.1:8770/listen >/dev/null
fi
systemctl --user restart care-reach-voice-capture-watchdog.timer
systemctl --user start care-reach-voice-capture-watchdog.service
printf 'Voice source: %s\nPrevious listening state restored: %s\nUnit backup: %s\n' "$repo" "$was_listening" "$backup"
