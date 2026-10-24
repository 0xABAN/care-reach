#!/usr/bin/env bash
set -euo pipefail

local_port=8766
socket="$HOME/.ssh/care-reach-camera-tunnel.sock"
target=robot@care-reach-robot-pi

if [[ ${1:-} == stop ]]; then
  ssh -o StrictHostKeyChecking=accept-new -S "$socket" -O exit "$target"
  exit
fi
if (( $# != 0 )); then
  echo 'Usage: camera-tunnel.sh [stop]' >&2
  exit 2
fi

if ssh -o StrictHostKeyChecking=accept-new -S "$socket" -O check "$target" >/dev/null 2>&1; then
  echo "Camera tunnel is already running: http://127.0.0.1:$local_port"
  exit
fi
if [[ -e $socket ]]; then
  echo "Stale SSH control socket at $socket; inspect and remove it before retrying." >&2
  exit 1
fi
if lsof -nP -iTCP:"$local_port" -sTCP:LISTEN >/dev/null 2>&1; then
  echo "Local port $local_port is already in use." >&2
  exit 1
fi

ssh -fN -M -S "$socket" -o StrictHostKeyChecking=accept-new -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 \
  -L "127.0.0.1:$local_port:127.0.0.1:8766" "$target"
echo "Camera tunnel started: http://127.0.0.1:$local_port"
