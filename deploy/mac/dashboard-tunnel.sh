#!/usr/bin/env bash
set -euo pipefail

local_port=8765
socket="$HOME/.ssh/care-reach-dashboard-tunnel.sock"
target=robot@pi.local

if [[ ${1:-} == stop ]]; then
  ssh -S "$socket" -O exit "$target"
  exit
fi
if (( $# != 0 )); then
  echo 'Usage: dashboard-tunnel.sh [stop]' >&2
  exit 2
fi

if ssh -S "$socket" -O check "$target" >/dev/null 2>&1; then
  echo "Dashboard tunnel is already running: http://127.0.0.1:$local_port"
  exit
fi
if [[ -e $socket ]]; then
  echo "Stale SSH control socket at $socket; inspect and remove it before retrying." >&2
  exit 1
fi
if lsof -nP -iTCP:"$local_port" -sTCP:LISTEN >/dev/null 2>&1; then
  echo "Local port $local_port is already in use; stop the old local dashboard before starting this tunnel." >&2
  exit 1
fi

ssh -fN -M -S "$socket" -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 \
  -L "127.0.0.1:$local_port:127.0.0.1:8765" "$target"
echo "Dashboard tunnel started: http://127.0.0.1:$local_port"
