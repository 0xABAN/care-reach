#!/usr/bin/env bash
set -euo pipefail

local_port=8765
socket="$HOME/.ssh/care-reach-dashboard-tunnel.sock"
ethernet_target=robot@pi.local
tailscale_target=robot@care-reach-robot-pi

if [[ ${1:-} == stop ]]; then
  ssh -o StrictHostKeyChecking=accept-new -S "$socket" -O exit "$ethernet_target"
  exit
fi
if (( $# != 0 )); then
  echo 'Usage: dashboard-tunnel.sh [stop]' >&2
  exit 2
fi

if ssh -o StrictHostKeyChecking=accept-new -S "$socket" -O check "$ethernet_target" >/dev/null 2>&1; then
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

if ssh -o BatchMode=yes -o ConnectTimeout=4 -o StrictHostKeyChecking=accept-new "$ethernet_target" true >/dev/null 2>&1; then
  target=$ethernet_target
elif ssh -o BatchMode=yes -o ConnectTimeout=4 -o StrictHostKeyChecking=accept-new "$tailscale_target" true >/dev/null 2>&1; then
  target=$tailscale_target
else
  echo 'Cannot SSH to the Pi over Ethernet or Tailscale.' >&2
  exit 1
fi

ssh -fN -M -S "$socket" -o BatchMode=yes -o ConnectTimeout=4 -o StrictHostKeyChecking=accept-new \
  -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 -o ServerAliveCountMax=3 \
  -L "127.0.0.1:$local_port:127.0.0.1:8765" "$target"
echo "Dashboard tunnel started via $target: http://127.0.0.1:$local_port"
