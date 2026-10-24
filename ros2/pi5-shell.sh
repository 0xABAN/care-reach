#!/bin/sh
set -eu

cd "$(dirname "$0")"
follower_port=${1:-/dev/ttyACM0}
leader_port=${2:-}

if [ ! -e "$follower_port" ]; then
  echo "Follower serial device not found: $follower_port" >&2
  echo "Check it with: ls -l /dev/serial/by-id/" >&2
  exit 2
fi
if [ -n "$leader_port" ] && [ ! -e "$leader_port" ]; then
  echo "Leader serial device not found: $leader_port" >&2
  echo "Check it with: ls -l /dev/serial/by-id/" >&2
  exit 2
fi

run_ros_shell() {
  if [ -d "$HOME/.cache/huggingface/lerobot" ]; then
    docker run --rm -it --network host "$@" \
      -v "$PWD/ros2_ws:/ros2_ws" \
      -v "$HOME/.cache/huggingface/lerobot:/root/.cache/huggingface/lerobot:ro" \
      -w /ros2_ws \
      ros2-jazzy-so101:local \
      bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && exec bash'
  else
    docker run --rm -it --network host "$@" \
      -v "$PWD/ros2_ws:/ros2_ws" \
      -w /ros2_ws \
      ros2-jazzy-so101:local \
      bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && exec bash'
  fi
}

if [ -n "$leader_port" ]; then
  run_ros_shell "--device=$follower_port" "--device=$leader_port"
else
  run_ros_shell "--device=$follower_port"
fi
