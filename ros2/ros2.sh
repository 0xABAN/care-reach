#!/bin/sh
set -eu

cd "$(dirname "$0")"

case "${1:-shell}" in
  shell)
    docker run --rm -it \
      --name ros2-lyrical \
      -v "$PWD/ros2_ws:/ros2_ws" \
      -w /ros2_ws \
      ros2-jazzy-so101:local \
      bash -lc 'source /opt/ros/jazzy/setup.bash; if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi; exec bash'
    ;;
  build)
    docker run --rm -i \
      -v "$PWD/ros2_ws:/ros2_ws" \
      -w /ros2_ws \
      ros2-jazzy-so101:local \
      bash -lc 'source /opt/ros/jazzy/setup.bash; colcon build --symlink-install'
    ;;
  talker)
    docker run --rm -it \
      -v "$PWD/ros2_ws:/ros2_ws" \
      ros2-jazzy-so101:local \
      bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 run demo_nodes_cpp talker'
    ;;
  listener)
    docker run --rm -it \
      -v "$PWD/ros2_ws:/ros2_ws" \
      ros2-jazzy-so101:local \
      bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 run demo_nodes_cpp listener'
    ;;
  fake)
    docker run --rm -it \
      -v "$PWD/ros2_ws:/ros2_ws" \
      -w /ros2_ws \
      ros2-jazzy-so101:local \
      bash -lc 'source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; ros2 launch so101_bringup so101_fake_moveit.launch.py'
    ;;
  *)
    echo "Usage: sh $0 [shell|build|fake|talker|listener]" >&2
    exit 2
    ;;
esac
