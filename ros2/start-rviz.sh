#!/bin/sh
set -eu

cd "$(dirname "$0")"
docker build -t ros2-jazzy-so101:local .
docker rm -f so101-rviz-local >/dev/null 2>&1 || true
docker run -d --name so101-rviz-local \
  -p 127.0.0.1:5900:5900 \
  -v "$PWD/ros2_ws:/ros2_ws" \
  -w /ros2_ws \
  -e DISPLAY=:99 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e QT_X11_NO_MITSHM=1 \
  ros2-jazzy-so101:local \
  bash -lc 'Xvfb :99 -screen 0 1600x1000x24 -nolisten tcp >/tmp/xvfb.log 2>&1 & x11vnc -display :99 -listen 0.0.0.0 -rfbport 5900 -forever -shared -passwd so101ros >/tmp/x11vnc.log 2>&1 & wait'

sleep 2
docker exec -d so101-rviz-local bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch so101_description robot_state_publisher.launch.py'
sleep 8
docker exec so101-rviz-local pgrep -af 'rviz2|joint_state_publisher_gui|robot_state_publisher' || true
open 'vnc://127.0.0.1:5900'
