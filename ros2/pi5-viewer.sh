#!/bin/sh
set -eu

cd "$(dirname "$0")"
docker build -t ros2-jazzy-so101:local .
docker rm -f so101-pi-rviz >/dev/null 2>&1 || true
docker run -d --name so101-pi-rviz \
  -p 127.0.0.1:5900:5900 \
  -v "$PWD/ros2_ws:/ros2_ws" \
  -w /ros2_ws \
  -e DISPLAY=:99 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e QT_X11_NO_MITSHM=1 \
  ros2-jazzy-so101:local \
  bash -lc 'Xvfb :99 -screen 0 1600x1000x24 -nolisten tcp >/tmp/xvfb.log 2>&1 & x11vnc -display :99 -listen 0.0.0.0 -rfbport 5900 -forever -shared -passwd so101ros >/tmp/x11vnc.log 2>&1 & wait'

sleep 2
docker exec -d so101-pi-rviz bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch so101_description robot_state_publisher.launch.py'
sleep 5
echo "RViz is running on the Pi, bound to its localhost only."
echo "From your computer, open another terminal and run:"
echo "  ssh -N -L 5900:127.0.0.1:5900 <pi-user>@<pi-hostname-or-ip>"
echo "Then connect Screen Sharing to vnc://127.0.0.1:5900 (password: so101ros)."
echo "This is a model-only viewer; it does not connect to arm motors."
echo "Stop it on the Pi with: docker stop so101-pi-rviz"
