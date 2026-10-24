# ROS 2 Jazzy on Raspberry Pi 5

These steps use the latest 64-bit Raspberry Pi OS on a Pi 5 and the SO-101 setup in this folder. ROS runs inside an ARM64 Ubuntu 24.04 Docker container. Raspberry Pi OS is currently Debian 13 (Trixie), while the SO-101 project requires Ubuntu 24.04 and ROS 2 Jazzy; the container avoids mixing packages from different Linux releases.

The pinned SO-101 source includes its real-arm ros2_control hardware driver. The Pi shell helper passes the arm's USB serial device into Docker. The first build compiles the ROS workspace on the Pi and may take a while.

## 1. Connect to the Pi

Use Raspberry Pi Imager to install the current Raspberry Pi OS (64-bit) image. In Imager settings, configure a username, password, Wi-Fi and SSH. Boot the Pi, then connect from your computer:

    ssh <pi-user>@<pi-hostname-or-ip>

On the Pi, confirm it is 64-bit Trixie:

    uname -m
    . /etc/os-release && echo "$PRETTY_NAME"

Expected: aarch64 and Raspberry Pi OS based on Debian 13/Trixie.

## 2. Install Docker Engine

Docker supports Debian Trixie on ARM64. On the Pi over SSH:

    sudo apt update
    sudo apt install -y ca-certificates curl git
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

Add Docker's package source and install Docker:

    echo "Types: deb
    URIs: https://download.docker.com/linux/debian
    Suites: $(. /etc/os-release && echo "$VERSION_CODENAME")
    Components: stable
    Architectures: $(dpkg --print-architecture)
    Signed-By: /etc/apt/keyrings/docker.asc" | sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null
    sudo apt update
    sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    sudo usermod -aG docker "$USER"

Disconnect and SSH back in so the new Docker group membership takes effect, then verify:

    docker run --rm hello-world

## 3. Get and build the ROS setup

On the Pi:

    git clone --single-branch --branch ROS https://github.com/0xABAN/care-reach.git
    cd care-reach/ros2
    sh ./setup.sh
    docker build -t ros2-jazzy-so101:local .
    sh ./ros2.sh build

setup.sh fetches the SO-101 source at the tested revision. The Docker image provides ROS 2 Jazzy, MoveIt, Gazebo and the dependencies. The first image and ROS builds can take a while on a Pi 5; keep the Pi powered and well cooled.

To try the arm model with sliders on the Pi and view it from your computer, run on the Pi:

    sh ./pi5-viewer.sh

Then, from a second terminal on your computer, create an SSH tunnel (replace the placeholders):

    ssh -N -L 5900:127.0.0.1:5900 <pi-user>@<pi-hostname-or-ip>

While that SSH command stays open, connect macOS Screen Sharing to vnc://127.0.0.1:5900. Password: so101ros. The viewer is bound to the Pi's localhost and reached through the SSH tunnel. It only displays a model; it does not connect to or move the arm. Stop it on the Pi with docker stop so101-pi-rviz.

## 4. Connect the real follower arm

First calibrate the follower with LeRobot and save its JSON on the Pi at:

    ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json

If it was calibrated on another computer, copy that calibration file to the same path on the Pi. Calibration is specific to the physical arm; do not make up a replacement. Connect the arm's USB cable and power supply, then identify its port:

    ls -l /dev/serial/by-id/

Plug the follower in first; it is usually /dev/ttyACM0. Start a ROS shell with that device passed through to Docker:

    sh ./pi5-shell.sh /dev/ttyACM0

If the stable /dev/serial/by-id/... path exists, you can pass that instead. Inside the container, check the servos in read-only mode (this does not enable torque):

    ros2 run so101_hardware so101_hardware_check --port /dev/ttyACM0 --calibration /root/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json --watch

Then check real joint feedback with torque still disabled:

    ros2 launch so101_bringup so101_real_moveit.launch.py port:=/dev/ttyACM0 follower_id:=my_follower read_only:=true use_rviz:=false

Press Ctrl+C to stop. For the first powered run, follow the upstream first-run procedure: https://github.com/adoodevv/so101_ros2/blob/ad6deb02043fec136698ef87c141e40aa4ed85fc/so101_hardware/README.md#first-run-on-your-arm. Only after read-only checks pass, start the real controller from the ROS shell:

    ros2 launch so101_bringup so101_real_moveit.launch.py port:=/dev/ttyACM0 follower_id:=my_follower use_rviz:=false

That launch enables motor torque and connects MoveIt to the real arm. Clear the workspace and support the arm before starting; when ROS stops, torque is disabled and the arm goes limp. use_rviz:=false is used because this command is running headless over SSH. For two-arm teleoperation, pass both ports to the shell helper, then follow the upstream teleoperation setup:

    sh ./pi5-shell.sh /dev/ttyACM0 /dev/ttyACM1
    ros2 launch so101_bringup so101_teleop.launch.py use_rviz:=false

This guide does not log into or configure a particular Pi. Replace the SSH username and host with the ones chosen in Raspberry Pi Imager.
