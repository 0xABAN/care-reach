# One-time SO-101 arm calibration

The ROS driver needs the calibration JSON for the exact physical arm. LeRobot writes matching calibration values into the arm's servos. Do not create an empty or copied-from-another-arm calibration file.

## Calibrate the follower

Use a computer with LeRobot and its Feetech support installed, with the follower arm connected over USB and its power supply connected. Follow the LeRobot installation guide first: https://huggingface.co/docs/lerobot/installation

For a new arm, set its servo motor IDs one servo at a time:

    lerobot-setup-motors --robot.type=so101_follower --robot.port=<follower-port>

Then calibrate it with a stable name. This example uses my_follower:

    lerobot-calibrate --robot.type=so101_follower --robot.port=<follower-port> --robot.id=my_follower

Follow the prompts and move the arm through the ranges it asks for. LeRobot saves the file locally at:

    ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json

For a leader arm, set up and calibrate it separately with the LeRobot teleoperator commands. See the SO-101 project's physical-arm setup instructions for details:
https://github.com/adoodevv/so101_ros2/blob/ad6deb02043fec136698ef87c141e40aa4ed85fc/README.md#setting-up-your-physical-arms-once

## Copy the file to the Pi

On the Pi, create the destination folder (replace my_follower if you chose a different id):

    mkdir -p ~/.cache/huggingface/lerobot/calibration/robots/so_follower

From the computer that has the calibration file, copy it to the Pi (replace the SSH username and host):

    scp ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json <pi-user>@<pi-hostname-or-ip>:.cache/huggingface/lerobot/calibration/robots/so_follower/

The Pi ROS helper mounts this folder read-only inside the container at /root/.cache/huggingface/lerobot. The real-arm launch and read-only diagnostic will use the file automatically when follower_id:=my_follower is selected.

## Safety check

Before a powered launch, connect the arm and its supply, then use the Pi ROS shell to run so101_hardware_check with the port and calibration file. It is read-only and does not enable torque. Verify that all six servos are found and the calibration matches. Follow the upstream first-run procedure before the first powered run. The powered driver enables torque and holds the current pose; on shutdown the arm goes limp, so support it if raised.
