# ROS 2 Jazzy setup for the SO-101

This folder runs the [adoodevv/so101_ros2](https://github.com/adoodevv/so101_ros2) project locally on a Mac using Docker and Colima. The source is pinned to the revision tested with this setup. ROS runs inside Ubuntu 24.04; you use it from Terminal on the Mac.

## First-time setup

Install Docker and [Colima](https://github.com/abiosoft/colima), then from this folder run:

```sh
sh ./setup.sh
colima start
docker build -t ros2-jazzy-so101:local .
sh ./ros2.sh build
```

The first build downloads ROS dependencies and can take a while. Later, open a ready-to-use ROS shell with:

```sh
sh ./ros2.sh shell
```

## Try it

Run a harmless ROS talker in one Terminal window:

```sh
sh ./ros2.sh talker
```

In another window, run the listener:

```sh
sh ./ros2.sh listener
```

For a simulated SO-101 MoveIt demo with mock hardware (no real motors), run:

```sh
sh ./ros2.sh fake
```

To see the robot model and joint sliders in macOS Screen Sharing, run:

```sh
sh ./start-rviz.sh
```

If prompted, the local viewer password is `so101ros`. It listens only on this Mac at `127.0.0.1:5900`. The sliders move a simulated model and do not command the robot. Stop the viewer with `docker stop so101-rviz-local`.

## What the commands mean

- `setup.sh` downloads the pinned SO-101 ROS source code.
- `Dockerfile` builds a Linux container with ROS 2 Jazzy, MoveIt, Gazebo and the project dependencies.
- `ros2.sh build` compiles the ROS packages; `shell` opens ROS with them loaded.
- `fake` starts a simulated robot; it does not connect to motors.
- `start-rviz.sh` opens a visual robot model with sliders.

This setup does not configure a physical SO-101 arm or connect it to a Raspberry Pi. ROS networking between a Mac container and a Pi needs additional configuration when you are ready for that.
