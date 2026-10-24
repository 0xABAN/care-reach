FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-moveit \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws/src/so101_ros2 /tmp/so101_ros2
RUN apt-get update \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update \
    && rosdep install --from-paths /tmp/so101_ros2 --ignore-src -r -y --skip-keys moveit_planners_pilz_industrial_motion_planner \
    && apt-get install -y --no-install-recommends ros-jazzy-pilz-industrial-motion-planner \
    && rm -rf /tmp/so101_ros2 /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y --no-install-recommends xvfb imagemagick x11vnc \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
CMD ["bash"]
