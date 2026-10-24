#!/bin/sh
set -eu

cd "$(dirname "$0")"
src=ros2_ws/src/so101_ros2
revision=ad6deb02043fec136698ef87c141e40aa4ed85fc

if [ -d "$src" ]; then
  echo "SO-101 source already exists at $src; leaving it unchanged."
  exit 0
fi

mkdir -p ros2_ws/src
git clone https://github.com/adoodevv/so101_ros2.git "$src"
git -C "$src" checkout --detach "$revision"
echo "SO-101 source ready at tested revision $revision."
