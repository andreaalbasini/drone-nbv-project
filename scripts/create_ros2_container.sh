#!/usr/bin/env bash
set -e

if docker ps -a --format '{{.Names}}' | grep -qx ros2_px4; then
  echo "Container ros2_px4 gia presente."
  exit 0
fi

docker run -d \
  --net=host \
  --name ros2_px4 \
  -v ~/Documents/Progetto_Drone:/ws_host \
  -w /ws_host \
  osrf/ros:humble-desktop \
  tail -f /dev/null

echo "Container ros2_px4 creato."
