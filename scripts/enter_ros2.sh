#!/usr/bin/env bash
set -e

docker start ros2_px4 >/dev/null 2>&1 || true
docker exec -it ros2_px4 bash
