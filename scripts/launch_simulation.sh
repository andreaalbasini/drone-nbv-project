#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="${HOME}/Documents/Progetto_Drone"
PX4_DIR="${BASE_DIR}/PX4-Autopilot"
AGENT_DIR="${BASE_DIR}/Micro-XRCE-DDS-Agent/build"
CONTAINER_NAME="ros2_px4"
ROS_IMAGE="osrf/ros:humble-desktop"

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "[ERROR] Command not found: $1"
    exit 1
  }
}

open_tab() {
  local title="$1"
  local cmd="$2"
  gnome-terminal --title="$title" -- bash -lc "$cmd" &
}

need_cmd gnome-terminal
need_cmd docker

if [ ! -d "$PX4_DIR" ]; then
  echo "[ERROR] PX4 folder not found: $PX4_DIR"
  exit 1
fi

if [ ! -x "${AGENT_DIR}/MicroXRCEAgent" ]; then
  echo "[ERROR] MicroXRCEAgent not found or not executable: ${AGENT_DIR}/MicroXRCEAgent"
  exit 1
fi

if ! docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  echo "[INFO] Creating Docker container: $CONTAINER_NAME"
  docker run -d \
    --net=host \
    --user $(id -u):$(id -g) \
    --name "$CONTAINER_NAME" \
    -v "${BASE_DIR}:/ws_host" \
    -w /ws_host \
    "$ROS_IMAGE" \
    tail -f /dev/null >/dev/null
else
  echo "[INFO] Reusing existing Docker container: $CONTAINER_NAME"
fi

docker start "$CONTAINER_NAME" >/dev/null 2>&1 || true


open_tab "PX4 SITL" "cd '$PX4_DIR'; make px4_sitl gazebo; exec bash"
sleep 1
open_tab "XRCE Agent" "cd '$AGENT_DIR'; ./MicroXRCEAgent udp4 -p 8888; exec bash"
sleep 1
open_tab "ROS2 Shell" "docker exec -it '$CONTAINER_NAME' bash -lc 'export HOME=/ws_host/.home; mkdir -p \$HOME/.ros/log; source /opt/ros/humble/setup.bash; [ -f /ws_host/ros2_ws/install/setup.bash ] && source /ws_host/ros2_ws/install/setup.bash; echo \"ROS 2 container ready.\"; exec bash'"

echo "[OK] Opening three terminals."
echo "[INFO] If you see less then 3, try manually:"
echo "  docker ps --format '{{.Names}}'"
echo "  docker exec -it $CONTAINER_NAME bash"