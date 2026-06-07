#!/usr/bin/env bash
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

SESSION="drone"
HOVER_Z="${HOVER_Z:--3.0}"
CONFIDENCE="${CONFIDENCE:-0.70}"
WARMUP="${WARMUP:-10.0}"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already active."
    exit 0
fi

tmux new-session -d -s "$SESSION" -x 220 -y 50

# Panel 0: DDS agent
tmux send-keys -t "$SESSION:0.0" \
    "MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600" C-m

sleep 5

# Panel 1: full ROS2 mission (nbv_planner + offboard + camera + aruco)
tmux split-window -v -t "$SESSION"
tmux send-keys -t "$SESSION:0.1" \
    "source /opt/ros/humble/setup.bash && \
     source /home/lp10/ros2_ws_AA/ros2_ws_AA/install/setup.bash && \
     ros2 launch whale_nbv mission.launch.py \
       hover_z:=${HOVER_Z} \
       confidence_threshold:=${CONFIDENCE} \
       gps_warmup_s:=${WARMUP}" C-m

echo "Drone started. To view logs: tmux attach -t drone"
