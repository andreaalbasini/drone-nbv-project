# Drone NBV Inspection — ArUco Marker Coverage

Autonomous drone inspection system based on **Next-Best-View (NBV)** planning.
The drone navigates iteratively to the viewpoints that maximise the visual quality of ArUco marker detections, stopping automatically when all target markers have been acquired above a configurable confidence threshold.

Tested on a real quadrotor with **PX4 v1.15.4**, **Pixhawk 6C**, and a **SIYI A8** nadir camera, controlled via a **LattePanda** companion computer running **ROS2 Humble**.

---

## Repository structure

```
ros2_ws_AA/src/
├── whale_nbv/          # Main package — NBV planner, offboard controller, ArUco detector
├── camera/             # SIYI A8 camera driver (UDP SDK → ROS2)
├── px4_msgs/           # PX4 ROS2 message definitions (compatible with PX4 v1.15.4)
└── whale_nbv_cpp/      # Gazebo camera bridge (simulation only, not needed on real drone)

calibration_data/       # SIYI A8 calibration images and results (K matrix, distortion)
scripts/
└── plot_flight_logs.py # Generates flight plots from ROS2 node logs (no rosbag needed)
```

---

## How it works

```
SIYI A8 Camera
      │  /camera/image_raw
      ▼
ArUco Detector ──────────────────────────────────────────────────────────────►
      │  /aruco/detections_json                          saves photo on goal_reached
      ▼                                                  saves photo on mission_complete
NBV Planner ──► computes next waypoint ──► /whale_nbv/goal_pose
      ▲                                            │
      │  /fmu/out/vehicle_odometry                 ▼
      └──────────────────── Offboard Controller ──► PX4 (TrajectorySetpoint, offboard mode)
```

**ArUco Detector** processes each frame and computes a confidence score per marker based on size, centring, shape regularity, and border clearance.

**NBV Planner** reads the detections and vehicle odometry, then publishes the next 3D waypoint using a ray–plane intersection strategy: the marker's pixel position is projected through the calibrated camera model onto the ground plane to estimate marker position, and the drone moves to improve coverage and confidence.

**Offboard Controller** receives PoseStamped goals and sends TrajectorySetpoint messages to PX4 at 20 Hz to maintain offboard mode. When mission is complete it performs a controlled descent at 0.30 m/s and waits for PX4 to auto-disarm.

---

## Hardware requirements

| Component | Details |
|-----------|---------|
| Flight controller | Pixhawk 6C, PX4 v1.15.4 |
| Companion computer | LattePanda (or equivalent x86/ARM SBC) |
| Camera | SIYI A8 (nadir mount), 1280×720, HFOV 70° |
| Positioning | GPS + barometer (EKF2), no external mocap needed |
| ArUco markers | 4×4\_50 dictionary (default) or APRILTAG\_36h11 |

---

## Software requirements

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- OpenCV with ArUco support (`opencv-contrib-python`)
- `numpy`, `matplotlib` (for plot script only)

---

## Build

```bash
cd ros2_ws_AA
colcon build --symlink-install --packages-skip whale_nbv_cpp
source install/setup.bash
```

> `whale_nbv_cpp` is a Gazebo simulation utility and does not compile on the real drone — skip it.

---

## Running on the drone

Everything starts together at drone boot. The companion computer launches the camera driver and the full mission stack in a single command:

```bash
ros2 launch whale_nbv mission.launch.py \
    hover_z:=-3.0 \
    confidence_threshold:=0.50 \
    gps_warmup_s:=20.0 \
    aruco_dict:=4X4_50
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hover_z` | `-3.0` | Hover altitude in NED metres (negative = above ground) |
| `confidence_threshold` | `0.70` | Minimum confidence to declare a marker acquired |
| `gps_warmup_s` | `20.0` | Seconds to wait for EKF convergence before takeoff |
| `aruco_dict` | `4X4_50` | ArUco dictionary (`4X4_50` or `APRILTAG_36h11`) |

The drone will:
1. Wait for GPS/EKF to converge (`gps_warmup_s` seconds)
2. Take off and hover at `hover_z`
3. Iterate NBV waypoints until all markers ≥ `confidence_threshold`
4. Save a photo at each waypoint (on `goal_reached` rising edge)
5. Save a final `MISSION_COMPLETE` photo
6. Perform a controlled landing and auto-disarm

Photos are saved to `~/aruco_output/` with filenames `waypoint_NNNN_HHMMSS.jpg`.

### Arm and switch to offboard (RC)

The offboard controller requires the drone to be **armed** and switched to **Offboard mode** via the RC transmitter (or QGroundControl) before it takes control. The node starts publishing setpoints immediately so PX4 can accept the offboard switch.
