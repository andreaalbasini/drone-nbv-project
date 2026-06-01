> **ℹ️ Info:** This is a clone of https://gitlab.com/mit-acl/fsw/mocap

ROS2 Motion Capture Drivers
==========================

This package contains ROS2 wrappers for VICON drivers (Optitrack coming soon). It allows the pose of multiple rigid bodies to be broadcast over the ROS network at the rate specified by underlying software (VICON Tracker or OptiTrack Motive).

In addition to streaming the pose of rigid bodies, this packages can stream twist and linear acceleration. To do so, a quaternion-based attitude EKF estimates `omega` and `q` from input measurements `q_mocap` and a GHK filter (nearly-constant acceleration model) is used to generate `p`, `v`, and `a` from input measurements `p_mocap`.

## Getting Started

Clone this package into your workspace and build (e.g., `colcon build`).

### Networking

The motion capture system is run on a Windows machine. This Windows machine and the ROS2 basestation (i.e., `aldrin`) must be on the same network.

Since the motion capture software uses multicast, this feature must be enabled for the specific network interface card. In most cases, this should be true by default (can check with `ip a`).

#### `netplan`

In Ubuntu 20.04 (July 2021) `netplan` was used to configure a static IP for the RAVEN / VICON connection. The default `NetworkManager` rendered was used. A new yaml `02-raven-static-vicon.yaml` was created with

```yaml
network:
  version: 2
  ethernets:
    enp4s0:
      dhcp4: false
      addresses:
        - 192.168.0.20/24
      gateway4: 192.168.0.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
    enp0s31f6: # OptiTrack connection (not tested)
      link-local: [ipv4]
      dhcp4: false
      addresses:
        - 192.168.1.119/24
```

This code will cause the relevant network interface to automatically join the appropriate multicast group. You can use `netstat -gn` to check.

### Running

1. Turn on motion capture system
2. Start mocap software on Windows computer (i.e., VICON Tracker or OptiTrack Motive)
3. Run `ros2 launch mocap vicon.launch.xml` or `ros2 launch mocap optitrack.launch.xml`

## Room Bounds

Since motion capture understands the valid flight volume, it makes since to have this package also set the global room bounds. These room bounds are set in the `/room_bounds` namespace. Other packages should respect these bounds by not allowing robots to leave them.

## Coordinate Frames
*The hardest thing in robotics*

ROS uses an East-North-Up (ENU) inertial coordinate frame. In the ACL Highbay, we have defined our `world` coordinate frame to have *x* aligned with the length of the highbay, positvely pointing East. The `world` *y* is aligned with the width of the room, positively pointing North (i.e., pointing away from the control room). The *z* coordinate is pointing up.

VICON and OptiTrack have their own coordinate frames, which will not necessarily be the same as we've defined our `world` frame above. To resolve this difference, we must define a transformation from VICON / OptiTrack into the `world`. Note that this transformation does not show up in the ROS tf tree---it happens behind the scenes whenever raw mocap measurements are received. This transformation is defined in each launch file as `mocap_wrt_parent_frame`
(and `body_wrt_mocap_body`---if you are unsure, more than likely this should be set as the inverse of the `mocap_wrt_parent_frame` transform).

Once these transformations are defined you never need to think of the raw mocap coordinate frame again. Instead, think of everything as defined in the `parent` (i.e., `world`) frame.

As an example, the OptiTrack coordinate frame has *z* going east, with *x* north and *y* up. How do we define the `mocap_wrt_parent_frame` and `body_wrt_mocap_body` parameters to transform OptiTrack rigid body measurements into our desired ENU world frame? The Euler angles of parameter `mocap_wrt_parent_frame` can be identified by starting with yaw (then pitch, then roll---about the intrinsic axes) to define how you would need to rotate the world frame to get into the OptiTrack frame. It turns out that the proper intrinsic ZYX rotation order is `1.5708 0 1.5708` (convince yourself!). Then, we need to find the inverse so we can define `body_wrt_mocap_body`. Because there are two angles involved, it is not as simple as negating the values. The following Python code demonstrates how you might find the inverse.

```python
In [1]: from scipy.spatial.transform import Rotation as Rot

In [2]: Rot.from_euler('ZYX', (90,0,90), degrees=True)
Out[2]: <scipy.spatial.transform._rotation.Rotation at 0x7f320ee97990>

In [3]: R = Rot.from_euler('ZYX', (90,0,90), degrees=True).as_matrix()
Out[3]:
array([[ 2.22044605e-16, -2.22044605e-16,  1.00000000e+00],
       [ 1.00000000e+00,  0.00000000e+00, -2.22044605e-16],
       [ 0.00000000e+00,  1.00000000e+00,  2.22044605e-16]])

In [4]: Rot.from_matrix(R.T).as_euler('ZYX')
Out[4]: array([-1.57079633, -1.57079633,  0.        ])
```

So `body_wrt_mocap_body` should have its YPR values as `-1.5708 -1.5708 0`.

## Implementation Details

There are three main components of this software:

1. The underlying motion capture client SDK, which retrieves raw multicast packets from the mocap software.
2. The ACL rigid body filtering to produce 6 DOF ground-truth state.
3. The ROS2 wrapper, which drives the client SDK to receive data, manages a list of rigid bodies, and provides measurement updates to existing rigid bodies.

## Motion Capture Software Settings (Windows)

### VICON Tracker

We depend on an accurate update rate from VICON. Make sure the **Genlock and Timecode** settings match with the following image. These settings correspond with `ViconSDK::TimecodeStandard::PAL`.

![tracker2.2_timecode_settings](media/timecode_settings.png)

### OptiTrack Motive

## Creating Rigid Bodies

For more details, see the following links to the wiki:

- [VICON Tracker](https://gitlab.com/mit-acl/fsw/mocap/-/wikis/VICON:-Creating-Rigid-Bodies)
- OptiTrack Motive

## Change Parameters (for better performance)

1. When creating a rigid body change these two parameters:
- Min Marker Count 6 or 8 (depending on how many markers you use)
- Max Deflection: 8mm
- Tracking Algorithm: Ray Based 
<img src="media/optitrack_rigid_body_settings.jpg" alt="drawing" width="200"/>  


2. Change these camera parameters
- Max Residual: 6.00
- Maximum Ray Length: 12.00

<img src="media/optitrack_camera_settings.jpg" alt="drawing" width="200"/>  



## RPC

A remote procedural call (RPC) server/client (client is in ROS code, server meant to be on AMSTRONG VICON computer) is included. The purpose of this was so that the ROS side could access the camera calibration files (i.e., camera poses) and rigid body definitions, which are stored as files on AMSTRONG. The goal was to detect when a particular rigid body needed to be recalibrated (i.e., due to collision related stress on the frame / vicon dots), but some critical VICON sdk functions are missing because we use old VICON Tracker software.

## FAQ

1. What frame are twist and accel published in?

    - All signals are w.r.t the `world` frame. Twist and accel are expressed in the `world` frame, **not** the body frame.

2. If a rigid body becomes occluded, will its pose still be broadcast?

    - No. Signals associated with a rigid body are only broadcast when a valid raw mocap measurement is received.

3. What version of VICON Tracker are we using?

    - We use version 2.2 of the software / API.

4. Fixing "Attempting to connect to VICON server..." issue when running `ros2 launch mocap vicon.launch.xml` on `aldrin`.

    - This issue rarely happens, but when it does, it is caused by having a duplicate session of `Tracker.exe` on ARMSTRONG PC(computer that has the VICON Tracker software). 
    - Fix:   
          1. Close VICON Tracker software.   
          2. Go into Windows Task Mangager (you can also find it by doing `Ctrl + Alt + Del`)   
          3. Look for `Tracker.exe` on the `Process` tab.  
          4. End all the running `Tracker.exe` processes.  
          5. Relaunch VICON Tracker software.  
          6. All set! You should not have any issues running `ros2 launch mocap vicon.launch.xml` on `aldrin` anymore.  

5. What version of the VICON DataStream SDK are we using?
    - We were using 1.5.0, but moved to 1.11.0 (Mar 2022). Some new functions work, some do not (see `sdk/vicon/notes.txt` for more info).

6. Seeing sdk-related errors: `Build step for vicon_datastream_sdk failed: 2`
    - Fix: remove /build folder and build it again.