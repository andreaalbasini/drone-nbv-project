# camera name - by default (if left blank in Tracker) it is "{type} ({deviceid})"
string name

# whether or not this camera is enabled / was used during calib
bool enabled

# the index of the camera is the number shown in Tracker, minus 1
uint32 index

# internal device id, also can  be seen in Tracker
uint32 deviceid

# type of camera (e.g., "MX F40" or "MX T40-S")
string type

# image error (in pixels) calculated during calibration
float64 image_error

# extrinsic calibration of camera w.r.t mocap origin
geometry_msgs/Pose pose

# intrinsic calibration of camera
sensor_msgs/CameraInfo camera_info

# centroids (i.e., of markers) visible in this camera
Centroid[] centroids