# This contains filter status info about rigid bodies being tracked by mocap

std_msgs/Header header

# Is the rigid body visible to the mocap system?
bool occluded

# How many markers are specified for this rigid body?
uint8 num_markers

# How many of those markers are visible?
uint8 num_visible_markers

# Are there too many markers that are occluded?
bool too_few_markers

# expected positions of markers due to rigid body model
Marker[] model_markers

# observed positions of markers
Marker[] markers

# angle difference btwn last quaternion measurement and current measurement
float32 q_diff

# if q_diff is above some threshold, then the orientation "jumped"
bool q_jumped

# did we have to flip the measurement quaternion?
bool q_flipped

# Was this measurement used or skipped?
bool skip_update

# How many times has a measurement been thrown away?
uint8 skip_count