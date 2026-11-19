ptc_interfaces/EventKey key
uint64 frame_id
string<=128 calibration_id
string<=128 depth_source_frame
sensor_msgs/Image color
# Registered to RGB pixels; 32FC1 metres, invalid depth is NaN.
sensor_msgs/Image depth
sensor_msgs/CameraInfo intrinsics
geometry_msgs/PoseStamped camera_world
geometry_msgs/PoseStamped tcp_world
bool depth_registered
