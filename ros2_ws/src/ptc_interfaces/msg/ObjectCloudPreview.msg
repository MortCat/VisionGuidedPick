# Read-only, downsampled surface observations; never inspection evidence.
std_msgs/Header header
EventKey key
uint64 revision
uint32 scan_count
bool frozen
string source_camera
string reason
sensor_msgs/PointCloud2 latest
sensor_msgs/PointCloud2 accumulated
