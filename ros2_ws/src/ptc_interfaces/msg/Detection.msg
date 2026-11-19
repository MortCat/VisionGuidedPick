# Coordinates are in original RGB pixels, not normalized/model input pixels.
string<=128 class_id
bool is_defect
float64 confidence
float64[4] bbox_xyxy
# Optional compressed mask; encoding and original dimensions identify its layout.
sensor_msgs/CompressedImage mask
bool position_valid
geometry_msgs/Point position_world
