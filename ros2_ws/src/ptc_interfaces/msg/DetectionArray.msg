ptc_interfaces/EventKey key
uint64 frame_id
builtin_interfaces/Time capture_stamp
string<=128 model_version
ptc_interfaces/Detection[<=4096] detections
bool complete
# model | assumed_ng | depth_workspace; assumed_ng carries no fabricated detections.
string<=32 decision_source
string<=512 message
