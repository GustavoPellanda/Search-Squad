# Local geometry at the original scan time, expressed in header.frame_id (robot base).
std_msgs/Header header
bool input_valid
string status
bool axis_valid
geometry_msgs/Pose axis
bool width_valid
float64 width
float64 lateral_error
float64 heading_error
# Valid means supported by this scan. Predicted means supported only by recent history.
bool left_wall_valid
bool right_wall_valid
bool left_predicted
bool right_predicted
float64 left_wall_distance
float64 right_wall_distance
float64 left_wall_age
float64 right_wall_age
float64 confidence
bool front_clearance_valid
float64 front_clearance
# A geometric candidate, never confirmation that the corridor mission has finished.
bool terminal_wall_candidate
# Consecutive pairs are segment endpoints in the base frame, in metres.
geometry_msgs/Point[] left_segments
geometry_msgs/Point[] right_segments
geometry_msgs/Point[] left_openings
geometry_msgs/Point[] right_openings
