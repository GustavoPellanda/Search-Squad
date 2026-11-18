# Confirmed geometric room entrance, not a room boundary or navigation goal.
# The stamp is the last selected geometry scan, projected into header.frame_id.
std_msgs/Header header
string id
# The pose centre lies on the opening; its +X axis points into the room.
geometry_msgs/Pose pose
float64 width
uint32 observation_count
