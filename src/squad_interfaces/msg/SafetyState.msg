# Decision of the gate's lidar_safety layer; Collision Monitor remains downstream.
std_msgs/Header header
string source
string state
string reason
geometry_msgs/Twist requested
geometry_msgs/Twist allowed
# Forward probe uses the gate's configured maximum linear speed, with current momentum.
bool forward_allowed
float64 scan_age
float64 nearest_clearance
float64 stopping_distance
float64 stopping_angle
