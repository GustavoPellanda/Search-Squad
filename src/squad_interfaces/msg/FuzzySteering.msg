# The unmodified scan-time observation that produced this steering result.
# Keeping the pair together prevents navigation from mixing different acquisitions.
CorridorObservation observation
# Mathematical inference succeeded; navigation still validates freshness and geometry.
bool valid
# Body yaw rate in rad/s: positive turns left. No acceleration ramp is applied here.
float64 angular_velocity
