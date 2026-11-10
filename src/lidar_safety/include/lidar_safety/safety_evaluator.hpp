#pragma once

#include <string>
#include <vector>

namespace lidar_safety
{

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Velocity {
    double linear = 0.0;
    double angular = 0.0;
};

struct Configuration {
    double front = 0.11;
    double rear = 0.12;
    double half_width = 0.11;
    double stop_margin = 0.04;
    double slow_margin = 0.20;
    double slowdown_ratio = 0.4;
    double linear_deceleration = 0.5;
    double angular_deceleration = 1.0;
    double latency = 0.10;
    double step = 0.02;
};

struct Decision {
    Velocity output;
    std::string state;
    std::string reason;
    double clearance = 0.0;
    double stopping_distance = 0.0;
    double stopping_angle = 0.0;
};

/*
Evaluate observed points already transformed into the current base frame.
The calculation only restricts a requested twist; it never selects a route.
*/
class SafetyEvaluator
{
    public:

    explicit SafetyEvaluator(const Configuration & configuration = Configuration());
    Decision evaluate(
        const std::vector<Point> & points,  // Finite hits in the current base frame, metres.
        const Velocity & requested,       // Selected command, m/s and rad/s.
        const Velocity & measured,        // Current encoder velocity in the base frame.
        double sensor_age                 // Scan age in seconds, excluding configured latency.
    ) const;

    private:

    struct PredictedPose {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    void validate_configuration() const;
    void calculate_stopping_bounds(
        const Velocity & requested,
        const Velocity & measured,
        double reaction_time,
        Decision & decision
    ) const;
    double footprint_distance(double x, double y) const;
    Velocity braking_velocity(const Velocity & initial, double braking_time) const;
    void advance_pose(PredictedPose & pose, const Velocity & velocity, double duration) const;
    bool footprint_at_pose_risk(
        const std::vector<Point> & points,
        const PredictedPose & pose,
        double expanded_margin
    ) const;
    bool trajectory_risk(
        const std::vector<Point> & points,
        const Velocity & velocity,
        double reaction_time,
        double margin
    ) const;

    Configuration configuration_;
};

}  // namespace lidar_safety
