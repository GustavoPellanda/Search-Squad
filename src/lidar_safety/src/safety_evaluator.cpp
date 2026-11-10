#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <lidar_safety/safety_evaluator.hpp>

namespace lidar_safety
{

// Establish valid footprint dimensions and bounded prediction settings:
SafetyEvaluator::SafetyEvaluator(const Configuration & configuration)
    : configuration_(configuration)
{
    validate_configuration();
}


// Validate geometry, braking assumptions and prediction resolution separately:
void SafetyEvaluator::validate_configuration() const
{
    for(double parameter : {
        configuration_.front, configuration_.rear, configuration_.half_width,
        configuration_.stop_margin, configuration_.slow_margin, configuration_.slowdown_ratio,
        configuration_.linear_deceleration, configuration_.angular_deceleration,
        configuration_.latency, configuration_.step
    }) {
        if(!std::isfinite(parameter) || parameter <= 0.0) {
            throw std::invalid_argument("Safety parameters must be finite and positive");
        }
    }

    if(configuration_.slowdown_ratio >= 1.0) {
        throw std::invalid_argument("slowdown_ratio must be less than 1.0");
    }
    if(configuration_.step < 0.005 || configuration_.step > 0.05) {
        throw std::invalid_argument("prediction step must be between 0.005 and 0.05 seconds");
    }
    if(configuration_.latency > 2.0) {
        throw std::invalid_argument("latency must not exceed 2.0 seconds");
    }
    if(configuration_.linear_deceleration < 0.05 || configuration_.angular_deceleration < 0.1) {
        throw std::invalid_argument("deceleration must be at least 0.05 m/s^2 and 0.1 rad/s^2");
    }
}


// Validate measurements, calculate stopping bounds, then select STOP, SLOW or CLEAR:
Decision SafetyEvaluator::evaluate(
    const std::vector<Point> & points,
    const Velocity & requested,
    const Velocity & measured,
    double sensor_age
) const
{
    // Invalid input retains the default zero output and an explicit failure reason:
    Decision decision;
    decision.state = "STOP";
    decision.reason = "INVALID_INPUT";
    decision.clearance = std::numeric_limits<double>::infinity();
    for(double measurement : {
        requested.linear, requested.angular, measured.linear, measured.angular, sensor_age
    }) {
        if(!std::isfinite(measurement)) return decision;
    }
    if(sensor_age < 0.0 || sensor_age > 2.0) return decision;

    for(const Point & obstacle : points) {
        if(!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y)) return decision;
        const double clearance = footprint_distance(obstacle.x, obstacle.y);
        decision.clearance = std::min(decision.clearance, clearance);
    }

    // Scan age adds to processing latency; measured momentum survives a command reversal:
    const double reaction_time = configuration_.latency + sensor_age;
    calculate_stopping_bounds(requested, measured, reaction_time, decision);
    const bool measured_motion_unsafe = trajectory_risk(
        points, measured, reaction_time, configuration_.stop_margin
    );
    const bool requested_motion_unsafe = trajectory_risk(
        points, requested, reaction_time, configuration_.stop_margin
    );
    if(measured_motion_unsafe || requested_motion_unsafe) {
        decision.reason = "BRAKING_ENVELOPE";
        return decision;
    }

    // An outer warning margin reduces both components without changing requested curvature:
    decision.output = requested;
    decision.state = "CLEAR";
    decision.reason = "PATH_CLEAR";
    const double warning_margin = configuration_.stop_margin + configuration_.slow_margin;
    const bool approaching_obstacle = trajectory_risk(
        points, requested, reaction_time, warning_margin
    );
    if(!approaching_obstacle) return decision;

    decision.output.linear *= configuration_.slowdown_ratio;
    decision.output.angular *= configuration_.slowdown_ratio;
    decision.state = "SLOW";
    decision.reason = "APPROACHING_OBSTACLE";

    // Independent linear/angular braking can change a curve; recheck the reduced command:
    const bool reduced_motion_unsafe = trajectory_risk(
        points, decision.output, reaction_time, configuration_.stop_margin
    );
    if(reduced_motion_unsafe) {
        decision.output = Velocity();
        decision.state = "STOP";
        decision.reason = "BRAKING_ENVELOPE";
    }
    return decision;
}


// Report reaction travel plus constant-deceleration stopping distance and angle:
void SafetyEvaluator::calculate_stopping_bounds(
    const Velocity & requested,
    const Velocity & measured,
    double reaction_time,
    Decision & decision
) const
{
    const double linear_speed = std::max(std::abs(requested.linear), std::abs(measured.linear));
    const double angular_speed = std::max(std::abs(requested.angular), std::abs(measured.angular));
    const double linear_braking_distance = linear_speed * linear_speed /
        (2.0 * configuration_.linear_deceleration);
    const double angular_braking_angle = angular_speed * angular_speed /
        (2.0 * configuration_.angular_deceleration);

    decision.stopping_distance = linear_speed * reaction_time + linear_braking_distance;
    decision.stopping_angle = angular_speed * reaction_time + angular_braking_angle;
}


// Sweep the body during reaction delay and until both velocity components have stopped:
bool SafetyEvaluator::trajectory_risk(
    const std::vector<Point> & points,
    const Velocity & velocity,
    double reaction_time,
    double margin
) const
{
    const double corner_radius = std::hypot(
        std::max(configuration_.front, configuration_.rear), configuration_.half_width
    );
    const double corner_speed = std::abs(velocity.linear) +
        corner_radius * std::abs(velocity.angular);
    const double sample_margin = configuration_.step * corner_speed;
    const double expanded_margin = margin + sample_margin;

    const double linear_braking_time = std::abs(velocity.linear) /
        configuration_.linear_deceleration;
    const double angular_braking_time = std::abs(velocity.angular) /
        configuration_.angular_deceleration;
    const double prediction_duration = reaction_time +
        std::max(linear_braking_time, angular_braking_time);

    PredictedPose pose;
    for(double elapsed = 0.0; elapsed < prediction_duration; elapsed += configuration_.step) {
        const double step_duration = std::min(configuration_.step, prediction_duration - elapsed);
        const double step_midpoint = elapsed + step_duration / 2.0;
        const double braking_time = std::max(0.0, step_midpoint - reaction_time);
        const Velocity predicted_velocity = braking_velocity(velocity, braking_time);
        advance_pose(pose, predicted_velocity, step_duration);
        if(footprint_at_pose_risk(points, pose, expanded_margin)) return true;
    }
    return false;
}


// Reduce speed magnitudes during braking, retaining each original direction:
Velocity SafetyEvaluator::braking_velocity(const Velocity & initial, double braking_time) const
{
    const double remaining_linear_speed = std::max(
        0.0, std::abs(initial.linear) - configuration_.linear_deceleration * braking_time
    );
    const double remaining_angular_speed = std::max(
        0.0, std::abs(initial.angular) - configuration_.angular_deceleration * braking_time
    );
    return {
        std::copysign(remaining_linear_speed, initial.linear),
        std::copysign(remaining_angular_speed, initial.angular)
    };
}


// Integrate one differential-drive step using its midpoint heading in the initial base frame:
void SafetyEvaluator::advance_pose(
    PredictedPose & pose,
    const Velocity & velocity,
    double duration
) const
{
    const double midpoint_yaw = pose.yaw + velocity.angular * duration / 2.0;
    pose.x += velocity.linear * std::cos(midpoint_yaw) * duration;
    pose.y += velocity.linear * std::sin(midpoint_yaw) * duration;
    pose.yaw += velocity.angular * duration;
}


// Transform each obstacle into the predicted body frame and check whether clearance decreases:
bool SafetyEvaluator::footprint_at_pose_risk(
    const std::vector<Point> & points,
    const PredictedPose & pose,
    double expanded_margin
) const
{
    constexpr double CLEARANCE_DECREASE_TOLERANCE_M = 1e-5;
    const double cosine = std::cos(pose.yaw);
    const double sine = std::sin(pose.yaw);
    for(const Point & obstacle : points) {
        const double initial_clearance = footprint_distance(obstacle.x, obstacle.y);
        if(initial_clearance <= 0.0) return true;

        const double relative_x = obstacle.x - pose.x;
        const double relative_y = obstacle.y - pose.y;
        const double x_in_body = cosine * relative_x + sine * relative_y;
        const double y_in_body = -sine * relative_x + cosine * relative_y;
        const double predicted_clearance = footprint_distance(x_in_body, y_in_body);

        // A nearby surface may be left behind, but cannot be approached inside the margin:
        const bool within_margin = predicted_clearance < expanded_margin;
        const bool approaching = predicted_clearance <
            initial_clearance - CLEARANCE_DECREASE_TOLERANCE_M;
        if(within_margin && approaching) return true;
    }
    return false;
}


// Calculate signed distance to the rectangular body; negative means physical overlap:
double SafetyEvaluator::footprint_distance(double x, double y) const
{
    const double center_x = (configuration_.front - configuration_.rear) / 2.0;
    const double half_length = (configuration_.front + configuration_.rear) / 2.0;
    const double distance_from_x_edge = std::abs(x - center_x) - half_length;
    const double distance_from_y_edge = std::abs(y) - configuration_.half_width;

    // Outside the rectangle use edge/corner distance; inside retain penetration depth:
    const double outside_distance = std::hypot(
        std::max(distance_from_x_edge, 0.0), std::max(distance_from_y_edge, 0.0)
    );
    const double inside_distance = std::min(
        std::max(distance_from_x_edge, distance_from_y_edge), 0.0
    );
    return outside_distance + inside_distance;
}

}  // namespace lidar_safety
