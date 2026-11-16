#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <squad_interfaces/msg/corridor_observation.hpp>
#include <squad_interfaces/msg/fuzzy_steering.hpp>
#include <squad_interfaces/msg/robot_event.hpp>
#include <std_srvs/srv/set_bool.hpp>

/*
Navigate the observed straight corridor using the separate FuzzySteering node's
angular result, paired with the original scan-time geometry. This node validates
inputs, regulates forward speed, applies ramps, handles stop/resume and confirms
the terminal wall. It requests cmd_vel_corridor; the gate, Collision Monitor and
timed drive retain the final command path. End recognition remains limited to
the unobstructed test corridor, not generic obstacles or fleet mission completion.
*/
class CorridorFollower : public rclcpp::Node
{
    public:

    CorridorFollower();

    private:

    using Observation = squad_interfaces::msg::CorridorObservation;
    using Steering = squad_interfaces::msg::FuzzySteering;
    enum class State {
        STOPPED, WAITING_INPUTS, FOLLOWING, VERIFYING_END, END_DETECTED, BLOCKED, FINISHED
    };

    void read_parameters();
    void initialize_communication();
    void enable_callback(const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response);
    void steering_callback(Steering::ConstSharedPtr message);
    void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr message);
    void control_loop();
    bool observation_is_usable() const;
    bool odometry_is_fresh() const;
    void update_terminal_evidence();
    void reset_terminal_evidence();
    geometry_msgs::msg::Twist tracking_command(double elapsed) const;
    void publish_stop(State state, const std::string & detail);
    void report_activity(State state, const std::string & detail);

    // Metres, radians and seconds; parameters remain fixed for this run:
    double linear_speed_ = 0.0;
    double max_angular_speed_ = 0.0;
    double lateral_error_scale_ = 0.0;
    double heading_error_scale_ = 0.0;
    double max_lateral_error_ = 0.0;
    double max_heading_error_ = 0.0;
    double min_wall_distance_ = 0.0;
    double min_confidence_ = 0.0;
    double observation_timeout_ = 0.0;
    double odom_timeout_ = 0.0;
    double front_stop_clearance_ = 0.0;
    double front_slow_clearance_ = 0.0;
    double terminal_clearance_tolerance_ = 0.0;
    double terminal_confirmation_time_ = 0.0;
    int terminal_min_observations_ = 0;
    double linear_acceleration_ = 0.0;
    double angular_acceleration_ = 0.0;

    // Distinct inputs; monotonic arrival ages also expire when /clock pauses:
    std::string robot_;
    Observation::ConstSharedPtr observation_;
    nav_msgs::msg::Odometry::ConstSharedPtr odometry_;
    std::chrono::steady_clock::time_point observation_arrival_;
    std::chrono::steady_clock::time_point odom_arrival_;
    double previous_observation_time_ = -1.0;
    double steering_angular_speed_ = 0.0;
    double previous_control_time_ = 0.0;
    geometry_msgs::msg::Twist previous_command_;

    // Local following and completion only; no dispatcher or Scout leases:
    bool enabled_ = false;
    bool finished_ = false;
    State state_ = State::STOPPED;
    std::string activity_detail_;
    double terminal_started_ = -1.0;
    int terminal_observations_ = 0;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
    rclcpp::Publisher<squad_interfaces::msg::RobotEvent>::SharedPtr event_publisher_;
    rclcpp::Subscription<Steering>::SharedPtr steering_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};


// Configure control before connecting callbacks or starting the 20 Hz wall timer:
CorridorFollower::CorridorFollower() : Node("corridor_follower")
{
    robot_ = std::string(get_namespace()).substr(1);
    read_parameters();
    initialize_communication();
    previous_control_time_ = now().seconds();
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&CorridorFollower::control_loop, this)
    );
}


// Shared scales/limit still regulate forward speed; fuzzy rules belong to FuzzySteering:
void CorridorFollower::read_parameters()
{
    enabled_ = declare_parameter("autostart", false);
    linear_speed_ = declare_parameter("linear_speed", 0.10);
    max_angular_speed_ = declare_parameter("max_angular_speed", 0.60);
    lateral_error_scale_ = declare_parameter("lateral_error_scale", 0.35);
    heading_error_scale_ = declare_parameter("heading_error_scale", 0.524);
    max_lateral_error_ = declare_parameter("max_lateral_error", 0.55);
    max_heading_error_ = declare_parameter("max_heading_error", 0.524);
    min_wall_distance_ = declare_parameter("min_wall_distance", 0.25);
    min_confidence_ = declare_parameter("min_confidence", 0.10);
    observation_timeout_ = declare_parameter("observation_timeout", 0.30);
    odom_timeout_ = declare_parameter("odom_timeout", 0.50);
    front_stop_clearance_ = declare_parameter("front_stop_clearance", 0.25);
    front_slow_clearance_ = declare_parameter("front_slow_clearance", 0.80);
    terminal_clearance_tolerance_ = declare_parameter("terminal_clearance_tolerance", 0.04);
    terminal_confirmation_time_ = declare_parameter("terminal_confirmation_time", 1.5);
    terminal_min_observations_ = declare_parameter("terminal_min_observations", 5);
    linear_acceleration_ = declare_parameter("linear_acceleration", 0.10);
    angular_acceleration_ = declare_parameter("angular_acceleration", 0.80);

    for(double limit : {
        linear_speed_, max_angular_speed_, lateral_error_scale_, heading_error_scale_,
        max_lateral_error_, max_heading_error_, min_wall_distance_, min_confidence_,
        observation_timeout_, odom_timeout_, front_stop_clearance_, front_slow_clearance_,
        terminal_clearance_tolerance_, terminal_confirmation_time_,
        linear_acceleration_, angular_acceleration_
    }) {
        if(!std::isfinite(limit) || limit <= 0.0) {
            throw std::invalid_argument("Scout control limits must be finite and positive");
        }
    }
    if(linear_speed_ > 0.15 || max_angular_speed_ > 0.8 || min_confidence_ > 1.0 ||
        front_stop_clearance_ + terminal_clearance_tolerance_ >= front_slow_clearance_ ||
        terminal_min_observations_ < 2) {
        throw std::invalid_argument("Invalid Scout speeds, clearance or confirmation");
    }
}


// Paired steering/geometry is reliable; odometry accepts the simulator's sensor QoS:
void CorridorFollower::initialize_communication()
{
    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_corridor", 1);
    event_publisher_ = create_publisher<squad_interfaces::msg::RobotEvent>("/fleet/events", 20);
    steering_subscription_ = create_subscription<Steering>(
        "corridor_steering", 1,
        std::bind(&CorridorFollower::steering_callback, this, std::placeholders::_1)
    );
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&CorridorFollower::odom_callback, this, std::placeholders::_1)
    );
    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "set_corridor_enabled",
        std::bind(&CorridorFollower::enable_callback, this,
            std::placeholders::_1, std::placeholders::_2)
    );
}


// Explicit stop bypasses ramps; completion cannot rearm itself:
void CorridorFollower::enable_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response
)
{
    if(request->data && finished_) {
        response->success = false;
        response->message = "Corridor already finished; restart the run for a new traversal";
        return;
    }
    enabled_ = request->data;
    reset_terminal_evidence();
    control_loop();
    response->success = true;
    response->message = enabled_ ? "Fuzzy following enabled" : "Scout stopped";
}


// The angular output and geometry travel together, so navigation never combines
// different acquisitions. Retain the sensor timestamp: processing is not a new scan.
// Count distinct acquisitions, never timer ticks or replays, as terminal evidence:
void CorridorFollower::steering_callback(Steering::ConstSharedPtr message)
{
    const auto & observation = message->observation;
    if(!message->valid || !std::isfinite(message->angular_velocity) ||
        std::abs(message->angular_velocity) > max_angular_speed_ + 1e-9 ||
        !observation.input_valid || observation.header.stamp.sec < 0 ||
        observation.header.stamp.nanosec >= 1000000000U) {
        observation_.reset();
        reset_terminal_evidence();
        return;
    }
    const double timestamp = rclcpp::Time(observation.header.stamp).seconds();
    const double age = now().seconds() - timestamp;
    if(age < -0.05 || age > observation_timeout_) {
        observation_.reset();
        reset_terminal_evidence();
        return;
    }
    // A simulation reset discards the old epoch; ordinary reordering cannot refresh it:
    if(now().seconds() < previous_observation_time_) {
        previous_observation_time_ = -1.0;
        reset_terminal_evidence();
    }
    if(timestamp <= previous_observation_time_) return;
    if(timestamp - previous_observation_time_ > observation_timeout_) reset_terminal_evidence();
    previous_observation_time_ = timestamp;
    observation_ = std::make_shared<Observation>(observation);
    steering_angular_speed_ = message->angular_velocity;
    observation_arrival_ = std::chrono::steady_clock::now();
    update_terminal_evidence();
}


// Keep body velocity for readiness and completion, not for corridor geometry:
void CorridorFollower::odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
    if(message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000U) {
        odometry_.reset();
        return;
    }
    if(odometry_ && message->header.stamp == odometry_->header.stamp) return;
    odometry_ = message;
    odom_arrival_ = std::chrono::steady_clock::now();
}


// Execution order: validate inputs, handle the terminal surface, then steer and publish:
void CorridorFollower::control_loop()
{
    const double control_time = now().seconds();
    const double elapsed = std::clamp(control_time - previous_control_time_, 0.0, 0.10);
    previous_control_time_ = control_time;
    if(!enabled_) {
        publish_stop(State::STOPPED, "Scout disabled by operator");
        return;
    }
    if(finished_) {
        publish_stop(State::FINISHED, "Test corridor end confirmed; Scout is stopped");
        return;
    }
    if(!observation_is_usable() || !odometry_is_fresh()) {
        reset_terminal_evidence();
        publish_stop(
            State::WAITING_INPUTS, "Waiting for fresh fuzzy steering, geometry and odometry"
        );
        return;
    }

    // Clearance starts at the physical front edge, not the laser. Stop first,
    // then require a persistent transverse wall and measured standstill:
    // Once stopped, tolerate a small change in range from sensor noise rather
    // than repeatedly moving forward and restarting the confirmation window:
    const double stop_distance = front_stop_clearance_ +
        (state_ == State::VERIFYING_END ? terminal_clearance_tolerance_ : 0.0);
    if(observation_->front_clearance <= stop_distance) {
        if(!observation_->terminal_wall_candidate) {
            publish_stop(State::BLOCKED, "Front blocked without corridor-end geometry");
            return;
        }
        const double evidence_time = previous_observation_time_ - terminal_started_;
        const auto & velocity = odometry_->twist.twist;
        const bool stopped = std::abs(velocity.linear.x) < 0.01 &&
            std::abs(velocity.angular.z) < 0.03;
        if(terminal_observations_ >= terminal_min_observations_ &&
            evidence_time >= terminal_confirmation_time_ && stopped) {
            finished_ = true;
            report_activity(State::END_DETECTED, "Persistent terminal wall in the test corridor");
            publish_stop(State::FINISHED, "Test corridor end confirmed; Scout is stopped");
        }
        else {
            publish_stop(State::VERIFYING_END, "Stopping and confirming the terminal wall");
        }
        return;
    }

    const auto command = tracking_command(elapsed);
    if(!std::isfinite(command.linear.x) || !std::isfinite(command.angular.z)) {
        publish_stop(State::WAITING_INPUTS, "Navigation produced an invalid command");
        return;
    }
    previous_command_ = command;
    command_publisher_->publish(command);
    report_activity(State::FOLLOWING, "Fuzzy corridor following requested");
}


// Missing walls require perception's bounded prediction; unavailable numbers
// never become a distant wall or a zero tracking error:
bool CorridorFollower::observation_is_usable() const
{
    if(!observation_) return false;
    const auto & observation = *observation_;
    const double age = now().seconds() - rclcpp::Time(observation.header.stamp).seconds();
    const double arrival_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - observation_arrival_
    ).count();
    if(age < -0.05 || age > observation_timeout_ || arrival_age > observation_timeout_ ||
        observation.header.frame_id != robot_ + "/base_footprint" || !observation.input_valid ||
        !observation.axis_valid || !observation.width_valid || !observation.front_clearance_valid ||
        !(observation.left_wall_valid || observation.left_predicted) ||
        !(observation.right_wall_valid || observation.right_predicted)) {
        return false;
    }
    for(double value : {
        observation.width, observation.lateral_error, observation.heading_error,
        observation.confidence, observation.front_clearance,
        observation.left_wall_distance, observation.right_wall_distance
    }) {
        if(!std::isfinite(value)) return false;
    }
    return observation.width > 2 * min_wall_distance_ &&
        observation.left_wall_distance > min_wall_distance_ &&
        observation.right_wall_distance > min_wall_distance_ &&
        std::abs(observation.lateral_error) <= max_lateral_error_ &&
        std::abs(observation.heading_error) <= max_heading_error_ &&
        observation.confidence >= min_confidence_ && observation.confidence <= 1.0 &&
        observation.front_clearance >= 0.0;
}


// Check both acquisition and receipt ages, including at standstill:
bool CorridorFollower::odometry_is_fresh() const
{
    if(!odometry_ || odometry_->header.frame_id != robot_ + "/odom" ||
        odometry_->child_frame_id != robot_ + "/base_footprint") {
        return false;
    }
    const double age = now().seconds() - rclcpp::Time(odometry_->header.stamp).seconds();
    const double arrival_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - odom_arrival_
    ).count();
    const auto & velocity = odometry_->twist.twist;
    return age >= -0.05 && age <= odom_timeout_ && arrival_age <= odom_timeout_ &&
        std::isfinite(velocity.linear.x) && std::isfinite(velocity.angular.z);
}


// A far visible wall cannot finish the run: evidence starts at the stopping distance:
void CorridorFollower::update_terminal_evidence()
{
    if(!enabled_ || finished_ || !observation_is_usable() ||
        !observation_->terminal_wall_candidate ||
        observation_->front_clearance > front_stop_clearance_ +
            (state_ == State::VERIFYING_END ? terminal_clearance_tolerance_ : 0.0)) {
        reset_terminal_evidence();
        return;
    }
    if(terminal_observations_ == 0) terminal_started_ = previous_observation_time_;
    ++terminal_observations_;
}


void CorridorFollower::reset_terminal_evidence()
{
    terminal_started_ = -1.0;
    terminal_observations_ = 0;
}


// Use the paired fuzzy yaw rate; speed factors and normal ramps remain navigation decisions:
geometry_msgs::msg::Twist CorridorFollower::tracking_command(double elapsed) const
{
    geometry_msgs::msg::Twist command;
    const auto & observation = *observation_;
    const double angular = steering_angular_speed_;

    // sqrt(distance fraction) slows continuously but reaches the stopping distance
    // in finite time; a linear distance factor would approach it asymptotically:
    const double front_factor = std::sqrt(std::clamp(
        (observation.front_clearance - front_stop_clearance_) /
        (front_slow_clearance_ - front_stop_clearance_), 0.0, 1.0
    ));
    const double heading_factor = 1.0 - 0.6 * std::min(
        1.0, std::abs(observation.heading_error) / heading_error_scale_
    );
    const double lateral_factor = 1.0 - 0.5 * std::min(
        1.0, std::abs(observation.lateral_error) / lateral_error_scale_
    );
    const double confidence_factor = std::min(1.0, observation.confidence / 0.5);
    const double turn_factor = 1.0 - 0.6 * std::abs(angular) / max_angular_speed_;
    const double linear = linear_speed_ * std::min({
        front_factor, heading_factor, lateral_factor, confidence_factor, turn_factor
    });

    command.linear.x = std::clamp(linear,
        previous_command_.linear.x - linear_acceleration_ * elapsed,
        previous_command_.linear.x + linear_acceleration_ * elapsed);
    command.angular.z = std::clamp(angular,
        previous_command_.angular.z - angular_acceleration_ * elapsed,
        previous_command_.angular.z + angular_acceleration_ * elapsed);
    return command;
}


// Faults and explicit stops bypass normal tracking ramps:
void CorridorFollower::publish_stop(State state, const std::string & detail)
{
    previous_command_ = geometry_msgs::msg::Twist();
    command_publisher_->publish(previous_command_);
    report_activity(state, detail);
}


// Report transitions only; never change the textual detail with every numeric sample:
void CorridorFollower::report_activity(State state, const std::string & detail)
{
    if(state == state_ && detail == activity_detail_) return;
    state_ = state;
    activity_detail_ = detail;
    squad_interfaces::msg::RobotEvent event;
    event.header.stamp = now();
    event.robot = robot_;
    switch(state) {
        case State::STOPPED:
            event.phase = "STOPPED";
            break;
        case State::WAITING_INPUTS:
            event.phase = "WAITING_GEOMETRY";
            break;
        case State::FOLLOWING:
            event.phase = "CORRIDOR_FOLLOWING";
            break;
        case State::VERIFYING_END:
            event.phase = "VERIFYING_END";
            break;
        case State::END_DETECTED:
            event.phase = "CORRIDOR_END_DETECTED";
            break;
        case State::BLOCKED:
            event.phase = "BLOCKED";
            break;
        case State::FINISHED:
            event.phase = "FINISHED";
            break;
    }
    event.detail = detail;
    event_publisher_->publish(event);
    RCLCPP_INFO(get_logger(), "%s: %s", event.phase.c_str(), detail.c_str());
}


// Serialize callbacks so observations, service calls and state updates cannot race:
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorFollower>());
    rclcpp::shutdown();
    return 0;
}
