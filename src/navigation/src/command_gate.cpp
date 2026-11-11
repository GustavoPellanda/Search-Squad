#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <lidar_safety/safety_evaluator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <squad_interfaces/msg/control_lease.hpp>
#include <squad_interfaces/msg/robot_event.hpp>
#include <squad_interfaces/msg/safety_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/*
Forward the selected planar command to the Collision Monitor only while commands,
scans and the scan-time TF chain are valid. The monitor owns the final cmd_vel.
The wall timer and monotonic watchdogs detect input loss even when ROS time stalls.
*/
class CommandGate : public rclcpp::Node
{
    public:

    CommandGate();

    private:

    void read_parameters();
    void configure_safety();
    void validate_parameters() const;
    void initialize_communication();
    void command_callback(geometry_msgs::msg::Twist::ConstSharedPtr message);
    void managed_command_callback(
        geometry_msgs::msg::Twist::ConstSharedPtr message, const std::string & source
    );
    void lease_callback(squad_interfaces::msg::ControlLease::ConstSharedPtr message);
    void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr message);
    bool is_command_valid(const geometry_msgs::msg::Twist & command) const;
    void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr message);
    bool is_scan_valid(const sensor_msgs::msg::LaserScan & scan) const;
    void control_loop();
    bool inputs_are_ready();
    bool inputs_are_fresh(double ros_time, std::chrono::steady_clock::time_point wall_time) const;
    bool scan_transforms_are_available();
    geometry_msgs::msg::Twist limit_command(const geometry_msgs::msg::Twist & command) const;
    void publish_safety(const geometry_msgs::msg::Twist & requested,
        const geometry_msgs::msg::Twist & allowed, const lidar_safety::Decision & decision);
    std::vector<lidar_safety::Point> obstacle_points() const;

    // Fixed sensor contract and existing timing tolerances:
    static constexpr std::size_t MIN_SCAN_RAYS = 180;
    static constexpr double MIN_SCAN_COVERAGE_RAD = 6.0;
    static constexpr double FUTURE_SCAN_TOLERANCE_SECONDS = 0.05;
    static constexpr std::uint32_t NANOSECONDS_PER_SECOND = 1000000000;
    static constexpr std::chrono::milliseconds CONTROL_PERIOD{50};

    // Startup-only configuration; speeds are in m/s and rad/s, timeouts in seconds:
    std::string command_source_;
    std::string scan_frame_;
    std::string base_frame_;
    std::string odom_frame_;
    double command_timeout_ = 0.0;
    double scan_timeout_ = 0.0;
    double max_linear_speed_ = 0.0;
    double max_angular_speed_ = 0.0;
    double odom_timeout_ = 0.5;
    std::unique_ptr<lidar_safety::SafetyEvaluator> safety_;
    std::string readiness_reason_;
    std::string reported_safety_;
    bool forward_allowed_ = false;
    lidar_safety::Velocity measured_velocity_;
    bool odom_valid_ = false;
    double odom_timestamp_ = 0.0;
    std::chrono::steady_clock::time_point odom_arrival_{};
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
    geometry_msgs::msg::TransformStamped scan_to_current_base_;

    // Invalid messages replace previous validity, rather than retaining usable motion:
    geometry_msgs::msg::Twist latest_command_;
    bool command_valid_ = false;
    bool scan_valid_ = false;
    double scan_timestamp_ = 0.0;
    double command_ros_time_ = 0.0;
    std::chrono::steady_clock::time_point command_arrival_{};
    std::chrono::steady_clock::time_point scan_arrival_{};

    bool managed_control_ = false;
    bool lease_valid_ = false;
    std::uint64_t generation_ = 0;
    std::chrono::steady_clock::time_point lease_arrival_{};
    rclcpp::Subscription<squad_interfaces::msg::ControlLease>::SharedPtr lease_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr reactive_subscription_;

    // Listener initialization depends on the buffer:
    tf2_ros::Buffer transform_buffer_;
    tf2_ros::TransformListener transform_listener_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
    rclcpp::Publisher<squad_interfaces::msg::SafetyState>::SharedPtr safety_publisher_;
    rclcpp::Publisher<squad_interfaces::msg::RobotEvent>::SharedPtr event_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};


// Configure the fixed source and connect the gate without starting a control workflow:
CommandGate::CommandGate()
    : Node("command_gate"),
      transform_buffer_(get_clock()),
      transform_listener_(transform_buffer_)
{
    read_parameters();
    validate_parameters();
    configure_safety();
    managed_control_ = command_source_ == "managed";
    if(managed_control_) command_source_ = "hold";
    initialize_communication();

    RCLCPP_INFO(
        get_logger(),
        "Selected source: %s; final output belongs to Collision Monitor",
        command_source_.c_str()
    );
}


// Keep source selection and protection settings read-only, as in the initial stage:
void CommandGate::read_parameters()
{
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    command_source_ = declare_parameter("source", "hold", fixed);
    scan_frame_ = declare_parameter("scan_frame", "base_scan", fixed);
    base_frame_ = declare_parameter("base_frame", "base_footprint", fixed);
    odom_frame_ = declare_parameter("odom_frame", "odom", fixed);
    command_timeout_ = declare_parameter("command_timeout", 0.35, fixed);
    scan_timeout_ = declare_parameter("scan_timeout", 0.6, fixed);
    max_linear_speed_ = declare_parameter("max_linear_speed", 0.15, fixed);
    max_angular_speed_ = declare_parameter("max_angular_speed", 0.8, fixed);
}


// Keep the footprint and braking assumptions in the installed lidar_safety configuration:
void CommandGate::configure_safety()
{
    lidar_safety::Configuration config;
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    config.front = declare_parameter("safety.footprint_front", config.front, fixed);
    config.rear = declare_parameter("safety.footprint_rear", config.rear, fixed);
    config.half_width = declare_parameter("safety.footprint_half_width", config.half_width, fixed);
    config.stop_margin = declare_parameter("safety.stop_margin", config.stop_margin, fixed);
    config.slow_margin = declare_parameter("safety.slow_margin", config.slow_margin, fixed);
    config.slowdown_ratio = declare_parameter(
        "safety.slowdown_ratio", config.slowdown_ratio, fixed);
    config.linear_deceleration = declare_parameter(
        "safety.linear_deceleration", config.linear_deceleration, fixed);
    config.angular_deceleration = declare_parameter(
        "safety.angular_deceleration", config.angular_deceleration, fixed);
    config.latency = declare_parameter("safety.latency", config.latency, fixed);
    config.step = declare_parameter("safety.prediction_step", config.step, fixed);
    odom_timeout_ = declare_parameter("safety.odom_timeout", 0.5, fixed);
    if(!std::isfinite(odom_timeout_) || odom_timeout_ <= 0.0 || odom_timeout_ > 1.0) {
        throw std::invalid_argument("safety.odom_timeout must be in (0, 1] seconds");
    }
    safety_ = std::make_unique<lidar_safety::SafetyEvaluator>(config);
}


// Reject unsupported sources and non-positive or non-finite protection limits:
void CommandGate::validate_parameters() const
{
    for(const std::string name : {
        "command_timeout", "scan_timeout", "max_linear_speed", "max_angular_speed"
    }) {
        const double configured_value = get_parameter(name).as_double();
        if(!std::isfinite(configured_value) || configured_value <= 0.0) {
            throw std::invalid_argument(name + " must be finite and positive");
        }
    }

    const bool known_source =
        command_source_ == "hold" || command_source_ == "test" ||
        command_source_ == "reactive" || command_source_ == "nav" ||
        command_source_ == "managed" || command_source_ == "corridor";
    if(!known_source) {
        throw std::invalid_argument("source must be hold, test, reactive, nav, managed or corridor");
    }
}


// Connect only the selected source; hold deliberately has no command subscription:
void CommandGate::initialize_communication()
{
    safety_publisher_ = create_publisher<squad_interfaces::msg::SafetyState>(
        "safety_state", rclcpp::QoS(1).reliable().transient_local());
    event_publisher_ = create_publisher<squad_interfaces::msg::RobotEvent>("/fleet/events", 20);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&CommandGate::odom_callback, this, std::placeholders::_1));
    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_guard_input",
        1
    );
    if(managed_control_) {
        lease_subscription_ = create_subscription<squad_interfaces::msg::ControlLease>(
            "control_lease", 1, std::bind(&CommandGate::lease_callback, this, std::placeholders::_1)
        );
        nav_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_nav", 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
                managed_command_callback(message, "nav");
            }
        );
        reactive_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_reactive", 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
                managed_command_callback(message, "reactive");
            }
        );
    }
    else if(command_source_ != "hold") {
        command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_" + command_source_,
            1,
            std::bind(&CommandGate::command_callback, this, std::placeholders::_1)
        );
    }

    // SensorDataQoS matches the best-effort Gazebo laser publisher:
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS(),
        std::bind(&CommandGate::scan_callback, this, std::placeholders::_1)
    );
    control_timer_ = create_wall_timer(
        CONTROL_PERIOD,
        std::bind(&CommandGate::control_loop, this)
    );
}


// Replace the selected command and record receipt in both clock domains:
void CommandGate::command_callback(geometry_msgs::msg::Twist::ConstSharedPtr message)
{
    command_valid_ = is_command_valid(*message);
    latest_command_ = *message;
    command_arrival_ = std::chrono::steady_clock::now();
    command_ros_time_ = now().seconds();
}


// Ignore every command whose source is not currently authorized:
void CommandGate::managed_command_callback(
    geometry_msgs::msg::Twist::ConstSharedPtr message, const std::string & source
)
{
    if(source == command_source_ && lease_valid_) command_callback(message);
}


// A new lease generation invalidates cached velocities before any new source can move:
void CommandGate::lease_callback(squad_interfaces::msg::ControlLease::ConstSharedPtr message)
{
    const bool supported = message->source == "hold" || message->source == "nav" ||
        message->source == "reactive";
    if(!supported || message->header.stamp.sec < 0 ||
        message->header.stamp.nanosec >= NANOSECONDS_PER_SECOND) {
        lease_valid_ = false;
        command_valid_ = false;
        return;
    }
    const double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
    lease_valid_ = age >= -0.05 && age < command_timeout_;
    if(!lease_valid_) return;
    if(generation_ != message->generation || command_source_ != message->source) {
        command_valid_ = false;
        generation_ = message->generation;
        command_source_ = message->source;
    }
    lease_arrival_ = std::chrono::steady_clock::now();
}


// Accept finite forward/yaw velocities and require all other axes to be zero:
bool CommandGate::is_command_valid(const geometry_msgs::msg::Twist & command) const
{
    const bool finite_planar_velocities =
        std::isfinite(command.linear.x) && std::isfinite(command.angular.z);
    const bool no_out_of_plane_motion =
        command.linear.y == 0.0 && command.linear.z == 0.0 &&
        command.angular.x == 0.0 && command.angular.y == 0.0;
    return finite_planar_velocities && no_out_of_plane_motion;
}


// Reject malformed timestamps before conversion and replace the previous scan validity:
void CommandGate::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
    // ROS Time requires normalized nanoseconds; rclcpp rejects negative seconds:
    const auto & timestamp = message->header.stamp;
    const bool timestamp_valid =
        timestamp.sec >= 0 && timestamp.nanosec < NANOSECONDS_PER_SECOND;
    scan_valid_ = timestamp_valid && is_scan_valid(*message);
    scan_timestamp_ = timestamp_valid ? rclcpp::Time(timestamp).seconds() : 0.0;
    scan_arrival_ = std::chrono::steady_clock::now();
    scan_ = message;
}


// A command reversal does not instantly remove the robot's measured momentum:
void CommandGate::odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
    const auto & stamp = message->header.stamp;
    const auto & velocity = message->twist.twist;
    odom_valid_ = stamp.sec >= 0 && stamp.nanosec < NANOSECONDS_PER_SECOND &&
        message->header.frame_id == odom_frame_ && message->child_frame_id == base_frame_ &&
        is_command_valid(velocity) && std::abs(velocity.linear.x) <= 2 * max_linear_speed_ &&
        std::abs(velocity.angular.z) <= 2 * max_angular_speed_;
    measured_velocity_ = {velocity.linear.x, velocity.angular.z};
    odom_timestamp_ = odom_valid_ ? rclcpp::Time(stamp).seconds() : 0.0;
    odom_arrival_ = std::chrono::steady_clock::now();
}


// Validate the sensor frame, near-full-circle coverage, range bounds and every ray:
bool CommandGate::is_scan_valid(const sensor_msgs::msg::LaserScan & scan) const
{
    if(scan.header.frame_id != scan_frame_) return false;
    if(scan.ranges.size() < MIN_SCAN_RAYS) return false;

    const bool finite_angles =
        std::isfinite(scan.angle_min) && std::isfinite(scan.angle_increment);
    if(!finite_angles || scan.angle_increment <= 0.0) return false;

    const double coverage_rad = scan.angle_increment * scan.ranges.size();
    if(coverage_rad < MIN_SCAN_COVERAGE_RAD) return false;

    const bool finite_bounds = std::isfinite(scan.range_min) && std::isfinite(scan.range_max);
    if(!finite_bounds || scan.range_min <= 0.0 || scan.range_max <= scan.range_min) return false;

    // +Inf means no return. NaN, -Inf and finite out-of-range rays stop motion:
    for(const float range_m : scan.ranges) {
        const bool no_return = std::isinf(range_m) && range_m > 0.0;
        const bool measured_in_range =
            std::isfinite(range_m) && range_m >= scan.range_min && range_m <= scan.range_max;
        if(!no_return && !measured_in_range) return false;
    }
    return true;
}


// Publish exactly one command per wall tick, with zero as the unavailable-input output:
void CommandGate::control_loop()
{
    geometry_msgs::msg::Twist output;
    const auto requested = limit_command(latest_command_);
    lidar_safety::Decision decision;
    decision.state = "STOP";
    forward_allowed_ = false;
    if(inputs_are_ready()) {
        const auto points = obstacle_points();
        const double age = std::max(0.0, now().seconds() - scan_timestamp_);
        decision = safety_->evaluate(points,
            {requested.linear.x, requested.angular.z}, measured_velocity_,
            age);
        forward_allowed_ = safety_->evaluate(points, {max_linear_speed_, 0.0},
            measured_velocity_, age).state != "STOP";
        output.linear.x = decision.output.linear;
        output.angular.z = decision.output.angular;
    }
    else {
        decision.reason = readiness_reason_;
        if(command_source_ == "hold") decision.state = "HOLD";
    }
    command_publisher_->publish(output);
    publish_safety(requested, output, decision);
}


// Check availability before freshness, and freshness before querying scan-time TF:
bool CommandGate::inputs_are_ready()
{
    const auto wall_time = std::chrono::steady_clock::now();
    const double ros_time = now().seconds();
    if(managed_control_) {
        const double lease_age = std::chrono::duration<double>(wall_time - lease_arrival_).count();
        if(!lease_valid_ || lease_age > command_timeout_) {
            readiness_reason_ = "SUPERVISOR_STALE";
            return false;
        }
    }
    readiness_reason_ = "HOLD";
    if(command_source_ == "hold") return false;
    readiness_reason_ = "SCAN_INVALID";
    if(!scan_valid_) return false;
    const double scan_age = ros_time - scan_timestamp_;
    readiness_reason_ = "SCAN_STALE";
    if(scan_age < -FUTURE_SCAN_TOLERANCE_SECONDS || scan_age > scan_timeout_ ||
        std::chrono::duration<double>(wall_time - scan_arrival_).count() > scan_timeout_) {
        return false;
    }
    readiness_reason_ = "COMMAND_INVALID_OR_STALE";
    if(!command_valid_ || ros_time <= 0.0 || !inputs_are_fresh(ros_time, wall_time)) return false;
    readiness_reason_ = "ODOM_INVALID_OR_STALE";
    const double odom_age = ros_time - odom_timestamp_;
    if(!odom_valid_ || odom_age < -0.05 || odom_age > odom_timeout_ ||
        std::chrono::duration<double>(wall_time - odom_arrival_).count() > odom_timeout_) {
        return false;
    }
    readiness_reason_ = "TF_UNAVAILABLE";
    return scan_transforms_are_available();
}


// Require both recent arrival and consistent ROS time, including clock-reset rejection:
bool CommandGate::inputs_are_fresh(
    double ros_time,
    std::chrono::steady_clock::time_point wall_time
) const
{
    const double command_wall_age =
        std::chrono::duration<double>(wall_time - command_arrival_).count();
    const double scan_wall_age =
        std::chrono::duration<double>(wall_time - scan_arrival_).count();
    if(command_wall_age > command_timeout_ || scan_wall_age > scan_timeout_) return false;

    const double scan_ros_age = ros_time - scan_timestamp_;
    const bool scan_time_valid =
        scan_ros_age >= -FUTURE_SCAN_TOLERANCE_SECONDS && scan_ros_age <= scan_timeout_;
    if(!scan_time_valid) return false;

    const double command_ros_age = ros_time - command_ros_time_;
    return command_ros_age >= 0.0 && command_ros_age <= command_timeout_;
}


// Require sensor-to-base and base-to-local-odom TF at the measurement time, without waiting:
bool CommandGate::scan_transforms_are_available()
{
    // Humble's monitor can ignore an unavailable source; enforce its required chain here:
    const rclcpp::Time scan_time(scan_->header.stamp);
    const auto no_wait = rclcpp::Duration::from_seconds(0.0);
    const bool sensor_to_base_available = transform_buffer_.canTransform(
        base_frame_,
        scan_frame_,
        scan_time,
        no_wait
    );
    if(!sensor_to_base_available) return false;

    try {
        const auto pose = transform_buffer_.lookupTransform(
            odom_frame_, base_frame_, tf2::TimePointZero);
        const double pose_age = (now() - rclcpp::Time(pose.header.stamp)).seconds();
        if(pose_age < -0.05 || pose_age > odom_timeout_) return false;
        // Compensate robot motion between the scan and the latest measured local pose:
        scan_to_current_base_ = transform_buffer_.lookupTransform(
            base_frame_, rclcpp::Time(pose.header.stamp), scan_frame_, scan_time, odom_frame_);
        return true;
    }
    catch(const tf2::TransformException &) {
        return false;
    }
}


// Ignore valid no-return rays; finite hits remain obstacles even when they are other robots:
std::vector<lidar_safety::Point> CommandGate::obstacle_points() const
{
    std::vector<lidar_safety::Point> points;
    const auto & transform = scan_to_current_base_.transform;
    const auto & q = transform.rotation;
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z));
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    for(std::size_t ray = 0; ray < scan_->ranges.size(); ++ray) {
        const double range = scan_->ranges[ray];
        if(!std::isfinite(range)) continue;
        const double angle = scan_->angle_min + ray * scan_->angle_increment;
        const double x = range * std::cos(angle);
        const double y = range * std::sin(angle);
        points.push_back({transform.translation.x + cosine * x - sine * y,
            transform.translation.y + sine * x + cosine * y});
    }
    return points;
}


// Publish every decision for inspection and send transitions to the central mission log:
void CommandGate::publish_safety(const geometry_msgs::msg::Twist & requested,
    const geometry_msgs::msg::Twist & allowed, const lidar_safety::Decision & decision)
{
    squad_interfaces::msg::SafetyState status;
    status.header.stamp = now();
    status.header.frame_id = base_frame_;
    status.source = command_source_;
    status.state = decision.state;
    status.reason = decision.reason;
    status.requested = requested;
    status.allowed = allowed;
    status.forward_allowed = forward_allowed_;
    status.scan_age = scan_ ? now().seconds() - scan_timestamp_ : -1.0;
    status.nearest_clearance = decision.clearance;
    status.stopping_distance = decision.stopping_distance;
    status.stopping_angle = decision.stopping_angle;
    safety_publisher_->publish(status);
    const std::string key = status.source + status.state + status.reason;
    if(key == reported_safety_) return;
    reported_safety_ = key;
    squad_interfaces::msg::RobotEvent event;
    event.header = status.header;
    event.robot = std::string(get_namespace()).substr(1);
    event.phase = "SAFETY_" + status.state;
    event.detail = status.reason;
    event_publisher_->publish(event);
}


// Saturate only the two supported axes before handing the command to Collision Monitor:
geometry_msgs::msg::Twist CommandGate::limit_command(
    const geometry_msgs::msg::Twist & command
) const
{
    geometry_msgs::msg::Twist limited_command;
    limited_command.linear.x = std::clamp(command.linear.x, -max_linear_speed_, max_linear_speed_);
    limited_command.angular.z = std::clamp(
        command.angular.z,
        -max_angular_speed_,
        max_angular_speed_
    );
    return limited_command;
}


// Spin the node with the existing single-threaded ROS execution model:
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandGate>());
    rclcpp::shutdown();
    return 0;
}
