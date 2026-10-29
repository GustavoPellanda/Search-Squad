#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
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
    void validate_parameters() const;
    void initialize_communication();
    void command_callback(geometry_msgs::msg::Twist::ConstSharedPtr message);
    bool is_command_valid(const geometry_msgs::msg::Twist & command) const;
    void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr message);
    bool is_scan_valid(const sensor_msgs::msg::LaserScan & scan) const;
    void control_loop();
    bool inputs_are_ready();
    bool inputs_are_fresh(double ros_time, std::chrono::steady_clock::time_point wall_time) const;
    bool scan_transforms_are_available();
    geometry_msgs::msg::Twist limit_command(const geometry_msgs::msg::Twist & command) const;

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

    // Invalid messages replace previous validity, rather than retaining usable motion:
    geometry_msgs::msg::Twist latest_command_;
    bool command_valid_ = false;
    bool scan_valid_ = false;
    double scan_timestamp_ = 0.0;
    double command_ros_time_ = 0.0;
    std::chrono::steady_clock::time_point command_arrival_{};
    std::chrono::steady_clock::time_point scan_arrival_{};

    // Listener initialization depends on the buffer:
    tf2_ros::Buffer transform_buffer_;
    tf2_ros::TransformListener transform_listener_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
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
        command_source_ == "reactive" || command_source_ == "nav";
    if(!known_source) {
        throw std::invalid_argument("source must be hold, test, reactive or nav");
    }
}


// Connect only the selected source; hold deliberately has no command subscription:
void CommandGate::initialize_communication()
{
    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_guard_input",
        1
    );
    if(command_source_ != "hold") {
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
    if(inputs_are_ready()) {
        output = limit_command(latest_command_);
    }
    command_publisher_->publish(output);
}


// Check availability before freshness, and freshness before querying scan-time TF:
bool CommandGate::inputs_are_ready()
{
    const auto wall_time = std::chrono::steady_clock::now();
    const double ros_time = now().seconds();
    if(command_source_ == "hold") return false;
    if(!command_valid_ || !scan_valid_) return false;
    if(ros_time <= 0.0) return false;
    if(!inputs_are_fresh(ros_time, wall_time)) return false;
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
    const rclcpp::Time scan_time(
        static_cast<int64_t>(scan_timestamp_ * NANOSECONDS_PER_SECOND),
        RCL_ROS_TIME
    );
    const auto no_wait = rclcpp::Duration::from_seconds(0.0);
    const bool sensor_to_base_available = transform_buffer_.canTransform(
        base_frame_,
        scan_frame_,
        scan_time,
        no_wait
    );
    if(!sensor_to_base_available) return false;

    return transform_buffer_.canTransform(
        odom_frame_,
        base_frame_,
        scan_time,
        no_wait
    );
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
