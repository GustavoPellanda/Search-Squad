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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/*
Reduce the local laser scan to front/left/right clearances and choose a bounded
forward or turning command. Output goes to cmd_vel_reactive, before protection.
The simulated duration is a demonstration budget, not a room-coverage criterion.
*/
class ReactiveDemo : public rclcpp::Node
{
    public:

    ReactiveDemo();

    private:

    enum class RunState { NOT_STARTED, RUNNING, FINISHED };
    enum class MotionState { FORWARD, TURNING };

    void read_parameters();
    void validate_parameters() const;
    void initialize_communication();
    void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr message);
    bool is_scan_metadata_valid(const sensor_msgs::msg::LaserScan & scan) const;
    bool update_sector_clearances(const sensor_msgs::msg::LaserScan & scan);
    void control_loop();
    bool scan_is_fresh(double ros_time) const;
    void update_run_state(double ros_time);
    geometry_msgs::msg::Twist calculate_velocity();
    void report_state(const std::string & state);

    // Existing sensor contract, sector boundaries and wall-timer period:
    static constexpr std::size_t MIN_SCAN_RAYS = 180;
    static constexpr double MIN_SCAN_COVERAGE_RAD = 6.0;
    static constexpr double FRONT_HALF_ANGLE_RAD = 0.45;
    static constexpr double SIDE_OUTER_ANGLE_RAD = 1.75;
    static constexpr double MIN_FORWARD_SPEED_RATIO = 0.15;
    static constexpr double FUTURE_SCAN_TOLERANCE_SECONDS = 0.05;
    static constexpr std::uint32_t NANOSECONDS_PER_SECOND = 1000000000;
    static constexpr std::chrono::milliseconds CONTROL_PERIOD{50};

    // ROS parameters: seconds, m/s, rad/s and distances in metres:
    double duration_ = 0.0;
    double forward_speed_ = 0.0;
    double turn_speed_ = 0.0;
    double stop_distance_ = 0.0;
    double slow_distance_ = 0.0;
    double release_distance_ = 0.0;
    double side_clearance_ = 0.0;
    double scan_timeout_ = 0.0;
    std::string expected_scan_frame_;

    // Latest scan, expressed in the robot's laser frame (X forward, Y left):
    double front_clearance_ = 0.0;
    double left_clearance_ = 0.0;
    double right_clearance_ = 0.0;
    bool scan_valid_ = false;
    double scan_timestamp_ = 0.0;
    std::chrono::steady_clock::time_point scan_arrival_{};

    RunState run_state_ = RunState::NOT_STARTED;
    MotionState motion_state_ = MotionState::FORWARD;
    double start_ros_time_ = 0.0;
    double turn_direction_ = 1.0;
    std::string reported_state_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};


// Configure the demonstration before connecting its scan and command interfaces:
ReactiveDemo::ReactiveDemo() : Node("reactive_demo")
{
    read_parameters();
    validate_parameters();
    initialize_communication();
}


// Read the existing experiment parameters and namespace-qualified laser frame:
void ReactiveDemo::read_parameters()
{
    duration_ = declare_parameter("duration", 30.0);
    forward_speed_ = declare_parameter("forward_speed", 0.12);
    turn_speed_ = declare_parameter("turn_speed", 0.55);
    stop_distance_ = declare_parameter("stop_distance", 0.50);
    slow_distance_ = declare_parameter("slow_distance", 1.0);
    release_distance_ = declare_parameter("release_distance", 0.75);
    side_clearance_ = declare_parameter("side_clearance", 0.35);
    scan_timeout_ = declare_parameter("scan_timeout", 0.6);
    expected_scan_frame_ = std::string(get_namespace()).substr(1) + "/base_scan";
}


// Require positive values and ordered thresholds for slowing, turning and release:
void ReactiveDemo::validate_parameters() const
{
    for(const std::string name : {
        "duration", "forward_speed", "turn_speed", "stop_distance", "slow_distance",
        "release_distance", "side_clearance", "scan_timeout"
    }) {
        const double configured_value = get_parameter(name).as_double();
        if(!std::isfinite(configured_value) || configured_value <= 0.0) {
            throw std::invalid_argument(name + " must be finite and positive");
        }
    }

    const bool ordered_distances =
        stop_distance_ < release_distance_ && release_distance_ < slow_distance_;
    if(!ordered_distances) {
        throw std::invalid_argument("Require stop_distance < release_distance < slow_distance");
    }
}


// Publish upstream of protection and receive best-effort laser measurements:
void ReactiveDemo::initialize_communication()
{
    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel_reactive",
        1
    );
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS(),
        std::bind(&ReactiveDemo::scan_callback, this, std::placeholders::_1)
    );
    control_timer_ = create_wall_timer(
        CONTROL_PERIOD,
        std::bind(&ReactiveDemo::control_loop, this)
    );
}


// Replace scan validity, sector clearances and both receipt timestamps on every message:
void ReactiveDemo::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
    // Validate the wire timestamp before passing it to rclcpp::Time:
    const auto & timestamp = message->header.stamp;
    const bool timestamp_valid =
        timestamp.sec >= 0 && timestamp.nanosec < NANOSECONDS_PER_SECOND;
    const bool metadata_valid = is_scan_metadata_valid(*message);
    const bool sectors_valid = update_sector_clearances(*message);
    scan_valid_ = timestamp_valid && metadata_valid && sectors_valid;

    scan_timestamp_ = timestamp_valid ? rclcpp::Time(timestamp).seconds() : 0.0;
    scan_arrival_ = std::chrono::steady_clock::now();
}


// Check frame, angular coverage and physical range limits before using a scan for motion:
bool ReactiveDemo::is_scan_metadata_valid(const sensor_msgs::msg::LaserScan & scan) const
{
    if(scan.header.frame_id != expected_scan_frame_) return false;
    if(scan.ranges.size() < MIN_SCAN_RAYS) return false;

    const bool finite_angles =
        std::isfinite(scan.angle_min) && std::isfinite(scan.angle_increment);
    if(!finite_angles || scan.angle_increment <= 0.0) return false;

    const double coverage_rad = scan.angle_increment * scan.ranges.size();
    if(coverage_rad < MIN_SCAN_COVERAGE_RAD) return false;

    const bool finite_bounds = std::isfinite(scan.range_min) && std::isfinite(scan.range_max);
    if(!finite_bounds) return false;
    return scan.range_min > 0.0 && scan.range_max > scan.range_min;
}


// Reduce valid rays to sector minima; keep invalidity even if other rays are usable:
bool ReactiveDemo::update_sector_clearances(const sensor_msgs::msg::LaserScan & scan)
{
    front_clearance_ = scan.range_max;
    left_clearance_ = scan.range_max;
    right_clearance_ = scan.range_max;
    int front_count = 0;
    int left_count = 0;
    int right_count = 0;
    bool all_ranges_valid = true;
    const double full_turn_rad = 2.0 * std::acos(-1.0);

    for(std::size_t beam_index = 0; beam_index < scan.ranges.size(); ++beam_index) {
        const double range_m = scan.ranges[beam_index];
        const bool no_return = std::isinf(range_m) && range_m > 0.0;
        const bool measured_in_range =
            std::isfinite(range_m) && range_m >= scan.range_min && range_m <= scan.range_max;
        if(!no_return && !measured_in_range) {
            all_ranges_valid = false;
            continue;
        }

        // Normalize the laser bearing around forward; +Inf clears only to range_max:
        const double beam_angle_rad = scan.angle_min + beam_index * scan.angle_increment;
        const double bearing_rad = std::remainder(beam_angle_rad, full_turn_rad);
        const double distance_m = no_return ? scan.range_max : range_m;
        if(std::abs(bearing_rad) <= FRONT_HALF_ANGLE_RAD) {
            front_clearance_ = std::min(front_clearance_, distance_m);
            ++front_count;
        }
        if(bearing_rad > FRONT_HALF_ANGLE_RAD && bearing_rad < SIDE_OUTER_ANGLE_RAD) {
            left_clearance_ = std::min(left_clearance_, distance_m);
            ++left_count;
        }
        if(bearing_rad < -FRONT_HALF_ANGLE_RAD && bearing_rad > -SIDE_OUTER_ANGLE_RAD) {
            right_clearance_ = std::min(right_clearance_, distance_m);
            ++right_count;
        }
    }

    const bool every_sector_observed = front_count > 0 && left_count > 0 && right_count > 0;
    return all_ranges_valid && every_sector_observed;
}


// Enforce the duration and scan watchdog before calculating and publishing motion:
void ReactiveDemo::control_loop()
{
    geometry_msgs::msg::Twist command;
    const double ros_time = now().seconds();
    const bool fresh_scan = scan_is_fresh(ros_time);
    update_run_state(ros_time);

    if(run_state_ == RunState::FINISHED) {
        report_state("FINISHED");
    }
    else if(!fresh_scan) {
        report_state("WAITING_FOR_SCAN");
    }
    else {
        if(run_state_ == RunState::NOT_STARTED) {
            run_state_ = RunState::RUNNING;
            start_ros_time_ = ros_time;
        }
        command = calculate_velocity();
    }
    command_publisher_->publish(command);
}


// Check ROS time and monotonic arrival age to detect scan loss even if ROS time stalls:
bool ReactiveDemo::scan_is_fresh(double ros_time) const
{
    if(!scan_valid_ || ros_time <= 0.0) return false;

    const double scan_ros_age = ros_time - scan_timestamp_;
    const bool timestamp_valid =
        scan_ros_age >= -FUTURE_SCAN_TOLERANCE_SECONDS && scan_ros_age <= scan_timeout_;
    const double scan_wall_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - scan_arrival_
    ).count();
    return timestamp_valid && scan_wall_age <= scan_timeout_;
}


// Once started, consume the same budget through scan loss and finish on clock rollback:
void ReactiveDemo::update_run_state(double ros_time)
{
    if(run_state_ == RunState::NOT_STARTED) return;

    const bool clock_rolled_back = ros_time < start_ros_time_;
    const bool duration_expired = ros_time - start_ros_time_ >= duration_;
    if(clock_rolled_back || duration_expired) {
        run_state_ = RunState::FINISHED;
    }
}


// Keep a chosen turn until the front clears; otherwise scale forward speed by clearance:
geometry_msgs::msg::Twist ReactiveDemo::calculate_velocity()
{
    if(motion_state_ == MotionState::FORWARD && front_clearance_ <= stop_distance_) {
        motion_state_ = MotionState::TURNING;
        turn_direction_ = left_clearance_ >= right_clearance_ ? 1.0 : -1.0;
    }
    if(motion_state_ == MotionState::TURNING && front_clearance_ >= release_distance_) {
        motion_state_ = MotionState::FORWARD;
    }

    geometry_msgs::msg::Twist command;
    if(motion_state_ == MotionState::TURNING) {
        // Preserve turn direction through blocked scans to avoid left/right chatter:
        const double chosen_side_clearance =
            turn_direction_ > 0.0 ? left_clearance_ : right_clearance_;
        if(chosen_side_clearance > side_clearance_) {
            command.angular.z = turn_direction_ * turn_speed_;
            report_state("TURNING");
        }
        else {
            report_state("BLOCKED");
        }
    }
    else {
        const double clearance_above_stop = front_clearance_ - stop_distance_;
        const double slowdown_interval = slow_distance_ - stop_distance_;
        const double speed_ratio = std::clamp(
            clearance_above_stop / slowdown_interval,
            MIN_FORWARD_SPEED_RATIO,
            1.0
        );
        command.linear.x = forward_speed_ * speed_ratio;
        report_state(front_clearance_ < slow_distance_ ? "SLOWING" : "FORWARD");
    }
    return command;
}


// Log transitions only, retaining the measured clearances that explain each decision:
void ReactiveDemo::report_state(const std::string & state)
{
    if(state == reported_state_) return;

    RCLCPP_INFO(
        get_logger(),
        "%s: front=%.2f left=%.2f right=%.2f",
        state.c_str(),
        front_clearance_,
        left_clearance_,
        right_clearance_
    );
    reported_state_ = state;
}


// Spin the node with the existing single-threaded ROS execution model:
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveDemo>());
    rclcpp::shutdown();
    return 0;
}
