#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace search_squad
{
/*
Apply final cmd_vel to the Burger wheels with an independent dual-clock timeout.
Integrate joint encoder increments into local planar odometry and publish its TF.
World pose is never used for odometry. The mutex serializes ROS commands with
Gazebo updates; all update helpers run under that same lock.
*/
class TimedDiffDrive : public gazebo::ModelPlugin
{
    public:

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

    private:

    void configure_drive(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
    void initialize_communication();
    void command_callback(geometry_msgs::msg::Twist::ConstSharedPtr message);
    bool is_command_valid(const geometry_msgs::msg::Twist & command) const;
    void world_update_callback(const gazebo::common::UpdateInfo & info);
    void reset_odometry(double simulation_time);
    bool command_is_fresh(double simulation_time) const;
    void apply_wheel_velocities(double elapsed_seconds, bool fresh_command);
    void integrate_encoder_odometry(double left_position_rad, double right_position_rad);
    nav_msgs::msg::Odometry make_odometry(const rclcpp::Time & timestamp) const;
    void publish_state(double simulation_time, double left_position_rad, double right_position_rad);

    // Fixed Burger geometry and the existing actuator limits (SI units):
    static constexpr double WHEEL_RADIUS_M = 0.033;
    static constexpr double WHEEL_SEPARATION_M = 0.160;
    static constexpr double MAX_WHEEL_TORQUE_NM = 20.0;
    static constexpr double MAX_LINEAR_SPEED_M_S = 0.15;
    static constexpr double MAX_ANGULAR_SPEED_RAD_S = 0.8;
    static constexpr double MAX_RIM_ACCELERATION_M_S2 = 1.0;
    static constexpr double PUBLISH_PERIOD_SECONDS = 1.0 / 30.0;
    static constexpr double NANOSECONDS_PER_SECOND = 1e9;
    static constexpr double POSITION_VARIANCE_M2 = 0.001;
    static constexpr double YAW_VARIANCE_RAD2 = 0.01;
    static constexpr double TWIST_VARIANCE = 0.01;
    // Covariance is a row-major 6x6 matrix over x, y, z, roll, pitch, yaw:
    static constexpr std::size_t X_VARIANCE_INDEX = 0;
    static constexpr std::size_t Y_VARIANCE_INDEX = 7;
    static constexpr std::size_t YAW_VARIANCE_INDEX = 35;

    gazebo_ros::Node::SharedPtr node_;
    gazebo::physics::JointPtr left_wheel_;
    gazebo::physics::JointPtr right_wheel_;
    gazebo::event::ConnectionPtr update_connection_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    std::mutex mutex_;
    std::string odom_frame_;
    std::string base_frame_;
    double command_timeout_ = 0.5;

    // Command receipt and the previous Gazebo update, in separate clock domains:
    bool command_received_ = false;
    double simulation_time_ = 0.0;
    double command_simulation_time_ = 0.0;
    double last_publish_time_ = 0.0;
    std::chrono::steady_clock::time_point command_wall_time_{};
    double linear_speed_ = 0.0;
    double angular_speed_ = 0.0;
    double applied_left_speed_ = 0.0;
    double applied_right_speed_ = 0.0;

    // Encoder-integrated pose in this robot's independent odom frame:
    double last_left_position_ = 0.0;
    double last_right_position_ = 0.0;
    double x_in_odom_ = 0.0;
    double y_in_odom_ = 0.0;
    double yaw_in_odom_ = 0.0;
};


// Bind the model and ROS interfaces before registering the Gazebo update callback:
void TimedDiffDrive::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    configure_drive(model, sdf);
    initialize_communication();
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&TimedDiffDrive::world_update_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(
        node_->get_logger(),
        "Encoder drive: final cmd_vel only, timeout %.2f s",
        command_timeout_
    );
}


// Resolve required joints, validate the SDF timeout and initialize encoder/time baselines:
void TimedDiffDrive::configure_drive(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    node_ = gazebo_ros::Node::Get(sdf);
    left_wheel_ = model->GetJoint("wheel_left_joint");
    right_wheel_ = model->GetJoint("wheel_right_joint");
    if(!left_wheel_ || !right_wheel_) {
        throw std::runtime_error("The Burger wheel joints are required");
    }

    command_timeout_ = sdf->Get<double>("command_timeout");
    if(!std::isfinite(command_timeout_) || command_timeout_ <= 0.0) {
        throw std::runtime_error("command_timeout must be finite and positive");
    }
    odom_frame_ = sdf->Get<std::string>("odometry_frame");
    base_frame_ = sdf->Get<std::string>("robot_base_frame");

    left_wheel_->SetParam("fmax", 0, MAX_WHEEL_TORQUE_NM);
    right_wheel_->SetParam("fmax", 0, MAX_WHEEL_TORQUE_NM);
    last_left_position_ = left_wheel_->Position(0);
    last_right_position_ = right_wheel_->Position(0);
    simulation_time_ = model->GetWorld()->SimTime().Double();
    last_publish_time_ = simulation_time_;
}


// Subscribe exclusively to final cmd_vel and publish local encoder state:
void TimedDiffDrive::initialize_communication()
{
    odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
        "odom",
        10
    );
    joint_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states",
        10
    );
    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    command_subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::QoS(1),
        std::bind(&TimedDiffDrive::command_callback, this, std::placeholders::_1)
    );
}


// Replace invalid commands with zero and refresh receipt times while holding the update lock:
void TimedDiffDrive::command_callback(geometry_msgs::msg::Twist::ConstSharedPtr message)
{
    std::lock_guard<std::mutex> guard(mutex_);
    const bool valid_command = is_command_valid(*message);
    linear_speed_ = 0.0;
    angular_speed_ = 0.0;
    if(valid_command) {
        linear_speed_ = std::clamp(message->linear.x, -MAX_LINEAR_SPEED_M_S, MAX_LINEAR_SPEED_M_S);
        angular_speed_ = std::clamp(
            message->angular.z,
            -MAX_ANGULAR_SPEED_RAD_S,
            MAX_ANGULAR_SPEED_RAD_S
        );
    }

    command_simulation_time_ = simulation_time_;
    command_wall_time_ = std::chrono::steady_clock::now();
    command_received_ = true;
}


// Require finite planar velocity and zero components along unsupported axes:
bool TimedDiffDrive::is_command_valid(const geometry_msgs::msg::Twist & command) const
{
    const bool finite_planar_velocities =
        std::isfinite(command.linear.x) && std::isfinite(command.angular.z);
    const bool no_out_of_plane_motion =
        command.linear.y == 0.0 && command.linear.z == 0.0 &&
        command.angular.x == 0.0 && command.angular.y == 0.0;
    return finite_planar_velocities && no_out_of_plane_motion;
}


// Advance time, apply protected wheel targets, integrate encoders and publish at 30 Hz:
void TimedDiffDrive::world_update_callback(const gazebo::common::UpdateInfo & info)
{
    std::lock_guard<std::mutex> guard(mutex_);
    const double simulation_time = info.simTime.Double();
    if(simulation_time < simulation_time_) {
        reset_odometry(simulation_time);
    }
    const double elapsed_seconds = std::max(0.0, simulation_time - simulation_time_);
    simulation_time_ = simulation_time;

    const bool fresh_command = command_is_fresh(simulation_time);
    apply_wheel_velocities(elapsed_seconds, fresh_command);

    const double left_position_rad = left_wheel_->Position(0);
    const double right_position_rad = right_wheel_->Position(0);
    integrate_encoder_odometry(left_position_rad, right_position_rad);

    if(simulation_time - last_publish_time_ < PUBLISH_PERIOD_SECONDS) return;
    last_publish_time_ = simulation_time;
    publish_state(simulation_time, left_position_rad, right_position_rad);
}


// Invalidate motion and rebase encoders on clock rollback, retaining the existing wheel ramp:
void TimedDiffDrive::reset_odometry(double simulation_time)
{
    command_received_ = false;
    x_in_odom_ = 0.0;
    y_in_odom_ = 0.0;
    yaw_in_odom_ = 0.0;
    last_left_position_ = left_wheel_->Position(0);
    last_right_position_ = right_wheel_->Position(0);
    last_publish_time_ = simulation_time;
}


// Expire motion independently on simulation time or monotonic elapsed time:
bool TimedDiffDrive::command_is_fresh(double simulation_time) const
{
    const double wall_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - command_wall_time_
    ).count();
    const double simulation_age = simulation_time - command_simulation_time_;
    return command_received_ && simulation_age <= command_timeout_ && wall_age <= command_timeout_;
}


// Convert body velocity into wheel-rim targets and limit acceleration before joint actuation:
void TimedDiffDrive::apply_wheel_velocities(double elapsed_seconds, bool fresh_command)
{
    const double forward_speed = fresh_command ? linear_speed_ : 0.0;
    const double yaw_rate = fresh_command ? angular_speed_ : 0.0;
    const double left_target = forward_speed - yaw_rate * WHEEL_SEPARATION_M / 2.0;
    const double right_target = forward_speed + yaw_rate * WHEEL_SEPARATION_M / 2.0;

    // The 1 m/s^2 rim ramp avoids exciting the light Burger body, including when stopping:
    const double maximum_speed_change = MAX_RIM_ACCELERATION_M_S2 * elapsed_seconds;
    applied_left_speed_ += std::clamp(
        left_target - applied_left_speed_,
        -maximum_speed_change,
        maximum_speed_change
    );
    applied_right_speed_ += std::clamp(
        right_target - applied_right_speed_,
        -maximum_speed_change,
        maximum_speed_change
    );
    left_wheel_->SetParam("vel", 0, applied_left_speed_ / WHEEL_RADIUS_M);
    right_wheel_->SetParam("vel", 0, applied_right_speed_ / WHEEL_RADIUS_M);
}


// Integrate wheel travel with midpoint heading in the independent local odom frame:
void TimedDiffDrive::integrate_encoder_odometry(
    double left_position_rad,
    double right_position_rad
)
{
    const double left_travel_m = WHEEL_RADIUS_M * (left_position_rad - last_left_position_);
    const double right_travel_m = WHEEL_RADIUS_M * (right_position_rad - last_right_position_);
    last_left_position_ = left_position_rad;
    last_right_position_ = right_position_rad;

    const double forward_travel_m = (left_travel_m + right_travel_m) / 2.0;
    const double yaw_change_rad = (right_travel_m - left_travel_m) / WHEEL_SEPARATION_M;
    const double midpoint_yaw_rad = yaw_in_odom_ + yaw_change_rad / 2.0;
    x_in_odom_ += forward_travel_m * std::cos(midpoint_yaw_rad);
    y_in_odom_ += forward_travel_m * std::sin(midpoint_yaw_rad);
    yaw_in_odom_ = std::remainder(yaw_in_odom_ + yaw_change_rad, 2.0 * std::acos(-1.0));
}


// Build planar pose in odom and body-frame twist from measured wheel velocities:
nav_msgs::msg::Odometry TimedDiffDrive::make_odometry(const rclcpp::Time & timestamp) const
{
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_in_odom_);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = timestamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_in_odom_;
    odom.pose.pose.position.y = y_in_odom_;
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.pose.pose.orientation.w = orientation.w();

    const double left_rate_rad_s = left_wheel_->GetVelocity(0);
    const double right_rate_rad_s = right_wheel_->GetVelocity(0);
    odom.twist.twist.linear.x = WHEEL_RADIUS_M * (left_rate_rad_s + right_rate_rad_s) / 2.0;
    odom.twist.twist.angular.z =
        WHEEL_RADIUS_M * (right_rate_rad_s - left_rate_rad_s) / WHEEL_SEPARATION_M;
    odom.pose.covariance[X_VARIANCE_INDEX] = POSITION_VARIANCE_M2;
    odom.pose.covariance[Y_VARIANCE_INDEX] = POSITION_VARIANCE_M2;
    odom.pose.covariance[YAW_VARIANCE_INDEX] = YAW_VARIANCE_RAD2;
    odom.twist.covariance[X_VARIANCE_INDEX] = TWIST_VARIANCE;
    odom.twist.covariance[YAW_VARIANCE_INDEX] = TWIST_VARIANCE;
    return odom;
}


// Publish odometry, its sole odom-to-base TF authority, then joint positions and velocities:
void TimedDiffDrive::publish_state(
    double simulation_time,
    double left_position_rad,
    double right_position_rad
)
{
    const rclcpp::Time timestamp(
        static_cast<int64_t>(simulation_time * NANOSECONDS_PER_SECOND),
        RCL_ROS_TIME
    );
    const nav_msgs::msg::Odometry odom = make_odometry(timestamp);
    odom_publisher_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x_in_odom_;
    transform.transform.translation.y = y_in_odom_;
    transform.transform.rotation = odom.pose.pose.orientation;
    transform_broadcaster_->sendTransform(transform);

    sensor_msgs::msg::JointState joints;
    joints.header.stamp = timestamp;
    joints.header.frame_id = base_frame_;
    joints.name = {"wheel_left_joint", "wheel_right_joint"};
    joints.position = {left_position_rad, right_position_rad};
    joints.velocity = {left_wheel_->GetVelocity(0), right_wheel_->GetVelocity(0)};
    joint_publisher_->publish(joints);
}


GZ_REGISTER_MODEL_PLUGIN(TimedDiffDrive)
}  // namespace search_squad
