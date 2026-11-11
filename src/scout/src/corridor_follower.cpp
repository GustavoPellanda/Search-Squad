#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <squad_interfaces/msg/robot_event.hpp>
#include <std_srvs/srv/set_bool.hpp>

/*
Initial Scout behavior: request forward motion with zero angular velocity.
The existing gate, Collision Monitor and actuator enforce sensor checks and stops.
Corridor perception runs independently; steering will be added in a later increment.
*/
class CorridorFollower : public rclcpp::Node
{
    public:

    CorridorFollower();

    private:

    void enable_callback(const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response);
    void publish_command();
    void report_activity();

    double linear_speed_ = 0.0;
    bool enabled_ = false;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
    rclcpp::Publisher<squad_interfaces::msg::RobotEvent>::SharedPtr event_publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};


// Configure the initial advance and expose an explicit stop/resume service:
CorridorFollower::CorridorFollower() : Node("corridor_follower")
{
    linear_speed_ = declare_parameter("linear_speed", 0.08);
    enabled_ = declare_parameter("autostart", false);
    if(!std::isfinite(linear_speed_) || linear_speed_ <= 0.0 || linear_speed_ > 0.15) {
        throw std::invalid_argument("linear_speed must be finite and in (0, 0.15] m/s");
    }
    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_corridor", 1);
    event_publisher_ = create_publisher<squad_interfaces::msg::RobotEvent>("/fleet/events", 20);
    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "set_corridor_enabled",
        std::bind(&CorridorFollower::enable_callback, this,
            std::placeholders::_1, std::placeholders::_2)
    );
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&CorridorFollower::publish_command, this)
    );
    report_activity();
}


// Stop immediately or resume the same forward request; stopping never rearms autostart:
void CorridorFollower::enable_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response
)
{
    enabled_ = request->data;
    publish_command();
    report_activity();
    response->success = true;
    response->message = enabled_ ? "Forward motion enabled" : "Forward motion disabled";
}


// Publish only forward velocity; all angular components remain zero:
void CorridorFollower::publish_command()
{
    geometry_msgs::msg::Twist command;
    command.linear.x = enabled_ ? linear_speed_ : 0.0;
    command_publisher_->publish(command);
}


// Report the requested activity without claiming motion that protection may refuse:
void CorridorFollower::report_activity()
{
    squad_interfaces::msg::RobotEvent event;
    event.header.stamp = now();
    event.robot = std::string(get_namespace()).substr(1);
    event.phase = enabled_ ? "ADVANCE_ENABLED" : "STOPPED";
    event.detail = enabled_ ? "Scout forward motion requested" : "Scout forward motion disabled";
    event_publisher_->publish(event);
    RCLCPP_INFO(get_logger(), "%s", event.detail.c_str());
}


// Run the simple behavior with serialized timer and service callbacks:
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorFollower>());
    rclcpp::shutdown();
    return 0;
}
