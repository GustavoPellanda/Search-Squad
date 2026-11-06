#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

/*
Simulation-assisted localization: measured Gazebo poses locate local odometry
origins in map. This node receives poses only, never walls, room bounds or maps.
*/
class SimPoseAdapter : public rclcpp::Node
{
    public:

    SimPoseAdapter();

    private:

    void request_poses();
    bool encoder_tree_ready(std::size_t index) const;
    bool publish_pose(std::size_t index);
    void receive_pose(
        rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future,
        std::size_t index
    );

    std::vector<std::string> robots_;
    std::vector<rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr> clients_;
    std::vector<bool> pending_;
    std::vector<gazebo_msgs::srv::GetEntityState::Response::SharedPtr> responses_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};


// Connect non-blocking requests and preserve the encoder odometry authorities:
SimPoseAdapter::SimPoseAdapter()
    : Node("sim_pose_adapter"), buffer_(get_clock()), listener_(buffer_), broadcaster_(*this)
{
    robots_ = declare_parameter<std::vector<std::string>>(
        "robots", {"scout", "explorer_1", "explorer_2"}
    );
    for(std::size_t index = 0; index < robots_.size(); ++index) {
        clients_.push_back(create_client<gazebo_msgs::srv::GetEntityState>(
            "/simulation/get_entity_state"
        ));
        pending_.push_back(false);
        responses_.push_back(nullptr);
    }
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&SimPoseAdapter::request_poses, this)
    );
    RCLCPP_WARN(get_logger(), "Localization: SIMULATION-ASSISTED POSES, not SLAM");
}


// Keep at most one pending request per robot:
void SimPoseAdapter::request_poses()
{
    for(std::size_t index = 0; index < robots_.size(); ++index) {
        if(responses_[index]) {
            if(publish_pose(index) ||
                (now() - rclcpp::Time(responses_[index]->header.stamp)).seconds() > 0.5) {
                responses_[index].reset();
            }
            else continue;
        }
        if(pending_[index] || !clients_[index]->service_is_ready()) continue;
        if(!encoder_tree_ready(index)) continue;
        auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        request->name = robots_[index];
        request->reference_frame = "world";
        pending_[index] = true;
        clients_[index]->async_send_request(
            request, [this, index](
                rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future
            ) { receive_pose(future, index); }
        );
    }
}


// Query readiness without flooding the startup log with canTransform's unknown-frame warnings:
bool SimPoseAdapter::encoder_tree_ready(std::size_t index) const
{
    try {
        buffer_.lookupTransform(robots_[index] + "/odom",
            robots_[index] + "/base_footprint", tf2::TimePointZero);
        return true;
    }
    catch(const tf2::TransformException &) {
        return false;
    }
}


// Compute map->odom from world pose and encoder TF at the same observation time:
void SimPoseAdapter::receive_pose(
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future,
    std::size_t index
)
{
    pending_[index] = false;
    const auto response = future.get();
    if(response->success && response->header.stamp.sec >= 0) responses_[index] = response;
}


// Wait for matching encoder TF instead of mixing a current world pose with older odometry:
bool SimPoseAdapter::publish_pose(std::size_t index)
{
    const auto response = responses_[index];
    try {
        const auto odom = buffer_.lookupTransform(
            robots_[index] + "/odom", robots_[index] + "/base_footprint",
            rclcpp::Time(response->header.stamp)
        );
        tf2::Transform odom_from_base;
        tf2::fromMsg(odom.transform, odom_from_base);
        const auto & pose = response->state.pose;
        const auto & q = pose.orientation;
        const double yaw = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        );
        tf2::Quaternion rotation;
        rotation.setRPY(0.0, 0.0, yaw);
        const tf2::Transform map_from_base(
            rotation, tf2::Vector3(pose.position.x, pose.position.y, 0.0)
        );
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "map";
        transform.header.stamp = response->header.stamp;
        transform.child_frame_id = robots_[index] + "/odom";
        transform.transform = tf2::toMsg(map_from_base * odom_from_base.inverse());
        broadcaster_.sendTransform(transform);
        return true;
    }
    catch(const tf2::TransformException &) {
        return false;
    }
}


// Serialize requests and transform publication:
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimPoseAdapter>());
    rclcpp::shutdown();
    return 0;
}
