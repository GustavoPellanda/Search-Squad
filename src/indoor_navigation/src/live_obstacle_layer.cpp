#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace indoor_navigation
{

/*
Keep Nav2's obstacle marking/clearing and replace only asynchronous TF filtering.
Humble's MessageFilter can deadlock its TF listener (geometry2 issue #992).
One pending scan waits for non-blocking, scan-time TF checks on a wall timer.
*/
class LiveObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
    public:

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    private:

    void receive_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr message);
    void integrate_pending();

    bool active_ = false;
    bool inf_is_valid_ = false;
    std::string sensor_frame_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr pending_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};


// Reuse standard observation buffers and discard their not-yet-active MessageFilters:
void LiveObstacleLayer::onInitialize()
{
    ObstacleLayer::onInitialize();
    observation_subscribers_.clear();
    observation_notifiers_.clear();
    auto node = node_.lock();
    if(!node) throw std::runtime_error("Live obstacle layer lost its costmap node");
    std::string source;
    node->get_parameter(name_ + ".observation_sources", source);
    if(observation_buffers_.size() != 1 || source.find(' ') != std::string::npos) {
        throw std::invalid_argument("LiveObstacleLayer requires one LaserScan source");
    }
    std::string data_type;
    node->get_parameter(name_ + "." + source + ".data_type", data_type);
    if(data_type != "LaserScan") {
        throw std::invalid_argument("LiveObstacleLayer supports LaserScan observations");
    }
    std::string topic;
    node->get_parameter(name_ + "." + source + ".topic", topic);
    node->get_parameter(name_ + "." + source + ".sensor_frame", sensor_frame_);
    node->get_parameter(name_ + "." + source + ".inf_is_valid", inf_is_valid_);

    // Serialize receipt, pending TF checks and lifecycle transitions on the costmap group:
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    subscription_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        topic,
        rclcpp::SensorDataQoS(),
        std::bind(&LiveObstacleLayer::receive_scan, this, std::placeholders::_1),
        options
    );
    timer_ = node->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&LiveObstacleLayer::integrate_pending, this),
        callback_group_
    );
}


// Activate the standard buffers while leaving their asynchronous filters absent:
void LiveObstacleLayer::activate()
{
    ObstacleLayer::activate();
    active_ = true;
}


// A deactivated layer must not insert observations received during shutdown or reconfiguration:
void LiveObstacleLayer::deactivate()
{
    active_ = false;
    pending_.reset();
    ObstacleLayer::deactivate();
}


// Retain at most one scan so a delayed transform cannot accumulate unbounded work:
void LiveObstacleLayer::receive_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
    if(!active_ || pending_) return;
    if(message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000U ||
        message->ranges.size() < 180 || message->header.frame_id.empty()) return;
    const bool valid_angles = std::isfinite(message->angle_min) &&
        std::isfinite(message->angle_increment) && message->angle_increment > 0.0;
    const bool valid_ranges = std::isfinite(message->range_min) &&
        std::isfinite(message->range_max) && message->range_min > 0.0 &&
        message->range_max > message->range_min;
    if(!valid_angles || !valid_ranges) return;
    pending_ = message;
}


// Check availability without waitForTransform, then use Nav2's unmodified scan processing:
void LiveObstacleLayer::integrate_pending()
{
    if(!active_ || !pending_) return;
    const auto node = node_.lock();
    if(!node) return;
    const double age = (node->now() - rclcpp::Time(pending_->header.stamp)).seconds();
    if(age > 0.5 || age < -0.05) {
        pending_.reset();
        return;
    }
    const auto stamp = rclcpp::Time(pending_->header.stamp);
    if(!tf_->canTransform(global_frame_, pending_->header.frame_id, stamp)) return;
    if(!sensor_frame_.empty() &&
        !tf_->canTransform(global_frame_, sensor_frame_, stamp)) return;
    const auto scan = pending_;
    pending_.reset();
    if(inf_is_valid_) laserScanValidInfCallback(scan, observation_buffers_.front());
    else laserScanCallback(scan, observation_buffers_.front());
}

}

PLUGINLIB_EXPORT_CLASS(indoor_navigation::LiveObstacleLayer, nav2_costmap_2d::Layer)
