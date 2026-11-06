#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/*
Fuse LiDAR into an initially unknown grid. Ray interiors clear cells and finite
hits mark obstacles. Peer footprints are excluded from structural hits only.
*/
class OccupancyMapper : public rclcpp::Node
{
    public:

    OccupancyMapper();

    private:

    void receive_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan, std::size_t index);
    void integrate_pending();
    bool integrate_scan(const sensor_msgs::msg::LaserScan & scan);
    void trace_ray(double x, double y, double end_x, double end_y, bool hit);
    void publish_map();
    int cell(double x, double y) const;

    std::vector<std::string> robots_;
    std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr> pending_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subscriptions_;
    std::vector<float> evidence_;
    std::vector<bool> observed_;
    nav_msgs::msg::OccupancyGrid map_;
    double resolution_ = 0.05;
    double max_range_ = 3.5;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr observation_publisher_;
    rclcpp::TimerBase::SharedPtr integration_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};


// Set a computational extent, not a room boundary; leave every unobserved cell unknown:
OccupancyMapper::OccupancyMapper()
    : Node("occupancy_mapper"), buffer_(get_clock()), listener_(buffer_)
{
    resolution_ = declare_parameter("resolution", 0.05);
    const double width = declare_parameter("width", 16.0);
    const double height = declare_parameter("height", 12.0);
    if(!std::isfinite(resolution_) || !std::isfinite(width) || !std::isfinite(height) ||
        resolution_ < 0.02 || width <= 0.0 || height <= 0.0 ||
        width * height / (resolution_ * resolution_) > 2000000.0) {
        throw std::invalid_argument("Map dimensions and resolution must be positive");
    }
    map_.header.frame_id = "map";
    map_.info.resolution = resolution_;
    map_.info.width = static_cast<unsigned int>(std::ceil(width / resolution_));
    map_.info.height = static_cast<unsigned int>(std::ceil(height / resolution_));
    map_.info.origin.position.x = declare_parameter("origin_x", -8.0);
    map_.info.origin.position.y = declare_parameter("origin_y", -6.0);
    map_.info.origin.orientation.w = 1.0;
    max_range_ = declare_parameter("max_range", 3.5);
    evidence_.resize(map_.info.width * map_.info.height, 0.0F);
    observed_.resize(evidence_.size(), false);
    map_.data.resize(evidence_.size(), -1);
    robots_ = declare_parameter<std::vector<std::string>>(
        "robots", {"scout", "explorer_1", "explorer_2"}
    );
    pending_.resize(robots_.size());
    for(std::size_t index = 0; index < robots_.size(); ++index) {
        subscriptions_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
            "/" + robots_[index] + "/scan", rclcpp::SensorDataQoS(),
            [this, index](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
                receive_scan(scan, index);
            }
        ));
    }
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).reliable().transient_local()
    );
    observation_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/mapping/scan_points", rclcpp::SensorDataQoS()
    );
    integration_timer_ = create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&OccupancyMapper::integrate_pending, this)
    );
    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&OccupancyMapper::publish_map, this)
    );
}


// Retain one pending observation per robot while scan-time TF catches up:
void OccupancyMapper::receive_scan(
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan, std::size_t index
)
{
    if(!pending_[index]) pending_[index] = scan;
}


// Bound TF waiting and discard malformed wire timestamps before rclcpp conversion:
void OccupancyMapper::integrate_pending()
{
    for(auto & scan : pending_) {
        if(!scan) continue;
        if(scan->header.stamp.sec < 0 || scan->header.stamp.nanosec >= 1000000000U) {
            scan.reset();
            continue;
        }
        const double age = (now() - rclcpp::Time(scan->header.stamp)).seconds();
        if(age > 0.8 || age < -0.05 || integrate_scan(*scan)) scan.reset();
    }
}


// Transform rays using the actual sensor mount and scan-time localization:
bool OccupancyMapper::integrate_scan(const sensor_msgs::msg::LaserScan & scan)
{
    if(scan.ranges.size() < 180 || !std::isfinite(scan.angle_min) ||
        !std::isfinite(scan.angle_increment) || !std::isfinite(scan.range_min) ||
        !std::isfinite(scan.range_max) || scan.angle_increment <= 0.0 ||
        scan.range_min <= 0.0 || scan.range_max <= scan.range_min) return true;
    for(const auto range : scan.ranges) {
        if(std::isnan(range) || range < scan.range_min ||
            (std::isfinite(range) && range > scan.range_max)) return true;
    }
    try {
        const auto transform = buffer_.lookupTransform(
            "map", scan.header.frame_id, rclcpp::Time(scan.header.stamp)
        );
        const auto & translation = transform.transform.translation;
        const auto & q = transform.transform.rotation;
        const double yaw = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        );
        std::vector<std::pair<double, double>> peers;
        for(const auto & robot : robots_) {
            if(scan.header.frame_id == robot + "/base_scan") continue;
            try {
                const auto peer = buffer_.lookupTransform(
                    "map", robot + "/base_footprint", rclcpp::Time(scan.header.stamp)
                );
                peers.emplace_back(peer.transform.translation.x, peer.transform.translation.y);
            }
            catch(const tf2::TransformException &) {
                return false;
            }
        }
        std::vector<std::array<float, 3>> visible_hits;
        for(std::size_t beam = 0; beam < scan.ranges.size(); ++beam) {
            const double measured = scan.ranges[beam];
            if(std::isnan(measured) || measured < scan.range_min) continue;
            const double distance = std::min({measured, max_range_, double(scan.range_max)});
            if(!std::isfinite(distance) || distance <= 0.0) continue;
            const double angle = yaw + scan.angle_min + beam * scan.angle_increment;
            const double end_x = translation.x + distance * std::cos(angle);
            const double end_y = translation.y + distance * std::sin(angle);
            bool hit = std::isfinite(measured) && measured < std::min(max_range_,
                double(scan.range_max)) - 0.01;
            if(hit) {
                visible_hits.push_back({float(end_x), float(end_y), float(translation.z)});
            }
            bool peer_hit = false;
            for(const auto & peer : peers) {
                if(std::hypot(end_x - peer.first, end_y - peer.second) < 0.22) peer_hit = true;
            }
            // Stop before a peer's body; do not mark it or clear the space behind it:
            const double stop_short = peer_hit ? std::max(0.0, distance - 0.22) : distance;
            trace_ray(translation.x, translation.y,
                translation.x + stop_short * std::cos(angle),
                translation.y + stop_short * std::sin(angle), hit && !peer_hit);
        }
        if(rclcpp::Time(scan.header.stamp) > rclcpp::Time(map_.header.stamp)) {
            map_.header.stamp = scan.header.stamp;
        }
        // RViz receives already-transformed points, avoiding asynchronous TF scan filtering:
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = scan.header;
        cloud.header.frame_id = "map";
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(visible_hits.size());
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
        for(const auto & point : visible_hits) {
            *x = point[0];
            *y = point[1];
            *z = point[2];
            ++x;
            ++y;
            ++z;
        }
        observation_publisher_->publish(cloud);
        return true;
    }
    catch(const tf2::TransformException &) {
        return false;
    }
}


// Convert metric positions to a checked row-major index:
int OccupancyMapper::cell(double x, double y) const
{
    const int column = std::floor((x - map_.info.origin.position.x) / resolution_);
    const int row = std::floor((y - map_.info.origin.position.y) / resolution_);
    if(column < 0 || row < 0 || column >= int(map_.info.width) ||
        row >= int(map_.info.height)) return -1;
    return row * map_.info.width + column;
}


// Half-cell sampling preserves thin walls; clamp confidence so later rays can correct it:
void OccupancyMapper::trace_ray(double x, double y, double end_x, double end_y, bool hit)
{
    const int steps = std::max(1, int(std::ceil(std::hypot(end_x - x, end_y - y) /
        (resolution_ * 0.5))));
    const int endpoint = cell(end_x, end_y);
    int previous = -1;
    for(int step = 0; step <= steps; ++step) {
        const double ratio = double(step) / steps;
        const int index = cell(x + ratio * (end_x - x), y + ratio * (end_y - y));
        if(index < 0 || index == previous) continue;
        previous = index;
        observed_[index] = true;
        const float increment = hit && index == endpoint ? 1.0F : -0.45F;
        evidence_[index] = std::clamp(evidence_[index] + increment, -4.0F, 4.0F);
    }
}


// Publish probabilities for observed cells and preserve unknown elsewhere:
void OccupancyMapper::publish_map()
{
    if(now().seconds() <= 0.0) return;
    for(std::size_t index = 0; index < evidence_.size(); ++index) {
        if(observed_[index]) {
            map_.data[index] = std::lround(100.0 / (1.0 + std::exp(-evidence_[index])));
        }
    }
    publisher_->publish(map_);
}


// Serialize scan integration and publication:
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyMapper>());
    rclcpp::shutdown();
    return 0;
}
