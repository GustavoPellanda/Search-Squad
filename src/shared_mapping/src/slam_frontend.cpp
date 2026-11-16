#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

/*
Bootstrap independent SLAM without inventing global odometry. Capture a measured
world<-odom transform once per robot, matching Gazebo and encoder timestamps.
Use these initial anchors and live encoder TF to exclude peer returns from the
SLAM-only scans. Raw scans still feed perception, costmaps and motor protection.
After SLAM initializes, place each explorer's local map in the Scout map once.
No map matching, cross-robot constraints or continuous Gazebo localization runs.
*/
class SlamFrontend : public rclcpp::Node
{
    public:

    SlamFrontend();

    private:

    using StateService = gazebo_msgs::srv::GetEntityState;
    struct Robot
    {
        std::string name;
        rclcpp::Client<StateService>::SharedPtr client;
        bool pending = false;
        bool seeded = false;
        bool aligned = false;
        StateService::Response::SharedPtr response;
        tf2::Transform world_from_odom;
        sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
        std::size_t masked_beams = 0;
    };

    void tick();
    void sample_initial_poses();
    void receive_pose(rclcpp::Client<StateService>::SharedFuture future, std::size_t index);
    bool capture_odometry_anchor(Robot & robot);
    void receive_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan, std::size_t index);
    void filter_pending_scans();
    bool filter_scan(Robot & robot);
    void align_local_maps();
    void publish_status();
    static tf2::Transform planar_pose(const geometry_msgs::msg::Pose & pose);

    std::vector<Robot> robots_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subscriptions_;
    double peer_radius_ = 0.22;
    double scan_timeout_ = 0.6;
    bool root_ready_ = false;
    tf2::Transform world_from_map_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::StaticTransformBroadcaster broadcaster_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    unsigned status_ticks_ = 0;
};


SlamFrontend::SlamFrontend()
    : Node("slam_frontend"), buffer_(get_clock()), listener_(buffer_), broadcaster_(*this)
{
    const auto names = declare_parameter<std::vector<std::string>>(
        "robots", {"scout", "explorer_1", "explorer_2"}
    );
    if(names != std::vector<std::string>{"scout", "explorer_1", "explorer_2"}) {
        throw std::invalid_argument("robots must be scout, explorer_1, explorer_2 in order");
    }
    peer_radius_ = declare_parameter("peer_radius", 0.22);
    scan_timeout_ = declare_parameter("scan_timeout", 0.6);
    if(!std::isfinite(peer_radius_) || peer_radius_ < 0.15 || peer_radius_ > 0.4 ||
        !std::isfinite(scan_timeout_) || scan_timeout_ <= 0.0) {
        throw std::invalid_argument("Invalid peer radius or scan timeout");
    }
    robots_.resize(names.size());
    for(std::size_t index = 0; index < names.size(); ++index) {
        auto & robot = robots_[index];
        robot.name = names[index];
        robot.client = create_client<StateService>("/simulation/get_entity_state");
        robot.publisher = create_publisher<sensor_msgs::msg::LaserScan>(
            "/" + robot.name + "/scan_mapping", rclcpp::SensorDataQoS()
        );
        subscriptions_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
            "/" + robot.name + "/scan", rclcpp::SensorDataQoS(),
            [this, index](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
                receive_scan(scan, index);
            }
        ));
    }
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        "/mapping/alignment", rclcpp::QoS(1).reliable().transient_local()
    );
    timer_ = create_wall_timer(std::chrono::milliseconds(25),
        std::bind(&SlamFrontend::tick, this));
    RCLCPP_WARN(get_logger(),
        "Parallel SLAM: measured Gazebo poses seed alignment once; no cooperative optimization");
}


// Initialization, filtered sensor delivery and map anchoring are distinct stages:
void SlamFrontend::tick()
{
    sample_initial_poses();
    filter_pending_scans();
    align_local_maps();
    if(++status_ticks_ >= 20) {
        publish_status();
        status_ticks_ = 0;
    }
}


void SlamFrontend::sample_initial_poses()
{
    for(std::size_t index = 0; index < robots_.size(); ++index) {
        auto & robot = robots_[index];
        if(robot.seeded) continue;
        if(robot.response) {
            const double age = (now() - rclcpp::Time(robot.response->header.stamp)).seconds();
            if(capture_odometry_anchor(robot) || age > scan_timeout_) robot.response.reset();
            continue;
        }
        if(robot.pending || !robot.client->service_is_ready()) continue;
        try {
            // Wait for measured encoder TF before requesting the one-time simulator sample.
            buffer_.lookupTransform(robot.name + "/odom",
                robot.name + "/base_footprint", tf2::TimePointZero);
        }
        catch(const tf2::TransformException &) {
            continue;
        }
        auto request = std::make_shared<StateService::Request>();
        request->name = robot.name;
        request->reference_frame = "world";
        robot.pending = true;
        robot.client->async_send_request(request,
            [this, index](rclcpp::Client<StateService>::SharedFuture future) {
                receive_pose(future, index);
            });
    }
}


void SlamFrontend::receive_pose(
    rclcpp::Client<StateService>::SharedFuture future, std::size_t index
)
{
    auto & robot = robots_[index];
    robot.pending = false;
    const auto response = future.get();
    if(response->success && response->header.stamp.sec >= 0) robot.response = response;
}


bool SlamFrontend::capture_odometry_anchor(Robot & robot)
{
    try {
        const auto odometry = buffer_.lookupTransform(
            robot.name + "/odom", robot.name + "/base_footprint",
            rclcpp::Time(robot.response->header.stamp)
        );
        tf2::Transform odom_from_base;
        tf2::fromMsg(odometry.transform, odom_from_base);
        const auto world_from_base = planar_pose(robot.response->state.pose);
        // world<-odom = world<-base(t) * inverse(odom<-base(t)). No spawn pose is used.
        robot.world_from_odom = world_from_base * odom_from_base.inverse();
        robot.seeded = true;
        RCLCPP_INFO(get_logger(), "%s: measured initial odometry anchor captured", robot.name.c_str());
        return true;
    }
    catch(const tf2::TransformException &) {
        return false;
    }
    catch(const std::invalid_argument & error) {
        RCLCPP_WARN(get_logger(), "Rejected initial pose: %s", error.what());
        robot.response.reset();
        return false;
    }
}


void SlamFrontend::receive_scan(
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan, std::size_t index
)
{
    // One pending scan per robot bounds memory and never renews acquisition time.
    robots_[index].scan = scan;
}


void SlamFrontend::filter_pending_scans()
{
    if(!std::all_of(robots_.begin(), robots_.end(),
        [](const Robot & robot) { return robot.seeded; })) return;
    for(auto & robot : robots_) {
        if(!robot.scan) continue;
        const auto & scan = *robot.scan;
        const bool metadata_valid = scan.header.frame_id == robot.name + "/base_scan" &&
            scan.header.stamp.sec >= 0 && scan.header.stamp.nanosec < 1000000000U &&
            scan.ranges.size() >= 180 && std::isfinite(scan.angle_min) &&
            std::isfinite(scan.angle_increment) && scan.angle_increment > 0.0 &&
            std::isfinite(scan.range_min) && std::isfinite(scan.range_max) &&
            scan.range_min > 0.0 && scan.range_max > scan.range_min;
        if(!metadata_valid) {
            robot.scan.reset();
            continue;
        }
        const double age = (now() - rclcpp::Time(scan.header.stamp)).seconds();
        if(age > scan_timeout_ || age < -0.05 || filter_scan(robot)) robot.scan.reset();
    }
}


bool SlamFrontend::filter_scan(Robot & robot)
{
    const auto & scan = *robot.scan;
    try {
        const auto stamp = rclcpp::Time(scan.header.stamp);
        tf2::Transform odom_from_sensor;
        tf2::fromMsg(buffer_.lookupTransform(
            robot.name + "/odom", scan.header.frame_id, stamp).transform, odom_from_sensor);
        const auto world_from_sensor = robot.world_from_odom * odom_from_sensor;
        std::vector<tf2::Vector3> peers_in_sensor;
        for(const auto & peer : robots_) {
            if(peer.name == robot.name) continue;
            tf2::Transform odom_from_peer;
            tf2::fromMsg(buffer_.lookupTransform(
                peer.name + "/odom", peer.name + "/base_footprint", stamp).transform,
                odom_from_peer);
            const auto sensor_from_peer = world_from_sensor.inverse() *
                peer.world_from_odom * odom_from_peer;
            peers_in_sensor.push_back(sensor_from_peer.getOrigin());
        }
        auto filtered = scan;
        for(std::size_t beam = 0; beam < scan.ranges.size(); ++beam) {
            const double range = scan.ranges[beam];
            if(std::isnan(range) || range < scan.range_min ||
                (std::isfinite(range) && range > scan.range_max)) return true;
            if(!std::isfinite(range)) continue;
            const double angle = scan.angle_min + beam * scan.angle_increment;
            const double x = range * std::cos(angle);
            const double y = range * std::sin(angle);
            for(const auto & peer : peers_in_sensor) {
                if(std::hypot(x - peer.x(), y - peer.y()) > peer_radius_) continue;
                // Karto ignores ranges <= range_min for both matching and ray tracing.
                // Zero suppresses this beam; +inf would incorrectly clear behind the peer.
                filtered.ranges[beam] = 0.0F;
                ++robot.masked_beams;
                break;
            }
        }
        robot.publisher->publish(filtered);
        return true;
    }
    catch(const tf2::TransformException &) {
        // The scan may precede a peer's encoder TF; retry only while its stamp is fresh.
        return false;
    }
}


void SlamFrontend::align_local_maps()
{
    if(!robots_.front().seeded) return;
    try {
        if(!root_ready_) {
            tf2::Transform map_from_odom;
            tf2::fromMsg(buffer_.lookupTransform(
                "map", "scout/odom", tf2::TimePointZero).transform, map_from_odom);
            world_from_map_ = robots_.front().world_from_odom * map_from_odom.inverse();
            root_ready_ = true;
            robots_.front().aligned = true;
        }
        for(auto & robot : robots_) {
            if(robot.aligned || !robot.seeded) continue;
            tf2::Transform local_from_odom;
            tf2::fromMsg(buffer_.lookupTransform(robot.name + "/slam_map",
                robot.name + "/odom", tf2::TimePointZero).transform, local_from_odom);
            const auto world_from_local = robot.world_from_odom * local_from_odom.inverse();
            geometry_msgs::msg::TransformStamped alignment;
            alignment.header.frame_id = "map";
            alignment.header.stamp = now();
            alignment.child_frame_id = robot.name + "/slam_map";
            alignment.transform = tf2::toMsg(world_from_map_.inverse() * world_from_local);
            broadcaster_.sendTransform(alignment);
            robot.aligned = true;
            RCLCPP_INFO(get_logger(), "%s: local SLAM map aligned to Scout reference",
                robot.name.c_str());
        }
    }
    catch(const tf2::TransformException &) {
        // Local SLAM must accept its first filtered scan before its map frame exists.
    }
}


void SlamFrontend::publish_status()
{
    std::ostringstream text;
    text << "{\"initialization\":\"measured_simulation_poses\",\"robots\":[";
    for(std::size_t index = 0; index < robots_.size(); ++index) {
        const auto & robot = robots_[index];
        if(index != 0) text << ',';
        text << "{\"name\":\"" << robot.name << "\",\"seeded\":" << robot.seeded
             << ",\"aligned\":" << robot.aligned << ",\"masked_beams\":"
             << robot.masked_beams << '}';
    }
    text << "]}";
    std_msgs::msg::String status;
    status.data = text.str();
    status_publisher_->publish(status);
}


tf2::Transform SlamFrontend::planar_pose(const geometry_msgs::msg::Pose & pose)
{
    const auto & p = pose.position;
    const auto & q = pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(norm) ||
        std::abs(norm - 1.0) > 0.01) {
        throw std::invalid_argument("Expected a finite measured planar pose");
    }
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, yaw);
    return tf2::Transform(rotation, tf2::Vector3(p.x, p.y, 0.0));
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamFrontend>());
    rclcpp::shutdown();
    return 0;
}
