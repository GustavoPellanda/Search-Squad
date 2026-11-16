#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/*
Compose independently optimized local SLAM grids into the sole structural /map.
Each input replaces its previous snapshot. Every output starts unknown and is
rebuilt from the latest complete snapshots, so changed local maps do not leave
permanently accumulated pixels behind. Alignment comes from TF, never spawn YAML.
This conservative raster fusion does not optimize trajectories between robots.
*/
class MapFusion : public rclcpp::Node
{
    public:

    MapFusion();

    private:

    struct Source
    {
        std::string name;
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
        tf2::Transform map_from_grid;
        std::size_t known = 0;
    };

    void receive_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid, std::size_t index);
    bool validate_map(const nav_msgs::msg::OccupancyGrid & grid, const Source & source) const;
    void compose_map();
    bool prepare_sources();
    bool allocate_shared_grid(nav_msgs::msg::OccupancyGrid & result);
    void project_source(const Source & source, nav_msgs::msg::OccupancyGrid & result) const;
    void publish_status(bool ready);

    std::vector<Source> sources_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> subscriptions_;
    double resolution_ = 0.05;
    double map_timeout_ = 3.0;
    std::size_t max_cells_ = 2000000;
    bool dirty_ = true;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


MapFusion::MapFusion() : Node("map_fusion"), buffer_(get_clock()), listener_(buffer_)
{
    const auto names = declare_parameter<std::vector<std::string>>(
        "robots", {"scout", "explorer_1", "explorer_2"}
    );
    if(names != std::vector<std::string>{"scout", "explorer_1", "explorer_2"}) {
        throw std::invalid_argument("robots must be scout, explorer_1, explorer_2 in order");
    }
    resolution_ = declare_parameter("resolution", 0.05);
    map_timeout_ = declare_parameter("map_timeout", 3.0);
    const int max_cells = declare_parameter("max_cells", 2000000);
    if(!std::isfinite(resolution_) || resolution_ < 0.02 || resolution_ > 0.2 ||
        !std::isfinite(map_timeout_) || map_timeout_ <= 0.0 || max_cells < 100) {
        throw std::invalid_argument("Invalid fusion resolution, timeout or grid size limit");
    }
    max_cells_ = static_cast<std::size_t>(max_cells);
    sources_.resize(names.size());
    for(std::size_t index = 0; index < names.size(); ++index) {
        sources_[index].name = names[index];
        subscriptions_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/" + names[index] + "/local_map", rclcpp::QoS(1).reliable().transient_local(),
            [this, index](nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid) {
                receive_map(grid, index);
            }
        ));
    }
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).reliable().transient_local()
    );
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        "/mapping/contributions", rclcpp::QoS(1).reliable().transient_local()
    );
    timer_ = create_wall_timer(std::chrono::milliseconds(500),
        std::bind(&MapFusion::compose_map, this));
}


// Validate before replacing a source; malformed messages never erase a valid snapshot:
void MapFusion::receive_map(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid, std::size_t index
)
{
    auto & source = sources_[index];
    if(!validate_map(*grid, source)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Rejected %s map: check frame, uniform resolution and grid metadata", source.name.c_str());
        return;
    }
    if(source.grid && rclcpp::Time(grid->header.stamp) <
        rclcpp::Time(source.grid->header.stamp)) return;
    source.grid = grid;
    source.known = static_cast<std::size_t>(std::count_if(grid->data.begin(), grid->data.end(),
        [](int8_t value) { return value >= 0; }));
    dirty_ = true;
}


bool MapFusion::validate_map(
    const nav_msgs::msg::OccupancyGrid & grid, const Source & source
) const
{
    const std::string expected_frame = source.name == "scout" ? "map" : source.name + "/slam_map";
    if(grid.header.frame_id != expected_frame || grid.header.stamp.sec < 0 ||
        grid.header.stamp.nanosec >= 1000000000U) return false;
    const auto cells = static_cast<std::size_t>(grid.info.width) * grid.info.height;
    if(cells == 0 || cells > max_cells_ || grid.data.size() != cells) return false;
    // This increment uses uniform-resolution SLAMs; do not silently undersample
    // coarser cells by projecting just their centers into a finer output grid.
    if(!std::isfinite(grid.info.resolution) ||
        std::abs(grid.info.resolution - resolution_) > 1e-6) return false;
    const auto & p = grid.info.origin.position;
    const auto & q = grid.info.origin.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm) ||
        std::abs(norm - 1.0) > 0.01 || std::abs(q.x) > 0.001 || std::abs(q.y) > 0.001) {
        return false;
    }
    return std::all_of(grid.data.begin(), grid.data.end(),
        [](int8_t value) { return value >= -1 && value <= 100; });
}


void MapFusion::compose_map()
{
    if(!prepare_sources()) {
        // Keep the last coherent retained map, without refreshing its acquisition stamp.
        publish_status(false);
        return;
    }
    if(!dirty_) return;
    nav_msgs::msg::OccupancyGrid result;
    if(!allocate_shared_grid(result)) {
        publish_status(false);
        return;
    }
    for(const auto & source : sources_) project_source(source, result);
    map_publisher_->publish(result);
    publish_status(true);
    dirty_ = false;
}


bool MapFusion::prepare_sources()
{
    for(auto & source : sources_) {
        if(!source.grid) return false;
        const double age = (now() - rclcpp::Time(source.grid->header.stamp)).seconds();
        if(age > map_timeout_ || age < -0.05) return false;
        try {
            // Whole-map frames are fixed initial anchors, not moving robot poses.
            // Latest TF is sufficient here; scan-time robot TF is handled inside each SLAM.
            tf2::Transform map_from_local;
            if(source.name == "scout") map_from_local.setIdentity();
            else {
                tf2::fromMsg(buffer_.lookupTransform(
                    "map", source.grid->header.frame_id, tf2::TimePointZero).transform,
                    map_from_local);
            }
            tf2::Transform local_from_grid;
            tf2::fromMsg(source.grid->info.origin, local_from_grid);
            source.map_from_grid = map_from_local * local_from_grid;
        }
        catch(const tf2::TransformException &) {
            return false;
        }
    }
    return true;
}


// Find the union of all four transformed corners, including rotated grid origins:
bool MapFusion::allocate_shared_grid(nav_msgs::msg::OccupancyGrid & result)
{
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = min_x;
    double max_x = -min_x;
    double max_y = -min_x;
    rclcpp::Time stamp(0, 0, get_clock()->get_clock_type());
    for(const auto & source : sources_) {
        const auto & info = source.grid->info;
        const double width = info.width * info.resolution;
        const double height = info.height * info.resolution;
        for(const double x : {0.0, width}) {
            for(const double y : {0.0, height}) {
                const auto corner = source.map_from_grid * tf2::Vector3(x, y, 0.0);
                min_x = std::min(min_x, corner.x());
                min_y = std::min(min_y, corner.y());
                max_x = std::max(max_x, corner.x());
                max_y = std::max(max_y, corner.y());
            }
        }
        stamp = std::max(stamp, rclcpp::Time(source.grid->header.stamp));
    }
    min_x = std::floor(min_x / resolution_) * resolution_;
    min_y = std::floor(min_y / resolution_) * resolution_;
    const double width = std::ceil((max_x - min_x) / resolution_);
    const double height = std::ceil((max_y - min_y) / resolution_);
    if(!std::isfinite(width * height) || width * height > max_cells_ ||
        width <= 0 || height <= 0) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
            "Aligned map union exceeds configured max_cells");
        return false;
    }
    result.header.frame_id = "map";
    result.header.stamp = stamp;
    result.info.resolution = resolution_;
    result.info.width = static_cast<uint32_t>(width);
    result.info.height = static_cast<uint32_t>(height);
    result.info.origin.position.x = min_x;
    result.info.origin.position.y = min_y;
    result.info.origin.orientation.w = 1.0;
    result.data.assign(result.info.width * result.info.height, -1);
    return true;
}


void MapFusion::project_source(
    const Source & source, nav_msgs::msg::OccupancyGrid & result
) const
{
    const auto & grid = *source.grid;
    for(uint32_t row = 0; row < grid.info.height; ++row) {
        for(uint32_t column = 0; column < grid.info.width; ++column) {
            const int8_t value = grid.data[column + row * grid.info.width];
            if(value < 0) continue;
            // ROS row-major cell centers -> local map -> shared map -> output row/column.
            const auto center = source.map_from_grid * tf2::Vector3(
                (column + 0.5) * grid.info.resolution,
                (row + 0.5) * grid.info.resolution, 0.0
            );
            const int x = static_cast<int>(std::floor(
                (center.x() - result.info.origin.position.x) / resolution_));
            const int y = static_cast<int>(std::floor(
                (center.y() - result.info.origin.position.y) / resolution_));
            if(x < 0 || y < 0 || x >= static_cast<int>(result.info.width) ||
                y >= static_cast<int>(result.info.height)) continue;
            auto & target = result.data[x + y * result.info.width];
            // Conservative independent-map fusion: occupied wins over free.
            // Start over for every composition; repeated grids do not increase confidence.
            target = std::max(target, value);
        }
    }
}


void MapFusion::publish_status(bool ready)
{
    std::ostringstream text;
    text << "{\"ready\":" << (ready ? "true" : "false") << ",\"sources\":[";
    for(std::size_t index = 0; index < sources_.size(); ++index) {
        if(index != 0) text << ',';
        text << "{\"robot\":\"" << sources_[index].name
             << "\",\"known_cells\":" << sources_[index].known << '}';
    }
    text << "]}";
    std_msgs::msg::String status;
    status.data = text.str();
    status_publisher_->publish(status);
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapFusion>());
    rclcpp::shutdown();
    return 0;
}
