#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace explorer
{

struct RoomAnalysis {
    std::vector<std::uint8_t> region;
    std::vector<std::uint8_t> safe;
    std::vector<std::vector<int>> frontiers;
    std::uint32_t known_cells = 0;
    std::uint32_t discarded_cells = 0;
    bool touches_extent = false;
    bool seed_valid = false;
};

/* Geometry-only frontier analysis, shared by the task node and synthetic tests. */
class RoomGeometry
{
    public:

    RoomGeometry(const nav_msgs::msg::OccupancyGrid & map,
        const geometry_msgs::msg::PoseStamped & threshold, double clearance,
        double minimum_frontier_length);
    RoomAnalysis analyze(const geometry_msgs::msg::Point & seed) const;
    bool choose_target(const RoomAnalysis & analysis, const geometry_msgs::msg::Point & robot,
        const std::vector<geometry_msgs::msg::Point> & visited,
        geometry_msgs::msg::Point & target, double visit_separation = 0.4) const;
    int index(double x, double y) const;
    geometry_msgs::msg::Point point(int index) const;
    bool inside(double x, double y) const;

    private:

    bool free(int index) const;
    bool unknown(int index) const;
    std::vector<int> neighbors(int index, bool diagonal = false) const;
    bool visible(int start, int end) const;

    const nav_msgs::msg::OccupancyGrid & map_;
    double door_x_;
    double door_y_;
    double normal_x_;
    double normal_y_;
    double clearance_;
    double minimum_frontier_length_;
};

}
