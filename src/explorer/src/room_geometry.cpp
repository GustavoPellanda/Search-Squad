#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include <explorer/room_geometry.hpp>

namespace explorer
{

// Record the observed grid and entrance normal, without importing true room dimensions:
RoomGeometry::RoomGeometry(const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::PoseStamped & threshold, double clearance,
    double minimum_frontier_length)
    : map_(map), door_x_(threshold.pose.position.x), door_y_(threshold.pose.position.y),
      clearance_(clearance), minimum_frontier_length_(minimum_frontier_length)
{
    const auto & q = threshold.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    normal_x_ = std::cos(yaw);
    normal_y_ = std::sin(yaw);
}


// Return a checked grid index for metric map coordinates:
int RoomGeometry::index(double x, double y) const
{
    const int column = std::floor((x - map_.info.origin.position.x) / map_.info.resolution);
    const int row = std::floor((y - map_.info.origin.position.y) / map_.info.resolution);
    if(column < 0 || row < 0 || column >= int(map_.info.width) ||
        row >= int(map_.info.height)) return -1;
    return row * map_.info.width + column;
}


// Use cell centers consistently for goals and visualization:
geometry_msgs::msg::Point RoomGeometry::point(int cell) const
{
    geometry_msgs::msg::Point result;
    result.x = map_.info.origin.position.x +
        (cell % map_.info.width + 0.5) * map_.info.resolution;
    result.y = map_.info.origin.position.y +
        (cell / map_.info.width + 0.5) * map_.info.resolution;
    return result;
}


// The doorway half-plane separates the corridor; observed connectivity supplies other bounds:
bool RoomGeometry::inside(double x, double y) const
{
    return (x - door_x_) * normal_x_ + (y - door_y_) * normal_y_ > 0.04;
}


// Keep uncertain probability cells unresolved instead of silently calling them free:
bool RoomGeometry::free(int cell) const
{
    return cell >= 0 && map_.data[cell] >= 0 && map_.data[cell] <= 35;
}


// Resolve room floor while tolerating a 10 cm uncertain edge around measured obstacle hits:
bool RoomGeometry::unknown(int cell) const
{
    if(cell < 0) return false;
    const int value = map_.data[cell];
    if(value >= 0 && (value <= 35 || value >= 65)) return false;
    // Sparse angular rays leave pinholes inside the measured surface of a thin wall.
    // This margin affects frontier classification only; /map remains unchanged:
    constexpr double WALL_EDGE_UNCERTAINTY_M = 0.10;
    const int radius = std::ceil(WALL_EDGE_UNCERTAINTY_M / map_.info.resolution);
    const int x = cell % map_.info.width;
    const int y = cell / map_.info.width;
    for(int dy = -radius; dy <= radius; ++dy) {
        for(int dx = -radius; dx <= radius; ++dx) {
            const double distance = std::hypot(dx, dy) * map_.info.resolution;
            if(distance > WALL_EDGE_UNCERTAINTY_M + 1e-6) continue;
            if(x + dx < 0 || y + dy < 0 || x + dx >= int(map_.info.width) ||
                y + dy >= int(map_.info.height)) continue;
            const int neighbor = (y + dy) * map_.info.width + x + dx;
            if(map_.data[neighbor] >= 65) return false;
        }
    }
    return true;
}


// Do not let row-major adjacency wrap around an image edge:
std::vector<int> RoomGeometry::neighbors(int cell, bool diagonal) const
{
    std::vector<int> result;
    const int x = cell % map_.info.width;
    const int y = cell / map_.info.width;
    for(int dy = -1; dy <= 1; ++dy) {
        for(int dx = -1; dx <= 1; ++dx) {
            if(dx == 0 && dy == 0) continue;
            if(!diagonal && std::abs(dx) + std::abs(dy) != 1) continue;
            if(x + dx < 0 || y + dy < 0 || x + dx >= int(map_.info.width) ||
                y + dy >= int(map_.info.height)) continue;
            result.push_back((y + dy) * map_.info.width + x + dx);
        }
    }
    return result;
}


// Extract free connectivity, footprint clearance and physically significant frontier groups:
RoomAnalysis RoomGeometry::analyze(const geometry_msgs::msg::Point & seed) const
{
    RoomAnalysis result;
    result.region.resize(map_.data.size(), 0);
    result.safe.resize(map_.data.size(), 0);
    const int first = index(seed.x, seed.y);
    if(!free(first) || !inside(seed.x, seed.y)) return result;
    result.seed_valid = true;
    std::queue<int> queue;
    queue.push(first);
    result.region[first] = 1;
    while(!queue.empty()) {
        const int current = queue.front();
        queue.pop();
        ++result.known_cells;
        const int x = current % map_.info.width;
        const int y = current / map_.info.width;
        if(x <= 1 || y <= 1 || x + 2 >= int(map_.info.width) ||
            y + 2 >= int(map_.info.height)) result.touches_extent = true;
        for(const int neighbor : neighbors(current)) {
            const auto coordinate = point(neighbor);
            if(!result.region[neighbor] && free(neighbor) &&
                inside(coordinate.x, coordinate.y)) {
                result.region[neighbor] = 1;
                queue.push(neighbor);
            }
        }
    }

    const int radius = std::ceil(clearance_ / map_.info.resolution);
    std::vector<std::uint8_t> frontier(map_.data.size(), 0);
    for(std::size_t cell = 0; cell < result.region.size(); ++cell) {
        if(!result.region[cell]) continue;
        bool safe = true;
        const auto center = point(cell);
        for(int dy = -radius; dy <= radius && safe; ++dy) {
            for(int dx = -radius; dx <= radius; ++dx) {
                if(std::hypot(dx, dy) * map_.info.resolution > clearance_) continue;
                const int neighbor = index(center.x + dx * map_.info.resolution,
                    center.y + dy * map_.info.resolution);
                if(neighbor < 0 || !result.region[neighbor]) {
                    safe = false;
                    break;
                }
            }
        }
        result.safe[cell] = safe;
        for(const int neighbor : neighbors(cell)) {
            const auto coordinate = point(neighbor);
            if(unknown(neighbor) && inside(coordinate.x, coordinate.y)) frontier[cell] = 1;
        }
    }
    for(std::size_t cell = 0; cell < frontier.size(); ++cell) {
        if(!frontier[cell]) continue;
        std::vector<int> cluster;
        queue.push(cell);
        frontier[cell] = 0;
        while(!queue.empty()) {
            const int current = queue.front();
            queue.pop();
            cluster.push_back(current);
            for(const int neighbor : neighbors(current, true)) {
                if(frontier[neighbor]) {
                    frontier[neighbor] = 0;
                    queue.push(neighbor);
                }
            }
        }
        if(cluster.size() * map_.info.resolution >= minimum_frontier_length_) {
            result.frontiers.push_back(std::move(cluster));
        }
        else {
            result.discarded_cells += cluster.size();
        }
    }
    return result;
}


// A view ray must traverse known free space to the frontier's free cell:
bool RoomGeometry::visible(int start, int end) const
{
    const auto a = point(start);
    const auto b = point(end);
    const int steps = std::max(1, int(std::ceil(std::hypot(a.x - b.x, a.y - b.y) /
        (map_.info.resolution * 0.5))));
    for(int step = 0; step <= steps; ++step) {
        const double fraction = double(step) / steps;
        if(!free(index(a.x + fraction * (b.x - a.x), a.y + fraction * (b.y - a.y)))) {
            return false;
        }
    }
    return true;
}


// Prefer reachable nearby observation poses and retain unresolved clusters when none work:
bool RoomGeometry::choose_target(const RoomAnalysis & analysis,
    const geometry_msgs::msg::Point & robot,
    const std::vector<geometry_msgs::msg::Point> & visited,
    geometry_msgs::msg::Point & target, double visit_separation) const
{
    int first = index(robot.x, robot.y);
    if(first < 0) return false;
    if(!analysis.safe[first]) {
        double nearest = 0.4;
        for(std::size_t cell = 0; cell < analysis.safe.size(); ++cell) {
            if(!analysis.safe[cell]) continue;
            const auto p = point(cell);
            const double distance = std::hypot(p.x - robot.x, p.y - robot.y);
            if(distance < nearest) {
                nearest = distance;
                first = cell;
            }
        }
        if(!analysis.safe[first]) return false;
    }
    std::vector<double> travel(map_.data.size(), -1.0);
    std::queue<int> queue;
    queue.push(first);
    travel[first] = 0.0;
    while(!queue.empty()) {
        const int current = queue.front();
        queue.pop();
        for(const int neighbor : neighbors(current)) {
            if(analysis.safe[neighbor] && travel[neighbor] < 0.0) {
                travel[neighbor] = travel[current] + map_.info.resolution;
                queue.push(neighbor);
            }
        }
    }
    double best_score = std::numeric_limits<double>::infinity();
    for(const auto & cluster : analysis.frontiers) {
        // Sample representative frontier cells, rather than evaluating every beam-sized artifact:
        const std::size_t stride = std::max<std::size_t>(1, cluster.size() / 12);
        for(std::size_t sample = 0; sample < cluster.size(); sample += stride) {
            const auto boundary = point(cluster[sample]);
            const int radius = std::ceil(1.1 / map_.info.resolution);
            for(int dy = -radius; dy <= radius; dy += 2) {
                for(int dx = -radius; dx <= radius; dx += 2) {
                    const int candidate = index(boundary.x + dx * map_.info.resolution,
                        boundary.y + dy * map_.info.resolution);
                    if(candidate < 0 || travel[candidate] < 0.25) continue;
                    const auto p = point(candidate);
                    const double view_distance = std::hypot(p.x - boundary.x, p.y - boundary.y);
                    if(view_distance < clearance_ || view_distance > 1.1) continue;
                    bool repeated = false;
                    for(const auto & previous : visited) {
                        if(std::hypot(previous.x - p.x, previous.y - p.y) < visit_separation) {
                            repeated = true;
                        }
                    }
                    if(repeated || !visible(candidate, cluster[sample])) continue;
                    // Favor substantial unknown boundaries over repeated tiny wall artifacts:
                    const double information_length = std::min(3.0,
                        cluster.size() * double(map_.info.resolution));
                    const double score = travel[candidate] + 0.4 * view_distance -
                        information_length;
                    if(score < best_score) {
                        best_score = score;
                        target = p;
                    }
                }
            }
        }
    }
    return std::isfinite(best_score);
}

}
