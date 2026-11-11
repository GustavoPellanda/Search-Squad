#pragma once

#include <cstddef>
#include <limits>
#include <vector>

namespace perception
{
// Positions and lengths are in metres in the robot's encoder odom frame.
struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Pose2D {
    Point position;
    double yaw = 0.0;  // Robot heading in odom, in radians.
};

struct LineSegment {
    Point first;
    Point last;
    Point center;
    double heading = 0.0;
    double length = 0.0;
    double residual = 0.0;  // Maximum perpendicular distance from support to the fitted line.
};

struct GeometryConfiguration {
    // Split ordered scan returns at gaps, then reject short or poorly fitted lines:
    double point_gap = 0.22;
    double max_residual = 0.035;
    double min_segment_length = 0.35;
    int min_segment_points = 6;

    // Associate approximately parallel support with each structural corridor wall:
    double max_heading_difference = 0.20;
    double wall_association_distance = 0.18;
    double min_width = 0.7;
    double max_width = 4.0;

    // Opening candidates are gaps along an observed wall, not registered entrances:
    double min_opening_width = 0.6;
    double max_opening_width = 2.0;

    // Missing support may reuse history only within BOTH limits:
    double prediction_time = 20.0;
    double prediction_distance = 1.5;
};

struct CorridorEstimate {
    bool axis_valid = false;
    bool width_valid = false;
    bool left_measured = false;
    bool right_measured = false;
    bool left_predicted = false;
    bool right_predicted = false;
    bool terminal_candidate = false;
    double heading = 0.0;  // Directed corridor axis in odom, in radians.
    Point center;          // Robot position projected onto that axis, in odom.
    double width = std::numeric_limits<double>::quiet_NaN();
    double lateral_error = std::numeric_limits<double>::quiet_NaN();  // Positive to axis left.
    double heading_error = std::numeric_limits<double>::quiet_NaN();  // Robot yaw minus axis yaw.
    double left_distance = std::numeric_limits<double>::quiet_NaN();
    double right_distance = std::numeric_limits<double>::quiet_NaN();
    double left_age = 0.0;
    double right_age = 0.0;
    double confidence = 0.0;
    std::vector<LineSegment> left_segments;
    std::vector<LineSegment> right_segments;
    std::vector<LineSegment> left_openings;
    std::vector<LineSegment> right_openings;
};

/*
Extract ordered scan segments and associate structural corridor walls in local odom.
History is bounded by both elapsed sensor time and robot travel. Missing points are
NaN separators, never distant walls. No simulator geometry or ROS state is used here.
*/
class CorridorGeometry
{
    public:

    explicit CorridorGeometry(const GeometryConfiguration & configuration);

    /*
    Interpret one distinct scan. Points retain beam order in encoder odom; NaN
    points separate missing returns. Robot pose uses the same frame and scan time.
    scan_time is acquisition time in seconds, not receipt time. Repeated times
    return an invalid estimate; backwards time resets the tracked walls.
    */
    CorridorEstimate estimate(
        const std::vector<Point> & points,
        const Pose2D & robot,
        double scan_time
    );
    void reset();

    private:

    struct WallTrack {
        bool valid = false;
        Point anchor;       // Length-weighted wall support in odom.
        Point observed_at;  // Robot position at the last real measurement.
        double timestamp = 0.0;  // Scan time of that measurement; prediction never renews it.
    };

    struct WallAssociation {
        std::vector<LineSegment> left_segments;
        std::vector<LineSegment> right_segments;
        double heading_correction = 0.0;  // Sum of length * signed angular difference.
        double total_support = 0.0;       // Sum of associated segment lengths, in metres.
    };

    WallAssociation associate_walls(
        const std::vector<LineSegment> & segments,
        const Pose2D & robot,
        double scan_time
    ) const;
    void calculate_axis(
        const std::vector<LineSegment> & segments,
        const Pose2D & robot,
        bool left_available,
        bool right_available,
        double total_support,
        CorridorEstimate & estimate
    ) const;
    std::vector<LineSegment> extract_segments(const std::vector<Point> & points) const;
    void split_segment(
        const std::vector<Point> & points,
        std::size_t begin,
        std::size_t end,
        std::vector<LineSegment> & segments,
        int depth
    ) const;
    bool track_is_fresh(const WallTrack & track, const Pose2D & robot, double time) const;
    void update_wall(
        const std::vector<LineSegment> & segments,
        WallTrack & track,
        const Pose2D & robot,
        double time
    );
    std::vector<LineSegment> find_openings(std::vector<LineSegment> segments) const;
    bool terminal_wall_evidence(
        const std::vector<LineSegment> & segments,
        const Pose2D & robot,
        double left,
        double right
    ) const;

    GeometryConfiguration configuration_;

    // Persistent evidence from distinct scans; all anchors remain in encoder odom:
    bool initialized_ = false;
    double heading_ = 0.0;
    double previous_time_ = -1.0;
    WallTrack left_;
    WallTrack right_;
};
}  // namespace perception
