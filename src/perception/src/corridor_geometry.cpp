#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "perception/corridor_geometry.hpp"

namespace perception
{
namespace
{
constexpr int MAX_SPLIT_DEPTH = 12;
constexpr double HEADING_UPDATE_GAIN = 0.25;
constexpr double FULL_CONFIDENCE_SUPPORT = 3.0;  // Metres of associated wall support.
constexpr double TERMINAL_COVERAGE_FRACTION = 0.75;

// Return a signed angle without changing its physical orientation:
double wrap_angle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}


// Compare positions in the same odometry frame:
double distance(const Point & first, const Point & second)
{
    return std::hypot(first.x - second.x, first.y - second.y);
}
}  // namespace


// Reject unusable geometry limits before processing any scans:
CorridorGeometry::CorridorGeometry(const GeometryConfiguration & configuration)
    : configuration_(configuration)
{
    for(double limit : {
        configuration_.point_gap, configuration_.max_residual,
        configuration_.min_segment_length, configuration_.max_heading_difference,
        configuration_.wall_association_distance, configuration_.min_width,
        configuration_.max_width, configuration_.min_opening_width,
        configuration_.max_opening_width, configuration_.prediction_time,
        configuration_.prediction_distance
    }) {
        if(!std::isfinite(limit) || limit <= 0.0) {
            throw std::invalid_argument("Corridor geometry limits must be finite and positive");
        }
    }
    if(configuration_.min_segment_points < 3 ||
        configuration_.max_width <= configuration_.min_width ||
        configuration_.max_opening_width <= configuration_.min_opening_width ||
        configuration_.max_heading_difference >= 0.5) {
        throw std::invalid_argument("Invalid segment support, corridor width or angular limits");
    }
}


// Forget geometric history after discontinuous time or localization:
void CorridorGeometry::reset()
{
    initialized_ = false;
    previous_time_ = -1.0;
    left_ = WallTrack{};
    right_ = WallTrack{};
}


// Associate longitudinal walls, estimate the centerline and expose missing support explicitly:
CorridorEstimate CorridorGeometry::estimate(
    const std::vector<Point> & points,
    const Pose2D & robot,
    double scan_time
)
{
    // Reject unusable poses and duplicate acquisitions before updating history:
    CorridorEstimate result;
    if(!std::isfinite(scan_time) || !std::isfinite(robot.yaw) ||
        !std::isfinite(robot.position.x) || !std::isfinite(robot.position.y)) {
        return result;
    }
    if(scan_time < previous_time_) reset();
    if(scan_time == previous_time_) return result;
    previous_time_ = scan_time;

    // Robot yaw seeds the initial search direction only. Subsequent scans refine
    // the corridor heading from wall support rather than treating robot yaw as truth:
    if(!initialized_) {
        heading_ = robot.yaw;
        initialized_ = true;
    }

    // Extract straight support in beam order and associate it with the two walls:
    const auto segments = extract_segments(points);
    const WallAssociation association = associate_walls(segments, robot, scan_time);

    // Smooth the length-weighted orientation correction, then refresh measured
    // walls. Empty support leaves a wall's last real timestamp and anchor intact:
    if(association.total_support > 0.0) {
        heading_ += HEADING_UPDATE_GAIN * association.heading_correction /
            association.total_support;
    }
    update_wall(association.left_segments, left_, robot, scan_time);
    update_wall(association.right_segments, right_, robot, scan_time);

    // Distinguish support measured now from history still within prediction limits:
    const bool left_available = track_is_fresh(left_, robot, scan_time);
    const bool right_available = track_is_fresh(right_, robot, scan_time);
    result.left_segments = association.left_segments;
    result.right_segments = association.right_segments;
    result.left_measured = !result.left_segments.empty();
    result.right_measured = !result.right_segments.empty();
    result.left_predicted = left_available && !result.left_measured;
    result.right_predicted = right_available && !result.right_measured;
    result.left_age = left_.valid ? scan_time - left_.timestamp :
        std::numeric_limits<double>::infinity();
    result.right_age = right_.valid ? scan_time - right_.timestamp :
        std::numeric_limits<double>::infinity();
    result.heading = heading_;

    // Gaps require measured segments on that wall; predicted walls create no gaps:
    result.left_openings = find_openings(result.left_segments);
    result.right_openings = find_openings(result.right_segments);

    // Derive width, signed errors and transverse evidence only from usable support:
    calculate_axis(
        segments,
        robot,
        left_available,
        right_available,
        association.total_support,
        result
    );
    return result;
}


// Select longitudinal support near each structural wall without adopting room back walls:
CorridorGeometry::WallAssociation CorridorGeometry::associate_walls(
    const std::vector<LineSegment> & segments,
    const Pose2D & robot,
    double scan_time
) const
{
    // The normal points left of the current directed axis. Its dot product with
    // an odom point gives a signed lateral coordinate shared by walls and robot:
    const Point normal{-std::sin(heading_), std::cos(heading_)};
    const double robot_offset = normal.x * robot.position.x + normal.y * robot.position.y;
    std::vector<LineSegment> longitudinal;
    double left_offset = std::numeric_limits<double>::infinity();
    double right_offset = -left_offset;

    // A fitted line has no forward direction: theta and theta + pi are identical.
    // Wrapping the doubled difference and halving it compares these equivalent lines:
    for(const auto & segment : segments) {
        const double difference = wrap_angle(2.0 * (segment.heading - heading_)) / 2.0;
        if(std::abs(difference) > configuration_.max_heading_difference) continue;
        longitudinal.push_back(segment);
        const double offset = normal.x * segment.center.x + normal.y * segment.center.y;
        const double side = offset - robot_offset;
        if(side > 0.0) left_offset = std::min(left_offset, offset);
        else right_offset = std::max(right_offset, offset);
    }

    // Initially choose the nearest wall on each side. While a track remains fresh,
    // its anchor overrides that choice so a doorway cannot shift the corridor into a room:
    if(track_is_fresh(left_, robot, scan_time)) {
        left_offset = normal.x * left_.anchor.x + normal.y * left_.anchor.y;
    }
    if(track_is_fresh(right_, robot, scan_time)) {
        right_offset = normal.x * right_.anchor.x + normal.y * right_.anchor.y;
    }

    // Keep only support close to the chosen wall; unrelated parallel surfaces do
    // not contribute to the orientation correction or to opening detection:
    WallAssociation association;
    for(const auto & segment : longitudinal) {
        const double offset = normal.x * segment.center.x + normal.y * segment.center.y;
        if(offset > robot_offset &&
            std::abs(offset - left_offset) < configuration_.wall_association_distance) {
            association.left_segments.push_back(segment);
        }
        else if(offset < robot_offset &&
            std::abs(offset - right_offset) < configuration_.wall_association_distance) {
            association.right_segments.push_back(segment);
        }
        else {
            continue;
        }
        const double doubled_heading_difference = wrap_angle(2 * (segment.heading - heading_));
        association.heading_correction += segment.length * doubled_heading_difference / 2;
        association.total_support += segment.length;
    }
    return association;
}


// Project wall anchors onto the refined normal to calculate width and robot-axis errors:
void CorridorGeometry::calculate_axis(
    const std::vector<LineSegment> & segments,
    const Pose2D & robot,
    bool left_available,
    bool right_available,
    double total_support,
    CorridorEstimate & estimate
) const
{
    // Reproject after heading refinement so width and errors use the same normal:
    const Point corrected_normal{-std::sin(heading_), std::cos(heading_)};
    const double position = corrected_normal.x * robot.position.x +
        corrected_normal.y * robot.position.y;
    const double left = corrected_normal.x * left_.anchor.x + corrected_normal.y * left_.anchor.y;
    const double right = corrected_normal.x * right_.anchor.x +
        corrected_normal.y * right_.anchor.y;
    if(left_available) estimate.left_distance = left - position;
    if(right_available) estimate.right_distance = position - right;

    // One available side still provides a distance, but cannot define a valid width:
    if(!left_available || !right_available) return;
    estimate.width = left - right;
    estimate.width_valid = estimate.width >= configuration_.min_width &&
        estimate.width <= configuration_.max_width;
    if(!estimate.width_valid) return;

    // Positive lateral error means the robot lies left of the midpoint. Subtracting
    // that normal offset projects its odom position onto the corridor centerline:
    estimate.axis_valid = true;
    estimate.lateral_error = position - (left + right) / 2.0;
    estimate.heading_error = wrap_angle(robot.yaw - heading_);
    estimate.center = {
        robot.position.x - estimate.lateral_error * corrected_normal.x,
        robot.position.y - estimate.lateral_error * corrected_normal.y
    };

    // Confidence is a support heuristic, not a calibrated probability. Prediction
    // halves it; sufficient measured wall length saturates it at one:
    const double support_quality = std::min(1.0, total_support / FULL_CONFIDENCE_SUPPORT);
    estimate.confidence = support_quality *
        (estimate.left_measured && estimate.right_measured ? 1.0 : 0.5);

    // A transverse segment must cover most of this valid corridor, not just face the robot:
    estimate.terminal_candidate = terminal_wall_evidence(segments, robot, left, right);
}


// Preserve angular ordering and gaps instead of joining points across a doorway:
std::vector<LineSegment> CorridorGeometry::extract_segments(const std::vector<Point> & points) const
{
    std::vector<LineSegment> segments;
    std::size_t begin = 0;

    // The final iteration flushes the last run; invalid returns remain separators.
    // A large distance between consecutive valid returns starts a separate run:
    for(std::size_t index = 0; index <= points.size(); ++index) {
        const bool at_end = index == points.size();
        const bool invalid = !at_end &&
            (!std::isfinite(points[index].x) || !std::isfinite(points[index].y));
        const bool gap = !at_end && !invalid && index > begin &&
            distance(points[index - 1], points[index]) > configuration_.point_gap;
        if(at_end || invalid || gap) {
            split_segment(points, begin, index, segments, 0);
            // Skip an invalid point, but keep a valid point beyond a gap in the next run:
            begin = invalid ? index + 1 : index;
        }
    }
    return segments;
}


// Fit orthogonal least squares, then divide curved or corner-shaped sequences:
void CorridorGeometry::split_segment(
    const std::vector<Point> & points,
    std::size_t begin,
    std::size_t end,
    std::vector<LineSegment> & segments,
    int depth
) const
{
    const std::size_t count = end - begin;
    if(count < static_cast<std::size_t>(configuration_.min_segment_points)) return;

    // The centroid is the origin for a line fit that treats x and y symmetrically:
    Point center;
    for(std::size_t index = begin; index < end; ++index) {
        center.x += points[index].x / count;
        center.y += points[index].y / count;
    }

    // The dominant covariance direction minimizes squared perpendicular errors.
    // Unlike y = a*x + b, this also handles vertical walls without a singular slope:
    double covariance_xx = 0.0;
    double covariance_xy = 0.0;
    double covariance_yy = 0.0;
    for(std::size_t index = begin; index < end; ++index) {
        const double dx = points[index].x - center.x;
        const double dy = points[index].y - center.y;
        covariance_xx += dx * dx;
        covariance_xy += dx * dy;
        covariance_yy += dy * dy;
    }
    const double heading = 0.5 * std::atan2(
        2.0 * covariance_xy, covariance_xx - covariance_yy
    );
    const Point tangent{std::cos(heading), std::sin(heading)};
    const Point normal{-tangent.y, tangent.x};

    // Project onto the tangent to find endpoints and onto the normal to test fit quality:
    double maximum_residual = 0.0;
    double minimum_projection = std::numeric_limits<double>::infinity();
    double maximum_projection = -minimum_projection;
    std::size_t split = begin;
    for(std::size_t index = begin; index < end; ++index) {
        const Point offset{points[index].x - center.x, points[index].y - center.y};
        const double residual = std::abs(normal.x * offset.x + normal.y * offset.y);
        if(residual > maximum_residual) {
            maximum_residual = residual;
            split = index;
        }
        const double projection = tangent.x * offset.x + tangent.y * offset.y;
        minimum_projection = std::min(minimum_projection, projection);
        maximum_projection = std::max(maximum_projection, projection);
    }

    // A poor fit may contain a corner. Refit both sides of the worst point;
    // retain it in both subsets so the corner does not create an artificial gap:
    if(maximum_residual > configuration_.max_residual) {
        if(depth >= MAX_SPLIT_DEPTH) return;
        if(split <= begin + 1 || split >= end - 2) split = begin + count / 2;
        split_segment(points, begin, split + 1, segments, depth + 1);
        split_segment(points, split, end, segments, depth + 1);
        return;
    }

    // Only sufficiently long, approximately straight support becomes a wall segment:
    const double length = maximum_projection - minimum_projection;
    if(length < configuration_.min_segment_length) return;
    segments.push_back({
        {center.x + minimum_projection * tangent.x, center.y + minimum_projection * tangent.y},
        {center.x + maximum_projection * tangent.x, center.y + maximum_projection * tangent.y},
        center, heading, length, maximum_residual
    });
}


// Bound a missing wall by its last real observation, not the latest publication:
bool CorridorGeometry::track_is_fresh(
    const WallTrack & track, const Pose2D & robot, double time
) const
{
    // Prediction expires with time OR travel; republishing cannot extend either limit:
    const double age = time - track.timestamp;
    return track.valid && age >= 0.0 && age <= configuration_.prediction_time &&
        distance(track.observed_at, robot.position) <= configuration_.prediction_distance;
}


// Combine collinear support while retaining the segments used to detect gaps:
void CorridorGeometry::update_wall(
    const std::vector<LineSegment> & segments,
    WallTrack & track,
    const Pose2D & robot,
    double time
)
{
    if(segments.empty()) return;

    // Longer measured fragments contribute more to the shared wall anchor:
    Point anchor;
    double support = 0.0;
    for(const LineSegment & segment : segments) {
        anchor.x += segment.center.x * segment.length;
        anchor.y += segment.center.y * segment.length;
        support += segment.length;
    }

    // Only a real measurement refreshes the observation position and scan timestamp:
    track = {true, {anchor.x / support, anchor.y / support}, robot.position, time};
}


// Report gaps between observed wall segments, without claiming a registered room entrance:
std::vector<LineSegment> CorridorGeometry::find_openings(std::vector<LineSegment> segments) const
{
    const Point tangent{std::cos(heading_), std::sin(heading_)};

    // Orient all endpoints along the corridor, then sort them by longitudinal position:
    for(LineSegment & segment : segments) {
        const double direction = tangent.x * (segment.last.x - segment.first.x) +
            tangent.y * (segment.last.y - segment.first.y);
        if(direction < 0.0) std::swap(segment.first, segment.last);
    }
    std::sort(segments.begin(), segments.end(), [&tangent](const auto & first, const auto & last) {
        return tangent.x * first.first.x + tangent.y * first.first.y <
            tangent.x * last.first.x + tangent.y * last.first.y;
    });
    std::vector<LineSegment> openings;
    if(segments.empty()) return openings;

    // Measure gaps along the axis, not between arbitrary LiDAR rays. Overlapping
    // fragments extend existing support and must not manufacture an opening:
    Point previous_end = segments.front().last;
    for(std::size_t index = 1; index < segments.size(); ++index) {
        const Point start = segments[index].first;
        const double gap = tangent.x * (start.x - previous_end.x) +
            tangent.y * (start.y - previous_end.y);
        if(gap >= configuration_.min_opening_width && gap <= configuration_.max_opening_width) {
            openings.push_back({previous_end, start, {}, heading_, gap, 0.0});
        }
        const Point end = segments[index].last;
        if(tangent.x * (end.x - previous_end.x) + tangent.y * (end.y - previous_end.y) > 0.0) {
            previous_end = end;
        }
    }
    return openings;
}


// Combine transverse fragments, including the two ends of a full-circle scan:
bool CorridorGeometry::terminal_wall_evidence(
    const std::vector<LineSegment> & segments,
    const Pose2D & robot,
    double left,
    double right
) const
{
    const Point tangent{std::cos(heading_), std::sin(heading_)};
    const Point normal{-tangent.y, tangent.x};
    std::vector<LineSegment> transverse;

    // Retain approximately perpendicular support in front of the robot:
    for(const auto & segment : segments) {
        const double parallel = std::abs(std::cos(segment.heading - heading_));
        const double forward = tangent.x * (segment.center.x - robot.position.x) +
            tangent.y * (segment.center.y - robot.position.y);
        if(parallel < std::sin(configuration_.max_heading_difference) && forward > 0.0) {
            transverse.push_back(segment);
        }
    }

    // Each reference defines a possible transverse plane. Project nearby fragments
    // onto the corridor normal and clip their coverage to the two structural walls:
    for(const auto & reference : transverse) {
        std::vector<std::pair<double, double>> intervals;
        for(const auto & segment : transverse) {
            const double longitudinal_difference = tangent.x *
                (segment.center.x - reference.center.x) + tangent.y *
                (segment.center.y - reference.center.y);
            if(std::abs(longitudinal_difference) > configuration_.wall_association_distance) {
                continue;
            }
            const double first_side = normal.x * segment.first.x + normal.y * segment.first.y;
            const double last_side = normal.x * segment.last.x + normal.y * segment.last.y;
            const double begin = std::max(right, std::min(first_side, last_side));
            const double end = std::min(left, std::max(first_side, last_side));
            if(end > begin) intervals.emplace_back(begin, end);
        }
        // Merge overlapping intervals before summing coverage. This also combines
        // a wall split between the beginning and end of a full-circle LiDAR scan:
        std::sort(intervals.begin(), intervals.end());
        double covered = 0.0;
        double covered_until = right;
        for(const auto & interval : intervals) {
            covered += std::max(0.0, interval.second - std::max(covered_until, interval.first));
            covered_until = std::max(covered_until, interval.second);
        }
        if(covered > TERMINAL_COVERAGE_FRACTION * (left - right)) return true;
    }
    return false;
}
}  // namespace perception
