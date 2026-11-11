#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>
#include <squad_interfaces/msg/corridor_observation.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "perception/corridor_geometry.hpp"

/*
Local corridor interpretation; this node never commands motion.

scan_callback stores the latest scan and its monotonic receipt time. The 20 Hz
process_scan timer checks metadata, acquisition age and duplicate timestamps.
interpret_scan then looks up scan-time TF, projects rays into encoder odom,
updates CorridorGeometry, and builds an observation in the scan-time base frame.
Finally, process_scan publishes corridor_observation and the matching RViz markers.

Wall history stays in encoder odom so robot motion does not move a tracked wall.
TF lookup is non-blocking: a fresh scan may wait for TF without renewing the
previous observation's timestamp. Invalid/stale inputs withdraw the geometry.
*/
class CorridorPerception : public rclcpp::Node
{
    public:

    CorridorPerception();

    private:

    using Observation = squad_interfaces::msg::CorridorObservation;
    using Marker = visualization_msgs::msg::Marker;

    void read_parameters();
    void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr message);
    void process_scan();
    bool scan_is_valid() const;
    Observation interpret_scan();
    void update_tracking_pose(const perception::Pose2D & pose, double scan_time);
    std::vector<perception::Point> project_scan_to_odom(
        const tf2::Transform & laser_to_odom,
        double & front_clearance
    ) const;
    Observation make_observation(
        const perception::CorridorEstimate & estimate,
        double front_clearance
    ) const;
    geometry_msgs::msg::Point point_in_base_frame(const perception::Point & point) const;
    void append_segment_endpoints(
        const std::vector<perception::LineSegment> & segments,
        std::vector<geometry_msgs::msg::Point> & endpoints
    ) const;
    void publish_invalid(const std::string & reason);
    void publish_markers(const Observation & observation);
    Marker make_lines(
        int id,
        const std_msgs::msg::Header & header,
        const std::vector<geometry_msgs::msg::Point> & points,
        float red,
        float green,
        float blue
    ) const;

    // Fixed processing bounds; these retain the initial implementation's tolerances:
    static constexpr std::size_t MAX_SCAN_POINTS = 8192;
    static constexpr double FUTURE_SCAN_TOLERANCE = 0.05;  // Seconds of ROS time.
    static constexpr double MAX_TRACKING_POSE_JUMP = 0.5;  // Metres between scans.

    // Configuration: robot-local frames, sensor freshness and frontal footprint band:
    std::string base_frame_;
    std::string scan_frame_;
    std::string odom_frame_;
    double scan_timeout_ = 0.0;
    double footprint_front_ = 0.0;
    double footprint_half_width_ = 0.0;
    double front_band_margin_ = 0.0;

    // Tracking state: history in odom and the base pose at the current scan's timestamp:
    std::unique_ptr<perception::CorridorGeometry> geometry_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
    bool processed_ = false;
    double previous_scan_time_ = -1.0;
    perception::Pose2D robot_pose_;
    bool previous_pose_valid_ = false;

    // Arrival age uses a monotonic clock; scan acquisition and TF use ROS time:
    std::chrono::steady_clock::time_point scan_arrival_{};
    tf2_ros::Buffer transform_buffer_;
    tf2_ros::TransformListener transform_listener_;

    // Observations and diagnostics share the same measurement timestamp:
    rclcpp::Publisher<Observation>::SharedPtr observation_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};


// Establish geometry limits, frames and communication before processing scans:
CorridorPerception::CorridorPerception()
    : Node("corridor_perception"),
      transform_buffer_(get_clock()),
      transform_listener_(transform_buffer_)
{
    read_parameters();

    // Keep only the newest output; consumers use validity flags and acquisition time:
    observation_publisher_ = create_publisher<Observation>("corridor_observation", 1);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "corridor_markers", 1
    );

    // SensorDataQoS accepts the LiDAR's best-effort stream without queueing old scans:
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS(),
        std::bind(&CorridorPerception::scan_callback, this, std::placeholders::_1)
    );

    // A wall timer keeps checking receipt age even when simulation time stops:
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&CorridorPerception::process_scan, this)
    );
}


// Read metre/radian/second limits and share the protection footprint through the launch:
void CorridorPerception::read_parameters()
{
    // Namespace isolates encoder odom and sensor frames for each robot:
    const std::string robot = std::string(get_namespace()).substr(1);
    base_frame_ = robot + "/base_footprint";
    scan_frame_ = robot + "/base_scan";
    odom_frame_ = robot + "/odom";

    // The launch supplies footprint dimensions from the existing safety configuration:
    scan_timeout_ = declare_parameter("scan_timeout", 0.3);
    footprint_front_ = declare_parameter("footprint_front", 0.11);
    footprint_half_width_ = declare_parameter("footprint_half_width", 0.11);
    front_band_margin_ = declare_parameter("front_band_margin", 0.08);
    for(double limit : {
        scan_timeout_, footprint_front_, footprint_half_width_, front_band_margin_
    }) {
        if(!std::isfinite(limit) || limit <= 0.0) {
            throw std::invalid_argument("Perception timeouts and footprint must be positive");
        }
    }

    // GeometryConfiguration owns the default fit/association limits. CorridorGeometry
    // validates them before any measurement can update history:
    perception::GeometryConfiguration configuration;
    configuration.point_gap = declare_parameter("point_gap", configuration.point_gap);
    configuration.max_residual = declare_parameter("max_residual", configuration.max_residual);
    configuration.min_segment_length = declare_parameter(
        "min_segment_length", configuration.min_segment_length
    );
    configuration.min_segment_points = declare_parameter(
        "min_segment_points", configuration.min_segment_points
    );
    configuration.max_heading_difference = declare_parameter(
        "max_heading_difference", configuration.max_heading_difference
    );
    configuration.wall_association_distance = declare_parameter(
        "wall_association_distance", configuration.wall_association_distance
    );
    configuration.min_width = declare_parameter("min_width", configuration.min_width);
    configuration.max_width = declare_parameter("max_width", configuration.max_width);
    configuration.min_opening_width = declare_parameter(
        "min_opening_width", configuration.min_opening_width
    );
    configuration.max_opening_width = declare_parameter(
        "max_opening_width", configuration.max_opening_width
    );
    configuration.prediction_time = declare_parameter(
        "prediction_time", configuration.prediction_time
    );
    configuration.prediction_distance = declare_parameter(
        "prediction_distance", configuration.prediction_distance
    );
    geometry_ = std::make_unique<perception::CorridorGeometry>(configuration);
}


// Retain the acquisition time and receipt time separately; repeated scans stay repeated:
void CorridorPerception::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
    scan_ = message;
    scan_arrival_ = std::chrono::steady_clock::now();
    processed_ = false;
}


// Process only fresh distinct measurements and invalidate outputs on missing inputs:
void CorridorPerception::process_scan()
{
    // Reject malformed input before timestamp arithmetic or transform lookups:
    if(!scan_is_valid()) {
        publish_invalid("SCAN_INVALID");
        return;
    }

    // Check acquisition age in ROS time AND receipt age on the monotonic clock.
    // A stopped clock or a repeated sensor packet must not preserve stale geometry:
    const double scan_time = rclcpp::Time(scan_->header.stamp).seconds();
    const double acquisition_age = now().seconds() - scan_time;
    const double receipt_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - scan_arrival_
    ).count();
    if(acquisition_age < -FUTURE_SCAN_TOLERANCE || acquisition_age > scan_timeout_ ||
        receipt_age > scan_timeout_) {
        publish_invalid("SCAN_STALE");
        return;
    }

    // One acquisition may update wall history only once, regardless of timer frequency:
    if(processed_ || scan_time == previous_scan_time_) return;
    try {
        // Publish geometry and diagnostics together, retaining the scan's timestamp:
        const Observation observation = interpret_scan();
        observation_publisher_->publish(observation);
        publish_markers(observation);
        previous_scan_time_ = scan_time;
        processed_ = true;
    }
    catch(const tf2::TransformException &) {
        // A newly arrived scan can precede its TF by one callback. Keep the previous
        // observation only within its original freshness window; never restamp it:
        const double previous_age = now().seconds() - previous_scan_time_;
        if(previous_scan_time_ < 0.0 || previous_age < 0.0 || previous_age > scan_timeout_) {
            publish_invalid("SCAN_TIME_TF_UNAVAILABLE");
        }
    }
}


// Bound the sensor work and validate metadata before allocating or looking up TF:
bool CorridorPerception::scan_is_valid() const
{
    if(!scan_ || scan_->header.frame_id != scan_frame_) return false;
    if(scan_->header.stamp.sec < 0 || scan_->header.stamp.nanosec >= 1000000000U) return false;
    if(scan_->ranges.size() < 3 || scan_->ranges.size() > MAX_SCAN_POINTS) return false;

    // Angle and range metadata define the polar-to-Cartesian projection. Individual
    // missing returns are handled later as separators rather than invalidating a scan:
    const bool valid_angles = std::isfinite(scan_->angle_min) &&
        std::isfinite(scan_->angle_max) && std::isfinite(scan_->angle_increment) &&
        scan_->angle_increment > 0.0;
    const bool valid_ranges = std::isfinite(scan_->range_min) &&
        std::isfinite(scan_->range_max) && scan_->range_min > 0.0 &&
        scan_->range_max > scan_->range_min;
    return valid_angles && valid_ranges;
}


// Transform rays to odom for tracking and return observations in the scan-time base frame:
CorridorPerception::Observation CorridorPerception::interpret_scan()
{
    const rclcpp::Time timestamp(scan_->header.stamp);

    // Use the acquisition-time transforms, never the latest robot pose. All rays
    // share this rigid transform; per-ray motion compensation is not implemented:
    const auto laser_transform = transform_buffer_.lookupTransform(
        odom_frame_, scan_frame_, timestamp
    );
    const auto base_transform = transform_buffer_.lookupTransform(
        odom_frame_, base_frame_, timestamp
    );
    tf2::Transform laser_to_odom;
    tf2::fromMsg(laser_transform.transform, laser_to_odom);

    // Extract the planar base pose in odom from the TF translation and quaternion:
    const auto & rotation = base_transform.transform.rotation;
    const double yaw = std::atan2(
        2 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z)
    );
    const perception::Pose2D pose{
        {base_transform.transform.translation.x, base_transform.transform.translation.y}, yaw
    };
    update_tracking_pose(pose, timestamp.seconds());

    // Keep ray order and missing-return gaps while projecting support into odom:
    double front_clearance = std::numeric_limits<double>::infinity();
    const auto points = project_scan_to_odom(laser_to_odom, front_clearance);

    // The geometry library fits/associates walls and estimates the corridor axis.
    // Output conversion returns that estimate to the same scan-time robot base:
    const auto estimate = geometry_->estimate(points, pose, timestamp.seconds());
    return make_observation(estimate, front_clearance);
}


// Discard wall history across clock resets or discontinuous localization, then store the pose:
void CorridorPerception::update_tracking_pose(const perception::Pose2D & pose, double scan_time)
{
    const bool time_moved_backwards = scan_time < previous_scan_time_;
    const double displacement = std::hypot(
        pose.position.x - robot_pose_.position.x,
        pose.position.y - robot_pose_.position.y
    );
    if(previous_pose_valid_ &&
        (time_moved_backwards || displacement > MAX_TRACKING_POSE_JUMP)) {
        geometry_->reset();
    }
    robot_pose_ = pose;
    previous_pose_valid_ = true;
}


// Convert scan returns to ordered odom points and measure clearance ahead of the footprint:
std::vector<perception::Point> CorridorPerception::project_scan_to_odom(
    const tf2::Transform & laser_to_odom,
    double & front_clearance
) const
{
    std::vector<perception::Point> points;
    points.reserve(scan_->ranges.size());
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    for(std::size_t index = 0; index < scan_->ranges.size(); ++index) {
        const double range = scan_->ranges[index];
        if(!std::isfinite(range) || range < scan_->range_min || range > scan_->range_max) {
            // Preserve an invalid ray's place so line extraction cannot bridge the gap:
            points.push_back({unavailable, unavailable});
            continue;
        }

        // Polar LiDAR coordinates -> Cartesian sensor coordinates -> encoder odom:
        const double angle = scan_->angle_min + index * scan_->angle_increment;
        const double sensor_x = range * std::cos(angle);
        const double sensor_y = range * std::sin(angle);
        const tf2::Vector3 point = laser_to_odom * tf2::Vector3(sensor_x, sensor_y, 0.0);
        points.push_back({point.x(), point.y()});

        // Evaluate forward clearance in the base frame: only returns in the frontal
        // footprint band count. Measure from its front edge, clamping overlap to zero:
        const auto base_point = point_in_base_frame(points.back());
        if(base_point.x >= 0.0 &&
            std::abs(base_point.y) < footprint_half_width_ + front_band_margin_) {
            front_clearance = std::min(
                front_clearance, std::max(0.0, base_point.x - footprint_front_)
            );
        }
    }
    return points;
}


// Encode validity, measured/predicted support and scan-time base-frame geometry for consumers:
CorridorPerception::Observation CorridorPerception::make_observation(
    const perception::CorridorEstimate & estimate,
    double front_clearance
) const
{
    // Input validity describes scan/TF availability; axis validity describes the
    // geometry. Fresh sensor data may legitimately contain no usable corridor:
    Observation observation;
    observation.header.stamp = scan_->header.stamp;
    observation.header.frame_id = base_frame_;
    observation.input_valid = true;
    observation.axis_valid = estimate.axis_valid;
    observation.width_valid = estimate.width_valid;
    observation.status = estimate.axis_valid ? "MEASURED" : "GEOMETRY_UNAVAILABLE";
    if(estimate.left_predicted || estimate.right_predicted) {
        observation.status = "PREDICTED";
    }

    // Rotate/translate the axis into base_footprint, with x forward and y left.
    // The planar quaternion represents axis heading relative to the measured base:
    observation.axis.position = point_in_base_frame(estimate.center);
    const double relative_heading = estimate.heading - robot_pose_.yaw;
    observation.axis.orientation.z = std::sin(relative_heading / 2.0);
    observation.axis.orientation.w = std::cos(relative_heading / 2.0);
    observation.width = estimate.width;
    observation.lateral_error = estimate.lateral_error;
    observation.heading_error = estimate.heading_error;

    // Measured flags and prediction flags remain separate; age is since real support:
    observation.left_wall_valid = estimate.left_measured;
    observation.right_wall_valid = estimate.right_measured;
    observation.left_predicted = estimate.left_predicted;
    observation.right_predicted = estimate.right_predicted;
    observation.left_wall_distance = estimate.left_distance;
    observation.right_wall_distance = estimate.right_distance;
    observation.left_wall_age = estimate.left_age;
    observation.right_wall_age = estimate.right_age;
    observation.confidence = estimate.confidence;

    // No return in the frontal band means unavailable clearance, not infinite free space:
    observation.front_clearance_valid = std::isfinite(front_clearance);
    observation.front_clearance = std::isfinite(front_clearance) ? front_clearance :
        std::numeric_limits<double>::quiet_NaN();
    observation.terminal_wall_candidate = estimate.terminal_candidate;

    // Keep endpoint pairs for measured segments and candidate gaps in the same frame:
    append_segment_endpoints(estimate.left_segments, observation.left_segments);
    append_segment_endpoints(estimate.right_segments, observation.right_segments);
    append_segment_endpoints(estimate.left_openings, observation.left_openings);
    append_segment_endpoints(estimate.right_openings, observation.right_openings);
    return observation;
}


// Express an odometry point in the measured robot base, preserving x-forward/y-left:
geometry_msgs::msg::Point CorridorPerception::point_in_base_frame(
    const perception::Point & point
) const
{
    // Invert the base pose: p_base = R(-yaw) * (p_odom - base_position_odom):
    const double dx = point.x - robot_pose_.position.x;
    const double dy = point.y - robot_pose_.position.y;
    geometry_msgs::msg::Point local;
    local.x = std::cos(robot_pose_.yaw) * dx + std::sin(robot_pose_.yaw) * dy;
    local.y = -std::sin(robot_pose_.yaw) * dx + std::cos(robot_pose_.yaw) * dy;
    return local;
}


// Keep observed support and openings as separate endpoint pairs for diagnostics:
void CorridorPerception::append_segment_endpoints(
    const std::vector<perception::LineSegment> & segments,
    std::vector<geometry_msgs::msg::Point> & endpoints
) const
{
    for(const auto & segment : segments) {
        endpoints.push_back(point_in_base_frame(segment.first));
        endpoints.push_back(point_in_base_frame(segment.last));
    }
}


// Withdraw geometry instead of retaining a previously valid estimate after input loss:
void CorridorPerception::publish_invalid(const std::string & reason)
{
    Observation observation;
    observation.header.frame_id = base_frame_;
    if(scan_) {
        observation.header.stamp = scan_->header.stamp;
    }
    else {
        observation.header.stamp = now();
    }

    // Message flags default to false; NaN prevents missing metrics from appearing as zero:
    observation.status = reason;
    observation.width = std::numeric_limits<double>::quiet_NaN();
    observation.lateral_error = observation.width;
    observation.heading_error = observation.width;
    observation.front_clearance = observation.width;
    observation_publisher_->publish(observation);
    publish_markers(observation);
}


// Show measured walls, candidate openings, estimated axis and validity without console spam:
void CorridorPerception::publish_markers(const Observation & observation)
{
    // Stable layers: measured left/right support, then candidate openings on each side:
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(make_lines(
        0,
        observation.header,
        observation.left_segments,
        0.1F,
        1.0F,
        0.2F
    ));
    markers.markers.push_back(make_lines(
        1,
        observation.header,
        observation.right_segments,
        0.1F,
        0.5F,
        1.0F
    ));
    markers.markers.push_back(make_lines(
        2,
        observation.header,
        observation.left_openings,
        1.0F,
        0.5F,
        0.0F
    ));
    markers.markers.push_back(make_lines(
        3,
        observation.header,
        observation.right_openings,
        1.0F,
        0.5F,
        0.0F
    ));

    // Draw the axis from one metre behind to two metres ahead of its projected center:
    std::vector<geometry_msgs::msg::Point> axis;
    if(observation.axis_valid) {
        const auto & orientation = observation.axis.orientation;
        const double heading = 2 * std::atan2(orientation.z, orientation.w);
        auto start = observation.axis.position;
        auto end = start;
        start.x -= std::cos(heading);
        start.y -= std::sin(heading);
        end.x += 2 * std::cos(heading);
        end.y += 2 * std::sin(heading);
        axis = {start, end};
    }
    markers.markers.push_back(make_lines(
        4,
        observation.header,
        axis,
        1.0F,
        1.0F,
        0.1F
    ));

    // A short lifetime also removes diagnostics if publication itself stops:
    Marker text;
    text.header = observation.header;
    text.ns = "corridor";
    text.id = 5;
    text.type = Marker::TEXT_VIEW_FACING;
    text.pose.position.z = 0.65;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.16;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;
    text.lifetime = rclcpp::Duration::from_seconds(0.5);
    text.text = observation.status;
    if(observation.axis_valid) {
        text.text += " | width " + std::to_string(observation.width).substr(0, 4) + " m";
    }
    if(observation.terminal_wall_candidate) {
        text.text += " | transverse wall candidate";
    }
    markers.markers.push_back(text);
    marker_publisher_->publish(markers);
}


// Build one short-lived visual layer so stale geometry disappears from RViz:
CorridorPerception::Marker CorridorPerception::make_lines(
    int id,
    const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Point> & points,
    float red,
    float green,
    float blue
) const
{
    Marker marker;
    marker.header = header;
    marker.ns = "corridor";
    marker.id = id;
    marker.type = Marker::LINE_LIST;

    // Stable IDs delete the previous layer explicitly when its support disappears:
    marker.action = points.empty() ? Marker::DELETE : Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.045;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.points = points;
    return marker;
}


// Serialize tracking and scan callbacks in a single executor:
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorPerception>());
    rclcpp::shutdown();
    return 0;
}
