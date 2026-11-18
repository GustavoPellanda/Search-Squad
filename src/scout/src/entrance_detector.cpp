#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <squad_interfaces/msg/corridor_observation.hpp>
#include <squad_interfaces/msg/entrance_array.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

/*
Scout room-entrance detection, independent of steering and velocity commands.

The callbacks retain corridor observations, a short scan queue and the shared map.
process_observation pairs an observation with its original scan, validates timing,
looks up acquisition-time TF and checks rays crossing each measured wall gap.
update_entrance associates these measurements in encoder odom, requiring distinct
scans and a changed viewpoint before assigning a stable session ID. The last
supported pose is projected into map and published with width, count and timestamp.
publish_registry retains a complete list and width/normal/label markers for RViz.

Unknown map cells are neutral evidence. Registered coordinates are historical
scan-time projections, not landmarks optimized by SLAM. Large map-to-odom jumps
withdraw old projections until reobservation; gradual drift and non-rigid pose-graph
revisions remain localization limitations. No simulator geometry is read here.
*/
class EntranceDetector : public rclcpp::Node
{
    public:

    EntranceDetector();

    private:

    using Observation = squad_interfaces::msg::CorridorObservation;
    using Scan = sensor_msgs::msg::LaserScan;
    using Grid = nav_msgs::msg::OccupancyGrid;
    using Marker = visualization_msgs::msg::Marker;
    using EntranceArray = squad_interfaces::msg::EntranceArray;

    struct Candidate {
        tf2::Vector3 center;  // Encoder odom, metres.
        double inward_yaw = 0.0;  // Room-facing normal in odom, radians.
        double width = 0.0;
    };

    struct EntranceTrack {
        Candidate geometry;
        tf2::Vector3 first_viewpoint;
        rclcpp::Time last_observed{0, 0, RCL_ROS_TIME};
        rclcpp::Time pose_stamp{0, 0, RCL_ROS_TIME};
        double best_view_distance = std::numeric_limits<double>::infinity();
        std::uint32_t observations = 0;
        unsigned int id = 0;  // Zero is an unconfirmed candidate, never published.
        geometry_msgs::msg::Pose map_pose;
        bool map_pose_valid = false;
    };

    void read_parameters();
    void validate_parameters() const;
    void initialize_communication();
    void observation_callback(Observation::ConstSharedPtr message);
    void scan_callback(Scan::ConstSharedPtr message);
    void map_callback(Grid::ConstSharedPtr message);
    void detection_loop();
    void process_observation();
    bool observation_is_valid() const;
    bool scan_is_valid(const Scan & scan) const;
    void update_reference_frames(
        const tf2::Transform & odom_from_base,
        const tf2::Transform & map_from_odom
    );
    void process_wall_openings(
        const std::vector<geometry_msgs::msg::Point> & endpoints,
        double side,
        const Scan & scan,
        const tf2::Transform & base_from_laser,
        const tf2::Transform & odom_from_base,
        const tf2::Transform & map_from_odom
    );
    std::optional<Candidate> interpret_opening(
        const geometry_msgs::msg::Point & first,
        const geometry_msgs::msg::Point & last,
        double side,
        const Scan & scan,
        const tf2::Transform & base_from_laser,
        const tf2::Transform & odom_from_base
    ) const;
    bool rays_cross_opening(
        const tf2::Vector3 & center,
        const tf2::Vector3 & tangent,
        const tf2::Vector3 & inward,
        double width,
        const Scan & scan,
        const tf2::Transform & base_from_laser
    ) const;
    bool refine_jambs(
        tf2::Vector3 & center,
        double & width,
        const tf2::Vector3 & tangent,
        const tf2::Vector3 & inward,
        const Scan & scan,
        const tf2::Transform & base_from_laser
    ) const;
    geometry_msgs::msg::Pose project_to_map(
        const Candidate & candidate,
        const tf2::Transform & map_from_odom
    ) const;
    bool map_blocks_opening(const geometry_msgs::msg::Pose & pose, double width) const;
    void update_entrance(
        const Candidate & candidate,
        const tf2::Vector3 & viewpoint,
        const tf2::Transform & map_from_odom
    );
    void reset_registry();
    void publish_registry();
    void append_entrance_markers(
        const EntranceTrack & track,
        visualization_msgs::msg::MarkerArray & markers
    ) const;
    static bool point_is_finite(const geometry_msgs::msg::Point & point);
    static bool pose_is_planar(const geometry_msgs::msg::Pose & pose);
    static double yaw_of(const tf2::Transform & transform);

    static constexpr double PI = 3.14159265358979323846;
    static constexpr std::size_t MAX_SCAN_POINTS = 8192;
    static constexpr std::size_t MAX_QUEUED_SCANS = 16;
    static constexpr std::size_t MAX_TRACKS = 128;
    static constexpr double FUTURE_TOLERANCE = 0.05;  // ROS seconds.

    // Parameters: frames, freshness, confirmation, association and passage evidence:
    std::string base_frame_;
    std::string odom_frame_;
    std::string map_frame_;
    double observation_timeout_ = 0.0;
    double map_timeout_ = 0.0;
    int min_observations_ = 0;
    double min_viewpoint_change_ = 0.0;
    double max_view_distance_ratio_ = 0.0;
    double candidate_timeout_ = 0.0;
    double association_distance_ = 0.0;
    double width_tolerance_ = 0.0;
    double orientation_tolerance_ = 0.0;
    double min_opening_width_ = 0.0;
    double max_opening_width_ = 0.0;
    double max_endpoint_offset_ = 0.0;
    double wall_return_tolerance_ = 0.0;
    double edge_margin_ = 0.0;
    double min_open_depth_ = 0.0;
    int min_crossing_rays_ = 0;
    int occupied_threshold_ = 0;
    double max_odom_jump_ = 0.0;
    double max_map_translation_jump_ = 0.0;
    double max_map_rotation_jump_ = 0.0;

    // Preserve original stamps; scan and observation callbacks may arrive in either order:
    Observation::ConstSharedPtr observation_;
    std::deque<Scan::ConstSharedPtr> scans_;
    Grid::ConstSharedPtr map_;
    tf2::Transform grid_from_map_;
    std::chrono::steady_clock::time_point observation_arrival_{};
    double last_processed_time_ = -1.0;
    double previous_clock_time_ = -1.0;

    // Session tracks stay in encoder odom, while each public pose records its last map projection:
    std::vector<EntranceTrack> tracks_;
    unsigned int next_id_ = 1;
    bool previous_frames_valid_ = false;
    tf2::Transform previous_odom_from_base_;
    tf2::Transform previous_map_from_odom_;
    bool registry_changed_ = true;
    std::set<unsigned int> published_marker_ids_;
    std::chrono::steady_clock::time_point last_publication_{};

    tf2_ros::Buffer transform_buffer_;
    tf2_ros::TransformListener transform_listener_;
    rclcpp::Subscription<Observation>::SharedPtr observation_subscription_;
    rclcpp::Subscription<Scan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<Grid>::SharedPtr map_subscription_;
    rclcpp::Publisher<EntranceArray>::SharedPtr entrance_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


// Establish valid parameters and interfaces before starting the non-blocking processing timer:
EntranceDetector::EntranceDetector()
    : Node("entrance_detector"),
      transform_buffer_(get_clock()),
      transform_listener_(transform_buffer_)
{
    read_parameters();
    validate_parameters();
    initialize_communication();
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&EntranceDetector::detection_loop, this)
    );
}


// Resolve robot-local frames and read the limits documented in entrance_detection.yaml:
void EntranceDetector::read_parameters()
{
    const std::string robot = std::string(get_namespace()).substr(1);
    const std::string prefix = robot.empty() ? "" : robot + "/";
    base_frame_ = declare_parameter("base_frame", prefix + "base_footprint");
    odom_frame_ = declare_parameter("odom_frame", prefix + "odom");
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    observation_timeout_ = declare_parameter("observation_timeout", 0.3);
    map_timeout_ = declare_parameter("map_timeout", 3.0);
    min_observations_ = declare_parameter("min_observations", 4);
    min_viewpoint_change_ = declare_parameter("min_viewpoint_change", 0.08);
    max_view_distance_ratio_ = declare_parameter("max_view_distance_ratio", 1.10);
    candidate_timeout_ = declare_parameter("candidate_timeout", 2.0);
    association_distance_ = declare_parameter("association_distance", 0.35);
    width_tolerance_ = declare_parameter("width_tolerance", 0.35);
    orientation_tolerance_ = declare_parameter("orientation_tolerance", 0.25);
    // The launch shares these two limits with CorridorPerception's existing YAML:
    min_opening_width_ = declare_parameter("min_opening_width", 0.6);
    max_opening_width_ = declare_parameter("max_opening_width", 2.0);
    max_endpoint_offset_ = declare_parameter("max_endpoint_offset", 0.18);
    wall_return_tolerance_ = declare_parameter("wall_return_tolerance", 0.06);
    edge_margin_ = declare_parameter("edge_margin", 0.10);
    min_open_depth_ = declare_parameter("min_open_depth", 0.25);
    min_crossing_rays_ = declare_parameter("min_crossing_rays", 3);
    occupied_threshold_ = declare_parameter("occupied_threshold", 65);
    max_odom_jump_ = declare_parameter("max_odom_jump", 0.5);
    max_map_translation_jump_ = declare_parameter("max_map_translation_jump", 0.25);
    max_map_rotation_jump_ = declare_parameter("max_map_rotation_jump", 0.15);
}


// Reject invalid limits before any observations can create tracks:
void EntranceDetector::validate_parameters() const
{
    if(base_frame_.empty() || odom_frame_.empty() || map_frame_.empty() ||
        base_frame_ == odom_frame_ || odom_frame_ == map_frame_) {
        throw std::invalid_argument("base_frame, odom_frame and map_frame must be distinct");
    }
    for(const auto & parameter : {
        "observation_timeout", "map_timeout", "min_viewpoint_change", "candidate_timeout",
        "association_distance", "width_tolerance", "orientation_tolerance",
        "min_opening_width", "max_opening_width", "max_endpoint_offset", "edge_margin",
        "min_open_depth", "max_odom_jump", "max_map_translation_jump", "max_map_rotation_jump",
        "max_view_distance_ratio", "wall_return_tolerance"
    }) {
        const double limit = get_parameter(parameter).as_double();
        if(!std::isfinite(limit) || limit <= 0.0) {
            throw std::invalid_argument(std::string(parameter) + " must be finite and positive");
        }
    }
    if(min_opening_width_ >= max_opening_width_ || 2 * edge_margin_ >= min_opening_width_) {
        throw std::invalid_argument("Opening widths must leave room for both edge_margin bands");
    }
    if(min_observations_ < 2 || min_crossing_rays_ < 1 ||
        occupied_threshold_ < 1 || occupied_threshold_ > 100 || orientation_tolerance_ >= PI / 2) {
        throw std::invalid_argument(
            "Invalid observation/ray counts, occupancy or orientation limits"
        );
    }
    if(max_view_distance_ratio_ < 1.0) {
        throw std::invalid_argument("max_view_distance_ratio must be at least 1.0");
    }
}


// Keep sensor QoS compatible with LiDAR, and retain complete map/entrance/marker snapshots:
void EntranceDetector::initialize_communication()
{
    observation_subscription_ = create_subscription<Observation>(
        "corridor_observation", rclcpp::QoS(1),
        std::bind(&EntranceDetector::observation_callback, this, std::placeholders::_1)
    );
    scan_subscription_ = create_subscription<Scan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&EntranceDetector::scan_callback, this, std::placeholders::_1)
    );
    const auto retained = rclcpp::QoS(1).reliable().transient_local();
    map_subscription_ = create_subscription<Grid>(
        "map", retained,
        std::bind(&EntranceDetector::map_callback, this, std::placeholders::_1)
    );
    entrance_publisher_ = create_publisher<EntranceArray>("entrances", retained);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "entrance_markers", retained
    );
}


// Retain geometry and monotonic receipt age without renewing its acquisition timestamp:
void EntranceDetector::observation_callback(Observation::ConstSharedPtr message)
{
    observation_ = message;
    observation_arrival_ = std::chrono::steady_clock::now();
}


// Allow either callback order while bounding scan storage independently of sensor rate:
void EntranceDetector::scan_callback(Scan::ConstSharedPtr message)
{
    scans_.push_back(message);
    while(scans_.size() > MAX_QUEUED_SCANS) {
        scans_.pop_front();
    }
}


// Validate the grid frame, geometry and storage before using it as corroborating evidence:
void EntranceDetector::map_callback(Grid::ConstSharedPtr message)
{
    const std::uint64_t cells = std::uint64_t(message->info.width) * message->info.height;
    const bool valid_grid = message->header.frame_id == map_frame_ && cells > 0 &&
        cells <= 2000000 && cells == message->data.size() &&
        std::isfinite(message->info.resolution) && message->info.resolution > 0.0 &&
        pose_is_planar(message->info.origin);
    if(!valid_grid) {
        map_.reset();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring invalid entrance map");
        return;
    }
    map_ = message;
    tf2::Transform map_from_grid;
    tf2::fromMsg(map_->info.origin, map_from_grid);
    grid_from_map_ = map_from_grid.inverse();
}


// Reset on ROS-time rollback, expire tentative gaps and publish historical confirmed entries:
void EntranceDetector::detection_loop()
{
    const double current_time = now().seconds();
    if(previous_clock_time_ >= 0.0 && current_time < previous_clock_time_) {
        reset_registry();
        scans_.clear();
        observation_.reset();
        map_.reset();
        last_processed_time_ = -1.0;
    }
    previous_clock_time_ = current_time;
    const auto expired = std::remove_if(tracks_.begin(), tracks_.end(),
        [this, current_time](const auto & track) {
            const double age = current_time - track.last_observed.seconds();
            return track.id == 0 && age > candidate_timeout_;
        }
    );
    tracks_.erase(expired, tracks_.end());

    process_observation();
    const auto elapsed = std::chrono::steady_clock::now() - last_publication_;
    if(registry_changed_ || elapsed >= std::chrono::milliseconds(500)) {
        publish_registry();
    }
}


// Pair distinct scan-time geometry, obtain real TF and check both observed corridor walls:
void EntranceDetector::process_observation()
{
    if(!observation_is_valid() || !map_) return;
    const rclcpp::Time stamp(observation_->header.stamp);
    const double acquisition_time = stamp.seconds();
    if(acquisition_time <= last_processed_time_) return;
    const double map_age = (now() - rclcpp::Time(map_->header.stamp)).seconds();
    if(map_age < -FUTURE_TOLERANCE || map_age > map_timeout_) return;

    const auto scan = std::find_if(scans_.begin(), scans_.end(), [this](const auto & item) {
        return item->header.stamp == observation_->header.stamp;
    });
    if(scan == scans_.end() || !scan_is_valid(**scan)) return;

    try {
        // All three transforms describe the same acquisition. Never replace missing
        // historical TF with the latest moving-base transform or a guessed pose:
        tf2::Transform base_from_laser;
        tf2::Transform odom_from_base;
        tf2::Transform map_from_odom;
        tf2::fromMsg(transform_buffer_.lookupTransform(
            base_frame_, (*scan)->header.frame_id, stamp
        ).transform, base_from_laser);
        tf2::fromMsg(transform_buffer_.lookupTransform(
            odom_frame_, base_frame_, stamp
        ).transform, odom_from_base);
        tf2::fromMsg(transform_buffer_.lookupTransform(
            map_frame_, odom_frame_, stamp
        ).transform, map_from_odom);
        update_reference_frames(odom_from_base, map_from_odom);

        // Predicted walls help steering but cannot create or confirm an entrance:
        if(observation_->left_wall_valid && !observation_->left_predicted) {
            process_wall_openings(
                observation_->left_openings,
                1.0,
                **scan,
                base_from_laser,
                odom_from_base,
                map_from_odom
            );
        }
        if(observation_->right_wall_valid && !observation_->right_predicted) {
            process_wall_openings(
                observation_->right_openings,
                -1.0,
                **scan,
                base_from_laser,
                odom_from_base,
                map_from_odom
            );
        }
        last_processed_time_ = acquisition_time;
    }
    catch(const tf2::TransformException & error) {
        // Retry only while the original observation remains fresh; the callback never waits:
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
            "Entrance observation waiting for acquisition-time TF: %s", error.what());
    }
}


// Reject invalid geometry, stale receipt/acquisition times and malformed endpoint arrays:
bool EntranceDetector::observation_is_valid() const
{
    if(!observation_ || !observation_->input_valid || !observation_->axis_valid ||
        observation_->header.frame_id != base_frame_ || !pose_is_planar(observation_->axis)) {
        return false;
    }
    const double age = (now() - rclcpp::Time(observation_->header.stamp)).seconds();
    const double receipt_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - observation_arrival_
    ).count();
    if(age < -FUTURE_TOLERANCE || age > observation_timeout_ ||
        receipt_age > observation_timeout_) return false;
    const auto & left = observation_->left_openings;
    const auto & right = observation_->right_openings;
    return left.size() % 2 == 0 && right.size() % 2 == 0 &&
        left.size() <= 2 * MAX_TRACKS && right.size() <= 2 * MAX_TRACKS;
}


// Check ray metadata without assuming a particular LiDAR resolution or field of view:
bool EntranceDetector::scan_is_valid(const Scan & scan) const
{
    return !scan.header.frame_id.empty() && !scan.ranges.empty() &&
        scan.ranges.size() <= MAX_SCAN_POINTS && std::isfinite(scan.angle_min) &&
        std::isfinite(scan.angle_increment) && scan.angle_increment > 0.0 &&
        std::isfinite(scan.range_min) && std::isfinite(scan.range_max) &&
        scan.range_min > 0.0 && scan.range_max > scan.range_min;
}


// Preserve local IDs through map corrections, but never silently move all old landmarks by one TF:
void EntranceDetector::update_reference_frames(
    const tf2::Transform & odom_from_base,
    const tf2::Transform & map_from_odom
)
{
    if(previous_frames_valid_) {
        const double odom_jump = (odom_from_base.getOrigin() -
            previous_odom_from_base_.getOrigin()).length();
        const double map_jump = (map_from_odom.getOrigin() -
            previous_map_from_odom_.getOrigin()).length();
        const double map_rotation = std::abs(std::remainder(
            yaw_of(map_from_odom) - yaw_of(previous_map_from_odom_), 2 * PI
        ));
        if(odom_jump > max_odom_jump_) {
            reset_registry();
            RCLCPP_WARN(get_logger(),
                "Entrance registry reset after encoder odometry discontinuity");
        }
        else if(map_jump > max_map_translation_jump_ || map_rotation > max_map_rotation_jump_) {
            for(auto & track : tracks_) {
                track.map_pose_valid = false;
            }
            registry_changed_ = true;
            RCLCPP_WARN(get_logger(),
                "Map correction withdrew entrance poses pending reobservation");
        }
    }
    previous_odom_from_base_ = odom_from_base;
    previous_map_from_odom_ = map_from_odom;
    previous_frames_valid_ = true;
}


// Interpret paired endpoints independently, rejecting map contradictions before updating tracks:
void EntranceDetector::process_wall_openings(
    const std::vector<geometry_msgs::msg::Point> & endpoints,
    double side,
    const Scan & scan,
    const tf2::Transform & base_from_laser,
    const tf2::Transform & odom_from_base,
    const tf2::Transform & map_from_odom
)
{
    for(std::size_t index = 0; index + 1 < endpoints.size(); index += 2) {
        const auto candidate = interpret_opening(
            endpoints[index],
            endpoints[index + 1],
            side,
            scan,
            base_from_laser,
            odom_from_base
        );
        if(!candidate) continue;
        const auto pose = project_to_map(*candidate, map_from_odom);
        if(map_blocks_opening(pose, candidate->width)) continue;
        update_entrance(*candidate, odom_from_base.getOrigin(), map_from_odom);
    }
}


// Derive a measured opening's centre, longitudinal width and room-facing normal:
std::optional<EntranceDetector::Candidate> EntranceDetector::interpret_opening(
    const geometry_msgs::msg::Point & first,
    const geometry_msgs::msg::Point & last,
    double side,
    const Scan & scan,
    const tf2::Transform & base_from_laser,
    const tf2::Transform & odom_from_base
) const
{
    if(!point_is_finite(first) || !point_is_finite(last)) return std::nullopt;
    const auto & rotation = observation_->axis.orientation;
    const double heading = 2 * std::atan2(rotation.z, rotation.w);
    const tf2::Vector3 tangent(std::cos(heading), std::sin(heading), 0.0);
    const tf2::Vector3 inward(-side * tangent.y(), side * tangent.x(), 0.0);
    const tf2::Vector3 a(first.x, first.y, 0.0);
    const tf2::Vector3 b(last.x, last.y, 0.0);
    tf2::Vector3 center = (a + b) / 2.0;
    double width = std::abs(tangent.dot(b - a));
    if(width < min_opening_width_ || width > max_opening_width_ ||
        std::abs(inward.dot(b - a)) > max_endpoint_offset_) return std::nullopt;

    // The opening must lie on the named side of the observed corridor axis:
    const auto & axis = observation_->axis.position;
    if(inward.dot(center - tf2::Vector3(axis.x, axis.y, 0.0)) <= 0.0) return std::nullopt;
    if(!refine_jambs(center, width, tangent, inward, scan, base_from_laser)) {
        return std::nullopt;
    }
    if(!rays_cross_opening(center, tangent, inward, width, scan, base_from_laser)) {
        return std::nullopt;
    }

    Candidate candidate;
    candidate.center = odom_from_base * center;
    candidate.inward_yaw = std::atan2(inward.y(), inward.x()) + yaw_of(odom_from_base);
    candidate.width = width;
    return candidate;
}


// Recover raw wall returns discarded by line fitting near doorway corners:
bool EntranceDetector::refine_jambs(
    tf2::Vector3 & center,
    double & width,
    const tf2::Vector3 & tangent,
    const tf2::Vector3 & inward,
    const Scan & scan,
    const tf2::Transform & base_from_laser
) const
{
    double first_jamb = -std::numeric_limits<double>::infinity();
    double last_jamb = std::numeric_limits<double>::infinity();
    const double search_limit = width / 2.0 + max_endpoint_offset_;
    for(std::size_t index = 0; index < scan.ranges.size(); ++index) {
        const double range = scan.ranges[index];
        if(!std::isfinite(range) || range < scan.range_min || range > scan.range_max) continue;
        const double angle = scan.angle_min + index * scan.angle_increment;
        const tf2::Vector3 point = base_from_laser * tf2::Vector3(
            range * std::cos(angle), range * std::sin(angle), 0.0
        );
        const tf2::Vector3 offset = point - center;
        if(std::abs(inward.dot(offset)) > wall_return_tolerance_) continue;
        const double along_wall = tangent.dot(offset);
        if(std::abs(along_wall) > search_limit) continue;
        if(along_wall < 0.0) first_jamb = std::max(first_jamb, along_wall);
        if(along_wall > 0.0) last_jamb = std::min(last_jamb, along_wall);
    }
    if(!std::isfinite(first_jamb) || !std::isfinite(last_jamb)) return false;

    // The nearest raw support on either side brackets the free interval. A room
    // back wall is excluded by the normal-distance band, not mistaken for a jamb:
    const double refined_width = last_jamb - first_jamb;
    if(refined_width < min_opening_width_ || refined_width > max_opening_width_) return false;
    center += tangent * ((first_jamb + last_jamb) / 2.0);
    width = refined_width;
    return true;
}


// Count valid rays crossing the central gap and continuing a measured depth into the room:
bool EntranceDetector::rays_cross_opening(
    const tf2::Vector3 & center,
    const tf2::Vector3 & tangent,
    const tf2::Vector3 & inward,
    double width,
    const Scan & scan,
    const tf2::Transform & base_from_laser
) const
{
    const tf2::Vector3 origin = base_from_laser.getOrigin();
    int crossing_rays = 0;
    for(std::size_t index = 0; index < scan.ranges.size(); ++index) {
        const double range = scan.ranges[index];
        const bool no_hit = std::isinf(range) && range > 0.0;
        if(!no_hit && (!std::isfinite(range) || range < scan.range_min ||
            range > scan.range_max)) continue;
        const double angle = scan.angle_min + index * scan.angle_increment;
        const tf2::Vector3 ray = base_from_laser.getBasis() *
            tf2::Vector3(std::cos(angle), std::sin(angle), 0.0);

        // Ray/threshold intersection: n dot (origin + distance * ray - centre) = 0.
        // A positive denominator means the ray travels from corridor into room:
        const double toward_room = inward.dot(ray);
        if(toward_room <= 1e-6) continue;
        const double distance = inward.dot(center - origin) / toward_room;
        if(distance < scan.range_min) continue;
        const tf2::Vector3 intersection = origin + distance * ray;
        const double along_opening = std::abs(tangent.dot(intersection - center));
        if(along_opening > width / 2.0 - edge_margin_) continue;

        // Project the remaining ray length onto the inward normal. +inf permits
        // only the sensor's finite no-hit horizon; NaN never supplies free space:
        const double horizon = no_hit ? scan.range_max : range;
        const double open_depth = (horizon - distance) * toward_room;
        if(open_depth < min_open_depth_) continue;
        if(++crossing_rays >= min_crossing_rays_) return true;
    }
    return false;
}


// Project the locally filtered centre and inward direction using the current scan's map TF:
geometry_msgs::msg::Pose EntranceDetector::project_to_map(
    const Candidate & candidate,
    const tf2::Transform & map_from_odom
) const
{
    const tf2::Vector3 center = map_from_odom * candidate.center;
    const double inward_yaw = candidate.inward_yaw + yaw_of(map_from_odom);
    geometry_msgs::msg::Pose pose;
    pose.position.x = center.x();
    pose.position.y = center.y();
    pose.orientation.z = std::sin(inward_yaw / 2.0);
    pose.orientation.w = std::cos(inward_yaw / 2.0);
    return pose;
}


// Reject a known occupied threshold while treating unknown/outside-map cells as neutral:
bool EntranceDetector::map_blocks_opening(
    const geometry_msgs::msg::Pose & pose,
    double width
) const
{
    if(!map_) return false;
    const double inward_yaw = 2 * std::atan2(pose.orientation.z, pose.orientation.w);
    int known_cells = 0;
    int occupied_cells = 0;
    // Sample the central 80% of the threshold, away from uncertain jamb pixels.
    // Map origin may be rotated: first transform map coordinates into the grid frame:
    for(int sample = -4; sample <= 4; ++sample) {
        const double offset = sample * width / 10.0;
        const tf2::Vector3 point = grid_from_map_ * tf2::Vector3(
            pose.position.x - std::sin(inward_yaw) * offset,
            pose.position.y + std::cos(inward_yaw) * offset, 0.0
        );
        const double column = std::floor(point.x() / map_->info.resolution);
        const double row = std::floor(point.y() / map_->info.resolution);
        if(column < 0 || row < 0 || column >= map_->info.width || row >= map_->info.height) {
            continue;
        }
        const auto index = std::size_t(column) + std::size_t(row) * map_->info.width;
        const int occupancy = map_->data[index];
        if(occupancy < 0) continue;
        ++known_cells;
        if(occupancy >= occupied_threshold_) ++occupied_cells;
    }
    // Require several occupied samples and a majority; one wall-edge pixel is insufficient:
    return occupied_cells >= 3 && occupied_cells * 2 > known_cells;
}


// Associate once per scan, smooth local geometry, then confirm persistence and viewpoint change:
void EntranceDetector::update_entrance(
    const Candidate & candidate,
    const tf2::Vector3 & viewpoint,
    const tf2::Transform & map_from_odom
)
{
    auto matched = tracks_.end();
    double closest_distance = association_distance_;
    for(auto track = tracks_.begin(); track != tracks_.end(); ++track) {
        const double distance = (candidate.center - track->geometry.center).length();
        const double angle = std::abs(std::remainder(
            candidate.inward_yaw - track->geometry.inward_yaw, 2 * PI
        ));
        // A closer view can resolve jambs omitted from an earlier wall fragment.
        // A substantially narrower result may refine the same spatial entrance:
        const bool nearer_narrower_view = candidate.width < track->geometry.width &&
            (candidate.center - viewpoint).length() < track->best_view_distance;
        const bool width_matches =
            std::abs(candidate.width - track->geometry.width) <= width_tolerance_;
        if(distance <= closest_distance && angle <= orientation_tolerance_ &&
            (width_matches || nearer_narrower_view)) {
            matched = track;
            closest_distance = distance;
        }
    }
    if(matched == tracks_.end()) {
        if(tracks_.size() >= MAX_TRACKS) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Entrance track limit reached");
            return;
        }
        EntranceTrack track;
        track.geometry = candidate;
        track.first_viewpoint = viewpoint;
        tracks_.push_back(track);
        matched = std::prev(tracks_.end());
    }
    const rclcpp::Time stamp(observation_->header.stamp);
    if(matched->observations > 0 && stamp <= matched->last_observed) return;
    matched->last_observed = stamp;

    // Endpoint spacing grows with oblique/range-limited views. Once a closer view
    // exists, distant views must not widen the measured gap again after passing it.
    // Continue associating them for identity/freshness, but retain the better pose:
    const double view_distance = (candidate.center - viewpoint).length();
    if(view_distance > matched->best_view_distance * max_view_distance_ratio_) return;
    matched->best_view_distance = std::min(matched->best_view_distance, view_distance);

    if(matched->observations < std::numeric_limits<std::uint32_t>::max()) ++matched->observations;
    // A running mean becomes a bounded-gain filter after twenty supporting scans:
    const double gain = 1.0 / std::min<std::uint32_t>(matched->observations, 20);
    matched->geometry.center += gain * (candidate.center - matched->geometry.center);
    matched->geometry.width += gain * (candidate.width - matched->geometry.width);
    matched->geometry.inward_yaw += gain * std::remainder(
        candidate.inward_yaw - matched->geometry.inward_yaw, 2 * PI
    );
    matched->pose_stamp = stamp;
    matched->map_pose = project_to_map(matched->geometry, map_from_odom);
    matched->map_pose_valid = !map_blocks_opening(matched->map_pose, matched->geometry.width);

    const double baseline = (viewpoint - matched->first_viewpoint).length();
    if(matched->id == 0 && matched->observations >= std::uint32_t(min_observations_) &&
        baseline >= min_viewpoint_change_ && matched->map_pose_valid) {
        matched->id = next_id_++;
        RCLCPP_INFO(
            get_logger(),
            "Registered entrance_%u at (%.2f, %.2f) in %s, width %.2f m",
            matched->id,
            matched->map_pose.position.x,
            matched->map_pose.position.y,
            map_frame_.c_str(),
            matched->geometry.width
        );
    }
    if(matched->id != 0) registry_changed_ = true;
}


// Clear session geometry while leaving prior marker IDs available for explicit deletion:
void EntranceDetector::reset_registry()
{
    tracks_.clear();
    next_id_ = 1;
    previous_frames_valid_ = false;
    registry_changed_ = true;
}


// Publish a retained complete snapshot and delete only this detector's obsolete marker IDs:
void EntranceDetector::publish_registry()
{
    EntranceArray snapshot;
    snapshot.header.frame_id = map_frame_;
    snapshot.header.stamp = now();  // Snapshot time; every entrance keeps its measurement time.
    visualization_msgs::msg::MarkerArray markers;
    std::set<unsigned int> current_ids;
    for(auto & track : tracks_) {
        if(track.id == 0 || !track.map_pose_valid) continue;
        // A later map may reveal a contradiction. Reobservation is required to restore it:
        if(map_blocks_opening(track.map_pose, track.geometry.width)) {
            track.map_pose_valid = false;
            continue;
        }
        squad_interfaces::msg::Entrance entrance;
        entrance.header.frame_id = map_frame_;
        entrance.header.stamp = track.pose_stamp;
        entrance.id = "entrance_" + std::to_string(track.id);
        entrance.pose = track.map_pose;
        entrance.width = track.geometry.width;
        entrance.observation_count = track.observations;
        snapshot.entrances.push_back(entrance);
        current_ids.insert(track.id);
        append_entrance_markers(track, markers);
    }
    for(unsigned int id : published_marker_ids_) {
        if(current_ids.count(id) != 0) continue;
        for(const auto * layer : {"entrance_width", "entrance_direction", "entrance_label"}) {
            Marker removal;
            removal.header = snapshot.header;
            removal.ns = layer;
            removal.id = static_cast<int>(id);
            removal.action = Marker::DELETE;
            markers.markers.push_back(removal);
        }
    }
    entrance_publisher_->publish(snapshot);
    marker_publisher_->publish(markers);
    published_marker_ids_ = current_ids;
    registry_changed_ = false;
    last_publication_ = std::chrono::steady_clock::now();
}


// Draw the measured threshold, inward normal and coordinates without altering OccupancyGrid:
void EntranceDetector::append_entrance_markers(
    const EntranceTrack & track,
    visualization_msgs::msg::MarkerArray & markers
) const
{
    Marker width;
    width.header.frame_id = map_frame_;
    width.header.stamp = track.pose_stamp;
    width.ns = "entrance_width";
    width.id = static_cast<int>(track.id);
    width.type = Marker::LINE_LIST;
    width.pose.orientation.w = 1.0;
    width.scale.x = 0.075;
    width.color.r = 0.1F;
    width.color.g = 1.0F;
    width.color.b = 0.8F;
    width.color.a = 1.0F;
    const double yaw = 2 * std::atan2(track.map_pose.orientation.z, track.map_pose.orientation.w);
    for(double sign : {-1.0, 1.0}) {
        auto endpoint = track.map_pose.position;
        endpoint.x -= sign * std::sin(yaw) * track.geometry.width / 2.0;
        endpoint.y += sign * std::cos(yaw) * track.geometry.width / 2.0;
        endpoint.z = 0.08;
        width.points.push_back(endpoint);
    }
    markers.markers.push_back(width);

    Marker direction = width;
    direction.ns = "entrance_direction";
    direction.type = Marker::ARROW;
    direction.points.clear();
    direction.pose = track.map_pose;
    direction.pose.position.z = 0.12;
    direction.scale.x = 0.45;
    direction.scale.y = 0.07;
    direction.scale.z = 0.10;
    markers.markers.push_back(direction);

    Marker label = width;
    label.ns = "entrance_label";
    label.type = Marker::TEXT_VIEW_FACING;
    label.points.clear();
    label.pose.position = track.map_pose.position;
    label.pose.position.x += 0.45 * std::cos(yaw);
    label.pose.position.y += 0.45 * std::sin(yaw);
    label.pose.position.z = 0.5;
    label.scale.x = 0.0;
    label.scale.z = 0.16;
    std::ostringstream text;
    text << "entrance_" << track.id << std::fixed << std::setprecision(2)
         << "\n(" << track.map_pose.position.x << ", " << track.map_pose.position.y
         << ")  " << track.geometry.width << " m";
    label.text = text.str();
    markers.markers.push_back(label);
}


// Require finite coordinates before geometric arithmetic or grid indexing:
bool EntranceDetector::point_is_finite(const geometry_msgs::msg::Point & point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}


// Accept only finite, normalized planar poses; an all-zero quaternion is unavailable data:
bool EntranceDetector::pose_is_planar(const geometry_msgs::msg::Pose & pose)
{
    const auto & q = pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return point_is_finite(pose.position) && std::isfinite(norm) &&
        std::abs(norm - 1.0) < 0.01 && std::abs(q.x) < 0.001 && std::abs(q.y) < 0.001;
}


// Extract planar heading from an acquisition-time TF without introducing a new frame authority:
double EntranceDetector::yaw_of(const tf2::Transform & transform)
{
    const auto & q = transform.getRotation();
    return std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
        1 - 2 * (q.y() * q.y() + q.z() * q.z()));
}


// Serialize callbacks and tracking updates in the standard single-threaded executor:
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EntranceDetector>());
    rclcpp::shutdown();
    return 0;
}
