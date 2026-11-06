#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <explorer/room_geometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/msg/costmap_filter_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <squad_interfaces/action/explore_room.hpp>
#include <squad_interfaces/msg/robot_event.hpp>
#include <squad_interfaces/srv/set_control.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

/*
One room task owns approach, entry, frontier observations, completion and return.
All goals go through navigation_supervisor. This node never publishes cmd_vel,
never reads the world file, and never treats a navigation mask as an observed wall.
*/
class RoomExplorer : public rclcpp::Node
{
    public:

    RoomExplorer();

    private:

    using Task = squad_interfaces::action::ExploreRoom;
    using TaskHandle = rclcpp_action::ServerGoalHandle<Task>;
    using Navigate = nav2_msgs::action::NavigateToPose;
    using Plan = nav2_msgs::action::ComputePathToPose;
    using NavHandle = rclcpp_action::ClientGoalHandle<Navigate>;
    enum class Stage { IDLE, CLAIM, APPROACH, ENTER, OBSERVE, CHECK_PATH, EXPLORE,
        RETURN, EXIT, REPORT, CANCEL_WAIT, FAULT };

    rclcpp_action::GoalResponse receive_task(
        const rclcpp_action::GoalUUID &, std::shared_ptr<const Task::Goal> goal
    );
    rclcpp_action::CancelResponse cancel_task(const std::shared_ptr<TaskHandle> goal);
    void accept_task(const std::shared_ptr<TaskHandle> goal);
    void map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr message);
    void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr message);
    bool current_pose(geometry_msgs::msg::PoseStamped & pose);
    bool mode_ready(const std::string & mode);
    void mode_response(rclcpp::Client<squad_interfaces::srv::SetControl>::SharedFuture future);
    void navigate(const geometry_msgs::msg::PoseStamped & pose, Stage stage);
    void navigation_response(const NavHandle::SharedPtr & handle, std::uint64_t generation);
    void navigation_result(const NavHandle::WrappedResult & result, std::uint64_t generation);
    void control_loop();
    void start_observation();
    void check_path(const geometry_msgs::msg::PoseStamped & pose);
    void path_result(const rclcpp_action::ClientGoalHandle<Plan>::WrappedResult & result,
        std::uint64_t generation);
    void validate_parameters() const;
    void evaluate_room(const geometry_msgs::msg::PoseStamped & pose);
    void begin_return(const std::string & reason);
    void finish_task(bool passage_clear);
    void transition(Stage stage, const std::string & detail);
    void publish_status();
    void publish_mask(const explorer::RoomAnalysis * analysis = nullptr);
    void publish_markers(const explorer::RoomAnalysis & analysis);
    void fault(const std::string & reason);
    std::string phase() const;

    double doorway_depth(const geometry_msgs::msg::Point & point) const;
    geometry_msgs::msg::PoseStamped exit_facing_pose(
        const geometry_msgs::msg::PoseStamped & pose) const;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_publisher_;
    std::string robot_;
    Stage stage_ = Stage::IDLE;
    std::shared_ptr<TaskHandle> task_;
    Task::Goal assignment_;
    std::string mapping_status_ = "NOT_STARTED";
    std::string reason_;
    std::string accepted_mode_;
    std::string requested_mode_;
    bool mode_pending_ = false;
    bool filter_info_sent_ = false;
    bool unrestricted_mask_sent_ = false;
    bool entered_ = false;
    bool returned_ = false;
    bool mask_active_ = false;
    bool cancel_requested_ = false;
    bool nav_pending_ = false;
    bool nav_done_ = false;
    bool nav_success_ = false;
    std::string observation_tree_;
    std::uint64_t planner_generation_ = 0;
    bool cancel_sent_ = false;
    std::uint64_t nav_generation_ = 0;
    NavHandle::SharedPtr nav_handle_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
    std::vector<geometry_msgs::msg::Point> visited_;
    geometry_msgs::msg::PoseStamped selected_goal_;
    std::uint32_t frontier_count_ = 0;
    std::uint32_t known_cells_ = 0;
    int stable_checks_ = 0;
    int failed_goals_ = 0;
    int maximum_failures_ = 6;
    double clearance_ = 0.40;
    double passage_clearance_ = 0.40;
    double minimum_frontier_ = 0.30;
    double observation_seconds_ = 2.5;
    double exploration_timeout_ = 600.0;
    double entered_time_ = 0.0;
    double observation_start_ = 0.0;
    double last_status_time_ = -1.0;
    double last_map_stamp_ = -1.0;
    bool scan_valid_ = false;
    double scan_stamp_ = 0.0;
    std::chrono::steady_clock::time_point scan_arrival_{};
    std::chrono::steady_clock::time_point stage_arrival_{};
    std::chrono::steady_clock::time_point mode_request_time_{};
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    rclcpp_action::Server<Task>::SharedPtr task_server_;
    rclcpp_action::Client<Plan>::SharedPtr planner_client_;
    rclcpp_action::Client<Navigate>::SharedPtr nav_client_;
    rclcpp::Client<squad_interfaces::srv::SetControl>::SharedPtr mode_client_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_publisher_;
    rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_publisher_;
    rclcpp::Publisher<squad_interfaces::msg::RobotEvent>::SharedPtr event_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


// Connect task feedback, the live structural map and supervised navigation interfaces:
RoomExplorer::RoomExplorer()
    : Node("room_explorer"), buffer_(get_clock()), listener_(buffer_)
{
    robot_ = std::string(get_namespace()).substr(1);
    clearance_ = declare_parameter("clearance", 0.40);
    passage_clearance_ = declare_parameter("passage_clearance", 0.40);
    minimum_frontier_ = declare_parameter("minimum_frontier_length", 0.30);
    observation_seconds_ = declare_parameter("observation_seconds", 2.5);
    exploration_timeout_ = declare_parameter("exploration_timeout", 600.0);
    maximum_failures_ = declare_parameter("maximum_failed_goals", 6);
    observation_tree_ = declare_parameter("observation_behavior_tree", "");
    validate_parameters();
    heartbeat_publisher_ = create_publisher<std_msgs::msg::Empty>("task_heartbeat", 1);
    mode_client_ = create_client<squad_interfaces::srv::SetControl>("set_control");
    planner_client_ = rclcpp_action::create_client<Plan>(this, "nav2/compute_path_to_pose");
    nav_client_ = rclcpp_action::create_client<Navigate>(this, "navigate_to_pose");
    task_server_ = rclcpp_action::create_server<Task>(
        this, "explore_room", std::bind(&RoomExplorer::receive_task, this,
            std::placeholders::_1, std::placeholders::_2),
        std::bind(&RoomExplorer::cancel_task, this, std::placeholders::_1),
        std::bind(&RoomExplorer::accept_task, this, std::placeholders::_1)
    );
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&RoomExplorer::map_callback, this, std::placeholders::_1)
    );
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&RoomExplorer::scan_callback, this, std::placeholders::_1)
    );
    mask_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "room_mask", rclcpp::QoS(1).reliable().transient_local()
    );
    filter_publisher_ = create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
        "room_filter_info", rclcpp::QoS(1).reliable().transient_local()
    );
    event_publisher_ = create_publisher<squad_interfaces::msg::RobotEvent>("/fleet/events", 20);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "room_markers", rclcpp::QoS(1).transient_local()
    );
    timer_ = create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&RoomExplorer::control_loop, this)
    );
}


// Reject configuration that cannot define a finite, bounded room exploration algorithm:
void RoomExplorer::validate_parameters() const
{
    for(const double value : {clearance_, passage_clearance_, minimum_frontier_,
        observation_seconds_, exploration_timeout_}) {
        if(!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("Room distances and durations must be finite and positive");
        }
    }
    if(maximum_failures_ < 1 || maximum_failures_ > 50) {
        throw std::invalid_argument("maximum_failed_goals must be between 1 and 50");
    }
}


// Accept a single reserved entrance, never a ground-truth room polygon:
rclcpp_action::GoalResponse RoomExplorer::receive_task(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Task::Goal> goal
)
{
    if(stage_ != Stage::IDLE || task_ || goal->task_id.empty() ||
        !std::isfinite(goal->doorway_width) || goal->doorway_width < 2.0 * clearance_ ||
        !std::isfinite(goal->exploration_timeout) || goal->exploration_timeout < 0.0) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    for(const auto * pose : {&goal->approach, &goal->threshold, &goal->entry}) {
        const auto & p = pose->pose.position;
        const auto & q = pose->pose.orientation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if(pose->header.frame_id != "map" || !std::isfinite(p.x) || !std::isfinite(p.y) ||
            !std::isfinite(norm) || std::abs(norm - 1.0) > 0.05) {
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    const auto & q = goal->threshold.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const auto & door = goal->threshold.pose.position;
    const auto & approach = goal->approach.pose.position;
    const auto & entry = goal->entry.pose.position;
    const double outside = (approach.x - door.x) * std::cos(yaw) +
        (approach.y - door.y) * std::sin(yaw);
    const double inside = (entry.x - door.x) * std::cos(yaw) +
        (entry.y - door.y) * std::sin(yaw);
    if(outside > -clearance_ - 0.05 + 1e-6 || inside < clearance_ + 0.05 - 1e-6) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


// Cancellation requests a controlled return, preserving the entrance reservation:
rclcpp_action::CancelResponse RoomExplorer::cancel_task(const std::shared_ptr<TaskHandle> goal)
{
    if(goal != task_) return rclcpp_action::CancelResponse::REJECT;
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}


// Clear the previous attempt only after its return and release have been confirmed:
void RoomExplorer::accept_task(const std::shared_ptr<TaskHandle> goal)
{
    task_ = goal;
    assignment_ = *goal->get_goal();
    mapping_status_ = "NOT_STARTED";
    entered_ = false;
    returned_ = false;
    mask_active_ = false;
    cancel_requested_ = false;
    cancel_sent_ = false;
    nav_done_ = false;
    stable_checks_ = 0;
    failed_goals_ = 0;
    known_cells_ = 0;
    frontier_count_ = 0;
    visited_.clear();
    accepted_mode_.clear();
    reason_.clear();
    transition(Stage::CLAIM, "Room task accepted; entrance reserved");
}


// Reject malformed grids rather than allowing invalid indexing in frontier analysis:
void RoomExplorer::map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
{
    if(message->header.frame_id != "map" || message->info.width < 3 ||
        message->info.height < 3 || !std::isfinite(message->info.resolution) ||
        message->info.resolution <= 0.0 || message->data.size() !=
        std::size_t(message->info.width) * message->info.height ||
        message->data.size() > 2000000 || message->header.stamp.sec < 0 ||
        message->header.stamp.nanosec >= 1000000000U) return;
    const auto & origin = message->info.origin;
    if(!std::isfinite(origin.position.x) || !std::isfinite(origin.position.y) ||
        std::abs(origin.orientation.x) > 1e-6 || std::abs(origin.orientation.y) > 1e-6 ||
        std::abs(origin.orientation.z) > 1e-6 ||
        std::abs(std::abs(origin.orientation.w) - 1.0) > 1e-6) return;
    map_ = message;
    if(!mask_active_ && !unrestricted_mask_sent_) publish_mask();
}


// Map freshness cannot hide a disconnected or malformed local LiDAR stream:
void RoomExplorer::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
    scan_valid_ = message->header.stamp.sec >= 0 &&
        message->header.stamp.nanosec < 1000000000U && message->ranges.size() >= 180;
    const bool valid_frame = message->header.frame_id == robot_ + "/base_scan";
    const bool valid_angles = std::isfinite(message->angle_min) &&
        std::isfinite(message->angle_increment) && message->angle_increment > 0.0 &&
        message->angle_increment * message->ranges.size() > 6.0;
    const bool valid_ranges = std::isfinite(message->range_min) &&
        std::isfinite(message->range_max) && message->range_min > 0.0 &&
        message->range_max > message->range_min;
    scan_valid_ = scan_valid_ && valid_frame && valid_angles && valid_ranges;
    for(const auto range : message->ranges) {
        if(std::isnan(range) || range < message->range_min ||
            (std::isfinite(range) && range > message->range_max)) scan_valid_ = false;
    }
    scan_stamp_ = scan_valid_ ? rclcpp::Time(message->header.stamp).seconds() : 0.0;
    scan_arrival_ = std::chrono::steady_clock::now();
}


// Every task decision uses a recent map-frame robot pose:
bool RoomExplorer::current_pose(geometry_msgs::msg::PoseStamped & pose)
{
    try {
        const auto transform = buffer_.lookupTransform("map", robot_ + "/base_footprint",
            tf2::TimePointZero);
        if((now() - rclcpp::Time(transform.header.stamp)).seconds() > 0.7) return false;
        pose.header = transform.header;
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.orientation = transform.transform.rotation;
        return true;
    }
    catch(const tf2::TransformException &) {
        return false;
    }
}


// Retry rejected mode requests after Nav2's finishing hold, without blocking callbacks:
bool RoomExplorer::mode_ready(const std::string & mode)
{
    if(accepted_mode_ == mode) return true;
    const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - mode_request_time_
    ).count();
    if(!mode_pending_ && mode_client_->service_is_ready() && age > 0.25) {
        auto request = std::make_shared<squad_interfaces::srv::SetControl::Request>();
        request->mode = mode;
        requested_mode_ = mode;
        mode_pending_ = true;
        mode_request_time_ = std::chrono::steady_clock::now();
        mode_client_->async_send_request(
            request, std::bind(&RoomExplorer::mode_response, this, std::placeholders::_1)
        );
    }
    return false;
}


// Only an acknowledged supervisor request enables the next room phase:
void RoomExplorer::mode_response(
    rclcpp::Client<squad_interfaces::srv::SetControl>::SharedFuture future
)
{
    mode_pending_ = false;
    const auto response = future.get();
    if(response->accepted) accepted_mode_ = requested_mode_;
    else if(response->message == "FAULT" && task_) fault("NAVIGATION_FAULT");
}


// All room navigation goes through the same public supervisor action as RViz goals:
void RoomExplorer::navigate(const geometry_msgs::msg::PoseStamped & pose, Stage stage)
{
    Navigate::Goal goal;
    goal.pose = pose;
    goal.pose.header.stamp = now();
    if(stage == Stage::EXPLORE) goal.behavior_tree = observation_tree_;
    selected_goal_ = goal.pose;
    accepted_mode_.clear();
    nav_done_ = false;
    nav_pending_ = true;
    cancel_sent_ = false;
    nav_handle_.reset();
    const auto generation = ++nav_generation_;
    rclcpp_action::Client<Navigate>::SendGoalOptions options;
    options.goal_response_callback = std::bind(&RoomExplorer::navigation_response,
        this, std::placeholders::_1, generation);
    options.result_callback = std::bind(&RoomExplorer::navigation_result,
        this, std::placeholders::_1, generation);
    nav_client_->async_send_goal(goal, options);
    transition(stage, "Sending supervised Nav2 goal x=" + std::to_string(pose.pose.position.x) +
        " y=" + std::to_string(pose.pose.position.y));
}


// Attempt IDs prevent a late response from completing a different observation:
void RoomExplorer::navigation_response(
    const NavHandle::SharedPtr & handle, std::uint64_t generation
)
{
    if(generation != nav_generation_) return;
    nav_pending_ = false;
    nav_handle_ = handle;
    if(!handle) {
        nav_done_ = true;
        nav_success_ = false;
    }
}


// Record terminal navigation before the state machine can issue another goal:
void RoomExplorer::navigation_result(
    const NavHandle::WrappedResult & result, std::uint64_t generation
)
{
    if(generation != nav_generation_) return;
    nav_handle_.reset();
    nav_pending_ = false;
    nav_done_ = true;
    nav_success_ = result.code == rclcpp_action::ResultCode::SUCCEEDED;
}


// Scanning is an explicit maneuver and cannot run concurrently with navigation:
void RoomExplorer::start_observation()
{
    observation_start_ = -1.0;
    transition(Stage::OBSERVE, "Acquiring fresh observations and updating room frontiers");
}


// A missing candidate preserves unresolved frontier evidence instead of manufacturing success:
void RoomExplorer::evaluate_room(const geometry_msgs::msg::PoseStamped & pose)
{
    explorer::RoomGeometry geometry(*map_, assignment_.threshold, clearance_, minimum_frontier_);
    const auto analysis = geometry.analyze(assignment_.entry.pose.position);
    frontier_count_ = analysis.frontiers.size();
    known_cells_ = analysis.known_cells;
    publish_mask(&analysis);
    publish_markers(analysis);
    if(!analysis.seed_valid || analysis.touches_extent || analysis.known_cells < 400) {
        begin_return("REGION_UNCERTAIN");
        return;
    }
    if(analysis.frontiers.empty()) {
        const double stamp = rclcpp::Time(map_->header.stamp).seconds();
        if(stamp > last_map_stamp_) {
            ++stable_checks_;
            last_map_stamp_ = stamp;
        }
        if(stable_checks_ >= 4) {
            mapping_status_ = "MAPPED";
            begin_return("NO_REACHABLE_FRONTIERS");
        }
        return;
    }
    stable_checks_ = 0;
    geometry_msgs::msg::Point target;
    bool target_found = geometry.choose_target(analysis, pose.pose.position, visited_, target);
    if(!target_found) {
        // Small changes in viewpoint can reveal a face hidden by another object:
        target_found = geometry.choose_target(analysis, pose.pose.position, visited_, target, 0.2);
    }
    if(!target_found) {
        begin_return("UNRESOLVED_FRONTIERS");
        return;
    }
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position = target;
    const double yaw = std::atan2(target.y - pose.pose.position.y,
        target.x - pose.pose.position.x);
    goal.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.orientation.w = std::cos(yaw / 2.0);
    check_path(goal);
}


// Ask the actual Nav2 planner before committing to a candidate selected on the live grid:
void RoomExplorer::check_path(const geometry_msgs::msg::PoseStamped & pose)
{
    selected_goal_ = pose;
    Plan::Goal goal;
    goal.goal = pose;
    goal.goal.header.stamp = rclcpp::Time(0);
    goal.planner_id = "GridBased";
    goal.use_start = false;
    rclcpp_action::Client<Plan>::SendGoalOptions options;
    const auto generation = ++planner_generation_;
    options.result_callback = std::bind(
        &RoomExplorer::path_result, this, std::placeholders::_1, generation
    );
    transition(Stage::CHECK_PATH, "Checking a room-contained observation route");
    planner_client_->async_send_goal(goal, options);
}


// Reject corridor shortcuts and blocked routes without treating a planner failure as completion:
void RoomExplorer::path_result(
    const rclcpp_action::ClientGoalHandle<Plan>::WrappedResult & result,
    std::uint64_t generation
)
{
    if(!task_ || stage_ != Stage::CHECK_PATH || generation != planner_generation_) return;
    bool valid = result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
        !result.result->path.poses.empty();
    explorer::RoomGeometry geometry(*map_, assignment_.threshold, clearance_, minimum_frontier_);
    if(valid) {
        for(const auto & pose : result.result->path.poses) {
            const auto & point = pose.pose.position;
            const int cell = geometry.index(point.x, point.y);
            if(cell < 0 || !geometry.inside(point.x, point.y) || map_->data[cell] < 0 ||
                map_->data[cell] > 35) valid = false;
        }
    }
    if(valid) {
        navigate(selected_goal_, Stage::EXPLORE);
    }
    else {
        visited_.push_back(selected_goal_.pose.position);
        transition(Stage::OBSERVE, "Candidate path unresolved; retaining frontier evidence");
        observation_start_ = now().seconds() - observation_seconds_ - 1.0;
    }
}


// Preserve mapping evidence separately from the return phase:
void RoomExplorer::begin_return(const std::string & reason)
{
    ++planner_generation_;
    reason_ = reason;
    if(mapping_status_ != "MAPPED") mapping_status_ = entered_ ? "PARTIAL" : "NOT_STARTED";
    accepted_mode_.clear();
    transition(Stage::RETURN, "Mapping result " + mapping_status_ + "; " + reason);
    nav_done_ = false;
}


// Report return and doorway clearance independently of the mapping outcome:
void RoomExplorer::finish_task(bool passage_clear)
{
    if(!task_) return;
    auto result = std::make_shared<Task::Result>();
    result->mapping_status = mapping_status_;
    result->reason = reason_;
    returned_ = passage_clear;
    result->returned = passage_clear;
    result->passage_clear = passage_clear;
    transition(passage_clear ? Stage::REPORT : Stage::FAULT,
        passage_clear ? "Returned to corridor; passage clear" :
        "Return failed; reservation retained");
    publish_status();
    if(task_->is_canceling()) task_->canceled(result);
    else if(passage_clear) task_->succeed(result);
    else task_->abort(result);
    task_.reset();
    if(passage_clear) {
        accepted_mode_.clear();
        mode_ready("release");
        transition(Stage::IDLE, "Available for another room task");
    }
}


// Keep task decisions asynchronous and enforce data, cancellation and return boundaries:
void RoomExplorer::control_loop()
{
    if(now().seconds() - last_status_time_ > 1.0) {
        publish_status();
        last_status_time_ = now().seconds();
    }
    if(!task_ || stage_ == Stage::FAULT) return;
    heartbeat_publisher_->publish(std_msgs::msg::Empty());
    geometry_msgs::msg::PoseStamped pose;
    const double scan_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - scan_arrival_).count();
    const bool ready = map_ && current_pose(pose) && scan_valid_ && scan_age < 0.8 &&
        now().seconds() - scan_stamp_ < 0.8 &&
        (now() - rclcpp::Time(map_->header.stamp)).seconds() < 1.2;
    if(stage_ == Stage::CLAIM) {
        if(std::chrono::duration<double>(std::chrono::steady_clock::now() -
            stage_arrival_).count() > 45.0) {
            fault("READINESS_TIMEOUT");
            return;
        }
        if(ready && nav_client_->action_server_is_ready() && mode_ready("task")) {
            publish_mask();
            navigate(assignment_.approach, Stage::APPROACH);
        }
        return;
    }
    if(stage_ == Stage::CHECK_PATH && std::chrono::duration<double>(
        std::chrono::steady_clock::now() - stage_arrival_).count() > 5.0) {
        begin_return("PLANNER_TIMEOUT");
        return;
    }
    if(!ready) {
        fault("DATA_INVALID");
        return;
    }
    if(cancel_requested_ && stage_ != Stage::RETURN && stage_ != Stage::EXIT &&
        stage_ != Stage::CANCEL_WAIT) {
        transition(Stage::CANCEL_WAIT, "Task canceled; waiting for navigation to stop");
    }
    if(stage_ == Stage::CANCEL_WAIT) {
        if(nav_handle_ && !cancel_sent_) {
            nav_client_->async_cancel_goal(nav_handle_);
            cancel_sent_ = true;
        }
        if(!nav_handle_ && !nav_pending_ && mode_ready("hold")) {
            begin_return(reason_.empty() ? "DISPATCHER_REQUEST" : reason_);
        }
        return;
    }
    const double limit = assignment_.exploration_timeout > 0.0 ?
        assignment_.exploration_timeout : exploration_timeout_;
    const bool exploring = stage_ == Stage::OBSERVE || stage_ == Stage::CHECK_PATH ||
        stage_ == Stage::EXPLORE;
    if(entered_ && exploring && now().seconds() - entered_time_ > limit) {
        cancel_requested_ = true;
        reason_ = "TIME_LIMIT";
        return;
    }
    if(stage_ == Stage::APPROACH && nav_done_) {
        if(!nav_success_) {
            fault("APPROACH_FAILED");
        }
        else if(mode_ready("hold")) navigate(assignment_.entry, Stage::ENTER);
    }
    else if(stage_ == Stage::ENTER && nav_done_) {
        if(!nav_success_) {
            fault("ENTRY_FAILED");
        }
        else if(mode_ready("hold")) {
            if(doorway_depth(pose.pose.position) < clearance_) {
                fault("ENTRY_NOT_CONFIRMED");
                return;
            }
            entered_ = true;
            entered_time_ = now().seconds();
            mapping_status_ = "IN_PROGRESS";
            mask_active_ = true;
            start_observation();
        }
    }
    else if(stage_ == Stage::OBSERVE) {
        if(observation_start_ < 0.0) {
            if(mode_ready("scan")) observation_start_ = now().seconds();
        }
        else if(now().seconds() - observation_start_ > observation_seconds_) {
            if(mode_ready("hold")) evaluate_room(pose);
        }
    }
    else if(stage_ == Stage::EXPLORE && nav_done_) {
        if(!mode_ready("hold")) return;
        visited_.push_back(selected_goal_.pose.position);
        if(!nav_success_) ++failed_goals_;
        if(failed_goals_ >= maximum_failures_) begin_return("BLOCKED");
        else start_observation();
    }
    else if(stage_ == Stage::RETURN && !nav_pending_ && !nav_handle_) {
        if(nav_done_) {
            if(!nav_success_) fault("RETURN_FAILED");
            else if(mode_ready("hold")) {
                mask_active_ = false;
                publish_mask();
                navigate(exit_facing_pose(assignment_.approach), Stage::EXIT);
            }
        }
        else if(mode_ready("hold")) {
            if(!entered_) {
                finish_task(true);
                return;
            }
            navigate(exit_facing_pose(assignment_.entry), Stage::RETURN);
        }
    }
    else if(stage_ == Stage::EXIT && nav_done_ && mode_ready("hold")) {
        explorer::RoomGeometry geometry(*map_, assignment_.threshold,
            clearance_, minimum_frontier_);
        const double distance = std::hypot(
            pose.pose.position.x - assignment_.approach.pose.position.x,
            pose.pose.position.y - assignment_.approach.pose.position.y);
        finish_task(nav_success_ && doorway_depth(pose.pose.position) < -passage_clearance_ &&
            distance < 0.25);
    }
}


// Return and exit face out of the room instead of requiring an extra turn after crossing:
geometry_msgs::msg::PoseStamped RoomExplorer::exit_facing_pose(
    const geometry_msgs::msg::PoseStamped & pose
) const
{
    auto outward = pose;
    const auto & q = assignment_.threshold.pose.orientation;
    const double inward_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double outward_yaw = inward_yaw + std::acos(-1.0);
    outward.pose.orientation.x = 0.0;
    outward.pose.orientation.y = 0.0;
    outward.pose.orientation.z = std::sin(outward_yaw / 2.0);
    outward.pose.orientation.w = std::cos(outward_yaw / 2.0);
    return outward;
}


// Confirm crossing using a footprint margin, not just a center point on the doorway line:
double RoomExplorer::doorway_depth(const geometry_msgs::msg::Point & point) const
{
    const auto & q = assignment_.threshold.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return (point.x - assignment_.threshold.pose.position.x) * std::cos(yaw) +
        (point.y - assignment_.threshold.pose.position.y) * std::sin(yaw);
}


// Fail closed without attempting a blind return; an occupied entrance stays reserved:
void RoomExplorer::fault(const std::string & reason)
{
    ++planner_generation_;
    reason_ = reason;
    if(mapping_status_ != "MAPPED") mapping_status_ = entered_ ? "PARTIAL" : "NOT_STARTED";
    if(nav_handle_) nav_client_->async_cancel_goal(nav_handle_);
    accepted_mode_.clear();
    auto request = std::make_shared<squad_interfaces::srv::SetControl::Request>();
    request->mode = "stop";
    if(mode_client_->service_is_ready()) mode_client_->async_send_request(request);
    finish_task(false);
}


// Expose readable state names without numeric protocol values:
std::string RoomExplorer::phase() const
{
    switch(stage_) {
        case Stage::IDLE: return "IDLE";
        case Stage::CLAIM: return "ASSIGNED";
        case Stage::APPROACH: return "GOING_TO_ROOM";
        case Stage::ENTER: return "ENTERING";
        case Stage::OBSERVE: return "OBSERVING";
        case Stage::CHECK_PATH: return "PLANNING_OBSERVATION";
        case Stage::EXPLORE: return "EXPLORING";
        case Stage::RETURN: return "RETURNING";
        case Stage::EXIT: return "EXITING";
        case Stage::REPORT: return mapping_status_ == "MAPPED" ? "COMPLETED" : "INCOMPLETE";
        case Stage::CANCEL_WAIT: return "CANCELING";
        case Stage::FAULT: return "FAILED";
    }
    return "FAULT";
}


// Record each transition once; periodic status remains available to late subscribers:
void RoomExplorer::transition(Stage stage, const std::string & detail)
{
    stage_ = stage;
    stage_arrival_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "%s: %s", phase().c_str(), detail.c_str());
    publish_status();
}


// One event interface feeds both the central terminal and task feedback:
void RoomExplorer::publish_status()
{
    squad_interfaces::msg::RobotEvent event;
    event.header.stamp = now();
    event.robot = robot_;
    event.task_id = assignment_.task_id;
    event.phase = phase();
    event.mapping_status = mapping_status_;
    event.detail = reason_;
    event.frontier_count = frontier_count_;
    event.known_cells = known_cells_;
    event.returned = returned_;
    event_publisher_->publish(event);
    if(task_ && task_->is_active()) {
        auto feedback = std::make_shared<Task::Feedback>();
        feedback->phase = event.phase;
        feedback->mapping_status = mapping_status_;
        feedback->frontier_count = frontier_count_;
        feedback->known_cells = known_cells_;
        task_->publish_feedback(feedback);
    }
}


// Restrictions affect costmaps only; the real open doorway remains unchanged in /map:
void RoomExplorer::publish_mask(const explorer::RoomAnalysis * analysis)
{
    if(!map_) return;
    nav_msgs::msg::OccupancyGrid mask = *map_;
    mask.header.stamp = now();
    // A mask value of -1 leaves the underlying costmap untouched. Zero would
    // incorrectly turn unknown structural cells into free space in KeepoutFilter:
    mask.data.assign(map_->data.size(), -1);
    if(mask_active_ && analysis) {
        for(std::size_t cell = 0; cell < mask.data.size(); ++cell) {
            mask.data[cell] = analysis->region[cell] ? -1 : 100;
        }
    }
    mask_publisher_->publish(mask);
    unrestricted_mask_sent_ = !mask_active_;
    if(filter_info_sent_) return;
    filter_info_sent_ = true;
    nav2_msgs::msg::CostmapFilterInfo info;
    info.header = mask.header;
    info.type = 0;
    info.filter_mask_topic = "/" + robot_ + "/room_mask";
    info.base = 0.0;
    info.multiplier = 1.0;
    filter_publisher_->publish(info);
}


// Show the inferred free region and frontiers, with no reference geometry in the display:
void RoomExplorer::publish_markers(const explorer::RoomAnalysis & analysis)
{
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker region;
    region.header = map_->header;
    region.ns = "room_region";
    region.id = 0;
    region.type = visualization_msgs::msg::Marker::POINTS;
    region.action = visualization_msgs::msg::Marker::ADD;
    region.pose.orientation.w = 1.0;
    region.scale.x = map_->info.resolution;
    region.scale.y = map_->info.resolution;
    region.color.g = 0.7;
    region.color.a = 0.18;
    explorer::RoomGeometry geometry(*map_, assignment_.threshold, clearance_, minimum_frontier_);
    for(std::size_t cell = 0; cell < analysis.region.size(); ++cell) {
        if(analysis.region[cell]) region.points.push_back(geometry.point(cell));
    }
    markers.markers.push_back(region);
    region.ns = "frontiers";
    region.id = 1;
    region.color.r = 1.0;
    region.color.g = 0.4;
    region.color.a = 1.0;
    region.points.clear();
    for(const auto & cluster : analysis.frontiers) {
        for(const auto cell : cluster) region.points.push_back(geometry.point(cell));
    }
    markers.markers.push_back(region);
    marker_publisher_->publish(markers);
}


// Keep action callbacks and grid decisions serialized:
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoomExplorer>());
    rclcpp::shutdown();
    return 0;
}
