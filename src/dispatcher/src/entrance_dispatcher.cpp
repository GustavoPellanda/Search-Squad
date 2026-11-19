#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <squad_interfaces/msg/entrance_array.hpp>
#include <squad_interfaces/msg/robot_event.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/*
Send one detected entrance to each configured explorer for the preliminary test.

entrances_callback retains each waiting robot's matching entrance by stable ID.
dispatch_loop waits for an active navigator, the public navigation action and
fresh map-to-base TF, then sends one corridor-side approach goal. After arrival
and supervisor idle acknowledgement, an observed entry point allows a second
goal just inside the room. Both poses use the doorway's inward normal.
Only the existing navigation supervisor may forward it to Nav2 and select motion.
Repeated entrance snapshots cannot resend an attempted goal. Rejection or failure
is reported for owner review; there is no retry, reassignment or room exploration.
*/
class EntranceDispatcher : public rclcpp::Node
{
    public:

    EntranceDispatcher();

    private:

    using Navigate = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;
    using GetState = lifecycle_msgs::srv::GetState;
    using Entrance = squad_interfaces::msg::Entrance;
    enum class Stage { WAITING, APPROACH_SENT, ENTRY_READY, ENTRY_SENT, FINISHED };

    struct Assignment {
        std::string robot;
        std::string entrance_id;
        Stage stage = Stage::WAITING;
        std::optional<Entrance> entrance;
        bool navigator_active = false;
        bool navigator_idle = false;
        bool state_request_pending = false;
        std::chrono::steady_clock::time_point last_state_request{};
        rclcpp_action::Client<Navigate>::SharedPtr navigation;
        rclcpp::Client<GetState>::SharedPtr navigator_state;
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription;
    };

    void read_parameters();
    void initialize_communication();
    void entrances_callback(squad_interfaces::msg::EntranceArray::ConstSharedPtr message);
    bool entrance_is_valid(const Entrance & entrance) const;
    void navigation_event_callback(squad_interfaces::msg::RobotEvent::ConstSharedPtr message);
    void costmap_callback(std::size_t index, nav_msgs::msg::OccupancyGrid::ConstSharedPtr message);
    void dispatch_loop();
    void request_navigator_state(std::size_t index);
    void navigator_state_callback(
        std::size_t index, rclcpp::Client<GetState>::SharedFuture response
    );
    bool localization_is_ready(const Assignment & assignment);
    geometry_msgs::msg::PoseStamped calculate_goal(const Entrance & entrance, double offset);
    std::optional<geometry_msgs::msg::PoseStamped> select_observed_entry(Assignment & assignment);
    void send_assignment(std::size_t index, const geometry_msgs::msg::PoseStamped & pose);
    void goal_response(std::size_t index, const GoalHandle::SharedPtr & goal);
    void goal_result(std::size_t index, const GoalHandle::WrappedResult & result);
    void report(const Assignment & assignment, const std::string & phase,
        const std::string & detail);

    double entry_distance_ = 0.0;  // Metres past the threshold into the room.
    double approach_distance_ = 0.0;  // Metres before the threshold in the corridor.
    double entry_lateral_search_ = 0.0;
    double entry_jamb_margin_ = 0.0;
    std::vector<Assignment> assignments_;
    tf2_ros::Buffer transform_buffer_;
    tf2_ros::TransformListener transform_listener_;
    rclcpp::Subscription<squad_interfaces::msg::EntranceArray>::SharedPtr entrance_subscription_;
    rclcpp::Publisher<squad_interfaces::msg::RobotEvent>::SharedPtr event_publisher_;
    rclcpp::Subscription<squad_interfaces::msg::RobotEvent>::SharedPtr event_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};


// Establish fixed assignments before connecting their callbacks and processing timer:
EntranceDispatcher::EntranceDispatcher()
    : Node("entrance_dispatcher"),
      transform_buffer_(get_clock()),
      transform_listener_(transform_buffer_)
{
    read_parameters();
    initialize_communication();
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&EntranceDispatcher::dispatch_loop, this)
    );
}


// Read robot/entrance pairs from YAML without embedding doorway coordinates in the node:
void EntranceDispatcher::read_parameters()
{
    const auto robots = declare_parameter("robots", std::vector<std::string>{});
    const auto entrances = declare_parameter("entrance_ids", std::vector<std::string>{});
    entry_distance_ = declare_parameter("entry_distance", 0.40);
    approach_distance_ = declare_parameter("approach_distance", 0.55);
    entry_lateral_search_ = declare_parameter("entry_lateral_search", 0.30);
    entry_jamb_margin_ = declare_parameter("entry_jamb_margin", 0.30);
    if(robots.size() != 2 || entrances.size() != robots.size()) {
        throw std::invalid_argument(
            "This preliminary dispatcher requires two robot/entrance pairs"
        );
    }
    if(!std::isfinite(entry_distance_) || entry_distance_ <= 0.0) {
        throw std::invalid_argument("entry_distance must be finite and positive");
    }
    if(!std::isfinite(approach_distance_) || approach_distance_ <= 0.0) {
        throw std::invalid_argument("approach_distance must be finite and positive");
    }
    if(!std::isfinite(entry_lateral_search_) || entry_lateral_search_ < 0.0 ||
        !std::isfinite(entry_jamb_margin_) || entry_jamb_margin_ <= 0.0) {
        throw std::invalid_argument("Entry search must be finite; jamb margin must be positive");
    }
    const std::set<std::string> unique_robots(robots.begin(), robots.end());
    const std::set<std::string> unique_entrances(entrances.begin(), entrances.end());
    if(unique_robots.size() != robots.size() || unique_entrances.size() != entrances.size()) {
        throw std::invalid_argument("Robot names and entrance IDs must be unique");
    }
    for(std::size_t index = 0; index < robots.size(); ++index) {
        if(robots[index].empty() || robots[index].front() == '/' || entrances[index].empty()) {
            throw std::invalid_argument(
                "Expected relative robot namespaces and nonempty entrance IDs"
            );
        }
        Assignment assignment;
        assignment.robot = robots[index];
        assignment.entrance_id = entrances[index];
        assignments_.push_back(assignment);
    }
}


// Subscribe to complete retained entrance lists and use only supervised public goal endpoints:
void EntranceDispatcher::initialize_communication()
{
    entrance_subscription_ = create_subscription<squad_interfaces::msg::EntranceArray>(
        "entrances",
        rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&EntranceDispatcher::entrances_callback, this, std::placeholders::_1)
    );
    event_publisher_ = create_publisher<squad_interfaces::msg::RobotEvent>("events", 20);
    event_subscription_ = create_subscription<squad_interfaces::msg::RobotEvent>(
        "events", 20,
        std::bind(&EntranceDispatcher::navigation_event_callback, this, std::placeholders::_1)
    );
    for(std::size_t index = 0; index < assignments_.size(); ++index) {
        auto & assignment = assignments_[index];
        assignment.navigation = rclcpp_action::create_client<Navigate>(
            this, assignment.robot + "/navigate_to_pose"
        );
        assignment.navigator_state = create_client<GetState>(
            assignment.robot + "/nav2/bt_navigator/get_state"
        );
        assignment.costmap_subscription = create_subscription<nav_msgs::msg::OccupancyGrid>(
            assignment.robot + "/nav2/global_costmap/costmap",
            rclcpp::QoS(1).reliable().transient_local(),
            [this, index](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
                costmap_callback(index, message);
            }
        );
    }
}


// Update only waiting assignments; a withdrawn map projection must not launch a stale goal:
void EntranceDispatcher::entrances_callback(
    squad_interfaces::msg::EntranceArray::ConstSharedPtr message
)
{
    for(auto & assignment : assignments_) {
        if(assignment.stage != Stage::WAITING) continue;
        assignment.entrance.reset();
        if(message->header.frame_id != "map") continue;
        for(const auto & entrance : message->entrances) {
            if(entrance.id == assignment.entrance_id && entrance_is_valid(entrance)) {
                assignment.entrance = entrance;
                break;
            }
        }
    }
}


// The public supervisor reports idle only after its stopping/handover stage finishes:
void EntranceDispatcher::navigation_event_callback(
    squad_interfaces::msg::RobotEvent::ConstSharedPtr message
)
{
    for(auto & assignment : assignments_) {
        if(message->robot != assignment.robot) continue;
        if(message->phase == "WAITING_FOR_TASK") assignment.navigator_idle = true;
        else if(message->phase == "NAVIGATING") assignment.navigator_idle = false;
    }
}


// Keep the actual private planning grid; it may initially cover only the corridor:
void EntranceDispatcher::costmap_callback(
    std::size_t index,
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr message
)
{
    assignments_[index].costmap = message;
}


// Accept only finite map-frame doorway geometry with a normalized planar inward orientation:
bool EntranceDispatcher::entrance_is_valid(const Entrance & entrance) const
{
    const auto & position = entrance.pose.position;
    const auto & q = entrance.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    const bool finite_geometry = std::isfinite(position.x) && std::isfinite(position.y) &&
        std::isfinite(position.z) && std::isfinite(entrance.width) && entrance.width > 0.0;
    const bool planar_orientation = std::isfinite(norm) && std::abs(norm - 1.0) < 0.01 &&
        std::abs(q.x) < 0.001 && std::abs(q.y) < 0.001;
    return entrance.header.frame_id == "map" && finite_geometry && planar_orientation;
}


// Dispatch each pair independently as soon as its entrance and navigation inputs are available:
void EntranceDispatcher::dispatch_loop()
{
    for(std::size_t index = 0; index < assignments_.size(); ++index) {
        auto & assignment = assignments_[index];
        if(!assignment.entrance) continue;
        if(assignment.stage == Stage::ENTRY_READY) {
            if(assignment.navigator_idle && localization_is_ready(assignment) &&
                assignment.navigation->action_server_is_ready()) {
                const auto entry = select_observed_entry(assignment);
                if(entry) send_assignment(index, *entry);
            }
            continue;
        }
        if(assignment.stage != Stage::WAITING) continue;
        request_navigator_state(index);
        if(!assignment.navigator_active || !assignment.navigation->action_server_is_ready()) {
            continue;
        }
        if(!localization_is_ready(assignment)) continue;
        send_assignment(index, calculate_goal(*assignment.entrance, -approach_distance_));
    }
}


// Check lifecycle readiness without a fixed startup delay or blocking the entrance subscription:
void EntranceDispatcher::request_navigator_state(std::size_t index)
{
    auto & assignment = assignments_[index];
    if(assignment.navigator_active || assignment.state_request_pending ||
        !assignment.navigator_state->service_is_ready()) return;
    const auto wall = std::chrono::steady_clock::now();
    if(wall - assignment.last_state_request < std::chrono::seconds(1)) return;
    assignment.last_state_request = wall;
    assignment.state_request_pending = true;
    assignment.navigator_state->async_send_request(
        std::make_shared<GetState::Request>(),
        [this, index](rclcpp::Client<GetState>::SharedFuture response) {
            navigator_state_callback(index, response);
        }
    );
}


// A discovered action server alone is insufficient while its lifecycle node is inactive:
void EntranceDispatcher::navigator_state_callback(
    std::size_t index,
    rclcpp::Client<GetState>::SharedFuture response
)
{
    auto & assignment = assignments_[index];
    assignment.state_request_pending = false;
    assignment.navigator_active = response.get()->current_state.id ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}


// Require a recent measured/estimated robot transform from the existing localization authority:
bool EntranceDispatcher::localization_is_ready(const Assignment & assignment)
{
    try {
        const auto pose = transform_buffer_.lookupTransform(
            "map", assignment.robot + "/base_footprint", tf2::TimePointZero
        );
        const double age = (now() - rclcpp::Time(pose.header.stamp)).seconds();
        const auto & position = pose.transform.translation;
        return age >= -0.1 && age < 0.6 &&
            std::isfinite(position.x) && std::isfinite(position.y);
    }
    catch(const tf2::TransformException &) {
        return false;
    }
}


// Negative offset approaches from the corridor; positive offset crosses into the room:
geometry_msgs::msg::PoseStamped EntranceDispatcher::calculate_goal(
    const Entrance & entrance,
    double offset
)
{
    const auto & q = entrance.pose.orientation;
    const double inward_yaw = 2 * std::atan2(q.z, q.w);
    const double inward_x = std::cos(inward_yaw);
    const double inward_y = std::sin(inward_yaw);
    geometry_msgs::msg::PoseStamped entry;
    entry.header.frame_id = "map";
    entry.header.stamp = now();  // Goal creation time, not a new doorway measurement.
    entry.pose = entrance.pose;
    // goal = threshold + signed_offset * inward_normal; preserve the room-facing heading:
    entry.pose.position.x += offset * inward_x;
    entry.pose.position.y += offset * inward_y;
    entry.pose.position.z = 0.0;
    return entry;
}


// Wait for a known, nonblocked entry point instead of planning into an unavailable map:
std::optional<geometry_msgs::msg::PoseStamped> EntranceDispatcher::select_observed_entry(
    Assignment & assignment
)
{
    if(!assignment.costmap || assignment.costmap->header.frame_id != "map") return std::nullopt;
    const auto & grid = *assignment.costmap;
    const auto & info = grid.info;
    const double age = (now() - rclcpp::Time(grid.header.stamp)).seconds();
    if(age < -0.1 || age > 2.0 || !std::isfinite(info.resolution) || info.resolution <= 0.0 ||
        std::size_t(info.width) * info.height != grid.data.size()) return std::nullopt;
    const auto centered = calculate_goal(*assignment.entrance, entry_distance_);
    const auto & rotation = centered.pose.orientation;
    const double inward_yaw = 2 * std::atan2(rotation.z, rotation.w);
    const double yaw = 2 * std::atan2(info.origin.orientation.z, info.origin.orientation.w);
    const double available_half_width = assignment.entrance->width / 2.0 - entry_jamb_margin_;
    if(available_half_width < 0.0) return std::nullopt;
    const double limit = std::min(entry_lateral_search_, available_half_width);
    // Test the centre first, then bounded offsets along the wall. Laser observations
    // may leave the exact centre unknown while a neighbouring strip is observed free:
    for(double fraction : {0.0, 0.25, -0.25, 0.5, -0.5, 0.75, -0.75, 1.0, -1.0}) {
        auto candidate = centered;
        const double offset = fraction * limit;
        candidate.pose.position.x -= std::sin(inward_yaw) * offset;
        candidate.pose.position.y += std::cos(inward_yaw) * offset;
        const double dx = candidate.pose.position.x - info.origin.position.x;
        const double dy = candidate.pose.position.y - info.origin.position.y;
        // Invert the grid origin before dividing map distances by cell resolution:
        const double grid_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
        const double column = std::floor(grid_x / info.resolution);
        const double row = std::floor((-std::sin(yaw) * dx + std::cos(yaw) * dy) / info.resolution);
        if(!std::isfinite(column) || !std::isfinite(row) || column < 0 || row < 0 ||
            column >= info.width || row >= info.height) continue;
        const int cost = grid.data[std::size_t(column) + std::size_t(row) * info.width];
        // 99/100 are inscribed/lethal obstacles; unknown cells remain unavailable:
        if(cost >= 0 && cost < 99) return candidate;
    }
    return std::nullopt;
}


// Latch the single attempt before invoking the asynchronous public navigation action:
void EntranceDispatcher::send_assignment(
    std::size_t index,
    const geometry_msgs::msg::PoseStamped & pose
)
{
    auto & assignment = assignments_[index];
    Navigate::Goal goal;
    const bool entering = assignment.stage == Stage::ENTRY_READY;
    goal.pose = pose;
    rclcpp_action::Client<Navigate>::SendGoalOptions options;
    options.goal_response_callback = std::bind(
        &EntranceDispatcher::goal_response, this, index, std::placeholders::_1
    );
    options.result_callback = std::bind(
        &EntranceDispatcher::goal_result, this, index, std::placeholders::_1
    );
    assignment.stage = entering ? Stage::ENTRY_SENT : Stage::APPROACH_SENT;
    assignment.navigator_idle = false;
    assignment.navigation->async_send_goal(goal, options);
    report(assignment, entering ? "ENTERING_ROOM" : "ENTRANCE_ASSIGNED", "Goal in map: x=" +
        std::to_string(goal.pose.pose.position.x) + ", y=" +
        std::to_string(goal.pose.pose.position.y));
}


// Rejection terminates this fixed attempt instead of resending every retained entrance update:
void EntranceDispatcher::goal_response(std::size_t index, const GoalHandle::SharedPtr & goal)
{
    if(goal) return;
    auto & assignment = assignments_[index];
    assignment.stage = Stage::FINISHED;
    report(assignment, "ENTRANCE_NAVIGATION_FAILED", "Public navigation action rejected the goal");
}


// Report arrival or the terminal failure; the supervisor restores hold when navigation finishes:
void EntranceDispatcher::goal_result(std::size_t index, const GoalHandle::WrappedResult & result)
{
    auto & assignment = assignments_[index];
    if(result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        if(assignment.stage == Stage::APPROACH_SENT) {
            assignment.stage = Stage::ENTRY_READY;
            report(assignment, "AT_ENTRANCE",
                "Approach reached; waiting for an observed entry point");
        }
        else {
            assignment.stage = Stage::FINISHED;
            report(assignment, "ROOM_ENTERED", "Reached the entry pose; waiting inside the room");
        }
    }
    else {
        const bool approach_failed = assignment.stage == Stage::APPROACH_SENT;
        assignment.stage = Stage::FINISHED;
        report(assignment, "ENTRANCE_NAVIGATION_FAILED",
            approach_failed ? "Navigation ended before reaching the approach" :
            "Navigation ended before reaching the entry pose");
    }
}


// Expose only assignment milestones in the existing mission log, never periodic goal chatter:
void EntranceDispatcher::report(
    const Assignment & assignment,
    const std::string & phase,
    const std::string & detail
)
{
    squad_interfaces::msg::RobotEvent event;
    event.header.frame_id = "map";
    event.header.stamp = now();
    event.robot = assignment.robot;
    event.task_id = assignment.entrance_id;
    event.phase = phase;
    event.detail = detail;
    event_publisher_->publish(event);
    RCLCPP_INFO(
        get_logger(),
        "%s -> %s | %s | %s",
        assignment.robot.c_str(),
        assignment.entrance_id.c_str(),
        phase.c_str(),
        detail.c_str()
    );
}


// Serialize entrance, lifecycle and action callbacks with the standard single-threaded executor:
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EntranceDispatcher>());
    rclcpp::shutdown();
    return 0;
}
