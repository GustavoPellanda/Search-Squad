#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <squad_interfaces/msg/control_lease.hpp>
#include <squad_interfaces/msg/robot_event.hpp>
#include <squad_interfaces/srv/set_control.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/*
Own the public navigation action and forward it to a private Nav2 endpoint.
Only this node issues control leases. Handover waits for stopping and terminal
cancellation; losing the supervisor expires the gate's independent wall watchdog.
*/
class NavigationSupervisor : public rclcpp::Node
{
    public:

    NavigationSupervisor();

    private:

    using Navigate = nav2_msgs::action::NavigateToPose;
    using PublicGoal = rclcpp_action::ServerGoalHandle<Navigate>;
    using BackendGoal = rclcpp_action::ClientGoalHandle<Navigate>;
    enum class Stage { IDLE, STOPPING, SENDING, ACTIVE, CANCELING, FINISHING, FAULT };

    rclcpp_action::GoalResponse receive_goal(
        const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal> goal
    );
    rclcpp_action::CancelResponse cancel_goal(const std::shared_ptr<PublicGoal> goal);
    void accept_goal(const std::shared_ptr<PublicGoal> goal);
    void backend_response(const BackendGoal::SharedPtr & goal);
    void backend_feedback(
        BackendGoal::SharedPtr goal, const std::shared_ptr<const Navigate::Feedback> feedback
    );
    void backend_result(const BackendGoal::WrappedResult & result);
    void control_request(
        const std::shared_ptr<squad_interfaces::srv::SetControl::Request> request,
        std::shared_ptr<squad_interfaces::srv::SetControl::Response> response
    );
    void task_heartbeat(std_msgs::msg::Empty::ConstSharedPtr message);
    void pose_goal_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr message);
    void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr message);
    void control_loop();
    void select_source(const std::string & source, const std::string & reactive_mode = "hold");
    void transition(Stage stage);
    bool localization_ready();
    bool stopped() const;
    bool backend_pose_fresh() const;
    void report_idle_activity();
    void report(const std::string & phase, const std::string & detail);
    void fail(const std::string & reason);

    std::string robot_;
    bool roam_on_idle_ = false;
    bool task_owned_ = false;
    bool room_owned_ = false;
    std::chrono::steady_clock::time_point room_heartbeat_{};
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_subscription_;
    bool cancel_requested_ = false;
    bool cancel_sent_ = false;
    Stage stage_ = Stage::IDLE;
    std::chrono::steady_clock::time_point stage_time_{};
    std::chrono::steady_clock::time_point goal_time_{};
    std::chrono::steady_clock::time_point odom_time_{};
    std::chrono::steady_clock::time_point feedback_arrival_{};
    double feedback_stamp_ = -1.0;
    double speed_ = 1.0;
    double turn_ = 1.0;
    double navigation_timeout_ = 120.0;
    squad_interfaces::msg::ControlLease lease_;
    std::string reported_phase_;
    std::shared_ptr<PublicGoal> public_goal_;
    BackendGoal::SharedPtr backend_goal_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    rclcpp_action::Server<Navigate>::SharedPtr action_server_;
    rclcpp_action::Client<Navigate>::SharedPtr action_client_;
    rclcpp::Service<squad_interfaces::srv::SetControl>::SharedPtr control_service_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_goal_subscription_;
    rclcpp_action::Client<Navigate>::SharedPtr public_client_;
    rclcpp::Publisher<squad_interfaces::msg::ControlLease>::SharedPtr lease_publisher_;
    rclcpp::Publisher<squad_interfaces::msg::RobotEvent>::SharedPtr event_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


// Wire the public action, private Nav2 client and bounded ownership heartbeat:
NavigationSupervisor::NavigationSupervisor()
    : Node("navigation_supervisor"), buffer_(get_clock()), listener_(buffer_)
{
    robot_ = std::string(get_namespace()).substr(1);
    roam_on_idle_ = declare_parameter("roam_on_idle", false);
    navigation_timeout_ = declare_parameter("navigation_timeout", 120.0);
    lease_.source = "hold";
    lease_.reactive_mode = "hold";
    lease_.generation = 1;
    lease_publisher_ = create_publisher<squad_interfaces::msg::ControlLease>("control_lease", 1);
    event_publisher_ = create_publisher<squad_interfaces::msg::RobotEvent>("/fleet/events", 20);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&NavigationSupervisor::odom_callback, this, std::placeholders::_1)
    );
    action_client_ = rclcpp_action::create_client<Navigate>(this, "nav2/navigate_to_pose");
    action_server_ = rclcpp_action::create_server<Navigate>(
        this, "navigate_to_pose",
        std::bind(&NavigationSupervisor::receive_goal, this,
            std::placeholders::_1, std::placeholders::_2),
        std::bind(&NavigationSupervisor::cancel_goal, this, std::placeholders::_1),
        std::bind(&NavigationSupervisor::accept_goal, this, std::placeholders::_1)
    );
    heartbeat_subscription_ = create_subscription<std_msgs::msg::Empty>(
        "task_heartbeat", 1,
        std::bind(&NavigationSupervisor::task_heartbeat, this, std::placeholders::_1)
    );
    public_client_ = rclcpp_action::create_client<Navigate>(this, "navigate_to_pose");
    pose_goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 1,
        std::bind(&NavigationSupervisor::pose_goal_callback, this, std::placeholders::_1)
    );
    control_service_ = create_service<squad_interfaces::srv::SetControl>(
        "set_control", std::bind(&NavigationSupervisor::control_request, this,
            std::placeholders::_1, std::placeholders::_2)
    );
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&NavigationSupervisor::control_loop, this)
    );
    transition(Stage::IDLE);
}


// Admit one finite map-frame goal only after localization and backend readiness:
rclcpp_action::GoalResponse NavigationSupervisor::receive_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal> goal
)
{
    const auto & p = goal->pose.pose.position;
    const auto & q = goal->pose.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    const bool valid = goal->pose.header.frame_id == "map" && std::isfinite(p.x) &&
        std::isfinite(p.y) && std::isfinite(norm) && std::abs(norm - 1.0) < 0.05;
    std::string rejection;
    if(!valid) rejection = "Expected a finite map-frame goal with a normalized orientation";
    else if(stage_ != Stage::IDLE || public_goal_) {
        rejection = "Navigation is busy; wait for the current goal and handover to finish";
    }
    else if(!action_client_->action_server_is_ready()) rejection = "Nav2 is not ready";
    else if(!localization_ready()) rejection = "Localization is not ready";
    if(!rejection.empty()) {
        report("GOAL_REJECTED", rejection);
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


// Stop immediately, then await the backend's terminal cancellation result:
rclcpp_action::CancelResponse NavigationSupervisor::cancel_goal(
    const std::shared_ptr<PublicGoal> goal
)
{
    if(goal != public_goal_) return rclcpp_action::CancelResponse::REJECT;
    cancel_requested_ = true;
    select_source("hold");
    return rclcpp_action::CancelResponse::ACCEPT;
}


// Invalidate old commands before forwarding the newly accepted goal:
void NavigationSupervisor::accept_goal(const std::shared_ptr<PublicGoal> goal)
{
    public_goal_ = goal;
    backend_goal_.reset();
    cancel_requested_ = false;
    cancel_sent_ = false;
    feedback_stamp_ = -1.0;
    goal_time_ = std::chrono::steady_clock::now();
    select_source("hold");
    transition(Stage::STOPPING);
    report("NAVIGATING", "Navigation goal accepted; stopping before handover");
}


// A rejected backend goal cannot leave the Nav2 source enabled:
void NavigationSupervisor::backend_response(const BackendGoal::SharedPtr & goal)
{
    if(stage_ == Stage::FAULT) {
        if(goal) action_client_->async_cancel_goal(goal);
        return;
    }
    backend_goal_ = goal;
    if(!goal) {
        fail("Nav2 rejected the forwarded goal");
        return;
    }
    transition(Stage::ACTIVE);
}


// Preserve standard feedback so RViz and action clients observe real Nav2 progress:
void NavigationSupervisor::backend_feedback(
    BackendGoal::SharedPtr goal, const std::shared_ptr<const Navigate::Feedback> feedback
)
{
    if(goal != backend_goal_) return;
    const auto & stamp = feedback->current_pose.header.stamp;
    const auto & position = feedback->current_pose.pose.position;
    if(stamp.sec >= 0 && stamp.nanosec < 1000000000U &&
        feedback->current_pose.header.frame_id == "map" &&
        std::isfinite(position.x) && std::isfinite(position.y)) {
        feedback_stamp_ = rclcpp::Time(stamp).seconds();
        feedback_arrival_ = std::chrono::steady_clock::now();
    }
    if(public_goal_ && public_goal_->is_active()) {
        public_goal_->publish_feedback(std::make_shared<Navigate::Feedback>(*feedback));
    }
}


// Terminal results close the public goal before any reactive fallback is permitted:
void NavigationSupervisor::backend_result(const BackendGoal::WrappedResult & result)
{
    select_source("hold");
    if(stage_ == Stage::FAULT) return;
    if(public_goal_ && public_goal_->is_active()) {
        auto response = std::make_shared<Navigate::Result>();
        if(public_goal_->is_canceling()) {
            public_goal_->canceled(response);
            report("CANCELED", "Nav2 cancellation completed");
        }
        else if(result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            public_goal_->succeed(response);
            report("GOAL_REACHED", "Navigation goal reached");
        }
        else {
            public_goal_->abort(response);
            report("NAVIGATION_FAILED", "Nav2 aborted the goal");
        }
    }
    public_goal_.reset();
    backend_goal_.reset();
    transition(Stage::FINISHING);
}


// Room tasks suppress free roaming between their observation goals:
void NavigationSupervisor::control_request(
    const std::shared_ptr<squad_interfaces::srv::SetControl::Request> request,
    std::shared_ptr<squad_interfaces::srv::SetControl::Response> response
)
{
    response->accepted = false;
    if(stage_ == Stage::FAULT) {
        response->message = "FAULT";
        return;
    }
    if(request->mode == "stop") {
        task_owned_ = true;
        cancel_requested_ = public_goal_ != nullptr;
        select_source("hold");
        if(room_owned_) {
            if(backend_goal_) action_client_->async_cancel_goal(backend_goal_);
            fail("Explicit stop during a reserved room task; reservation retained");
        }
        response->accepted = true;
        response->message = "Stop latched; canceling any active navigation";
        return;
    }
    if(stage_ != Stage::IDLE || public_goal_) {
        response->message = "Wait for terminal navigation and handover before changing mode";
        return;
    }
    if(request->mode == "task") {
        task_owned_ = true;
        room_owned_ = true;
        room_heartbeat_ = std::chrono::steady_clock::now();
        select_source("hold");
    }
    else if(request->mode == "release") {
        task_owned_ = true;
        room_owned_ = false;
        select_source("hold");
    }
    else if(request->mode == "hold") {
        task_owned_ = true;
        select_source("hold");
    }
    else if(request->mode == "scan") {
        if(!task_owned_) return;
        select_source("reactive", "scan");
    }
    else if(request->mode == "roam") {
        if(room_owned_) {
            response->message = "A reserved room task owns this robot";
            return;
        }
        task_owned_ = false;
        roam_on_idle_ = true;
        select_source("reactive", "roam");
    }
    else {
        response->message = "Expected task, release, hold, scan or roam";
        return;
    }
    response->accepted = true;
    response->message = request->mode;
}


// Loss of a task owner cannot leave its navigation goal running unattended:
void NavigationSupervisor::task_heartbeat(std_msgs::msg::Empty::ConstSharedPtr)
{
    room_heartbeat_ = std::chrono::steady_clock::now();
}


// The ordinary RViz goal tool follows exactly the same public action path:
void NavigationSupervisor::pose_goal_callback(
    geometry_msgs::msg::PoseStamped::ConstSharedPtr message
)
{
    if(task_owned_ || stage_ != Stage::IDLE) {
        report("GOAL_REJECTED", task_owned_ ?
            "RViz goal ignored: task ownership or explicit hold is active" :
            "RViz goal ignored: navigation is busy; wait for the current goal and handover");
        return;
    }
    Navigate::Goal goal;
    goal.pose = *message;
    public_client_->async_send_goal(goal);
}


// Track physical stopping with an independent arrival watchdog:
void NavigationSupervisor::odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
    speed_ = std::hypot(message->twist.twist.linear.x, message->twist.twist.linear.y);
    turn_ = std::abs(message->twist.twist.angular.z);
    odom_time_ = std::chrono::steady_clock::now();
}


// Confirm the localization authority is publishing recent, finite robot poses:
bool NavigationSupervisor::localization_ready()
{
    try {
        const auto pose = buffer_.lookupTransform("map", robot_ + "/base_footprint",
            tf2::TimePointZero);
        const double age = (now() - rclcpp::Time(pose.header.stamp)).seconds();
        return age >= -0.1 && age < 0.6 && std::isfinite(pose.transform.translation.x);
    }
    catch(const tf2::TransformException &) {
        return false;
    }
}


// A stale odometry sample is not evidence that the robot has stopped:
bool NavigationSupervisor::stopped() const
{
    return speed_ < 0.01 && turn_ < 0.03 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - odom_time_).count() < 0.5;
}


// The planner/controller TF cache must be fresh independently of this node's own TF cache:
bool NavigationSupervisor::backend_pose_fresh() const
{
    const double age = now().seconds() - feedback_stamp_;
    const double arrival_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - feedback_arrival_).count();
    return feedback_stamp_ >= 0.0 && age >= -0.1 && age < 0.8 && arrival_age < 0.8;
}


// Increment the lease epoch on every source or maneuver change:
void NavigationSupervisor::select_source(
    const std::string & source, const std::string & reactive_mode
)
{
    if(source != lease_.source || reactive_mode != lease_.reactive_mode) {
        ++lease_.generation;
        lease_.source = source;
        lease_.reactive_mode = reactive_mode;
    }
}


// Stage time uses monotonic time so paused simulation cannot block cancellation forever:
void NavigationSupervisor::transition(Stage stage)
{
    stage_ = stage;
    stage_time_ = std::chrono::steady_clock::now();
}


// Keep lifecycle decisions, cancellation and command selection in one serialized loop:
void NavigationSupervisor::control_loop()
{
    const auto wall = std::chrono::steady_clock::now();
    const double stage_age = std::chrono::duration<double>(wall - stage_time_).count();
    if(room_owned_ && std::chrono::duration<double>(wall - room_heartbeat_).count() > 1.5) {
        if(backend_goal_ && !cancel_sent_) action_client_->async_cancel_goal(backend_goal_);
        fail("Room task owner heartbeat expired; entrance reservation retained");
    }
    if(stage_ == Stage::STOPPING && stopped() && stage_age > 0.35) {
        if(cancel_requested_) {
            public_goal_->canceled(std::make_shared<Navigate::Result>());
            public_goal_.reset();
            transition(Stage::FINISHING);
        }
        else {
            rclcpp_action::Client<Navigate>::SendGoalOptions options;
            options.goal_response_callback = std::bind(
                &NavigationSupervisor::backend_response, this, std::placeholders::_1);
            options.feedback_callback = std::bind(&NavigationSupervisor::backend_feedback,
                this, std::placeholders::_1, std::placeholders::_2);
            options.result_callback = std::bind(
                &NavigationSupervisor::backend_result, this, std::placeholders::_1);
            auto backend_goal = *public_goal_->get_goal();
            // A goal in fixed map coordinates remains valid as localization updates:
            backend_goal.pose.header.stamp = rclcpp::Time(0);
            action_client_->async_send_goal(backend_goal, options);
            transition(Stage::SENDING);
        }
    }
    if(stage_ == Stage::ACTIVE && (!localization_ready() ||
        std::chrono::duration<double>(wall - goal_time_).count() > navigation_timeout_)) {
        cancel_requested_ = true;
    }
    if(stage_ == Stage::ACTIVE && !cancel_requested_) {
        if(backend_pose_fresh()) select_source("nav");
        else if(std::chrono::duration<double>(wall - stage_time_).count() > 2.0) {
            action_client_->async_cancel_goal(backend_goal_);
            cancel_sent_ = true;
            fail("Nav2 pose feedback is stale; refusing commands from an old TF cache");
        }
    }
    if(cancel_requested_ && backend_goal_ && !cancel_sent_) {
        select_source("hold");
        cancel_sent_ = true;
        action_client_->async_cancel_goal(backend_goal_);
        transition(Stage::CANCELING);
    }
    if((stage_ == Stage::CANCELING || stage_ == Stage::SENDING ||
        stage_ == Stage::STOPPING) &&
        std::chrono::duration<double>(wall - stage_time_).count() > 8.0) {
        fail("Navigation handover timed out; manual recovery required");
    }
    if(stage_ == Stage::FINISHING && stopped() && stage_age > 0.4) {
        cancel_requested_ = false;
        transition(Stage::IDLE);
    }
    if(stage_ == Stage::IDLE && !task_owned_) {
        select_source(roam_on_idle_ && localization_ready() ? "reactive" : "hold", "roam");
    }
    lease_.header.stamp = now();
    lease_publisher_->publish(lease_);
    report_idle_activity();
}


// An idle action server can still authorize reactive motion; report the selected activity:
void NavigationSupervisor::report_idle_activity()
{
    if(stage_ != Stage::IDLE || room_owned_) return;
    std::string phase;
    std::string detail;
    if(lease_.source == "reactive" && lease_.reactive_mode == "roam") {
        phase = "REACTIVE_NAVIGATION";
        detail = "Autonomous obstacle-reactive navigation";
    }
    else if(!roam_on_idle_ && !task_owned_) {
        phase = "WAITING_FOR_TASK";
        detail = "Stationary; waiting for a room assignment";
    }
    else {
        phase = "HOLD";
        detail = task_owned_ ? "Stationary; autonomous roaming disabled" :
            "Stationary; waiting for localization before reactive navigation";
    }
    if(phase != reported_phase_) report(phase, detail);
}


// Publish structured status separately from velocity commands:
void NavigationSupervisor::report(const std::string & phase, const std::string & detail)
{
    reported_phase_ = phase;
    squad_interfaces::msg::RobotEvent event;
    event.header.stamp = now();
    event.robot = robot_;
    event.phase = phase;
    event.detail = detail;
    event_publisher_->publish(event);
    RCLCPP_INFO(get_logger(), "%s: %s", phase.c_str(), detail.c_str());
}


// Failure leaves an explicit fault hold, including when the backend never confirms stopping:
void NavigationSupervisor::fail(const std::string & reason)
{
    select_source("hold");
    if(stage_ == Stage::FAULT) return;
    if(public_goal_ && public_goal_->is_active()) {
        public_goal_->abort(std::make_shared<Navigate::Result>());
    }
    public_goal_.reset();
    transition(Stage::FAULT);
    report("FAULT", reason);
}


// Serialize asynchronous action callbacks and mode changes:
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationSupervisor>());
    rclcpp::shutdown();
    return 0;
}
