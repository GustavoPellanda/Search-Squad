#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <squad_interfaces/msg/robot_event.hpp>

/*
Show what each robot is doing, with task progress and relevant warnings.
Filtering affects this viewer only; control and detailed ROS topics remain independent.
*/
class MissionLogger : public rclcpp::Node
{
    public:

    MissionLogger();

    private:

    using WallClock = std::chrono::steady_clock;
    using RobotEvent = squad_interfaces::msg::RobotEvent;

    struct SafetyReport {
        RobotEvent candidate;
        WallClock::time_point changed{};
        WallClock::time_point printed{};
        std::string last_signature;
        bool stop_visible = false;
    };

    void event_callback(RobotEvent::ConstSharedPtr event);
    void safety_event(const RobotEvent & event);
    void flush_safety_reports();
    void warning_callback(rcl_interfaces::msg::Log::ConstSharedPtr message);
    void write_event(const RobotEvent & event);
    void write_line(const std::string & line);

    double progress_interval_ = 30.0;
    double warning_interval_ = 30.0;
    double safety_interval_ = 10.0;
    double safety_settle_time_ = 1.0;
    std::ofstream output_;
    std::map<std::string, std::string> phases_;
    std::map<std::string, WallClock::time_point> reports_;
    std::map<std::string, SafetyReport> safety_reports_;
    std::map<std::string, bool> room_active_;
    rclcpp::Subscription<RobotEvent>::SharedPtr event_subscription_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr warning_subscription_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
};


// Configure the human-readable summary independently of control-loop reporting:
MissionLogger::MissionLogger() : Node("mission_logger")
{
    progress_interval_ = declare_parameter("progress_interval", 30.0);
    warning_interval_ = declare_parameter("warning_interval", 30.0);
    safety_interval_ = declare_parameter("safety_interval", 10.0);
    safety_settle_time_ = declare_parameter("safety_settle_time", 1.0);
    for(double interval : {
        progress_interval_, warning_interval_, safety_interval_, safety_settle_time_
    }) {
        if(!std::isfinite(interval) || interval <= 0.0) {
            throw std::invalid_argument("Mission log intervals must be finite and positive");
        }
    }

    const auto path = declare_parameter("log_file", "/tmp/search_squad_mission/events.log");
    const auto directory = std::filesystem::path(path).parent_path();
    if(!directory.empty()) std::filesystem::create_directories(directory);
    output_.open(path, std::ios::out | std::ios::trunc);
    if(!output_) throw std::runtime_error("Cannot open mission log: " + path);

    event_subscription_ = create_subscription<RobotEvent>(
        "/fleet/events",
        50,
        std::bind(&MissionLogger::event_callback, this, std::placeholders::_1)
    );
    warning_subscription_ = create_subscription<rcl_interfaces::msg::Log>(
        "/rosout",
        100,
        std::bind(&MissionLogger::warning_callback, this, std::placeholders::_1)
    );
    safety_timer_ = create_wall_timer(
        std::chrono::milliseconds(250),
        std::bind(&MissionLogger::flush_safety_reports, this)
    );
    write_line("SEARCH SQUAD | Mission log");
}


// Keep task milestones immediate and summarize internal exploration cycles as progress:
void MissionLogger::event_callback(RobotEvent::ConstSharedPtr event)
{
    if(event->phase.rfind("SAFETY_", 0) == 0) {
        safety_event(*event);
        return;
    }

    const bool terminal_task = event->phase == "COMPLETED" || event->phase == "INCOMPLETE" ||
        event->phase == "FAILED" || event->phase == "IDLE";
    if(!event->task_id.empty()) room_active_[event->robot] = !terminal_task;
    // Room IDLE means no room task; the supervisor reports the robot's actual activity:
    if(event->phase == "IDLE") return;
    if(event->task_id.empty() && room_active_[event->robot] &&
        (event->phase == "NAVIGATING" || event->phase == "GOAL_REACHED")) {
        return;
    }

    const bool exploration_cycle = event->phase == "OBSERVING" ||
        event->phase == "EXPLORING" || event->phase == "PLANNING_OBSERVATION";
    const std::string phase_group = exploration_cycle ? "EXPLORING" : event->phase;
    const std::string category = event->task_id.empty() ? "navigation" : "task/" + event->task_id;
    const std::string key = event->robot + "/" + category;
    const std::string signature = phase_group + "/" + event->mapping_status + "/" + event->detail;
    const auto wall = WallClock::now();
    const double elapsed = std::chrono::duration<double>(wall - reports_[key]).count();
    const bool unchanged = phases_[key] == signature;
    if(unchanged && (!exploration_cycle || elapsed < progress_interval_)) return;

    phases_[key] = signature;
    reports_[key] = wall;
    RobotEvent summary = *event;
    if(exploration_cycle) summary.phase = "EXPLORING";
    write_event(summary);
}


// Coalesce frequent CLEAR/SLOW/HOLD changes and remember persistent stop candidates:
void MissionLogger::safety_event(const RobotEvent & event)
{
    RobotEvent candidate = event;
    if(event.phase != "SAFETY_STOP") {
        candidate.phase = "SAFETY_CLEAR";
        candidate.detail = "Protection recovered";
    }
    auto & report = safety_reports_[event.robot];
    if(report.candidate.phase == candidate.phase && report.candidate.detail == candidate.detail) {
        return;
    }
    report.candidate = candidate;
    report.changed = WallClock::now();
}


// A wall timer confirms persistence even when a producer publishes transitions only:
void MissionLogger::flush_safety_reports()
{
    const auto wall = WallClock::now();
    for(auto & entry : safety_reports_) {
        auto & report = entry.second;
        const bool stopped = report.candidate.phase == "SAFETY_STOP";
        if(!stopped && !report.stop_visible) continue;

        const double stable_age = std::chrono::duration<double>(wall - report.changed).count();
        if(stable_age < safety_settle_time_) continue;
        const std::string signature = report.candidate.phase + "/" + report.candidate.detail;
        if(signature == report.last_signature) continue;
        const double elapsed = std::chrono::duration<double>(wall - report.printed).count();
        if(stopped && elapsed < safety_interval_) continue;

        RobotEvent summary = report.candidate;
        if(!stopped) summary.phase = "SAFETY_RECOVERED";
        write_event(summary);
        report.stop_visible = stopped;
        report.last_signature = signature;
        report.printed = wall;
    }
}


// Show new warnings/errors and throttle recurrence; mask refresh notices are routine traffic:
void MissionLogger::warning_callback(rcl_interfaces::msg::Log::ConstSharedPtr message)
{
    if(message->level < rcl_interfaces::msg::Log::WARN) return;
    if(message->level == rcl_interfaces::msg::Log::WARN &&
        message->msg.find("KeepoutFilter: New filter mask arrived") != std::string::npos) {
        return;
    }
    const auto wall = WallClock::now();
    const std::string key = "warning/" + message->name + "/" + message->msg.substr(0, 55);
    const double elapsed = std::chrono::duration<double>(wall - reports_[key]).count();
    if(elapsed < warning_interval_) return;
    reports_[key] = wall;
    write_line("ROS " + message->name + " | " + message->msg);
}


// Omit irrelevant empty task fields and zero frontier counters from ordinary status lines:
void MissionLogger::write_event(const RobotEvent & event)
{
    std::string line = "[" + std::to_string(rclcpp::Time(event.header.stamp).seconds()) +
        "] " + event.robot + " | " + event.phase;
    if(!event.task_id.empty()) line += " | task=" + event.task_id;
    if(!event.mapping_status.empty()) line += " | " + event.mapping_status;
    if(event.phase == "EXPLORING") {
        line += " | frontiers=" + std::to_string(event.frontier_count) +
            " known=" + std::to_string(event.known_cells);
    }
    if(!event.detail.empty()) line += " | " + event.detail;
    write_line(line);
}


// Flush summaries to the log file and the launch output at the same time:
void MissionLogger::write_line(const std::string & line)
{
    std::cout << line << std::endl;
    output_ << line << std::endl;
}


// Let launch own the collector process and its shutdown:
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionLogger>());
    rclcpp::shutdown();
    return 0;
}
