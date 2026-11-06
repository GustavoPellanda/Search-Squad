#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <squad_interfaces/msg/robot_event.hpp>

/*
Collect robot task transitions and application warnings into a single flushed log.
The graphical terminal follows this file; closing the viewer does not stop robots.
*/
class MissionLogger : public rclcpp::Node
{
    public:

    MissionLogger();

    private:

    void event_callback(squad_interfaces::msg::RobotEvent::ConstSharedPtr event);
    void warning_callback(rcl_interfaces::msg::Log::ConstSharedPtr message);
    void write_line(const std::string & line);

    std::ofstream output_;
    std::map<std::string, std::string> phases_;
    std::map<std::string, std::chrono::steady_clock::time_point> reports_;
    rclcpp::Subscription<squad_interfaces::msg::RobotEvent>::SharedPtr event_subscription_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr warning_subscription_;
};


// Open a runtime output outside the source tree and subscribe to every robot's events:
MissionLogger::MissionLogger() : Node("mission_logger")
{
    const auto path = declare_parameter("log_file", "/tmp/search_squad_mission/events.log");
    const auto directory = std::filesystem::path(path).parent_path();
    if(!directory.empty()) std::filesystem::create_directories(directory);
    output_.open(path, std::ios::out | std::ios::trunc);
    if(!output_) throw std::runtime_error("Cannot open mission log: " + path);
    event_subscription_ = create_subscription<squad_interfaces::msg::RobotEvent>(
        "/fleet/events", 50, std::bind(&MissionLogger::event_callback, this, std::placeholders::_1)
    );
    warning_subscription_ = create_subscription<rcl_interfaces::msg::Log>(
        "/rosout", 100, std::bind(&MissionLogger::warning_callback, this, std::placeholders::_1)
    );
    write_line("SEARCH SQUAD | Central mission status | localization mode must be declared");
}


// Print phase changes immediately and throttle repeated progress heartbeats:
void MissionLogger::event_callback(squad_interfaces::msg::RobotEvent::ConstSharedPtr event)
{
    const std::string key = event->robot + "/" + event->task_id;
    const std::string phase = event->phase + "/" + event->mapping_status;
    const auto wall = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(wall - reports_[key]).count();
    if(phases_[key] == phase && elapsed < 3.0) return;
    phases_[key] = phase;
    reports_[key] = wall;
    write_line("[" + std::to_string(rclcpp::Time(event->header.stamp).seconds()) + "] " +
        event->robot + " | " + event->task_id + " | " + event->phase + " | " +
        event->mapping_status + " | frontiers=" + std::to_string(event->frontier_count) +
        " known=" + std::to_string(event->known_cells) + " | " + event->detail);
}


// Show warnings/errors without repeating every node's ordinary ROS informational output:
void MissionLogger::warning_callback(rcl_interfaces::msg::Log::ConstSharedPtr message)
{
    if(message->level < rcl_interfaces::msg::Log::WARN) return;
    const auto wall = std::chrono::steady_clock::now();
    const std::string key = "warning/" + message->name + "/" + message->msg.substr(0, 55);
    if(std::chrono::duration<double>(wall - reports_[key]).count() < 5.0) return;
    reports_[key] = wall;
    write_line("ROS " + message->name + " | " + message->msg);
}


// Flush each event so the terminal and a post-run report see the same record:
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
