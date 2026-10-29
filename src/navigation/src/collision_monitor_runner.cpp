#include <chrono>
#include <csignal>
#include <memory>

#include <lifecycle_msgs/msg/state.hpp>
#include <nav2_collision_monitor/collision_monitor_node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;
constexpr std::chrono::milliseconds SPIN_POLL_PERIOD{50};

// Defer lifecycle cleanup until the executor has finished its current callback:
void request_stop(int)
{
    stop_requested = 1;
}


// Release only the lifecycle resources present in the monitor's current state:
void clean_up_monitor(nav2_collision_monitor::CollisionMonitor & monitor)
{
    if(monitor.get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        monitor.deactivate();
    }
    if(monitor.get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        monitor.cleanup();
    }
}
}  // namespace


// Run the upstream monitor and serialize lifecycle cleanup after callback completion:
int main(int argc, char ** argv)
{
    // Keep the upstream Humble library unchanged and own only signal/cleanup ordering:
    rclcpp::init(
        argc,
        argv,
        rclcpp::InitOptions(),
        rclcpp::SignalHandlerOptions::None
    );
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    auto monitor = std::make_shared<nav2_collision_monitor::CollisionMonitor>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(monitor->get_node_base_interface());

    // A stock-main cleanup racing a scan callback was observed during initial validation:
    while(rclcpp::ok() && !stop_requested) {
        executor.spin_once(SPIN_POLL_PERIOD);
    }
    executor.remove_node(monitor->get_node_base_interface());
    clean_up_monitor(*monitor);
    rclcpp::shutdown();
    return 0;
}
