#!/bin/bash
# Start the implemented fleet demonstration from a clean, workspace-local ROS environment.
set -e -o pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Keep display access and explicit isolation settings, but discard inherited ROS overlays.
exec env -i \
    HOME="$HOME" \
    USER="${USER:-}" \
    PATH=/usr/bin:/bin:/usr/sbin:/sbin \
    LANG="${LANG:-C.UTF-8}" \
    DISPLAY="${DISPLAY:-}" \
    XAUTHORITY="${XAUTHORITY:-}" \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}" \
    GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11461}" \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    bash --noprofile --norc -c '
        set -e -o pipefail
        project_root="$1"
        shift

        if [[ ! -f /opt/ros/humble/setup.bash ]]; then
            echo "ROS 2 Humble is required at /opt/ros/humble." >&2
            exit 1
        fi
        source /opt/ros/humble/setup.bash

        if [[ ! -f "$project_root/install/local_setup.bash" ]]; then
            echo "Build this workspace with colcon before running the demonstration." >&2
            exit 1
        fi
        source "$project_root/install/local_setup.bash"

        if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
            echo "Install the middleware: sudo apt install ros-humble-rmw-cyclonedds-cpp" >&2
            exit 1
        fi

        exec ros2 launch mission fleet.launch.py demo:=true "$@"
    ' search-squad-demo "$project_root" "$@"
