#!/usr/bin/env bash
# Source the same message installation selected by CMake for runtime dlopen.
dts_setup_ros_runtime() {
    local project_root="$1"
    local cache="$project_root/build/CMakeCache.txt"
    local message_cmake_dir message_prefix
    [[ -f "$cache" ]] || return 0
    grep -q '^BUILD_WITH_NAV_ROS2:BOOL=ON$' "$cache" || return 0
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        source /opt/ros/humble/setup.bash || return 1
    fi
    message_cmake_dir=$(sed -n 's/^point_msgs_DIR:[^=]*=//p' "$cache" | head -n 1)
    message_prefix="${message_cmake_dir%/share/point_msgs/cmake}"
    if [[ -z "$message_cmake_dir" || ! -f "$message_prefix/share/point_msgs/local_setup.bash" ]]; then
        echo "ROS runtime: point_msgs setup missing for $message_cmake_dir" >&2
        return 1
    fi
    source "$message_prefix/share/point_msgs/local_setup.bash"
    # Shared memory segments are created mode 0644, so a peer running as a
    # different user cannot map them and no samples are delivered in that
    # direction. Announce UDPv4 locators only instead.
    local dds_profile="$project_root/config/fastdds_udp_only.xml"
    if [[ -f "$dds_profile" ]]; then
        export FASTRTPS_DEFAULT_PROFILES_FILE="$dds_profile"
    fi
}
