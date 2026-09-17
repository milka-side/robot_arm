#!/usr/bin/bash

set -e

source_if_exists() {
    local script_path="$1"

    if [ -f "$script_path" ]; then
        # shellcheck disable=SC1090
        source "$script_path"
    fi
}

# -------------------- ROS2 Workspace Setup --------------------

TARGET_ROS_DISTRO="${TARGET_ROS_DISTRO:-humble}"
WS="${WS:-/opt/ws}"

source_if_exists "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
source_if_exists "${WS}/install/setup.bash"

echo "[ROS] SUCCESS: Environment ready (${TARGET_ROS_DISTRO})."

exec "$@"
