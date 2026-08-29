#!/usr/bin/env bash
# Sourced-in ROS and the mod101 overlay, once, for every service.
#
# Without this each `command:` in the compose file has to be a `bash -lc`
# wrapping a `source ... && ros2 ...` chain, which is why they used to be
# unreadable folded scalars. Here the shell setup happens once and every
# service is the bare command it actually runs.
#
# Not `set -u`: ROS's own setup scripts read unset variables
# (AMENT_TRACE_SETUP_FILES, COLCON_TRACE, ...) and would abort under it.
set -eo pipefail
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/mod101/setup.bash
exec "$@"
