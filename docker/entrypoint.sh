#!/bin/bash
# Sourced-in ROS, once, for every service.
#
# Without this each `command:` in the compose file has to be a `bash -lc`
# wrapping a `source ... && ros2 ...` chain, which is why they were unreadable
# folded scalars. Here the shell setup happens once and every service is the
# bare command it actually runs.
#
# The workspace overlay is sourced only IF it exists: on a clean checkout the
# `workspace` service has to be able to start and build it, and it cannot do
# that if the entrypoint fails on the missing setup.bash first.
set -e
source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
exec "$@"
