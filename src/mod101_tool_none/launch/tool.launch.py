#!/usr/bin/env python3
"""No-tool launch — nothing to spawn, but the file must exist so that
mod101_gazebo's IncludeLaunchDescription resolves regardless of tool selection.

`spawn_controllers` is declared but unused: mod101_gazebo passes it to every
tool launch, and an include that doesn't declare an argument it's handed is an
error. Any new tool package needs the same declaration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('spawn_controllers', default_value='true'),
    ])
