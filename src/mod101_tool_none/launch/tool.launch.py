#!/usr/bin/env python3
"""No-tool launch — nothing to spawn, but the file must exist so that
mod101_gazebo's IncludeLaunchDescription resolves regardless of tool selection."""

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
