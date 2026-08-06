#!/usr/bin/env python3
"""SO-101 jaws tool launch. Spawns the gripper_controller after the arm's
joint_state_broadcaster and arm_controller have come up."""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        TimerAction(
            period=10.0,  # after JSB (3s) + arm (5s) in the main gazebo launch
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller',
                           '--controller-manager', '/controller_manager'],
                output='screen',
            )],
        ),
    ])
