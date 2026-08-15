#!/usr/bin/env python3
"""Parallel-jaw tool launch. Spawns the gripper_controller after the arm's\njoint_state_broadcaster and arm_controller have come up.

`spawn_controllers:=false` suppresses the gripper_controller — used when MoveIt
drives the tool over FollowJointTrajectory instead (mod101_moveit_config), since
gripper_controller and gripper_trajectory_controller both claim joint 6 and
controller_manager rejects two active claims on one joint.

The 10 s delay is deliberate; see docs/performance-notes.md before removing it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'spawn_controllers', default_value='true',
            description='Spawn this tool\'s position controller.'),
        TimerAction(
            period=10.0,  # after JSB (3s) + arm (5s) in the main gazebo launch
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                condition=IfCondition(LaunchConfiguration('spawn_controllers')),
                arguments=['gripper_controller',
                           '--controller-manager', '/controller_manager',
                           '--controller-manager-timeout', '60'],
                output='screen',
            )],
        ),
    ])
