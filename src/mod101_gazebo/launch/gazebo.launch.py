#!/usr/bin/env python3
"""Bring up the mod101 arm in Gazebo Sim with ros2_control wired through the
gz_ros2_control plugin. Spawns the robot, then loads controllers via the
controller_manager spawner Node (the canonical path — `ros2 control
load_controller` has historically tripped JTC's parameter library on Jazzy).
arm_trajectory_controller is loaded inactive for MoveIt to activate on demand.
"""

import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_description = get_package_share_directory('mod101_description')
    pkg_gazebo = get_package_share_directory('mod101_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    install_prefix = get_package_prefix('mod101_description')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(install_prefix, 'share'),
    )

    world_file = os.path.join(pkg_gazebo, 'worlds', 'empty.sdf')
    urdf_file = os.path.join(pkg_description, 'urdf', 'mod101.xacro')
    bridge_config = os.path.join(pkg_gazebo, 'config', 'gz_ros_bridge.yaml')

    robot_description = xacro.process_file(urdf_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mod101',
            '-z', '0.05',
        ],
        output='screen',
    )

    def spawner(name, *extra_args, delay):
        return TimerAction(
            period=delay,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=[name, '--controller-manager', '/controller_manager',
                           *extra_args],
                output='screen',
            )],
        )

    # Time-staged spawners; let the controller_manager come up before we hit it.
    # arm_trajectory_controller is loaded inactive — MoveIt (or a manual
    # switch_controllers call) activates it and deactivates arm_controller.
    spawn_jsb = spawner('joint_state_broadcaster', delay=3.0)
    spawn_arm = spawner('arm_controller', delay=5.0)
    spawn_arm_traj = spawner('arm_trajectory_controller', '--inactive', delay=7.0)
    spawn_gripper = spawner('gripper_controller', delay=9.0)

    return LaunchDescription([
        gz_resource_path,
        robot_state_publisher,
        gz_sim,
        clock_bridge,
        bridge,
        spawn_robot,
        spawn_jsb,
        spawn_arm,
        spawn_arm_traj,
        spawn_gripper,
    ])
