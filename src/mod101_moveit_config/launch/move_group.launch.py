#!/usr/bin/env python3
"""move_group for mod101, plus (optionally) RViz.

Assumes a controller_manager is already up — from the Gazebo plugin
(mod101_gazebo/launch/gazebo.launch.py) or from mock hardware
(mod101_moveit_config/launch/mock.launch.py). It does NOT start one.

Because both the URDF and the SRDF are parametric, every arg here has to match
whatever the running robot was expanded with, or MoveIt will plan against a
different robot than the one in the sim. demo.launch.py forwards them for you.

Launch args:
    tool                                     mod101_tool_<name>, default jaws
    shoulder_ext_length / elbow_ext_length    2020 rail length, m
    shoulder_mount / elbow_mount              small | big
    use_sim_time                              default true
    rviz                                      launch RViz too, default true
"""

import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

BUILD_ARGS = ('shoulder_ext_length', 'elbow_ext_length',
              'shoulder_mount', 'elbow_mount')


def _configured_tool():
    """The tool the configurator last saved, from mod101_config.xacro.

    Used as the launch default so `ros2 launch` agrees with the configurator
    instead of pinning one tool forever. `tool:=parallel` still overrides.
    """
    cfg = os.path.join(get_package_share_directory('mod101_description'),
                       'urdf', 'mod101_config.xacro')
    m = re.search(r'<xacro:arg\s+name="tool"\s+default="([^"]+)"', open(cfg).read())
    return m.group(1) if m else 'jaws'


def _drop_unset(mappings):
    """Args left empty fall through to mod101_config.xacro's defaults.

    The configurator writes that file, so a launch must not shadow it with a
    stale hardcoded number. Anything explicitly passed still wins.
    """
    return {k: v for k, v in mappings.items() if v != ''}


def build_moveit_config(context):
    """Assemble MoveIt's parameters for the requested build configuration."""
    tool = LaunchConfiguration('tool').perform(context)
    mappings = _drop_unset(
        {k: LaunchConfiguration(k).perform(context) for k in BUILD_ARGS})

    pkg_moveit = get_package_share_directory('mod101_moveit_config')
    urdf = os.path.join(
        get_package_share_directory('mod101_description'), 'urdf', 'mod101.xacro')
    srdf = os.path.join(pkg_moveit, 'srdf', 'mod101.srdf.xacro')

    moveit_config = (
        MoveItConfigsBuilder('mod101', package_name='mod101_moveit_config')
        # use_sim=true keeps the gz_ros2_control blocks in the description so
        # the URDF here is byte-identical to the one the sim spawned.
        .robot_description(file_path=urdf,
                           mappings={**mappings, 'tool': tool, 'use_sim': 'true'})
        .robot_description_semantic(file_path=srdf, mappings={'tool': tool})
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )

    _merge_tool_controllers(moveit_config, tool)
    return moveit_config


def _merge_tool_controllers(moveit_config, tool):
    """Fold mod101_tool_<tool>'s gripper controller into the execution config.

    The tool layer owns its controllers everywhere else (ros2_control block,
    controllers.yaml, launch fragment), so it owns its MoveIt entry too rather
    than this package carrying a hardcoded table of every gripper that exists.
    """
    try:
        path = os.path.join(
            get_package_share_directory(f'mod101_tool_{tool}'),
            'config', 'moveit_controllers.yaml')
        with open(path) as f:
            entries = yaml.safe_load(f)
    except (FileNotFoundError, KeyError):
        print(f'[mod101_moveit_config] no MoveIt controllers for tool "{tool}"')
        return

    if not entries:  # mod101_tool_none: file exists but is comments only
        return

    mgr = moveit_config.trajectory_execution['moveit_simple_controller_manager']
    for name, spec in entries.items():
        mgr[name] = spec
        if name not in mgr['controller_names']:
            mgr['controller_names'].append(name)
    print(f'[mod101_moveit_config] tool "{tool}" contributes: {list(entries)}')


def _build(context):
    moveit_config = build_moveit_config(context)

    # ParameterValue(..., value_type=bool) is load-bearing. A bare
    # LaunchConfiguration resolves to the STRING "true", and use_sim_time is a
    # built-in bool parameter — passing a string means it silently never takes
    # effect. move_group and RViz then run on wall clock while TF arrives
    # stamped with Gazebo's sim time, and tf2 spams "Detected jump back in
    # time. Clearing TF buffer." on every transform.
    use_sim_time = {'use_sim_time': ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)}

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), use_sim_time],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(
            get_package_share_directory('mod101_moveit_config'),
            'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
        ],
    )

    return [move_group, rviz]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('tool', default_value=_configured_tool()),
        # Empty = "whatever the configurator last saved": mod101_config.xacro
        # holds the defaults, and _drop_unset() below keeps unset args out of
        # the xacro mappings so that file wins. Pass a value to override.
        DeclareLaunchArgument('shoulder_ext_length', default_value=''),
        DeclareLaunchArgument('elbow_ext_length', default_value=''),
        DeclareLaunchArgument('shoulder_mount', default_value=''),
        DeclareLaunchArgument('elbow_mount', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_build)])
