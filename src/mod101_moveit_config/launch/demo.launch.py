#!/usr/bin/env python3
"""Full MoveIt demo: Gazebo sim + move_group + RViz, with the right controllers.

    ros2 launch mod101_moveit_config demo.launch.py
    ros2 launch mod101_moveit_config demo.launch.py tool:=parallel

Controllers: the plain gazebo launch brings up `arm_controller`
(JointGroupPositionController, Float64MultiArray) and the tool brings up
`gripper_controller`. MoveIt can't drive either — it speaks
FollowJointTrajectory — and the trajectory controllers claim the same joints,
which controller_manager won't allow twice. So rather than spawning the position
controllers and then switching away from them (racy: the switch can land before
the spawner has finished loading, and `ros2 control switch_controllers` then
blocks), this passes `spawn_controllers:=false` to the gazebo launch and brings
up only the trajectory controllers.
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BUILD_ARGS = ('tool', 'shoulder_ext_length', 'elbow_ext_length',
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


def _build(context):
    params = _drop_unset(
        {k: LaunchConfiguration(k).perform(context) for k in BUILD_ARGS})
    tool = params['tool']

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mod101_gazebo'),
            'launch', 'gazebo.launch.py')),
        launch_arguments={**params, 'spawn_controllers': 'false'}.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mod101_moveit_config'),
            'launch', 'move_group.launch.py')),
        launch_arguments={**params,
                          'use_sim_time': 'true',
                          'rviz': LaunchConfiguration('rviz')}.items(),
    )

    # mod101_tool_none has no actuated end-effector.
    controllers = ['arm_trajectory_controller']
    if tool != 'none':
        controllers.append('gripper_trajectory_controller')

    # Timed, not event-chained — see the note in mock.launch.py and
    # docs/performance-notes.md. move_group goes last because its execution
    # manager enumerates controllers at startup: start it early and it comes up
    # with none, and every plan fails at execution time.
    spawners = [
        TimerAction(period=8.0 + 2.0 * i, actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60'],
            output='screen')])
        for i, name in enumerate(controllers)
    ]

    return [gazebo, *spawners, TimerAction(period=16.0, actions=[move_group])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tool', default_value=_configured_tool()),
        # Empty = "whatever the configurator last saved": mod101_config.xacro
        # holds the defaults, and _drop_unset() below keeps unset args out of
        # the xacro mappings so that file wins. Pass a value to override.
        DeclareLaunchArgument('shoulder_ext_length', default_value=''),
        DeclareLaunchArgument('elbow_ext_length', default_value=''),
        DeclareLaunchArgument('shoulder_mount', default_value=''),
        DeclareLaunchArgument('elbow_mount', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_build),
    ])
