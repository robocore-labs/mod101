#!/usr/bin/env python3
"""The harness robot on real hardware - a prepared skeleton, not a driver.

Brings up robot_state_publisher (use_sim=false) and a controller_manager
against the placeholder `mock_components/GenericSystem` declared in
mod101_harness_arm.xacro, then spawns the same controllers the sim uses. It
runs as-is (fake hardware), so it's a working bench target while the real
integration lands.

TO GO LIVE:
  - swap `mock_components/GenericSystem` in urdf/mod101_harness_arm.xacro for
    the real servo-bus hardware_interface plugin (see
    mod101_control/urdf/mod101.hardware.xacro for the arm-only version);
  - add the RealSense driver and the pan/tilt servo driver here.

Build args (tool + the four build params) behave exactly as in sim.launch.py.
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

BUILD_ARGS = ('tool', 'shoulder_ext_length', 'elbow_ext_length',
              'shoulder_mount', 'elbow_mount')


def _configured_tool():
    cfg = os.path.join(get_package_share_directory('mod101_description'),
                       'urdf', 'mod101_config.xacro')
    m = re.search(r'<xacro:arg\s+name="tool"\s+default="([^"]+)"', open(cfg).read())
    return m.group(1) if m else 'jaws'


def _drop_unset(mappings):
    return {k: v for k, v in mappings.items() if v != ''}


def _build(context):
    params = _drop_unset(
        {k: LaunchConfiguration(k).perform(context) for k in BUILD_ARGS})

    pkg_harness = get_package_share_directory('mod101_harness')
    pkg_control = get_package_share_directory('mod101_control')

    urdf_file = os.path.join(pkg_harness, 'urdf', 'mod101_harness_arm.xacro')
    robot_description = xacro.process_file(
        urdf_file, mappings={**params, 'use_sim': 'false'}).toxml()

    controllers = [
        os.path.join(pkg_control, 'config', 'controllers.sim.yaml'),
        os.path.join(pkg_harness, 'config', 'pan_tilt_controllers.yaml'),
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, *controllers],
    )

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60'],
            output='screen',
        )

    spawn_controllers = LaunchConfiguration('spawn_controllers').perform(context)
    ctrl_spawners = []
    if spawn_controllers.lower() == 'true':
        ctrl_spawners = [spawner('joint_state_broadcaster'),
                         spawner('arm_controller'),
                         spawner('pan_tilt_controller')]

    tool = params.get('tool', _configured_tool())
    tool_launch_actions = []
    try:
        tool_launch_file = os.path.join(
            get_package_share_directory(f'mod101_tool_{tool}'),
            'launch', 'tool.launch.py')
        tool_launch_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tool_launch_file),
            launch_arguments={'spawn_controllers': spawn_controllers}.items()))
    except Exception:
        pass

    return [robot_state_publisher, controller_manager,
            *ctrl_spawners, *tool_launch_actions]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tool', default_value=_configured_tool()),
        DeclareLaunchArgument('shoulder_ext_length', default_value=''),
        DeclareLaunchArgument('elbow_ext_length', default_value=''),
        DeclareLaunchArgument('shoulder_mount', default_value=''),
        DeclareLaunchArgument('elbow_mount', default_value=''),
        DeclareLaunchArgument('spawn_controllers', default_value='true'),
        OpaqueFunction(function=_build),
    ])
