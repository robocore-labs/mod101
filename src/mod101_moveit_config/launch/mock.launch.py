#!/usr/bin/env python3
"""MoveIt against mock hardware — no Gazebo, no physics, no meshes to render.

Fastest way to check that the SRDF, the IK solver and the planning pipeline are
sane, and the one bringup that runs on a headless box:

    ros2 launch mod101_moveit_config mock.launch.py rviz:=false

Uses mod101_control/urdf/mod101.hardware.xacro (use_sim:=false), whose
mock_components/GenericSystem echoes commands straight back as state. Swap that
plugin for the real servo bus driver and this same launch drives the physical
arm — that's the point of the overlay.
"""

import os
import re
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

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


def _build(context):
    tool = LaunchConfiguration('tool').perform(context)
    params = _drop_unset(
        {k: LaunchConfiguration(k).perform(context) for k in BUILD_ARGS})

    hardware_xacro = os.path.join(
        get_package_share_directory('mod101_control'),
        'urdf', 'mod101.hardware.xacro')
    robot_description = _force_mock_hardware(xacro.process_file(
        hardware_xacro,
        mappings={**params, 'tool': tool, 'use_sim': 'false'}).toxml())

    controllers = _controller_params(tool)

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False},
                    controllers],
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
    )

    # NOTE: these delays look like something that should be event-driven, and
    # an attempt to do that (spawners chained on OnProcessExit, starting
    # immediately) BROKE bringup — starting a spawner in the same instant as
    # ros2_control_node makes the controller manager never receive
    # /robot_description, and it hangs on "Waiting for data on
    # 'robot_description' topic". Reproduced 3/3 with a clean /dev/shm. See
    # docs/performance-notes.md before trying again.
    def spawner(name, delay):
        return TimerAction(period=delay, actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60'],
            output='screen')])

    spawners = [spawner('joint_state_broadcaster', 3.0),
                spawner('arm_trajectory_controller', 5.0)]
    if tool != 'none':
        spawners.append(spawner('gripper_trajectory_controller', 7.0))

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mod101_moveit_config'),
            'launch', 'move_group.launch.py')),
        launch_arguments={**params, 'tool': tool,
                          'use_sim_time': 'false',
                          'rviz': LaunchConfiguration('rviz')}.items(),
    )

    return [ros2_control_node, rsp, *spawners,
            TimerAction(period=9.0, actions=[move_group])]


def _force_mock_hardware(urdf):
    """Point every <ros2_control> block at mock_components/GenericSystem.

    use_sim:=false means "real hardware", and a tool is entitled to name a real
    driver there — mod101_tool_pincopen names pinc_open_driver/PincOpenDriver,
    which only exists if you've cloned CNURobotics/pinc_open_driver into the
    workspace. That's right for a real bringup and wrong for this launch, whose
    whole purpose is to exercise MoveIt with no hardware and no extra
    dependencies. Rewriting here keeps the tool packages honest about what they
    actually drive, instead of adding a third mode to every tool's contract.

    Driver <param>s (serial_port, baud_rate, ...) are dropped with the plugin —
    they mean nothing to the mock and would just be noise in the description.
    """
    import xml.etree.ElementTree as ET

    root = ET.fromstring(urdf)
    swapped = []
    for hw in root.iter('hardware'):
        plugin = hw.find('plugin')
        if plugin is None or plugin.text == 'mock_components/GenericSystem':
            continue
        swapped.append(plugin.text)
        plugin.text = 'mock_components/GenericSystem'
        for param in hw.findall('param'):
            hw.remove(param)

    if swapped:
        print(f'[mod101_moveit_config] mock bringup: replaced hardware '
              f'plugin(s) {swapped} with mock_components/GenericSystem')
    return ET.tostring(root, encoding='unicode')


def _controller_params(tool):
    """Merge the arm + tool controller YAMLs, with use_sim_time forced off.

    Written to one temp file rather than handed to Node(parameters=[...]) as a
    list: controllers.sim.yaml hardcodes `use_sim_time: true` for the Gazebo
    case, and with no /clock publisher the controller manager's update loop
    never cycles — every activation then dies with "Switch controller timed out
    after 5 seconds". Appending an override dict after the file is not reliable
    enough to undo that, so merge it here where the precedence is explicit.
    """
    paths = [os.path.join(get_package_share_directory('mod101_control'),
                          'config', 'controllers.sim.yaml')]
    if tool != 'none':
        paths.append(os.path.join(
            get_package_share_directory(f'mod101_tool_{tool}'),
            'config', 'controllers.yaml'))

    merged = {}
    for path in paths:
        with open(path) as f:
            for node, cfg in (yaml.safe_load(f) or {}).items():
                dst = merged.setdefault(node, {}).setdefault('ros__parameters', {})
                dst.update(cfg.get('ros__parameters', {}))

    merged['controller_manager']['ros__parameters']['use_sim_time'] = False

    # Stable path per tool, overwritten each run — a fresh NamedTemporaryFile
    # would leak one file per launch, and these have to outlive this function
    # (controller_manager reads the path later), so they can't be cleaned up
    # on exit either.
    path = os.path.join(tempfile.gettempdir(),
                        f'mod101_mock_controllers_{tool}.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(merged, f)
    return path


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
