#!/usr/bin/env python3
"""Bring up mod101 in Gazebo Sim with ros2_control. The tool layer is
decoupled: this launch only knows how to bring up the arm + sim plumbing,
then defers to `mod101_tool_<tool>/launch/tool.launch.py` to spawn any
tool-specific controllers / nodes (image bridges, suction-pump drivers, etc.).

Launch arg:
    tool   default "parallel"   matches a package named mod101_tool_<tool>
"""

import os
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
    PackageNotFoundError,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _build(context):
    tool = LaunchConfiguration('tool').perform(context)

    pkg_description = get_package_share_directory('mod101_description')
    pkg_gazebo      = get_package_share_directory('mod101_gazebo')
    pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')

    # Gazebo resolves `package://<pkg>/...` mesh URIs by walking
    # GZ_SIM_RESOURCE_PATH for a directory named <pkg>. Under colcon's
    # default isolated-install layout each package has its own prefix
    # (install/<pkg>/share/<pkg>/...), so we need to add every package
    # whose meshes are referenced — the arm, plus every installed tool.
    resource_dirs = [os.path.join(get_package_prefix('mod101_description'), 'share')]
    for pkg in (f'mod101_tool_{t}' for t in (tool,)):
        try:
            resource_dirs.append(
                os.path.join(get_package_prefix(pkg), 'share')
            )
        except PackageNotFoundError:
            pass
    # Preserve any pre-existing GZ_SIM_RESOURCE_PATH (e.g. fuel cache mounts).
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        resource_dirs.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(resource_dirs),
    )

    world_file    = os.path.join(pkg_gazebo, 'worlds', 'empty.sdf')
    urdf_file     = os.path.join(pkg_description, 'urdf', 'mod101.xacro')
    bridge_config = os.path.join(pkg_gazebo, 'config', 'gz_ros_bridge.yaml')

    # Expand the URDF with the chosen tool so the tool's links/joints/blocks
    # come along inside robot_description.
    robot_description = xacro.process_file(
        urdf_file, mappings={'tool': tool}
    ).toxml()

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

    # Wrist camera lives on the arm (between elbow and wrist flange), not in
    # a tool — bridge it here.
    wrist_camera_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='wrist_camera_image_bridge',
        arguments=['/wrist_camera/image_raw'],
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

    # Arm spawners only. JTC stays out of the launch (segfaults on Jazzy —
    # see ros2_control issue #2400). The tool brings up its own controllers
    # via its own launch fragment below.
    spawn_jsb = spawner('joint_state_broadcaster', delay=3.0)
    spawn_arm = spawner('arm_controller', delay=5.0)

    # Defer to the active tool's launch — spawns gripper/vacuum controllers,
    # tool-specific bridges, anything else the tool needs at runtime.
    tool_launch_actions = []
    tool_pkg = f'mod101_tool_{tool}'
    try:
        tool_launch_file = os.path.join(
            get_package_share_directory(tool_pkg), 'launch', 'tool.launch.py'
        )
        tool_launch_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tool_launch_file)
        ))
    except PackageNotFoundError:
        print(f'[mod101_gazebo] warning: tool package "{tool_pkg}" not found; '
              f'continuing without tool-side launch.')

    return [
        gz_resource_path,
        robot_state_publisher,
        gz_sim,
        clock_bridge,
        bridge,
        wrist_camera_image_bridge,
        spawn_robot,
        spawn_jsb,
        spawn_arm,
        *tool_launch_actions,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tool',
            default_value='parallel',
            description='End-effector package suffix (matches mod101_tool_<tool>).',
        ),
        OpaqueFunction(function=_build),
    ])
