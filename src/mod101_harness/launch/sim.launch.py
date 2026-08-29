#!/usr/bin/env python3
"""The whole harness robot in Gazebo: mod101 arm + camera tower + actuated
pan/tilt head + RealSense depth camera + the arm's wrist camera, all bridged.

This is an overlay launch - it spins up gz_sim, spawns the robot, and brings up
controllers, reusing mod101's controller set verbatim (the arm is unprefixed)
plus the one pan/tilt controller this package adds. No MoveIt: point
mod101_moveit_config's move_group at this package's xacro if you want planning.

Build args (tool, shoulder_ext_length, elbow_ext_length, shoulder_mount,
elbow_mount) default to whatever the configurator last saved; pass one to
override it. spawn_controllers:=false leaves the controller_manager empty (for
a MoveIt overlay that brings up its own).

world:=<name|path> picks the world, defaulting to `table` - the harness on a
bench with graspable objects laid out in front of the arm (worlds/table.sdf).
world:=empty restores the bare ground plane.
"""

import os
import re

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

BUILD_ARGS = ('tool', 'shoulder_ext_length', 'elbow_ext_length',
              'shoulder_mount', 'elbow_mount')

# The bench world is the default because that is what this robot IS - an arm
# bolted to a harness on a table, with things on the table to pick up. `empty`
# is still one argument away for anyone who wants the arm in a void.
DEFAULT_WORLD = 'table'


def _resolve_world(name, pkg_harness, pkg_gazebo):
    """A path, or a bare name looked up in this package then mod101_gazebo."""
    if os.path.isabs(name) or os.sep in name:
        return name
    stem = name[:-4] if name.endswith('.sdf') else name
    for d in (os.path.join(pkg_harness, 'worlds'),
              os.path.join(pkg_gazebo, 'worlds')):
        cand = os.path.join(d, f'{stem}.sdf')
        if os.path.exists(cand):
            return cand
    raise RuntimeError(
        f'world "{name}" not found in {pkg_harness}/worlds or '
        f'{pkg_gazebo}/worlds. Pass a full path, or one of: '
        + ', '.join(sorted(
            os.path.splitext(f)[0]
            for d in (os.path.join(pkg_harness, 'worlds'),
                      os.path.join(pkg_gazebo, 'worlds'))
            if os.path.isdir(d) for f in os.listdir(d) if f.endswith('.sdf'))))


def _configured_tool():
    cfg = os.path.join(get_package_share_directory('mod101_description'),
                       'urdf', 'mod101_config.xacro')
    m = re.search(r'<xacro:arg\s+name="tool"\s+default="([^"]+)"', open(cfg).read())
    return m.group(1) if m else 'jaws'


def _drop_unset(mappings):
    """Unset args fall through to mod101_config.xacro's defaults."""
    return {k: v for k, v in mappings.items() if v != ''}


def _build(context):
    params = _drop_unset(
        {k: LaunchConfiguration(k).perform(context) for k in BUILD_ARGS})
    tool = params.get('tool', _configured_tool())

    pkg_harness    = get_package_share_directory('mod101_harness')
    pkg_gazebo     = get_package_share_directory('mod101_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Gazebo walks GZ_SIM_RESOURCE_PATH for a dir named <pkg> to resolve mesh
    # URIs: the arm, the harness, and the active tool.
    resource_dirs = [
        os.path.join(get_package_prefix('mod101_description'), 'share'),
        os.path.join(get_package_prefix('mod101_harness'), 'share'),
    ]
    try:
        resource_dirs.append(
            os.path.join(get_package_prefix(f'mod101_tool_{tool}'), 'share'))
    except PackageNotFoundError:
        pass
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        resource_dirs.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=os.pathsep.join(resource_dirs))

    world_file    = _resolve_world(
        LaunchConfiguration('world').perform(context), pkg_harness, pkg_gazebo)
    urdf_file     = os.path.join(pkg_harness, 'urdf', 'mod101_harness_arm.xacro')
    bridge_config = os.path.join(pkg_harness, 'config', 'gz_bridge.yaml')

    robot_description = xacro.process_file(
        urdf_file, mappings={**params, 'use_sim': 'true'}).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # camera_info + point cloud (image streams go through image_bridge below).
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='harness_sensor_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    depth_camera_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='harness_depth_camera_image_bridge',
        arguments=['/harness/depth_camera/image',
                   '/harness/depth_camera/depth_image'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

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
        arguments=['-topic', 'robot_description', '-name', 'mod101_harness_arm'],
        output='screen',
    )

    def spawner(name, delay):
        return TimerAction(period=delay, actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60'],
            output='screen',
        )])

    spawn_controllers = LaunchConfiguration('spawn_controllers').perform(context)
    spawn_jsb = spawner('joint_state_broadcaster', 3.0)
    position_spawners = ([spawner('arm_controller', 5.0),
                          spawner('pan_tilt_controller', 5.0)]
                         if spawn_controllers.lower() == 'true' else [])

    tool_launch_actions = []
    tool_pkg = f'mod101_tool_{tool}'
    try:
        tool_launch_file = os.path.join(
            get_package_share_directory(tool_pkg), 'launch', 'tool.launch.py')
        tool_launch_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tool_launch_file),
            launch_arguments={'spawn_controllers': spawn_controllers}.items()))
    except PackageNotFoundError:
        print(f'[mod101_harness] warning: tool package "{tool_pkg}" not found; '
              f'continuing without tool-side launch.')

    return [
        gz_resource_path,
        robot_state_publisher,
        gz_sim,
        clock_bridge,
        sensor_bridge,
        depth_camera_image_bridge,
        wrist_camera_image_bridge,
        spawn_robot,
        spawn_jsb,
        *position_spawners,
        *tool_launch_actions,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tool', default_value=_configured_tool()),
        DeclareLaunchArgument('shoulder_ext_length', default_value=''),
        DeclareLaunchArgument('elbow_ext_length', default_value=''),
        DeclareLaunchArgument('shoulder_mount', default_value=''),
        DeclareLaunchArgument('elbow_mount', default_value=''),
        DeclareLaunchArgument(
            'world', default_value=DEFAULT_WORLD,
            description='World name (looked up in mod101_harness/worlds then '
                        'mod101_gazebo/worlds) or a full path to an .sdf. '
                        '"table" is the bench with graspable objects; '
                        '"empty" is a bare ground plane.'),
        DeclareLaunchArgument(
            'spawn_controllers', default_value='true',
            description='Spawn joint_state_broadcaster + arm_controller + '
                        'pan_tilt_controller (+ the tool gripper). Set false '
                        'when a MoveIt overlay brings up its own.'),
        OpaqueFunction(function=_build),
    ])
