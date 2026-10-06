"""Display the configured arm; explicit build arguments override saved defaults."""
from mod101_description.tool_config import tool_option_arguments
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BUILD_ARGS = (*tool_option_arguments(), 'wrist_camera', 'arm_dof', 'tool', 'shoulder_ext_length', 'elbow_ext_length',
              'shoulder_mount', 'elbow_mount')


def _build(context):
    share = get_package_share_directory('mod101_description')
    mappings = {key: LaunchConfiguration(key).perform(context) for key in BUILD_ARGS}
    description = xacro.process_file(os.path.join(share, 'urdf', 'mod101.xacro'),
                                     mappings={k: v for k, v in mappings.items() if v}).toxml()
    gui = LaunchConfiguration('gui')
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(gui)),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'config', 'display.rviz')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        *(DeclareLaunchArgument(key, default_value='') for key in BUILD_ARGS),
        OpaqueFunction(function=_build),
    ])
