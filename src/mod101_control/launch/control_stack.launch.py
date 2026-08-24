#!/usr/bin/env python3
"""Spawn the mod101 controllers against an already-running controller_manager.

Used both by the gazebo launch (where the gz_ros2_control plugin provides the
controller_manager) and by future hardware bringup.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Position controllers only. The trajectory twins are declared in
    # controllers.sim.yaml but deliberately not loaded here: they claim the
    # same joints, and nothing in the plain stack drives FollowJointTrajectory.
    # The robocore agent loads one on demand at control-session entry
    # (load_controller + configure_controller + switch_controller) and drops
    # it again on exit — declaring them is what makes that possible.
    spawners = []
    for ctrl in (
        'joint_state_broadcaster',
        'arm_controller',
        'gripper_controller',
    ):
        spawners.append(ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', ctrl],
            output='screen',
        ))
    return LaunchDescription(spawners)
