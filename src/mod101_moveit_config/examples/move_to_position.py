#!/usr/bin/env python3
"""Move the tool tip to an (x, y, z) point. Your first mod101 MoveIt script.

    python3 move_to_position.py 0.235 0.074 0.359
    python3 move_to_position.py 0.20 0.00 0.30 --tip jaws_moving

Note what this does NOT take: an orientation. mod101 is a 5-DOF arm — the tool's
azimuth is rigidly tied to joint_base, so a full 6-DOF pose goal is
unsatisfiable except by coincidence. Ask for a POSITION and let the arm pick the
orientation it can actually reach. See docs/moveit.md.

The goal is expressed as a PositionConstraint over a small sphere rather than a
pose target, which is the honest way to say "put the tip here, I don't care how".

Talks to move_group over its /move_action interface, so it needs no MoveIt
Python bindings — just rclpy and moveit_msgs. Run it alongside either bringup:

    ros2 launch mod101_moveit_config demo.launch.py     # or mock.launch.py
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, MotionPlanRequest,
                             PlanningOptions, PositionConstraint)
from shape_msgs.msg import SolidPrimitive

DEFAULT_TIP = 'wrist_flange'   # the tool mount contract point (tool-convention.md)

# moveit_msgs/MoveItErrorCodes, for messages a human can act on.
ERRORS = {
    1: 'SUCCESS', -1: 'PLANNING_FAILED', -2: 'INVALID_MOTION_PLAN',
    -3: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE', -4: 'CONTROL_FAILED',
    -5: 'UNABLE_TO_ACQUIRE_SENSOR_DATA', -6: 'TIMED_OUT', -7: 'PREEMPTED',
    -10: 'START_STATE_IN_COLLISION', -12: 'GOAL_IN_COLLISION',
    -31: 'NO_IK_SOLUTION', 99999: 'UNDEFINED',
}
HINTS = {
    -1: '  — out of reach, or blocked. Try a closer point.',
    -31: '  — out of reach.',
    -4: '  — planned fine but the controller refused it. Are the trajectory '
        'controllers up? Wait for "You can start planning now!".',
    99999: '  — usually an unreachable goal that failed before planning began.',
    -10: '  — the arm is already in self-collision; move it out first.',
}


def position_goal(x, y, z, tip, frame, tolerance):
    """A 'put the tip in this sphere' constraint — position only, no orientation."""
    region = BoundingVolume()
    region.primitives.append(SolidPrimitive(
        type=SolidPrimitive.SPHERE, dimensions=[tolerance]))
    centre = Pose()
    centre.position.x, centre.position.y, centre.position.z = x, y, z
    centre.orientation.w = 1.0
    region.primitive_poses.append(centre)

    pc = PositionConstraint()
    pc.header.frame_id = frame
    pc.link_name = tip
    pc.constraint_region = region
    pc.weight = 1.0
    # No offset: constrain the link origin itself.
    pc.target_point_offset.x = 0.0

    goal = Constraints()
    goal.position_constraints.append(pc)
    return goal


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    ap.add_argument('z', type=float)
    ap.add_argument('--tip', default=DEFAULT_TIP,
                    help=f'link to place at the target (default {DEFAULT_TIP})')
    ap.add_argument('--frame', default='world')
    ap.add_argument('--group', default='arm')
    ap.add_argument('--tolerance', type=float, default=0.01,
                    help='radius of the acceptable sphere, m (default 0.01)')
    ap.add_argument('--speed', type=float, default=0.3,
                    help='velocity/acceleration scaling, 0-1 (default 0.3)')
    ap.add_argument('--plan-only', action='store_true',
                    help='plan and report, but do not move the arm')
    opts = ap.parse_args()

    rclpy.init()
    node = Node('mod101_move_to_position')
    client = ActionClient(node, MoveGroup, '/move_action')

    if not client.wait_for_server(timeout_sec=30.0):
        print('move_group is not running — start demo.launch.py or mock.launch.py')
        sys.exit(1)

    req = MotionPlanRequest()
    req.group_name = opts.group
    req.num_planning_attempts = 10
    req.allowed_planning_time = 10.0
    req.max_velocity_scaling_factor = opts.speed
    req.max_acceleration_scaling_factor = opts.speed
    req.goal_constraints.append(position_goal(
        opts.x, opts.y, opts.z, opts.tip, opts.frame, opts.tolerance))

    goal = MoveGroup.Goal()
    goal.request = req
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = opts.plan_only

    action = 'Planning' if opts.plan_only else 'Moving'
    print(f'{action}: {opts.tip} -> ({opts.x}, {opts.y}, {opts.z}) '
          f'in {opts.frame}, +/-{opts.tolerance * 1000:.0f} mm')

    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print('goal rejected by move_group')
        sys.exit(1)

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=60.0)
    result = result_future.result()

    if result is None:
        print('no result from move_group (timed out)')
        sys.exit(1)

    code = result.result.error_code.val
    if code == 1:
        # planning_time comes back 0 on the execute path — only report it when
        # move_group actually filled it in.
        secs = result.result.planning_time
        timing = f' in {secs:.3f}s' if secs > 0 else ''
        print(f'OK — {"planned" if opts.plan_only else "planned and executed"}'
              + timing)
    else:
        print(f'FAILED — {ERRORS.get(code, "?")} ({code}){HINTS.get(code, "")}')

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if code == 1 else 1)


if __name__ == '__main__':
    main()
