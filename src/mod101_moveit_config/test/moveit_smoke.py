#!/usr/bin/env python3
"""Smoke-test the mod101 MoveIt config against a running move_group.

Run it against either bringup, in a second terminal:

    ros2 launch mod101_moveit_config mock.launch.py rviz:=false   # or demo.launch.py
    python3 $(ros2 pkg prefix --share mod101_moveit_config)/test/moveit_smoke.py

Exits non-zero if any check fails, so it works as a CI gate.

The point of interest is IK: mod101 is 5-DOF, so full-pose IK is impossible and
pick_ik is configured position-only (rotation_scale: 0.0). This checks that
claim rather than assuming it.

  1. FK on the SRDF 'ready' state          -> a definitely-reachable pose
  2. IK back to that pose                  -> pick_ik solves 5-DOF
  3. IK to a deliberately impossible pose  -> solver reports failure, not a lie
  4. Joint-space plan                      -> OMPL pipeline works end to end
"""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PositionIKRequest, RobotState)
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

ARM = ['joint_base', 'joint_shoulder', 'joint_elbow',
       'joint_wrist_tilt', 'joint_wrist_roll']
READY = [0.0, 1.2, 1.5, 0.3, 0.0]          # SRDF group_state 'ready'
TIP = 'wrist_flange'

results = []


def check(name, ok, detail=''):
    results.append((name, ok, detail))
    print(f'{"PASS" if ok else "FAIL"}  {name}' + (f'  — {detail}' if detail else ''),
          flush=True)


def robot_state(positions):
    rs = RobotState()
    rs.joint_state = JointState(name=list(ARM), position=list(positions))
    return rs


def main():
    rclpy.init()
    node = Node('mod101_moveit_smoke')

    fk = node.create_client(GetPositionFK, '/compute_fk')
    ik = node.create_client(GetPositionIK, '/compute_ik')
    plan = node.create_client(GetMotionPlan, '/plan_kinematic_path')
    for cli in (fk, ik, plan):
        if not cli.wait_for_service(timeout_sec=30.0):
            print(f'FAIL  service {cli.srv_name} never appeared')
            sys.exit(1)

    def call(cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
        return fut.result()

    # --- 1. FK on 'ready' ------------------------------------------------
    req = GetPositionFK.Request()
    req.header.frame_id = 'world'
    req.fk_link_names = [TIP]
    req.robot_state = robot_state(READY)
    res = call(fk, req)
    ok = res is not None and res.error_code.val == 1 and len(res.pose_stamped) == 1
    if not ok:
        check('FK on ready state', False, f'error_code={getattr(res, "error_code", None)}')
        sys.exit(1)
    pose = res.pose_stamped[0]
    p = pose.pose.position
    check('FK on ready state', True, f'{TIP} at ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})')

    # --- 2. IK back to that pose (the 5-DOF question) --------------------
    target = PoseStamped()
    target.header.frame_id = 'world'
    target.pose = pose.pose

    req = GetPositionIK.Request()
    req.ik_request = PositionIKRequest()
    req.ik_request.group_name = 'arm'
    req.ik_request.ik_link_name = TIP
    req.ik_request.pose_stamped = target
    req.ik_request.robot_state = robot_state([0.0] * 5)
    req.ik_request.timeout.sec = 2
    req.ik_request.avoid_collisions = True
    res = call(ik, req)
    ok = res is not None and res.error_code.val == 1
    detail = ''
    if ok:
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        # Verify by FK: position must land back on target, orientation may not.
        vreq = GetPositionFK.Request()
        vreq.header.frame_id = 'world'
        vreq.fk_link_names = [TIP]
        vreq.robot_state = robot_state([sol[j] for j in ARM])
        vres = call(fk, vreq)
        q = vres.pose_stamped[0].pose.position
        err = math.dist((p.x, p.y, p.z), (q.x, q.y, q.z))
        ok = err < 5e-3
        detail = (f'position error {err * 1000:.2f} mm; '
                  f'q=[{", ".join(f"{sol[j]:.3f}" for j in ARM)}]')
    else:
        detail = f'error_code={getattr(res, "error_code", None)}'
    check('IK to reachable pose (position-only, 5-DOF)', ok, detail)

    # --- 3. IK to an unreachable pose ------------------------------------
    far = PoseStamped()
    far.header.frame_id = 'world'
    far.pose.position.x = 5.0
    far.pose.orientation.w = 1.0
    req.ik_request.pose_stamped = far
    res = call(ik, req)
    ok = res is not None and res.error_code.val != 1
    check('IK correctly fails on unreachable pose', ok,
          f'error_code={getattr(res, "error_code", None)}')

    # --- 4. Joint-space plan ---------------------------------------------
    req = GetMotionPlan.Request()
    mpr = MotionPlanRequest()
    mpr.group_name = 'arm'
    mpr.num_planning_attempts = 5
    mpr.allowed_planning_time = 10.0
    mpr.max_velocity_scaling_factor = 0.5
    mpr.max_acceleration_scaling_factor = 0.5
    mpr.start_state = robot_state([0.0] * 5)
    c = Constraints()
    for j, v in zip(ARM, READY):
        c.joint_constraints.append(JointConstraint(
            joint_name=j, position=v,
            tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
    mpr.goal_constraints.append(c)
    req.motion_plan_request = mpr
    res = call(plan, req)
    ok = (res is not None
          and res.motion_plan_response.error_code.val == 1
          and len(res.motion_plan_response.trajectory.joint_trajectory.points) > 1)
    detail = ''
    if res is not None:
        traj = res.motion_plan_response.trajectory.joint_trajectory
        if traj.points:
            detail = (f'{len(traj.points)} points, '
                      f'{traj.points[-1].time_from_start.sec}'
                      f'.{traj.points[-1].time_from_start.nanosec // 10**8}s')
        else:
            detail = f'error_code={res.motion_plan_response.error_code.val}'
    check('OMPL joint-space plan home -> ready', ok, detail)

    # --- 5. Execute it (MoveIt -> moveit_controllers.yaml -> ros2_control) --
    exec_cli = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
    if not exec_cli.wait_for_server(timeout_sec=20.0):
        check('MoveIt executes through to ros2_control', False,
              '/execute_trajectory action server never appeared')
    else:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.motion_plan_response.trajectory
        fut = exec_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            check('MoveIt executes through to ros2_control', False, 'goal rejected')
        else:
            rfut = handle.get_result_async()
            rclpy.spin_until_future_complete(node, rfut, timeout_sec=30.0)
            result = rfut.result()
            code = result.result.error_code.val if result else None
            check('MoveIt executes through to ros2_control', code == 1,
                  f'error_code={code}')

    print('\n' + '=' * 60)
    npass = sum(1 for _, ok, _ in results if ok)
    print(f'{npass}/{len(results)} passed')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if npass == len(results) else 1)


if __name__ == '__main__':
    main()
