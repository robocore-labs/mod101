# MoveIt 2

`mod101_moveit_config` gives the arm motion planning, collision-aware IK and
trajectory execution, in Gazebo or against mock/real hardware.

**New to this? Start with [moveit-getting-started.md](moveit-getting-started.md)** —
a step-by-step walkthrough to a first executed motion. This page is the reference.
Picking up the work? [moveit-handover.md](moveit-handover.md) has status and open
items; [performance-notes.md](performance-notes.md) has the measurements and the
DDS traps.

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-pick-ik

colcon build --packages-select mod101_moveit_config
source install/setup.bash

ros2 launch mod101_moveit_config demo.launch.py               # Gazebo + move_group + RViz
ros2 launch mod101_moveit_config demo.launch.py tool:=parallel
ros2 launch mod101_moveit_config mock.launch.py               # no Gazebo, no physics
```

Drag the interactive marker in RViz's MotionPlanning panel, **Plan**, then
**Execute**.

## The 5-DOF problem — read this first

mod101 has five actuated joints: base yaw (Z), then shoulder, elbow and
wrist-tilt, which are **all pitch joints about parallel ±Y axes**, then wrist
roll (X).

That reaches any position in the workspace, and it can pitch the tool up and
down. What it *cannot* do is choose the tool's azimuth independently — the
direction the tool points around vertical is whatever `joint_base` is set to.
A 6-DOF pose goal is therefore unsatisfiable except by coincidence.

MoveIt's default `KDLKinematicsPlugin` solves for full 6-DOF pose and needs at
least 6 joints. On this arm it fails or times out on essentially every goal,
which looks exactly like a broken install.

So `config/kinematics.yaml` uses **`pick_ik` with `rotation_scale: 0.0`** —
position-only IK. It solves for where the tool tip goes and accepts whatever
orientation results:

```yaml
arm:
  kinematics_solver: pick_ik/PickIkPlugin
  position_scale: 1.0
  rotation_scale: 0.0     # <- the accommodation
```

To *bias* the approach direction (e.g. prefer pointing down at a table), raise
`rotation_scale` to about **0.2–0.5** — a soft preference the solver trades off
against position. Do not set it to 1.0; that reproduces the KDL failure mode.

Practical consequence: **plan to positions, not poses.** `set_position_target`
works; `set_pose_target` will usually fail. Grasps need their approach designed
around the arm's fixed azimuth, not specified as an arbitrary quaternion.

## What's where

| File | What it is |
|---|---|
| `srdf/mod101.srdf.xacro` | Planning groups, named states, virtual joint. **Parametric** — takes `tool`, same as the URDF |
| `config/kinematics.yaml` | pick_ik, position-only (above) |
| `config/joint_limits.yaml` | Velocity/acceleration for time parameterisation. The URDF's `velocity="100"` is placeholder CAD data |
| `config/ompl_planning.yaml` | RRTConnect by default |
| `config/moveit_controllers.yaml` | Arm's FollowJointTrajectory handle. The tool contributes its own |
| `config/collisions/<tool>.srdf.xacro` | **Generated** self-collision matrices — see below |
| `test/moveit_smoke.py` | FK / IK / plan / execute checks; exits non-zero on failure |

Groups: **`arm`** (`base_link` → `wrist_flange`, 5 joints) and **`gripper`**
(the tool's joint `6`), with named states `home` / `ready` and `open` / `closed`.

### The SRDF is generated per build, on purpose

The URDF is parametric — rail lengths and the `small|big` mount swap move link
geometry around — so "which link pairs never collide" is a property of *your
build*, not of the design. A frozen matrix would either over-disable pairs on a
long-armed build (the planner drives the forearm through the base) or
under-disable on a short one (everything self-collides and nothing plans).

Regenerate after changing any build parameter:

```bash
python3 tools/gen_collision_matrix.py                 # all tools, current args
python3 tools/gen_collision_matrix.py --tool jaws
```

It reads the current build args straight out of `mod101.xacro` (so it picks up
whatever the configurator last saved) and rewrites
`config/collisions/<tool>.srdf.xacro`. Rebuild afterwards.

Each tool contributes its planning semantics from
`mod101_tool_<name>/srdf/tool.srdf.xacro`, the same way it contributes its URDF
and its ros2_control block — so adding a tool stays a one-package job.

## Controllers

MoveIt speaks `FollowJointTrajectory`; the plain sim's `arm_controller` and
`gripper_controller` are `JointGroupPositionController` and take
`Float64MultiArray`. Both sets claim the same joints, and `controller_manager`
allows only one active claim per joint.

So the MoveIt launches pass **`spawn_controllers:=false`** to
`gazebo.launch.py` (forwarded to the tool launch too) and bring up
`arm_trajectory_controller` + `gripper_trajectory_controller` instead. Nothing
is spawned and then switched away from — that ordering is racy and
`ros2 control switch_controllers` blocks when it loses the race.

| Bringup | Hardware | Controllers |
|---|---|---|
| `demo.launch.py` | Gazebo (`gz_ros2_control`) | JSB + arm/gripper trajectory controllers |
| `mock.launch.py` | `mock_components/GenericSystem` | same |
| `move_group.launch.py` | none — attaches to a running controller_manager | none |

`mock.launch.py` rewrites every `<ros2_control>` hardware plugin to
`mock_components/GenericSystem` before handing the description over. That
matters for `mod101_tool_pincopen`, which legitimately names the vendored
`pinc_open_driver/PincOpenDriver` for real hardware — without the rewrite, mock
bringup dies with a `LibraryLoadException` unless you've cloned that driver.

## Verifying an install

```bash
ros2 launch mod101_moveit_config mock.launch.py rviz:=false
# in another terminal:
python3 $(ros2 pkg prefix --share mod101_moveit_config)/test/moveit_smoke.py
```

Checks FK on `ready`, IK back to that pose (the 5-DOF question), that IK
*fails* on an unreachable pose, an OMPL joint-space plan, and execution all the
way through ros2_control. Current status: **5/5 on all four tools**, in both
Gazebo and mock.

## Known rough edges

- **move_group segfaults on Ctrl-C.** In `~TrajectoryExecutionManager` during
  `rclcpp` teardown — an upstream MoveIt 2.12 / rclcpp shutdown bug. Harmless:
  it happens after everything has already stopped.
- **Collision meshes are the full-resolution visual STLs**, up to 7.3 MB
  (`servo_shoulder_big_1.stl`). Planning is fine at the current 122 disabled
  pairs, but convex hulls would make collision checking materially cheaper —
  and would help Gazebo too.
- **Joint limits are estimates.** `joint_wrist_tilt` and `joint_wrist_roll` are
  still `continuous` in the URDF; once `calibration.yaml` exists they should
  become bounded `revolute` with measured limits. See
  [calibration.md](calibration.md).
- **The tool joint is named `6`.** Legal, but it has to be quoted in every YAML
  and reads badly in SRDF. Renaming it to `joint_gripper` would touch all four
  tool packages.
