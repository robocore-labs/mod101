# Getting started with MoveIt on mod101

From nothing to a planned, executed motion. About fifteen minutes, no hardware
required.

If you just want the reference — solver settings, the SRDF layout, controller
wiring — that's [moveit.md](moveit.md). This page is the walkthrough.

---

## 1. Install

You need ROS 2 Jazzy on Ubuntu 24.04 already working. Then:

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-pick-ik
```

`pick_ik` is not optional. mod101 is a 5-DOF arm and MoveIt's default IK solver
cannot handle it — [step 5](#5-the-one-thing-that-will-confuse-you) explains
why. Installing MoveIt without it gives you a stack that fails every goal.

## 2. Build

```bash
cd ~/robots/mod101
colcon build --packages-select \
  mod101_description mod101_control mod101_gazebo mod101_moveit_config \
  mod101_tool_parallel mod101_tool_pincopen mod101_tool_jaws mod101_tool_none
source install/setup.bash
```

## 3. First launch — no physics

Start with mock hardware. It's fast, it runs on a headless box, and if
something is wrong you find out in ten seconds instead of waiting for Gazebo.

```bash
ros2 launch mod101_moveit_config mock.launch.py
```

Wait for this line:

```
You can start planning now!
```

That's move_group telling you the robot model, the SRDF and the planning
pipeline all loaded. If you don't get it, jump to
[troubleshooting](#troubleshooting).

### Check it properly

In a second terminal:

```bash
source ~/robots/mod101/install/setup.bash
python3 $(ros2 pkg prefix --share mod101_moveit_config)/test/moveit_smoke.py
```

```
PASS  FK on ready state — wrist_flange at (0.2350, 0.0742, 0.3592)
PASS  IK to reachable pose (position-only, 5-DOF) — position error 0.21 mm
PASS  IK correctly fails on unreachable pose
PASS  OMPL joint-space plan home -> ready
PASS  MoveIt executes through to ros2_control
5/5 passed
```

Anything less than 5/5 means the config is broken, not your code. It exits
non-zero, so it works as a CI gate too.

## 4. Move the arm

Still with `mock.launch.py` running, in the second terminal:

```bash
EX=$(ros2 pkg prefix --share mod101_moveit_config)/examples
python3 $EX/move_to_position.py 0.20 0.05 0.30
```

```
Moving: wrist_flange -> (0.2, 0.05, 0.3) in world, +/-10 mm
OK — planned and executed
```

That's the whole loop: plan, execute, arm moves. Add `--plan-only` to see the
plan without committing to it, and `--speed 0.1` to slow it down.

More points to try — all of these are reachable at the default rail lengths
except the last, which is deliberately out of range:

```bash
python3 $EX/move_to_position.py 0.15 0.10 0.25
python3 $EX/move_to_position.py 0.235 0.074 0.359    # the SRDF 'ready' pose
python3 $EX/move_to_position.py 3.0 0.0 0.5          # deliberately too far
```

The last one fails, as it should.

## 5. The one thing that will confuse you

**mod101 plans to positions, not poses.**

Its three pitch joints are parallel, so the arm can put the tool tip anywhere in
its envelope and pitch it up or down — but it cannot aim the tool sideways
independently. That direction is whatever `joint_base` is set to. A full 6-DOF
pose goal is unsatisfiable except by luck.

| In your code | Result |
|---|---|
| `set_position_target(x, y, z)` | works |
| joint-space goals | always work |
| `set_pose_target(pose)` | usually fails |

That's why `move_to_position.py` takes `x y z` and no orientation. It is geometry,
not configuration — no solver setting fixes it, though `rotation_scale` can turn
orientation into a soft preference.

**Why, and how to bias the approach direction:**
[moveit.md § The 5-DOF problem](moveit.md#the-5-dof-problem--read-this-first).

## 6. Now with physics and a GUI

```bash
ros2 launch mod101_moveit_config demo.launch.py
```

Gazebo, move_group and RViz together. In RViz's **MotionPlanning** panel: drag
the blue interactive marker to a goal, hit **Plan**, then **Execute**. The same
scripts from step 4 work against this too.

Pick your end-effector with `tool:=`:

```bash
ros2 launch mod101_moveit_config demo.launch.py tool:=parallel
ros2 launch mod101_moveit_config demo.launch.py tool:=pincopen
ros2 launch mod101_moveit_config demo.launch.py tool:=none
```

The gripper is its own planning group. Named states `open` and `closed` are in
the MotionPlanning panel's group dropdown.

## 7. If you changed the arm's dimensions

Rail lengths and mount sizes move link geometry around, which changes which link
pairs can actually collide. The self-collision matrix is generated, not
hand-written, so **regenerate it after any build change**:

```bash
python3 tools/gen_collision_matrix.py --trials 1000000
```

Skip this and you get one of two bad outcomes: pairs disabled that shouldn't be
(the planner drives the forearm through the base), or pairs enabled that
shouldn't be (everything reports self-collision and nothing plans).

The configurator's Save does this for you — for mod101 only. If you have a robot
that embeds the arm, it runs its own sync step. Full rules, including what
*doesn't* regenerate itself:
[moveit.md § When you change the build](moveit.md#when-you-change-the-build).

## Troubleshooting

The two you are most likely to hit first are below. The full table — including
the failures that look identical but aren't — is in
[moveit.md § When planning fails](moveit.md#when-planning-fails).

**Every goal fails with `NO_IK_SOLUTION` (-31) even for close points.**
`pick_ik` probably isn't installed, or `kinematics.yaml` got reverted to KDL.
Check `kinematics_solver: pick_ik/PickIkPlugin`.

**`CONTROL_FAILED` (-4), plans fine but won't move.**
The trajectory controllers aren't up. Confirm with:

```bash
ros2 control list_controllers
```

You want `arm_trajectory_controller` **active** — not `arm_controller`. The two
claim the same joints and only one may be active at a time; the MoveIt launches
handle this by passing `spawn_controllers:=false` to the Gazebo launch.

**Controllers fail to load, or spawners report "already loaded" then "no
controller with this name exists".**
Another ROS system is on your DDS domain, and the spawners are talking to *its*
controller_manager. Check with `ros2 node list` — if you see nodes you didn't
start, set `ROS_DOMAIN_ID` to something unused, or shut the other one down.

**`LibraryLoadException: pinc_open_driver/PincOpenDriver does not exist`.**
Only bites outside `mock.launch.py` (which rewrites hardware plugins to
mock_components). For real `tool:=pincopen` hardware you need
[CNURobotics/pinc_open_driver](https://github.com/CNURobotics/pinc_open_driver)
cloned into `src/`.

**move_group segfaults when you Ctrl-C it.**
Known upstream MoveIt 2.12 / rclcpp teardown bug, in `~TrajectoryExecutionManager`.
It happens after everything has already stopped. Ignore it.

**`Detected jump back in time. Clearing TF buffer.` on repeat.**
Something is publishing `/clock` twice. Gazebo's clock must have exactly one
bridge — `gazebo.launch.py` runs a dedicated `clock_bridge` node, so `/clock`
must *not* also appear in `mod101_gazebo/config/gz_ros_bridge.yaml`. Two
publishers sample Gazebo at different moments, sim time appears to step
backwards, and MoveIt's TF lookups break. Check with
`ros2 topic info /clock --verbose` — you want one publisher.

**Trajectories are absurdly fast or slow.**
`config/joint_limits.yaml` holds conservative estimates, not measured servo
data — the URDF's `velocity="100"` is placeholder CAD output. Tune there, and
see [calibration.md](calibration.md) for getting real numbers off the arm.

## Where to go next

- [moveit.md](moveit.md) — the reference: SRDF layout, solver tuning, how tools
  contribute planning semantics
- [ros-architecture.md](ros-architecture.md) — packages, controllers, the tool
  contract
- Adding a tool? It needs `srdf/tool.srdf.xacro` and
  `config/moveit_controllers.yaml` alongside the URDF — see
  [ros-architecture.md](ros-architecture.md#adding-a-new-tool)
