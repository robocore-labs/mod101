# MoveIt 2

`mod101_moveit_config` gives the arm motion planning, collision-aware IK and
trajectory execution, in Gazebo or against mock/real hardware.

This is the reference: how it works, what to change, and what to do when it
breaks. **New to it? [moveit-getting-started.md](moveit-getting-started.md) is
the walkthrough** — install to first executed motion, about fifteen minutes.

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-pick-ik

colcon build --symlink-install
source install/setup.bash

ros2 launch mod101_moveit_config demo.launch.py               # Gazebo + move_group + RViz
ros2 launch mod101_moveit_config demo.launch.py tool:=parallel
ros2 launch mod101_moveit_config mock.launch.py               # no Gazebo, no physics
```

Drag the interactive marker in RViz's MotionPlanning panel, **Plan**, then
**Execute**.

---

## The 5-DOF problem — read this first

mod101 has five actuated joints: base yaw (Z), then shoulder, elbow and
wrist-tilt, which are **all pitch joints about parallel ±Y axes**, then wrist
roll (X).

That reaches any position in the workspace, and it can pitch the tool up and
down. What it *cannot* do is choose the tool's azimuth independently — the
direction the tool points around vertical is whatever `joint_base` is set to.
A 6-DOF pose goal is therefore unsatisfiable except by coincidence. That is
geometry, not configuration: no solver setting fixes it.

MoveIt's default `KDLKinematicsPlugin` solves for full 6-DOF pose and needs at
least 6 joints. On this arm it fails or times out on essentially every goal,
which looks exactly like a broken install.

So `config/kinematics.yaml` uses **`pick_ik` with `rotation_scale: 0.0`** —
position-only IK. It solves for where the tool tip goes and accepts whatever
orientation results:

```yaml
arm:
  kinematics_solver: pick_ik/PickIkPlugin
  mode: global
  position_scale: 1.0
  rotation_scale: 0.0     # <- the accommodation
```

**Practical consequence: plan to positions, not poses.**

| In your code | Result |
|---|---|
| `set_position_target(x, y, z)` | works |
| joint-space goals | always work |
| `set_pose_target(pose)` | usually fails |

To *bias* the approach direction (e.g. prefer pointing down at a table), raise
`rotation_scale` to about **0.2–0.5** — a soft preference the solver trades off
against position. Do not set it to 1.0; that reproduces the KDL failure mode.
Grasps need their approach designed around the arm's fixed azimuth, not
specified as an arbitrary quaternion.

---

## The mental model

MoveIt knows nothing about your arm. It reads four descriptions, and everything
that goes wrong goes wrong in one of them.

| | Where | Answers |
|---|---|---|
| **Shape** | URDF | Where are the parts, and how far does each joint turn? |
| **Meaning** | SRDF | Which joints do I plan with, and what counts as a self-collision? |
| **Solver** | `kinematics.yaml`, `ompl_planning.yaml`, `joint_limits.yaml` | How do I turn a target into joint angles, and how fast may I move? |
| **Muscle** | `moveit_controllers.yaml` | Where do I send the finished plan? |

The chain runs one way — **configurator → URDF → SRDF → MoveIt**. Change
something near the front and everything behind it is stale until regenerated.
That is what [*When you change the build*](#when-you-change-the-build) is about.

## What's where

| File | What it is |
|---|---|
| `srdf/mod101.srdf.xacro` | Planning groups, named states, virtual joint. **Parametric** — takes `tool`, same as the URDF |
| `config/kinematics.yaml` | pick_ik, position-only (above) |
| `config/joint_limits.yaml` | Velocity/acceleration for time parameterisation. **No position limits** — those live in the URDF |
| `config/ompl_planning.yaml` | RRTConnect by default |
| `config/moveit_controllers.yaml` | Arm's FollowJointTrajectory handle. The tool contributes its own |
| `config/collisions/<tool>.srdf.xacro` | **Generated** self-collision matrices — see below |
| `test/moveit_smoke.py` | FK / IK / plan / execute checks; exits non-zero on failure |

Groups: **`arm`** (`base_link` → `wrist_flange`, 5 joints) and **`gripper`**
(the tool's joint `6`), with named states `home` / `ready` and `open` / `closed`.

The `arm` group deliberately terminates at `wrist_flange`, so it is identical
for every tool. Each tool contributes its own semantics from
`mod101_tool_<name>/srdf/tool.srdf.xacro`, the same way it contributes its URDF
and its ros2_control block — adding a tool stays a one-package job.

---

## When you change the build

The configurator writes four build args plus the active tool into
`mod101_description/urdf/mod101_config.xacro`. Everything downstream derives
from those five values.

**Save to xacro** writes those five values and nothing else — it is instant.
**Rebuild MoveIt config** is a separate button that regenerates the collision
matrices at 1,000,000 samples (~20 s for all four tools). The configurator marks
the rebuild as owed whenever the generated matrices no longer carry the same
build stamp as `mod101_config.xacro`.

> **A finished rebuild is not proof.** It reports `skipped: workspace not built`
> and carries on. Check `GET /collisions`.

> **If a robot embeds this arm, the configurator does not touch it.** mod101
> regenerates itself and nothing else — it deliberately doesn't know where its
> consumers live. Each consumer owns its own sync step, and the `/save` response
> carries a `downstream` note naming it. For base101:
>
> ```bash
> cd ~/robots/base101
> ./src/base101_arm/base101_arm_moveit_config/scripts/sync_arm_change.sh
> ```

### You changed a rail length — `shoulder_ext_length` / `elbow_ext_length`

The arm's reach changed, so *which link pairs can never touch* changed with it.
This is the case the generated matrices exist for.

| What | Status | Notes |
|---|---|---|
| URDF geometry | **rebuild** | xacro picks up the new args, but see the build-type note below |
| Collision matrices | **press Rebuild** | Save marks them stale; the Rebuild button regenerates all four tools at 1M samples |
| Payload / reach readout | **automatic** | Live in the configurator |
| Named poses `home` / `ready` | **check** | These are *joint angles*, not positions. Still legal, but a longer forearm means `ready` points somewhere else |
| Link masses | **re-run** | `python3 tools/estimate_masses.py` if you reprinted parts |

### You changed a servo bracket — `shoulder_mount` / `elbow_mount`

Picking an ST3120 flips that joint to the `big` mount: a heavier servo body
*and* a fixed downstream lever extension.

| What | Status | Notes |
|---|---|---|
| URDF geometry & inertia | **rebuild** | A different module file is included wholesale; ament_python means it needs a build |
| Collision matrices | **press Rebuild** | The lever extension shifts everything downstream |
| Joint *structure* | **unchanged** | `small` and `big` modules have identical link/joint graphs — only geometry differs |
| Velocity / acceleration caps | **edit** | `joint_limits.yaml` is hand-written and conservative. A bigger servo can take more; it will not raise itself |
| Travel limits | **recalibrate** | New bracket, new mechanical stops. See [calibration.md](calibration.md) |

### You changed the end effector — `tool`

Each tool is its own package with its own URDF fragment, SRDF fragment and
collision matrix. The SRDF picks the matching set with an `xacro:if`.

| What | Status | Notes |
|---|---|---|
| Tool geometry | **automatic** | Mount joint is always identity — see [tool-convention.md](tool-convention.md) |
| Gripper group & matrix | **automatic** | One matrix per tool already exists; the SRDF selects it |
| Planning group `arm` | **unchanged** | Terminates at `wrist_flange` |
| A brand-new tool package | **rebuild** | `colcon build` — a package that didn't exist can't be symlinked |
| Controllers | **check** | `tool:=none` has no actuated jaw, so no `gripper_trajectory_controller` is spawned |

### Why the matrices are generated

The URDF is parametric — rail lengths and the `small|big` mount swap move link
geometry around — so "which link pairs never collide" is a property of *your
build*, not of the design. A frozen matrix would either over-disable pairs on a
long-armed build (the planner drives the forearm through the base) or
under-disable on a short one (everything self-collides and nothing plans).

By hand:

```bash
python3 tools/gen_collision_matrix.py --trials 1000000
python3 tools/gen_collision_matrix.py --tool jaws --trials 1000000
```

Output has two parts:

- **Sampled pairs** — "never collides" decided by random search. Build-dependent.
- **Derived pairs** — emitted by `rigid_adjacency()` from the joint graph alone.
  Build-invariant. These exist because `collisions_updater` only marks *directly*
  connected links `Adjacent`: two links one hop apart across a **fixed** joint
  are never `Adjacent` and do collide when sampled, so they fall through both
  nets forever. That is the two halves of a hinge — and with the elbow's pair
  enabled, its range breaks into disconnected islands and OMPL cannot plan
  through it at all.

> **The trials trap.** "Never collides" is statistical, not a proof, and wrongly
> disabling a pair that *can* collide is the dangerous direction. At 10,000
> samples, measured on this build, `base_cover_1 ↔ wrist_camera_v1_1` was wrongly
> disabled for **all four tools** — the planner was free to drive the wrist
> camera through the base cover.
>
> The configurator's Rebuild button passes **1,000,000** and base101's generator
> defaults to it. But `gen_collision_matrix.py`'s own default is still **10,000**,
> so a hand-run without `--trials` produces the bad matrix. Always pass it.

> **`--symlink-install` does not cover the URDF.** Measured on a clean build:
> `ament_cmake` packages get their `share/` files symlinked back to `src/`, so
> edits are live — that is why `mod101_moveit_config`'s matrices and SRDF are.
> **`ament_python` packages get copies.** `mod101_description` and
> `base101_description` are both `ament_python`, so every `.xacro`, `.gazebo`
> and `.ros2control` edit needs `colcon build` before it reaches a launch. If a
> URDF change appears to do nothing, this is why.

## What never regenerates itself

Four things are hand-owned. Nothing in the pipeline touches them, and three of
the four have bitten.

| File | Holds | Watch for |
|---|---|---|
| `joint_limits.yaml` | Velocity and acceleration only | **Position limits are not here** — they live in the URDF |
| URDF `<limit>` tags | Real joint travel | A limit of exactly `0.0` is a trap — see below |
| `kinematics.yaml` | Solver choice, `rotation_scale` | Only touch it to constrain approach direction |
| SRDF group states | `home`, `ready` | Fixed joint angles. Still legal after a resize, but they no longer mean the same place |

---

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

---

## When planning fails

Each of these produces the same symptom — "planning failed" in RViz — and each
hides the next.

| Symptom | Cause | Fix |
|---|---|---|
| `NO_IK_SOLUTION` (−31) on every goal, even close ones | `pick_ik` isn't installed, or `kinematics.yaml` reverted to KDL | `sudo apt install ros-jazzy-pick-ik`; check `kinematics_solver: pick_ik/PickIkPlugin` |
| `pick_ik: Initial guess exceeds joint limits` | A joint is `type="continuous"` with no `lower`/`upper`, so there is no interval to sample | Make it `revolute` with real travel. Servo joints are never continuous |
| `CheckStartStateBounds failed … Start state out of bounds`, off by ~1e-13 | A limit sits at exactly `0.0` and the sim settles a picoradian below it | Give the stop ~1 mrad of margin (`lower="-0.001"`). `fix_start_state` does *not* help — it only normalizes continuous joints |
| `OMPL: Unable to solve the planning problem` after the full timeout, though start and goal are each valid | A self-collision splits a joint's range into disconnected islands | Regenerate the matrices; sweep the joint and check validity across its range |
| `CONTROL_FAILED` (−4) — plans fine, won't move | Trajectory controllers aren't up | `ros2 control list_controllers` — you want `arm_trajectory_controller` **active**, not `arm_controller` |
| Everything silent, `/joint_states` empty | Controller spawners lost a startup race and died | Check for `Configured and activated` on all of them. Faster boots make this likelier |

> **The one diagnostic worth knowing.** When a plan fails but the start and goal
> both look fine, sweep the suspect joint through its range and ask
> `/check_state_validity` at each step. A joint whose valid range comes back in
> disconnected bands cannot be planned through, no matter how good the endpoints
> are.

## Known rough edges

- **move_group segfaults on Ctrl-C.** In `~TrajectoryExecutionManager` during
  `rclcpp` teardown — an upstream MoveIt 2.12 / rclcpp shutdown bug. Harmless:
  it happens after everything has already stopped.
- **Collision meshes are the full-resolution visual STLs**, up to 7.3 MB
  (`servo_shoulder_big_1.stl`). Planning is fine at the current 167 disabled
  pairs (tool=jaws, of 276 total), but convex hulls would make collision
  checking materially cheaper — and would help Gazebo too.
- **Joint limits are partly measured.** `joint_wrist_tilt` (−1.23 … 1.54) and
  `joint_wrist_roll` (0 … π) are bounded `revolute`; the shoulder and elbow
  carry a −0.001 margin below their zero hard stop. The remaining estimates are
  the *dynamics* in `joint_limits.yaml`. See [calibration.md](calibration.md).
- **The tool joint is named `6`.** Legal, but it has to be quoted in every YAML
  and reads badly in SRDF. Renaming it to `joint_gripper` would touch all four
  tool packages.

---

Related: [performance-notes.md](performance-notes.md) for measurements and the
DDS traps · [ros-architecture.md](ros-architecture.md) for the package layout and
joint table · [worklogs/moveit.md](worklogs/moveit.md) for the original handover
note and the collision-matrix measurements that led here.
