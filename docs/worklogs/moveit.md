# MoveIt 2 — handover

> **ARCHIVED WORKLOG — 2026-08.** This was the handover note written when
> MoveIt support landed. It is kept for the reasoning and the measurements, not
> as current documentation: several items here have since been resolved, and the
> collision-matrix numbers predate `rigid_adjacency()`.
>
> **For current documentation see [../moveit.md](../moveit.md).**

State of the MoveIt work as of 2026-08-13. What exists, what's proven, what
isn't, and what I'd do next.

Nothing here is committed — it's all in the working tree.

Companion docs: [moveit-getting-started.md](../moveit-getting-started.md) (the
walkthrough), [moveit.md](../moveit.md) (the reference),
[performance-notes.md](../performance-notes.md) (why bringup feels slow, and the
DDS traps).

---

## What was added

**New package `mod101_moveit_config`**

```
srdf/mod101.srdf.xacro            parametric SRDF (takes `tool`, like the URDF)
config/kinematics.yaml            pick_ik, position-only  <- the important one
config/joint_limits.yaml          velocity/accel for time parameterisation
config/ompl_planning.yaml         RRTConnect default
config/moveit_controllers.yaml    arm's FollowJointTrajectory handle
config/collisions/<tool>.srdf.xacro   GENERATED self-collision matrices
config/moveit.rviz                MotionPlanning panel config
launch/demo.launch.py             Gazebo + move_group + RViz
launch/mock.launch.py             mock hardware, no Gazebo, headless-friendly
launch/move_group.launch.py       move_group alone (attaches to a running CM)
test/moveit_smoke.py              FK/IK/plan/execute checks, non-zero on failure
examples/move_to_position.py      "move the tip to x y z" reference script
```

**New `tools/gen_collision_matrix.py`** — regenerates the collision matrices.

**Each `mod101_tool_*` gained** `srdf/tool.srdf.xacro` (its planning group + end
effector) and `config/moveit_controllers.yaml`, plus a
`gripper_trajectory_controller` in its `controllers.yaml`. The one-package-per-
tool contract is preserved; `docs/ros-architecture.md` documents the new files.

**Modified outside the new package**

| File | Change |
|---|---|
| `mod101_gazebo/launch/gazebo.launch.py` | new `spawn_controllers` arg (default true) |
| `mod101_gazebo/config/gz_ros_bridge.yaml` | **removed duplicate `/clock` bridge** — see below |
| `mod101_control/urdf/mod101.hardware.xacro` | robot name `mod101_hardware` → `mod101` (MoveIt requires URDF and SRDF names to match) |
| `mod101_control/config/controllers.sim.yaml` | corrected the stale JTC-segfault comment |
| `mod101_tool_*/launch/tool.launch.py` | accept `spawn_controllers` |

## Verification status

`test/moveit_smoke.py` checks FK on `ready` → IK back to that pose → IK
correctly *failing* on an unreachable pose → an OMPL joint-space plan →
execution through ros2_control.

| Bringup | Tools | Result |
|---|---|---|
| `mock.launch.py` | jaws, parallel, pincopen, none | **5/5 each** |
| `demo.launch.py` (Gazebo) | jaws | **5/5** |

Also verified: all four SRDFs expand; `pick_ik` solves the 5-DOF chain to
0.07–0.21 mm; planning takes 0.07–0.26 s; Gazebo runs at **RTF 0.992**;
`examples/move_to_position.py` plans, executes, and fails correctly out of range.

**Not verified: anything visual.** I could never see RViz — WSLg composites
through Wayland, so an X root-window grab returns black, and there's no
`Xvfb`/`grim`/`xdotool` installed. `config/moveit.rviz` is correct as far as
RViz's logs go (plugins load, interactive-marker server connects, no parse
errors) but nobody has confirmed the panel *looks* right. In particular the
config carries no `QMainWindow State` blob, so panel docking falls back to
RViz's defaults; if it lands badly, arrange it and **File → Save Config As**
over `config/moveit.rviz`.

## The two decisions that matter

### 1. 5 DOF → position-only IK

Joints are base yaw (Z), then shoulder/elbow/wrist-tilt — **all pitch about
parallel ±Y** — then wrist roll (X). The tool's azimuth is rigidly tied to
`joint_base`, so arbitrary 6-DOF poses are unreachable.

MoveIt's default KDL solver needs ≥6 DOF and fails on essentially every goal,
which reads as a broken install. So `kinematics.yaml` uses **`pick_ik` with
`rotation_scale: 0.0`**. Consequence for all downstream code: **plan to
positions, not poses.** `ros-jazzy-pick-ik` is a hard dependency.

### 2. Controllers: don't spawn-then-switch

`arm_controller`/`gripper_controller` (position) and the trajectory controllers
claim the same joints, and controller_manager permits one active claim per
joint. The first implementation spawned the position controllers then switched
away; that races the spawner and `ros2 control switch_controllers` hangs when
it loses. Now the MoveIt launches pass `spawn_controllers:=false` and bring up
only the trajectory controllers. The plain Gazebo launch is unchanged by
default (re-verified).

## The collision-matrix question — measured

The matrices are generated, not hand-written, because rail lengths and the
`small|big` mount swap move geometry. I measured how much they actually change
across the configurator's full 80–280 mm range (tool=jaws):

These counts are a **historical measurement**, taken at 10,000 trials and before
`rigid_adjacency()` existed. Today the same default build reports 167 pairs at
1,000,000 trials. The *shape* of the result below still holds — that is what the
section is for — but do not compare the absolute numbers against a current file.

| Config | pairs | extra vs default | **missing vs default** |
|---|---|---|---|
| short + small (default) | 122 | — | — |
| long + small | 133 | 11 | **0** |
| short + **big** | 114 | 0 | **8** |
| long + **big** | 129 | 11 | **4** |

**Rail length is harmless** — lengthening only ever *adds* disabled pairs, so
the default matrix on a long arm is conservative: slightly slower, never wrong.
Moving the length sliders does **not** require regeneration.

**The `small` → `big` mount swap is dangerous.** It removes 8 pairs that the
larger servos genuinely bring into contact (`arm_extrusion_1`↔`servo_shoulder_1`,
`elbow_adapter_1`↔`elbow_link_1`, `camera_wing_1`↔`elbow_link_1`, …). Shipping
the small-mount matrix on a big-mount build silently blinds the planner to
those.

**Resolved — the alternative won.** The generator is wired into the
configurator's Save: `regen_collisions_async()` runs it in a background thread
with a 900 s timeout, and `GET /collisions` reports status. Shipping an
intersection was not needed.

Three caveats worth carrying forward:

- **It regenerates mod101 and nothing else.** An earlier version reached into a
  base101 workspace via a `BASE101_WS` setting. That inverted the dependency —
  the arm had to know where its consumers lived, which breaks with two of them,
  or one on another machine, or a consumer that isn't base101. Consumers now own
  their own regeneration; the Save response carries a `downstream` note naming
  the script to run. For base101 that is
  `base101_arm_moveit_config/scripts/sync_arm_change.sh`.
- **A green Save is not proof.** Regeneration reports
  `skipped: workspace not built` and carries on. Check `GET /collisions`.
- **The 8 "missing" pairs above were two different problems.** Some were genuine
  build-dependence, which regenerate-on-save fixes. But
  `elbow_adapter_1`↔`elbow_link_1` was never a sampling question at all: those
  are the two halves of the elbow hinge, one link apart across a *fixed* joint,
  so `collisions_updater` can never mark them `Adjacent` and they always collide
  when sampled. They fell through both nets on every build. `rigid_adjacency()`
  in the generator now emits that class of pair from the joint graph directly.

Nothing else is a precomputed artifact — URDF and SRDF are both xacro, expanded
at launch. And `colcon build --symlink-install` means config edits need no
rebuild.

## Open items

**Should do**

1. ~~**Collision matrix strategy**~~ — **resolved.** Regenerate-on-save won:
   `configurator/server.py` calls `regen_collisions_async()` on every Save. It
   regenerates **mod101 only** — consumers own their own matrices and the Save
   response carries a `downstream` note telling you to run theirs. The specific
   pair this section worried about, `elbow_adapter_1`↔`elbow_link_1`, is now
   emitted structurally by `rigid_adjacency()` rather than left to sampling.
2. **Verify RViz visually** — first thing worth doing on native Linux.
3. **Joint dynamics are still estimates.** `config/joint_limits.yaml` holds
   conservative guesses and the URDF's `velocity="100"` is placeholder CAD data.
   Joint *travel* is no longer an estimate: the wrists are bounded `revolute`
   (tilt −1.23 … 1.54, roll 0 … π) and the shoulder/elbow stops carry a −0.001
   margin.

**Nice to have**

4. **Collision meshes are the full-res visual STLs** — 16 meshes, 111,766
   triangles, same geometry for visual and collision. Planning is fine, but
   convex hulls would cut collision-checking cost and help Gazebo.
5. **The tool joint is named `6`.** Legal, but needs quoting in every YAML and
   reads badly in SRDF. `joint_gripper` would touch all four tool packages.
6. **Bringup takes ~16 s of fixed timers.** See
   [performance-notes.md](../performance-notes.md) — there's a failed experiment
   documented there; read it before retrying.

**Known upstream, not ours**

- `move_group` segfaults in `~TrajectoryExecutionManager` on Ctrl-C (MoveIt
  2.12 / rclcpp teardown). Happens after everything has stopped.
- `/recognize_objects not available` on startup — stock MoveIt, harmless.

## Gotcha worth knowing before debugging anything

Intermittent "controller failed to load", "Waiting for data on
`robot_description`", or `CONTROL_FAILED` are usually **environment, not code** —
stale FastDDS shared-memory segments, or another ROS system on your DDS domain.
Both cost me hours. See [performance-notes.md](../performance-notes.md#dds-traps).
