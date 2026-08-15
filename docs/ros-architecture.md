# ROS 2 architecture

mod101 follows the standard ROS 2 description / control / gazebo layout, with a **modular tool layer**: every end-effector lives in its own `mod101_tool_<name>` package that ships everything it needs (URDF, ros2_control hardware block, gazebo extensions, controllers YAML, launch).

Target distro: **Jazzy** (Ubuntu 24.04, Python 3.12, Gazebo Harmonic).

## Packages

| Package | Build type | Purpose |
|---|---|---|
| `mod101_description` | `ament_cmake` | Arm URDF/xacro, meshes, RViz config, `display.launch.py`. The arm itself is the **`mod101_arm` macro** in `mod101_macro.xacro` (prefix-parameterized so multiple instances can coexist in one robot); `mod101.xacro` is the thin standalone wrapper; `mod101.ros2control` / `mod101.gazebo` hold the matching per-instance macros. Terminates at the `wrist_flange` link — the contract point for tools. |
| `mod101_control` | `ament_cmake` | Arm controller YAML (`controllers.sim.yaml`), real-hardware overlay (`mod101.hardware.xacro` — placeholder plugin) |
| `mod101_gazebo` | `ament_cmake` | World (`empty.sdf`, bullet-featherstone physics), bridge config, `gazebo.launch.py` (accepts `tool:=<name>`) |
| `mod101_tool_parallel` | `ament_cmake` | Parallel-jaw gripper |
| `mod101_tool_pincopen` | `ament_cmake` | PincOpen pincer gripper (vendored from [CNURobotics/pinc_open_driver](https://github.com/CNURobotics/pinc_open_driver)) |
| `mod101_tool_jaws` | `ament_cmake` | SO-101 single-jaw gripper (URDF + meshes adapted from the SO-101 lineage) |
| `mod101_tool_none` | `ament_cmake` | Empty end-cap. No actuators, no controllers — useful as a baseline / template |
| `mod101_moveit_config` | `ament_cmake` | MoveIt 2: parametric SRDF (`srdf/mod101.srdf.xacro`), 5-DOF-aware IK, generated collision matrices, `demo`/`mock`/`move_group` launches. See [moveit.md](moveit.md) |

## The `mod101_arm` macro

Everything that *is* the arm — links, joints, ros2_control hardware block,
gazebo extensions, the selected tool — is emitted by one macro:

```xml
<xacro:include filename="$(find mod101_description)/urdf/mod101_macro.xacro"/>
<xacro:mod101_arm prefix="" parent="world" xyz="0 0 0" rpy="0 0 0"
                  tool="jaws" use_sim="true"/>
```

`prefix` is prepended to every link/joint/system name and the wrist-camera
topic, so multiple instances coexist cleanly (`left_arm_1…6`, etc.).
`parent` adds a fixed `<prefix>base_mount` joint to the given link (empty =
no mount joint). `use_sim` gates the gz_ros2_control system blocks + gazebo
tags per instance.

Two things deliberately stay **outside** the macro, in the standalone
wrapper `mod101.xacro`: the `world` anchor link and the gz_ros2_control
**plugin** declaration (one per robot — an integrator with its own plugin
block, like base101, must not inherit a second one). `mod101.xacro` keeps
the original `use_sim` / `tool` args, so single-arm bringup is unchanged.

## Tool layer

The macro's `tool` param (arg `tool` on the standalone wrapper, default
`jaws`) selects which `mod101_tool_<tool>` macro to invoke; the tool's links
are fixed-jointed to `<prefix>wrist_flange`. The Gazebo plugin block in
`mod101.xacro` loads `mod101_control`'s arm YAML **plus** the active tool's
YAML (multiple `<parameters>` files merge into the single
`controller_manager` namespace); the launch file `IncludeLaunchDescription`s
the tool's `launch/tool.launch.py` to spawn whatever controllers / bridges /
drivers the tool needs.

### Adding a new tool

Create `mod101_tool_<name>/` with:

```
urdf/tool.urdf.xacro        # macro mod101_tool_<name>(prefix, use_sim): links + fixed joint to ${prefix}wrist_flange
urdf/tool.ros2control       # optional: macro mod101_tool_<name>_ros2control(prefix, use_sim)
urdf/tool.gazebo            # optional: macro mod101_tool_<name>_gazebo(prefix)
srdf/tool.srdf.xacro        # optional: macro mod101_tool_<name>_srdf(prefix) — MoveIt group + end effector
config/controllers.yaml     # optional
config/moveit_controllers.yaml  # optional: this tool's FollowJointTrajectory entry for MoveIt
launch/tool.launch.py       # spawners, image bridges, drivers, ...
```

`launch/tool.launch.py` must declare a `spawn_controllers` argument (even if it
ignores it, as `mod101_tool_none` does) — `mod101_gazebo` passes it to every
tool launch, and an include handed an argument it doesn't declare is an error.

Every link/joint name and parent/child reference inside the macros takes a
`${prefix}` (copy an existing tool — `jaws` is the smallest complete
example). Then add one include + one `<xacro:if>` invocation branch in
`mod101_macro.xacro`, one `<parameters>` branch in `mod101.xacro`'s
plugin block (if the tool ships controllers), and — if it ships SRDF semantics —
one include + branch in `mod101_moveit_config/srdf/mod101.srdf.xacro`, then run
`python3 tools/gen_collision_matrix.py --tool <name>`. The configurator
auto-discovers the package on disk and lists it in the dropdown.

## Joints

Arm, base outward — 5 DOF, all `<prefix>`-prefixed:

| Joint | Type | Axis | Limits |
|---|---|---|---|
| `joint_base` | revolute | +Z (yaw) | ±1.5708 |
| `joint_shoulder` | revolute | +Y (pitch) | 0 … 3.1416 |
| `joint_elbow` | revolute | −Y (pitch) | 0 … 3.1416 |
| `joint_wrist_tilt` | continuous | −Y (pitch) | — |
| `joint_wrist_roll` | continuous | +X (roll) | — |

One yaw, three *parallel* pitches, one roll. That spans position plus tool
pitch, but the tool's azimuth is rigidly tied to `joint_base` — the arm cannot
reach an arbitrary 6-DOF pose, which is why MoveIt uses a position-only IK
solver (see [moveit.md](moveit.md)).

`effort="100" velocity="100"` on every joint is placeholder data from the CAD
export, not servo spec — real dynamics limits live in
`mod101_moveit_config/config/joint_limits.yaml`.

- Tool joints live in the tool package, and the drive joint is always named `<prefix>6`. For `mod101_tool_parallel`: `<prefix>6` (prismatic, drives `left_jaw`); `<prefix>right_jaw_slider` follows via URDF `<mimic>` (bullet-featherstone honors it; dartsim doesn't).

## Build

```bash
cd ~/Work/mod101
colcon build --packages-select \
  mod101_description mod101_control mod101_gazebo mod101_moveit_config \
  mod101_tool_parallel mod101_tool_pincopen mod101_tool_jaws mod101_tool_none
source install/setup.bash
```

## Run

RViz only (no physics, joint sliders):

```bash
ros2 launch mod101_description display.launch.py
```

Gazebo sim with ros2_control:

```bash
ros2 launch mod101_gazebo gazebo.launch.py                 # tool:=jaws (default)
ros2 launch mod101_gazebo gazebo.launch.py tool:=parallel
ros2 launch mod101_gazebo gazebo.launch.py tool:=pincopen
ros2 launch mod101_gazebo gazebo.launch.py tool:=none
```

With MoveIt (planning + execution; walkthrough in [moveit-getting-started.md](moveit-getting-started.md), reference in [moveit.md](moveit.md)):

```bash
ros2 launch mod101_moveit_config demo.launch.py               # Gazebo + move_group + RViz
ros2 launch mod101_moveit_config mock.launch.py               # no Gazebo, mock hardware
```

## Controllers

Controllers active after launch (with `tool:=parallel`):

| Controller | Type | Joints | Owner | State at boot |
|---|---|---|---|---|
| `joint_state_broadcaster` | broadcaster | — | `mod101_gazebo` | active |
| `arm_controller` | `position_controllers/JointGroupPositionController` | arm 5 | `mod101_gazebo` | **active** (default) |
| `gripper_controller` | `position_controllers/JointGroupPositionController` | 6 | `mod101_tool_parallel` | active |
| `arm_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` (JTC) | arm 5 | YAML in `mod101_control` | defined, **not spawned** here — MoveIt spawns it, see note below |
| `gripper_trajectory_controller` | JTC | 6 | `mod101_tool_<tool>` | defined, **not spawned** here — MoveIt spawns it |

Send a position command:

```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.5, -0.5, 0.0, 0.0]}"
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.013]}"
```

### JTC is not spawned by the *plain* gazebo launch

Not because it's broken — because `arm_trajectory_controller` and
`arm_controller` claim the same five joints, and `controller_manager` allows
only one active claim per joint. The plain sim gives you the simple
Float64MultiArray interface; MoveIt needs the trajectory one.

Don't spawn-then-switch. Pass `spawn_controllers:=false` to
`gazebo.launch.py` (it also forwards to the active tool's launch, suppressing
`gripper_controller`) and spawn the trajectory controllers instead — which is
exactly what `mod101_moveit_config/launch/demo.launch.py` does:

```bash
ros2 launch mod101_moveit_config demo.launch.py
```

**Historical note:** JTC used to segfault in `on_init()` under `gz_ros2_control`
([ros2_control issue #2400](https://github.com/ros-controls/ros2_control/issues/2400)).
That is **fixed** as of `ros2_control` 4.45.2 / `gz_ros2_control` 1.2.19 —
JTC loads, activates, and executes `FollowJointTrajectory` goals in Gazebo.
Verified by `mod101_moveit_config/test/moveit_smoke.py`.

## Wiring real hardware

Override `mod101.xacro`'s `use_sim` arg to `false` and switch `mod101.hardware.xacro`'s placeholder `mock_components/GenericSystem` plugin for the real servo bus driver.

Calibrate the physical arm first — the configurator's Calibrate tab sweeps each
joint and generates `mod101_control/config/calibration.yaml` (joint limits in
radians, per-servo tick ranges) plus a LeRobot calibration JSON. See
[calibration.md](calibration.md).

## Gotchas

- `/clock` must have exactly **one** publisher. `gazebo.launch.py` runs a dedicated `clock_bridge`; do not also list `/clock` in `config/gz_ros_bridge.yaml`. Two bridges sample Gazebo independently, sim time appears to run backwards, and every sim-time subscriber spams `Detected jump back in time. Clearing TF buffer.` (this broke MoveIt's TF lookups).
- `update_rate` is **int** (`100`, not `100.0`) — Jazzy crashes on float.
- `state_publish_rate`, `action_monitor_rate` are **double** (`50.0`, `20.0`).
- `controller_manager.catch_exceptions: true` keeps gazebo alive when a controller's `on_init()` throws — real error is in the line just above the C++ stack trace.
- Physics engine is **bullet-featherstone** (set in `worlds/empty.sdf`) so URDF `<mimic>` works (used by `mod101_tool_parallel` for `right_jaw_slider`, and by `mod101_tool_pincopen` for all four 4-bar joints). Switching back to dartsim drops mimicked joints.
- Each `mod101_tool_*` package needs at least one node in its `launch/tool.launch.py` for `IncludeLaunchDescription` to be happy — `mod101_tool_none` ships an empty `LaunchDescription([])` and that's fine.
- `GZ_SIM_RESOURCE_PATH` must include each tool's install prefix so Gazebo can resolve `package://mod101_tool_<name>/meshes/...`. The main `gazebo.launch.py` builds this list automatically from the active tool.
