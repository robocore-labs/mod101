# ROS 2 architecture

mod101 follows the standard ROS 2 description / control / gazebo layout, with a **modular tool layer**: every end-effector lives in its own `mod101_tool_<name>` package that ships everything it needs (URDF, ros2_control hardware block, gazebo extensions, controllers YAML, launch).

Target distro: **Jazzy** (Ubuntu 24.04, Python 3.12, Gazebo Harmonic).

## Packages

| Package | Build type | Purpose |
|---|---|---|
| `mod101_description` | `ament_cmake` | Arm URDF/xacro (`mod101.xacro`, `materials.xacro`, `mod101.ros2control`, `mod101.gazebo`), meshes, RViz config, `display.launch.py`. Terminates at the `wrist_flange` link — the contract point for tools. |
| `mod101_control` | `ament_cmake` | Arm controller YAML (`controllers.sim.yaml`), real-hardware overlay (`mod101.hardware.xacro` — placeholder plugin) |
| `mod101_gazebo` | `ament_cmake` | World (`empty.sdf`, bullet-featherstone physics), bridge config, `gazebo.launch.py` (accepts `tool:=<name>`) |
| `mod101_tool_parallel` | `ament_cmake` | Parallel-jaw gripper |
| `mod101_tool_pincopen` | `ament_cmake` | PincOpen pincer gripper (vendored from [CNURobotics/pinc_open_driver](https://github.com/CNURobotics/pinc_open_driver)) |
| `mod101_tool_jaws` | `ament_cmake` | SO-101 single-jaw gripper (URDF + meshes adapted from the SO-101 lineage) |
| `mod101_tool_none` | `ament_cmake` | Empty end-cap. No actuators, no controllers — useful as a baseline / template |

## Tool layer

The arm xacro takes a `tool` arg (default `parallel`). At expansion time it `<xacro:include>`s `mod101_tool_<tool>/urdf/tool.urdf.xacro`, which is fixed-jointed to `wrist_flange`. The Gazebo plugin block in `mod101.gazebo` loads `mod101_control`'s arm YAML **plus** the active tool's YAML (multiple `<parameters>` files merge into the single `controller_manager` namespace); the launch file `IncludeLaunchDescription`s the tool's `launch/tool.launch.py` to spawn whatever controllers / bridges / drivers the tool needs.

### Adding a new tool

Create `mod101_tool_<name>/` with:

```
urdf/tool.urdf.xacro        # links + a fixed joint to wrist_flange
urdf/tool.ros2control       # optional, included by tool.urdf.xacro
urdf/tool.gazebo            # optional, included by tool.urdf.xacro
config/controllers.yaml     # optional
launch/tool.launch.py       # spawners, image bridges, drivers, ...
```

Then add one `<xacro:if>` branch in `mod101.xacro` (include the tool's xacro) and one in `mod101.gazebo` (point the plugin at the tool's controllers YAML if it has one). The configurator auto-discovers the package on disk and lists it in the dropdown.

## Joints

- Arm (`mod101_description`): `1`, `2`, `3`, `4`, `5` (all continuous)
- Tool joints live in the tool package. For `mod101_tool_parallel`: `6` (prismatic, drives `left_jaw`); `right_jaw_slider` follows via URDF `<mimic>` (bullet-featherstone honors it; dartsim doesn't).

## Build

```bash
cd ~/Work/mod101
colcon build --packages-select \
  mod101_description mod101_control mod101_gazebo \
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
ros2 launch mod101_gazebo gazebo.launch.py                 # tool:=parallel (default)
ros2 launch mod101_gazebo gazebo.launch.py tool:=pincopen
ros2 launch mod101_gazebo gazebo.launch.py tool:=jaws
ros2 launch mod101_gazebo gazebo.launch.py tool:=none
```

## Controllers

Controllers active after launch (with `tool:=parallel`):

| Controller | Type | Joints | Owner | State at boot |
|---|---|---|---|---|
| `joint_state_broadcaster` | broadcaster | — | `mod101_gazebo` | active |
| `arm_controller` | `position_controllers/JointGroupPositionController` | 1-5 | `mod101_gazebo` | **active** (default) |
| `gripper_controller` | `position_controllers/JointGroupPositionController` | 6 | `mod101_tool_parallel` | active |
| `arm_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` (JTC) | 1-5 | YAML in `mod101_control` | defined, **not spawned** — see note below |

Send a position command:

```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.5, -0.5, 0.0, 0.0]}"
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.013]}"
```

### JTC is intentionally NOT spawned

The current Jazzy stack (`gz_ros2_control` + `joint_trajectory_controller`) segfaults in `JTC::on_init()` because `gz_ros2_control` predates the Resource Manager constructor change ([ros2_control issue #2400](https://github.com/ros-controls/ros2_control/issues/2400)) — the hardware's executor isn't set, JTC dereferences a null at offset `0x18`. When you bring up MoveIt, spawn JTC from there in a separate launch and swap it in:

```bash
ros2 run controller_manager spawner arm_trajectory_controller --inactive
ros2 control switch_controllers \
  --activate arm_trajectory_controller --deactivate arm_controller
```

## Wiring real hardware

Override `mod101.xacro`'s `use_sim` arg to `false` and switch `mod101.hardware.xacro`'s placeholder `mock_components/GenericSystem` plugin for the real servo bus driver.

## Gotchas

- `update_rate` is **int** (`100`, not `100.0`) — Jazzy crashes on float.
- `state_publish_rate`, `action_monitor_rate` are **double** (`50.0`, `20.0`).
- `controller_manager.catch_exceptions: true` keeps gazebo alive when a controller's `on_init()` throws — real error is in the line just above the C++ stack trace.
- Physics engine is **bullet-featherstone** (set in `worlds/empty.sdf`) so URDF `<mimic>` works (used by `mod101_tool_parallel` for `right_jaw_slider`, and by `mod101_tool_pincopen` for all four 4-bar joints). Switching back to dartsim drops mimicked joints.
- Each `mod101_tool_*` package needs at least one node in its `launch/tool.launch.py` for `IncludeLaunchDescription` to be happy — `mod101_tool_none` ships an empty `LaunchDescription([])` and that's fine.
- `GZ_SIM_RESOURCE_PATH` must include each tool's install prefix so Gazebo can resolve `package://mod101_tool_<name>/meshes/...`. The main `gazebo.launch.py` builds this list automatically from the active tool.
