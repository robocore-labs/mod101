# mod101

**An open-source, universal robot arm platform.** mod101 is a 5-DOF arm designed as a *base* you build on — resize it for your reach envelope, snap in whichever tool the job needs (gripper, vacuum, camera, dispenser…), and drive it with whatever stack you like (ROS 2, MoveIt, direct serial). Dual 2020 aluminum extrusion links, PLA-CF brackets, Feetech STS32xx serial bus servos.

![](img/imagine.jpg)

## Features

- **Modular link sizing** — shoulder and elbow extrusions are parametric. Set their lengths from the web configurator (live three.js preview, payload + reach recompute on the fly) and the xacro is rewritten in place.
- **Hot-swappable tooling** — every end-effector is its own ROS 2 package (`mod101_tool_*`) shipping its own URDF, controllers, gazebo extensions, and launch file. The arm terminates at a single `wrist_flange` mount; tools attach there mechanically *and* in software. Ships with `mod101_tool_parallel` (parallel-jaw gripper) and `mod101_tool_none` (blank end-cap); add new ones (vacuum gripper, inspection camera, glue dispenser, soldering iron, …) by dropping a package into `src/`.
- **Configurable servo budget** — same brackets, same firmware. BASE config (8× STS3215, ~$134) holds 547 g continuous at full extension / 852 g at 70 % reach. PRO swaps two shoulder servos for STS3250s (~$211) and jumps to 1,105 g / 1,649 g.
- **Real structure, not a printed shell** — dual 2020 aluminum extrusions between every joint, PLA-CF brackets at the joints only. Stiffer and more durable than fully-printed alternatives, no special hardware to source.
- **First-class sim** — Gazebo Sim (bullet-featherstone for `<mimic>`) with `gz_ros2_control`, a wrist camera bridged to ROS, and a `forge` manifest (`simulation.yaml`) for one-command container deploy.
- **Web configurator** — `python3 configurator/server.py` opens a static page that edits the xacro live, swaps tools from a dropdown, and shows the assembled robot in three.js.
- **Open** — MIT licensed, no proprietary parts, BOM under $135 entry-level.

## Why mod101?

The SO-101 is a great starting point for hobby robotics, but its all-3D-printed structure is fragile, the link lengths are fixed, and the gripper is permanently integrated. mod101 fixes all of that — and treats the end-effector as a first-class swappable layer, not a hard-coded part of the arm.

| | SO-101 | mod101 BASE | mod101 PRO | reBot B601 |
|---|---|---|---|---|
| DOF | 5+1 | 5+1 | 5+1 | 6+1 |
| Reach | 351mm | 502mm | 502mm | 767mm |
| Payload (continuous) | 346g | 547g | 1,105g | ~750g |
| Payload @ 70% reach | — | 852g | 1,649g | 1,500g |
| Structure | 3D printed PLA | Dual 2020 + PLA-CF | Dual 2020 + PLA-CF | CNC 5052 aluminum |
| Quick-change end effector | No | Yes | Yes | No |
| Configurable link lengths | No | Yes | Yes | No |
| BOM cost | ~$85 | ~$134 | ~$211 | ~$1,200 |
| License | Apache 2.0 | MIT | MIT | CC BY-NC-SA |


## Configurations

Same brackets, same extrusion, same firmware. Just swap servos.

### BASE — 8× ST3215 ($134)

- 2× ST3215 shoulder pitch (doubled)
- 2× ST3215 elbow (doubled)
- 1× ST3215 wrist tilt
- 1× ST3215 wrist roll
- 1× ST3215 gripper

| Condition | Payload |
|---|---|
| Continuous (70% stall) | 547g |
| Stall (100%) | 906g |
| Continuous @ 70% reach | 852g |

Bottleneck: shoulder pitch (doubled elbow has 2× headroom).

### PRO — 2× STS3250 shoulder + 6× ST3215 ($211)

Swap two shoulder servos from ST3215 to STS3250. Everything else stays identical.

| Condition | Payload |
|---|---|
| Continuous (70% stall) | 1,105g |
| Stall (100%) | 1,703g |
| Continuous @ 70% reach | 1,649g |

## Payload Notes

All payload numbers assume worst-case: arm fully extended horizontal, sustained hold. Real-world working payloads are significantly higher because the arm is rarely at full horizontal extension.

At a 45° working angle (typical tabletop pick-and-place), available payload roughly doubles. The robot can *move* 1kg+ through most of its workspace on the BASE config; it just can't *hold* 1kg at full horizontal extension.


## ROS 2 Packages

ROS 2 (Jazzy) packages following the standard description / control / gazebo pattern, with a **modular tool layer**: every end-effector lives in its own `mod101_tool_<name>` package that ships everything it needs (URDF, ros2_control hardware block, gazebo extensions, controllers YAML, launch).

| Package | Build type | Purpose |
|---|---|---|
| `mod101_description` | `ament_cmake` | Arm URDF/xacro (`mod101.xacro`, `materials.xacro`, `mod101.ros2control`, `mod101.gazebo`), meshes, RViz config, `display.launch.py`. Terminates at the `wrist_flange` link — the contract point for tools. |
| `mod101_control` | `ament_cmake` | Arm controller YAML (`controllers.sim.yaml`), real-hardware overlay (`mod101.hardware.xacro` — placeholder plugin) |
| `mod101_gazebo` | `ament_cmake` | World (`empty.sdf`, bullet-featherstone physics), bridge config, `gazebo.launch.py` (accepts `tool:=<name>`) |
| `mod101_tool_parallel` | `ament_cmake` | Parallel-jaw gripper: URDF + meshes, own ros2_control block, own controllers YAML, own launch (spawns `gripper_controller`) |
| `mod101_tool_none` | `ament_cmake` | Empty end-cap. No actuators, no controllers — useful as a baseline / template |

### Tool architecture

The arm xacro takes a `tool` arg (default `parallel`). At expansion time it `<xacro:include>`s `mod101_tool_<tool>/urdf/tool.urdf.xacro`, which is fixed-jointed to `wrist_flange`. The Gazebo plugin block in `mod101.gazebo` loads `mod101_control`'s arm YAML **plus** the active tool's YAML (multiple `<parameters>` files merge into the single `controller_manager` namespace); the launch file `IncludeLaunchDescription`s the tool's `launch/tool.launch.py` to spawn whatever controllers / bridges / drivers the tool needs.

Adding a new tool = create `mod101_tool_<name>/` with:

```
urdf/tool.urdf.xacro        # links + a fixed joint to wrist_flange
urdf/tool.ros2control       # optional, included by tool.urdf.xacro
urdf/tool.gazebo            # optional, included by tool.urdf.xacro
config/controllers.yaml     # optional
launch/tool.launch.py       # spawners, image bridges, drivers, ...
```

…plus one `<xacro:if>` branch in `mod101.xacro` (include the tool's xacro) and one in `mod101.gazebo` (point the plugin at the tool's controllers YAML if it has one). The configurator auto-discovers the package on disk and lists it in the dropdown.

### Joints

- Arm (`mod101_description`): `1`, `2`, `3`, `4`, `5` (all continuous)
- Tool joints live in the tool package. For `mod101_tool_parallel`: `6` (prismatic, drives `left_jaw`); `right_jaw_slider` follows via URDF `<mimic>` (bullet-featherstone honors it; dartsim doesn't).

### Build

```bash
cd ~/Work/mod101
colcon build --packages-select \
  mod101_description mod101_control mod101_gazebo \
  mod101_tool_parallel mod101_tool_none
source install/setup.bash
```

### Run

RViz only (no physics, joint sliders):

```bash
ros2 launch mod101_description display.launch.py
```

Gazebo sim with ros2_control (default tool is the parallel gripper):

```bash
ros2 launch mod101_gazebo gazebo.launch.py                 # tool:=parallel (default)
ros2 launch mod101_gazebo gazebo.launch.py tool:=none      # no end-effector
```

Controllers active after launch (tool=parallel):

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

**JTC is intentionally NOT spawned from `gazebo.launch.py`.** The current Jazzy stack (`gz_ros2_control` + `joint_trajectory_controller`) segfaults in `JTC::on_init()` because `gz_ros2_control` predates the Resource Manager constructor change ([ros2_control issue #2400](https://github.com/ros-controls/ros2_control/issues/2400)) — the hardware's executor isn't set, JTC dereferences a null at offset `0x18`. When you bring up MoveIt, spawn JTC from there in a separate launch and swap it in:

```bash
ros2 run controller_manager spawner arm_trajectory_controller --inactive
ros2 control switch_controllers \
  --activate arm_trajectory_controller --deactivate arm_controller
```

### Configurator (web app)

`configurator/` is a self-contained tool for sizing and tooling the arm. It edits `src/mod101_description/urdf/mod101.xacro` in place and shows a live three.js view (`urdf-loader`) that re-renders on every save.

```bash
cd ~/Work/mod101
python3 configurator/server.py
# open http://localhost:8000/
```

What you can change:

- **Extrusion lengths** — `shoulder_ext_length` and `elbow_ext_length` xacro properties. Reach, total mass, and payload (continuous + stall, at full extension and at 70 % reach) recompute live for both BASE and PRO servo configs.
- **Tool** — dropdown lists every `mod101_tool_*` package discovered under `src/`; selecting one rewrites the `tool` xacro arg's `default` and reloads the viewer with the new end-effector attached.

Backend (`server.py`) is stdlib-only Python — no Flask. Endpoints:

| Method + path | Purpose |
|---|---|
| `GET /load` / `POST /save` | shoulder + elbow extrusion lengths |
| `GET /tool` / `POST /tool` | active tool + discovered tools |
| `GET /urdf` | runs `xacro` and rewrites mesh URIs (`package://` and `file://`) to `/pkg/<pkgname>/meshes/<file>` |
| `GET /pkg/<pkgname>/meshes/<file>` | serves binary meshes from any package's `meshes/` dir |

After changing anything, rebuild the affected packages (`mod101_description` for lengths, `mod101_tool_<name>` if you also added/edited a tool) to pick up the changes in sim. You can also open `configurator/index.html` directly via `file://` for a read-only preview; Save needs the server.

### Wiring Real Hardware

Override `mod101.xacro`'s `use_sim` arg to `false` and switch `mod101.hardware.xacro`'s placeholder `mock_components/GenericSystem` plugin for the real servo bus driver.

### Gotchas

- `update_rate` is **int** (`100`, not `100.0`) — Jazzy crashes on float.
- `state_publish_rate`, `action_monitor_rate` are **double** (`50.0`, `20.0`).
- `controller_manager.catch_exceptions: true` keeps gazebo alive when a controller's `on_init()` throws — real error is in the line just above the C++ stack trace.
- Physics engine is **bullet-featherstone** (set in `worlds/empty.sdf`) so URDF `<mimic>` works (used by `mod101_tool_parallel` for `right_jaw_slider`). Switching back to dartsim drops mimicked joints.
- Each `mod101_tool_*` package needs at least one node in its `launch/tool.launch.py` for `IncludeLaunchDescription` to be happy — `mod101_tool_none` ships an empty `LaunchDescription([])` and that's fine.

## BOM

### Base Config (~$134)

| Part | Qty | Unit Price | Subtotal |
|---|---|---|---|
| STS3215 servo (12V, 30 kg·cm) | 6 | $16.50 | $99.00 |
| Waveshare SC09 gripper servo | 1 | $5.00 | $5.00 |
| 2020 aluminum extrusion (cut to length) | ~0.6m | $5.00/m | $3.00 |
| M5 T-nuts | ~20 | — | $3.00 |
| M5×8 bolts | ~20 | — | $2.00 |
| PLA-CF filament (~150g) | — | — | $8.00 |
| Pogo pin connector (4-pin) | 1 | $3.00 | $3.00 |
| Misc hardware (bolts, nuts, wires) | — | — | $10.00 |
| **Total** | | | **~$134** |

### PRO Upgrade (+$77)

Replace 2× STS3215 shoulder servos with 2× STS3250 ($55 each). Everything else unchanged.

## Related Projects

- **[Axon](https://github.com/robocore-dev/axon)** — RP2350-based multi-protocol bridge (CAN FD / RS485 / Dynamixel / Feetech)
- **[Bolt](https://github.com/robocore-dev/bolt)** — Power distribution hub
- **[Forge](https://github.com/robocore-dev/forge)** — ROS 2 deployment orchestration
- **[vision-factory](https://github.com/robocore-dev/vision-factory)** — Plug-and-play computer vision pipeline generator

## Acknowledgments
* Derived from the [SO-101](https://github.com/TheRobotStudio/SO-ARM100) by The Robot Studio. 
* Gripper is the [PathOn 6DOF symmetric gripper](https://github.com/PathOn-AI/pathon_opensource).

## License
MIT