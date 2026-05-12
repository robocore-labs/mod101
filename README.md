# mod101

A modular, open-source 5+1 DOF robot arm derived from the SO-101. Built around dual 2020 aluminum extrusion links, PLA-CF printed brackets, and Feetech STS32xx serial bus servos.

## Why mod101?

The SO-101 is a great starting point for hobby robotics, but its all-3D-printed structure is fragile, non-parametric, and the gripper is permanently integrated. The mod101 fixes all of that.

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

The mod101 PRO exceeds the reBot B601's payload at every reach percentage, at 18% of the cost.

## Architecture

The arm is built from standardized, swappable components.

**Structure:** Dual 2020 T-slot aluminum extrusion rails with a 15mm gap. The dual-rail design provides torsion resistance at the bracket mounting points (8.5× safety factor vs doubled servo torque) and a natural cable routing channel between the rails. Link lengths are configurable — cut extrusion to length.

**Brackets:** PLA-CF (carbon fiber filled PLA), 6mm wall thickness. Exceeds the strength and stiffness of the 2mm aluminum brackets that ship with STS3250 servos (1.43× strength, 1.38× stiffness). Print flat, 4+ perimeters, 0.2mm layer height.

**Servos:** Feetech STS3215 (30 kg·cm, ~$16.50) or STS3250 (50 kg·cm, ~$55). Same form factor, same TTL bus, same brackets. Drop-in upgrade. The shoulder and elbow joints are doubled (two servos per joint) for increased torque and redundancy.


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

ROS 2 (Jazzy) packages following the standard description / control / gazebo pattern.

| Package | Build type | Purpose |
|---|---|---|
| `mod101_description` | `ament_python` | URDF/xacro (`mod101.xacro`, `materials.xacro`, `mod101.ros2control`, `mod101.gazebo`), meshes, RViz config, `display.launch.py` |
| `mod101_control` | `ament_cmake` | Controller YAML (`controllers.sim.yaml`), real-hardware overlay (`mod101.hardware.xacro` — placeholder plugin), spawner launch |
| `mod101_gazebo` | `ament_cmake` | World (`empty.sdf`, bullet-featherstone physics), bridge config, `gazebo.launch.py` |

### Joints

- Arm: `1`, `2`, `3`, `4`, `5` (continuous)
- Gripper: `6` (prismatic, drives `left_jaw`); `right_jaw_slider` follows via URDF `<mimic>` (bullet-featherstone honors it; dartsim doesn't)

### Build

```bash
cd ~/ros_ws
colcon build --packages-select mod101_description mod101_control mod101_gazebo
source install/setup.bash
```

### Run

RViz only (no physics, joint sliders):

```bash
ros2 launch mod101_description display.launch.py
```

Gazebo sim with ros2_control:

```bash
ros2 launch mod101_gazebo gazebo.launch.py
```

Controllers loaded on gazebo launch:

| Controller | Type | Joints | State at boot |
|---|---|---|---|
| `joint_state_broadcaster` | broadcaster | — | active |
| `arm_controller` | `forward_command_controller/ForwardCommandController` (position) | 1-5 | **active** (default) |
| `arm_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | 1-5 | inactive (for MoveIt) |
| `gripper_controller` | `joint_trajectory_controller/JointTrajectoryController` | 6 | active |

Send a position command to the arm:

```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.5, -0.5, 0.0, 0.0]}"
```

Switch to trajectory control (e.g. when MoveIt comes up):

```bash
ros2 control switch_controllers \
  --activate arm_trajectory_controller --deactivate arm_controller
```

### Wiring Real Hardware

Override `mod101.xacro`'s `use_sim` arg to `false` and switch `mod101.hardware.xacro`'s placeholder `mock_components/GenericSystem` plugin for the real servo bus driver.

### Gotchas

- `update_rate` is **int** (`100`, not `100.0`) — Jazzy crashes on float.
- `state_publish_rate`, `action_monitor_rate` are **double** (`50.0`, `20.0`).
- `controller_manager.catch_exceptions: true` keeps gazebo alive when a controller's `on_init()` throws — real error is in the line just above the C++ stack trace.
- Physics engine is **bullet-featherstone** (set in `worlds/empty.sdf`) so `<mimic>` on `right_jaw_slider` works. Switching back to dartsim drops the right jaw.

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

Derived from the [SO-101](https://github.com/TheRobotStudio/SO-ARM100) by The Robot Studio. Gripper inspired by the [PathOn 6DOF symmetric gripper](https://github.com/PathOn-AI/pathon_opensource).

## License

MIT