# mod101_hw_bringup

The mod101 arm on its camera-tower harness, on **real hardware**.

```bash
ros2 launch mod101_hw_bringup bringup.launch.py
```

This package is configuration, not code. Every node it starts comes from
somewhere else, and none of those places know what a mod101 is:

| what | package | where it comes from |
|---|---|---|
| Feetech ST3215 bus | `st3215_manager` | [robocore-labs/ros2_feetech_manager](https://github.com/robocore-labs/ros2_feetech_manager) |
| ros2_control hardware plugin | `ros2_control_bridge` | [robocore-labs/ros2_control_bridge](https://github.com/robocore-labs/ros2_control_bridge) |
| RealSense | `realsense2_camera` | upstream |
| wrist camera | `usb_cam` | upstream |
| description, controllers | `mod101_harness`, `mod101_description` | this repo |

The two robocore drivers are separate repos. Get them into the workspace with
`vcs import src/hardware < src/hardware/hardware.repos`, or symlink a local
checkout — see `hardware.repos`.

## The shape of it

```
  controllers                    ros2_control_node
  (arm / pan_tilt / gripper)            │
                                        │  ros2_control_bridge/TopicBridge
                                        ▼
        /motor_manager/arm_cmd   ┌──────────────┐   /motor_manager/joint_states
        /motor_manager/tool_cmd  │              │   ─────────────────────────▶
        /motor_manager/head_cmd  │ Float64Multi │        (back to the bridge)
        ────────────────────────▶│   Array      │
                                 └──────┬───────┘
                                        ▼
                                 servo_manager_node
                                        │
                                 one RS-485 chain, 8x STS3215
```

**Why the bus is a separate node instead of a SystemInterface that opens the
port.** `read()` and `write()` run inside controller_manager's update loop, so a
blocking serial read there stalls every controller on the robot — and eight
servos over one 1 Mbaud line is milliseconds per cycle. Splitting them puts the
serial latency on a topic instead of in the control loop. The cost is a hop and
the loss of hard sync between command and state, which a hobby servo bus does
not offer anyway.

## In a container

Parity with the sim stack, and the same shape: one image built from this repo,
one pulled for the agent.

```bash
docker compose -f docker/hardware.compose.yaml up --build
docker compose -f docker/hardware.compose.yaml down
```

**Run it this way rather than from a shell.** A `kill` on a host launch takes
the launcher and misses its `robot_state_publisher`, which keeps publishing
`/robot_description` and TF into the next run's graph; three of those piled up
here in one afternoon, and the symptom is a duplicate-node warning and TF that
flickers between two robots. `docker compose down` cannot leave one behind.

Three things the compose file gets right that are easy to get wrong by hand:

- **The bus is mapped by its udev name**, `/dev/mod101-servo`, which is what
  `servos.yaml` already names. The Link101 exposes five CDC endpoints and their
  numbering moves — it shifted once mid-session on this bench. Never map
  `/dev/ttyACMn`.
- **A different ROS domain from the sim** (2 vs 1). Sharing one would put a
  simulated `controller_manager` on the same graph as the real one, and a
  `ros2 topic pub` meant for the bench would reach the robot.
- **`restart: "no"`.** The driver refuses to start when motors are silent or a
  joint is resting outside its calibrated travel. Those want a person, not a
  supervisor retrying into them.

`up` starts three services: the robot, **ROSBoard** on
<http://localhost:8888>, and the agent on `ws://localhost:10101`.

**ROSBoard can move the arm.** The robocore fork's publish allowlist includes
`std_msgs/Float64MultiArray`, so its Joint sliders card publishes onto
`/arm_controller/commands` — and that controller treats what it receives as a
step, so a slider dragged across its range is a full-speed sweep with no
trajectory and no collision checking. It also binds `0.0.0.0:8888`, and with
`network_mode: host` there is no port mapping to narrow that: anything that can
reach this machine can drive the arm. On a bench that is what you want it for.

Its zero-on-silence watchdog deliberately skips position commands — a "safe"
zero is a stop for a Twist and a trip to home for a joint angle — so closing
the tab holds the arm rather than moving it.

The driver packages are cloned during the build, by the same `vcs import` line
a host workspace uses. So the image holds what is on `main` — a local edit that
has not been pushed does not silently become what the robot runs. The flip side
is that Docker cannot see a remote branch move, so after pushing a driver
change:

```bash
docker compose -f docker/hardware.compose.yaml build \
  --build-arg DRIVERS_CACHEBUST=$(date +%s)
```

Cameras are not wired into the image yet; see the note at the foot of
`hardware.compose.yaml`.

## Calibrate, then launch

The configurator writes everything that has to be discovered from the actual
robot, so a bring-up after calibration is one command:

```bash
python3 configurator/server.py     # Calibrate -> sweep each joint -> Write to repo
ros2 launch mod101_hw_bringup bringup.launch.py
```

**Three files make up the servo config, and which one owns what is the design:**

| file | written by | holds |
|---|---|---|
| `config/servos.yaml` | you | bus port, rates, which groups exist, speeds, accelerations, safety flags |
| `config/servos.generated.yaml` | the configurator | motor IDs, joint names, directions, array indices |
| `mod101_control/config/calibration.yaml` | the configurator | home tick and travel per joint |

`bringup.launch.py` loads the first, overlays the second, injects the third, and
hands the result to the driver as parameters.

**Two generated files rather than one big one, and neither is the authored
file.** A generator that rewrites `servos.yaml` eats the comments explaining why
the numbers are what they are — why the shoulder runs at half speed, why
`require_motors` is on — and those comments are the only record of it. Keeping
the machine's half separate means neither can clobber the other, and the overlay
only replaces the five keys it owns, so adding a tuning parameter to
`servos.yaml` needs no change to the generator.

**Group membership is read, not inferred.** The configurator asks `servos.yaml`
which joints belong to which group rather than sorting them by name — that
guessing is exactly what used to put the pan/tilt joints on the arm's topic. A
mapped servo whose joint no group claims, or a group joint with no servo, is a
warning in the write log, not a silent omission.

If the driver refuses a group with `min_ticks >= max_ticks`, the joint's travel
crosses the encoder's `4095 -> 0` seam. Recalibrate it in the configurator's
guided flow and, with the joint at its URDF default, press **Set centre here**
if the joint moves both ways from that pose (base, pan, tilt, wrist tilt) or
**Set zero here** if that pose is a hard stop (shoulder, elbow, wrist roll,
jaw). Either writes an offset into the servo placing the travel clear of the
seam; it is a servo-side fix, so it holds across power cycles and for this
driver too.

Without the calibration file the driver runs **uncalibrated**: home at mid-scale (2048) and
travel over the full encoder range. Every joint angle is then offset by however
far its servo horn sits from centre, in both directions at once — commands land
short and the reported position is wrong by the same amount, so the error
cancels on a plot and does not cancel on a robot. The launch says so loudly at
startup; it is not a silent default.

## Configuration

`config/controllers.yaml` — the same controller set as the sim, in one file,
with `use_sim_time: false`. There is no `/clock` here, and a controller waiting
for simulated time on a robot with no simulator never ticks.

**Three groups — `arm`, `tool`, `head` — and they are declared twice.** Once as
`<param name="group">` on each joint in the URDF (`mod101_harness_arm.xacro`,
`mod101_tool_jaws/urdf/tool.ros2control`), and once as a group block in
`servos.yaml`. The URDF side decides what the bridge publishes and in what
order; the YAML side decides which motor reads which index. **They have to
agree**, including the order of joints within a group, because that order *is*
the array layout on the wire. Reorder one side and the wrong servo moves.

## Arguments

| arg | default | |
|---|---|---|
| `hardware` | `bridge` | `mock` loops commands back as state — the whole graph comes up and nothing moves |
| `cameras` | `true` | `false` skips both cameras |
| `serial_port` | from yaml | override the bus device |
| `calibration_file` | `mod101_control/config/calibration.yaml` | |
| `servos_file`, `controllers_file`, `generated_servos_file` | this package | |
| `tool`, `shoulder_ext_length`, … | configurator's saved values | same build args as `sim.launch.py` |

A first bring-up worth doing in this order:

```bash
# 1. nothing moves, everything loads — checks the URDF, plugin and controllers
ros2 launch mod101_hw_bringup bringup.launch.py hardware:=mock cameras:=false

# 2. the bus, for real, with the arm powered
ros2 launch mod101_hw_bringup bringup.launch.py cameras:=false
```

## Cameras

`launch/cameras.launch.py`, also runnable alone. It **remaps the drivers onto
the topic names the simulator uses** (`/harness/depth_camera/*`,
`/wrist_camera/*`) so `profiles/mod101_harness.yaml` — and anything written
against it — is the same file on hardware and in sim. Publishing the RealSense's
native names would fork the profile in two, and the two copies would drift.

One thing does not match sim: the driver stamps images with its **own** optical
frames (`head_camera_color_optical_frame`), not the URDF's
`hn_realsense_optical_frame`. The camera's frames hang off the URDF link
`hn_realsense_1` via `base_frame_id`, so TF resolves — but if you deproject
against the profile on hardware, the head camera's `frame:` has to name the
driver's frame.

Prefer a `/dev/v4l/by-id/...` path for `wrist_camera_device`: `/dev/video0` is
whichever camera enumerated first, which changes when the RealSense is plugged
in.

## Motor IDs

1–5 arm, 6 jaw, 7–8 head. 1–6 are the SO-101 factory order; 7 and 8 are a
convention this package chose.

The IDs in `servos.yaml` are only the **fallback** — what runs before anything
has been calibrated. Once the configurator has written
`servos.generated.yaml`, the IDs and directions come from what actually
answered on the bus, and editing `servos.yaml` will not change them. IDs live in
servo EEPROM; the configurator's Hardware tab both scans for them and reassigns
them.

`require_motors: true` means the driver refuses to start unless all eight
answer. The usual reason they do not is motor power: the logic side is USB-fed,
so the port opens and the chain stays silent, which looks like a working driver
with a dead robot.

## Not done yet

- **Only the `jaws` tool has a hardware path.** `parallel`, `pincopen` and
  `none` still emit `mock_components/GenericSystem` when `use_sim:=false`;
  `tool.ros2control` in each needs the same `$(arg hardware)` block the jaws
  tool now has.
- **The head's motor IDs and directions are unverified** — nothing has been on
  the bus yet.
- **No e-stop.** `brake_method: torque_disable` drops torque when a group is
  commanded to stop, but nothing here cuts power, and a servo holding position
  with a stalled motor draws current until it cooks. The configurator's Disarm
  is the only thing that drops torque on demand, and it needs the bus to itself.
