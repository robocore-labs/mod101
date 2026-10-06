# Tool options API (version 1)

A `mod101_tool_<name>` package can supply `config/configurator.yaml`. The
configurator discovers these files, renders their controls with the standard
panel styling, validates values, forwards them to xacro, and saves them as
arguments in `mod101_description/urdf/mod101_config.xacro`. ROS description,
Gazebo, mock, MoveIt and hardware launches discover the same arguments from
installed package metadata. The shared implementation is
`mod101_description/tool_config.py`; the frontend has no camera-specific logic.

```yaml
schema_version: 1
label: Camera mount
actuated: false
servo_count: 0
options:
  camera_sensor:
    type: enum
    label: Depth camera
    default: luxonis
    choices:
      - {value: luxonis, label: "Luxonis · 61 g"}
      - {value: realsense, label: "RealSense · 72 g"}
  camera_gopro:
    type: boolean
    label: Top GoPro · 153 g
    default: false
```

Version 1 supports `enum` and `boolean`. Option names must use lowercase letters,
digits and underscores, start with a letter, and be unique across tool packages;
namespace them by tool. Core arguments such as `tool`, `arm_dof` and
`wrist_camera` are reserved. Enum choices define accepted values; arbitrary
strings and non-boolean values are rejected before saving. Omitted values retain
their saved settings or use the schema default. Packages without a manifest keep
the existing UI and derive actuator presence from their controller YAML.

The schema describes controls; it does not generate robot geometry. The tool's
authored xacro declares the corresponding arguments and uses them to select
links, meshes and inertials. `mod101_tool_camera/urdf/tool_config.xacro` reads
its defaults directly from the YAML, avoiding a second default-value source.
Arm wrappers pass the arguments into the tool macro; imported arm instances can
pass their own values independently. A new tool still registers its URDF/SRDF
macro with the arm and provides appropriate collision data.

`GET /tool` returns `schemas` and saved `options`. `GET /load` returns saved
`tool_options`. Live `/urdf` and `/payload` requests accept the declared argument
names as query parameters. `POST /save` accepts `tool_options` as an object:

```json
{"shoulder": 0.14, "elbow": 0.14,
 "tool_options": {"camera_sensor": "realsense", "camera_gopro": true}}
```

Passive tools ship an empty `config/controllers.yaml` and an empty
`launch/tool.launch.py`. They expose no joint `6`, spawn no gripper controller,
and hardware bring-up omits the tool's servo group. The configurator uses
`servo_count` only for its estimated actuator cost.
