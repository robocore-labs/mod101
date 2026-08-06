# Tool URDF convention

Every `mod101_tool_*` package follows the same coordinate convention so tools are interchangeable, the configurator's preview lines up cleanly, and the wrist adapter bracket has a single well-defined geometry.

## The rule

> **A tool's root link sits at the bolt pattern that mates with `wrist_flange`, oriented "upright" and projecting outward.**

Concretely, in the tool's root-link frame:

| Axis | Direction |
|---|---|
| `+X` | **outward** — the direction the tool projects away from the wrist (aligned with the wrist-roll axis) |
| `+Y` | right (when looking at the gripper from behind, along +X) |
| `+Z` | **up** — matches world `+Z` when the arm is in its canonical horizontal "pointing forward" pose |
| origin `(0,0,0)` | the geometric center of the bolt pattern that contacts `wrist_flange` |

This is a right-handed frame with `+X` as the "long" axis of the tool. A parallel gripper opens in `±Y`; a single jaw rotates in the `XZ` plane.

## What this means for the mount joint

The fixed joint that attaches the tool to the arm is **always identity**.
Tool URDFs are macros (so the arm can be instantiated multiple times in one
robot), so every link/joint name and reference carries the `${prefix}`
macro param:

```xml
<joint name="${prefix}<tool>_mount" type="fixed">
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <parent link="${prefix}wrist_flange"/>
  <child  link="${prefix}<tool>_root_link"/>
</joint>
```

If the tool has a mounting adapter / spacer bracket, model that as part of the tool — either with its own link or by offsetting the body link inside the tool's URDF. Don't bake adapter geometry into the mount joint.

## What this means for visual origins

If you author a tool from scratch, place each mesh so the link origin coincides with the link's natural pivot point (or the bolt pattern, for the root link). Visual `<origin>` should usually be near zero — large offsets are a sign that the mesh was authored in a different CAD-assembly frame and is being shimmed into place.

If you vendor a tool from another project (`jaws` from LLMy, `pincopen` from CNURobotics, etc.), the source URDF will typically have one of:

- **All meshes authored at a common CAD-world origin**, with each link's `<visual><origin>` carrying a large negative offset that brings the meshes back to that common point. (LLMy / SO-101 are like this.)
- **Each mesh authored at its own link origin**, with `<visual><origin>` near zero. (PincOpen is like this.)

For the first case, encode the CAD-frame-to-tool-frame transform as a **single fixed joint inside the tool** between the convention root link and the vendored sub-tree:

```xml
<link name="<tool>_root_link"><inertial>tiny</inertial></link>  <!-- convention origin -->

<link name="<tool>_vendor_body"> ...mesh with original visual_origin... </link>

<joint name="<tool>_vendor_to_root" type="fixed">
  <origin xyz="..." rpy="..."/>   <!-- bake CAD-frame offset + rotation here -->
  <parent link="<tool>_root_link"/>
  <child  link="<tool>_vendor_body"/>
</joint>
```

That keeps the vendored sub-tree internally consistent with its source URDF (preserving inter-mesh geometry, joint pivots, dynamics) while exposing a clean convention-following root link to the arm.

## Joint naming

The active joint(s) inside a tool continue mod101's joint numbering: `6`, `7`, … (always written `${prefix}6` etc. in the macro — the standalone arm's prefix is empty, so the runtime name is just `6`; in a multi-arm robot it becomes e.g. `left_arm_6`). Stick to integer joint names for the active DOFs; descriptive names (`gripper`, `wrist_roll`) only for fixed cosmetic joints or `<mimic>` followers. `<mimic joint=...>` references must carry the prefix too.

The primary controller a tool exposes should be named `gripper_controller` regardless of internal joint name, so downstream code that publishes to `/gripper_controller/commands` is tool-agnostic. (Multi-arm integrators define their own prefixed controllers — e.g. `left_gripper_controller` on `left_arm_6` — in their controller YAML.)

## Checklist for a new tool

- [ ] `urdf/tool.urdf.xacro` defines `<xacro:macro name="mod101_tool_<name>" params="prefix:='' use_sim:='true'">`; every link/joint name and parent/child/mimic reference inside carries `${prefix}`. (Copy `mod101_tool_jaws` — smallest complete example.)
- [ ] Root link `${prefix}<tool>_root_link` exists, mount joint to `${prefix}wrist_flange` is identity.
- [ ] In the root frame: `+X` outward, `+Z` up, origin at the bolt pattern.
- [ ] Active joint is named `${prefix}6` (or next available integer).
- [ ] Controllers YAML exposes `gripper_controller`.
- [ ] `launch/tool.launch.py` spawns `gripper_controller` on a `TimerAction(10s)` so it runs after the arm's broadcaster + arm_controller.
- [ ] `urdf/tool.ros2control` defines `mod101_tool_<name>_ros2control(prefix, use_sim)` and switches hardware plugin on `${use_sim}` (`gz_ros2_control/GazeboSimSystem` vs. real driver); the `<ros2_control>` system name is `${prefix}mod101_tool_<name>_system`.
- [ ] One include + `<xacro:if>` invocation branch added to `mod101_macro.xacro`; one `<parameters>` branch in `mod101.xacro`'s plugin block (if the tool ships controllers).
