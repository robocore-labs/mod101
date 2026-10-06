# Camera mount end effector

Passive wrist-flange tool extracted from `camera_tool.step`. The mount origin
is the rear face at the centre of the +/-7 mm bolt pattern. +X points out of
the lenses; +Z places the GoPro above the selected depth camera.

In the configurator select **Camera mount**, choose **Luxonis** or **RealSense**,
and optionally enable **Top GoPro**. Save to persist the configuration.
The primary cameras are mutually exclusive in the URDF, not merely hidden
in the preview. The GoPro body, mass, and optical frame are all removed when off.

```bash
ros2 launch mod101_moveit_config mock.launch.py \
  tool:=camera camera_sensor:=realsense camera_gopro:=true arm_dof:=7
```

Imported `mod101_arm` macros accept `camera_sensor` and `camera_gopro` per
instance. The paired `mod101_arm_srdf` accepts the same GoPro flag. No extra
joint, controller, or servo is created; camera drivers are separate from this
mechanical description package.

| Component | Mass |
|---|---:|
| PETG holder (15% infill, 1.6 mm shell, 1.27 g/cm³) | 46.2 g estimated |
| Luxonis | 61 g (user supplied) |
| RealSense | 72 g (user supplied) |
| GoPro | 153 g (user supplied) |

The holder includes its GoPro mounting fork even with the camera absent.
COM/inertia assume uniform effective density in each CAD component; camera
masses use the supplied weights. Source hash, geometry measurements, tensors,
and the CAD-to-tool transform are recorded in `config/measurements.json`.
`tools/export_camera_tool.py` reproduces the meshes and mass measurements;
`urdf/tool.urdf.xacro` remains the authored robot description.

`<prefix>camera_tool_gopro_optical_frame` follows REP-103: +Z forward, +X image
right, +Y down. Its origin is the front lens-window centre measured from CAD,
at `(35.1851, 21.5930, 66)` mm in tool coordinates. It is a mechanical reference;
use camera calibration for imaging intrinsics and precise optical extrinsics.
The GoPro stays fixed at its exported pose.

Visual meshes preserve the vendor STEP surfaces, including imported seams;
collision shapes are conservative boxes. The primary box is the union of both
camera envelopes so the sampled collision matrix is valid for either choice.
Collision pairs involving the optional GoPro are omitted when it is disabled.
