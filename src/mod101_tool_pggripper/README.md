# PGGripper for mod101

Adapted from [NormaCore Parallel Jaw Gripper](https://github.com/norma-core/norma-core/tree/main/hardware/pgripper), licensed Apache-2.0. Original license and STEP are included. Meshes and kinematic description are adapted for mod101; upstream optional webcam/mount are omitted. Keep mod101 wrist camera enabled if required.

Select **PGGripper** in the configurator, or pass `tool:=pggripper` to the normal ROS launch commands. Supports both 6DOF and 7DOF, prefixed multi-arm macros, gripper position and trajectory controllers.

Joint `6` is the motor rotation in radians, zero closed and approximately 2.271 radians fully open. Two passive prismatic mimic joints convert rotation through the CAD rack pitch (4.15mm, 18 teeth); 27mm travel per jaw / 54mm aperture. Imported STEP is at a measured 50mm aperture. Servo mounting orientation and phase are retained; calibrate motor zero/sign on hardware before commanding it.

Mass estimate: PETG 15% infill with 1.6mm shell, 56g ST3215-family actuator, steel hardware at 7850kg/m³. Total approximately 154g, with CAD-derived COM and inertia. These are estimates, not weighed parts; upstream filament requirement includes print supports and differs. Cameras are not included in this package mass.

`config/measurements.json` records datums, source hash and computed mass properties. `tools/export_pggripper.py` exports meshes/measurements; authored xacro owns the robot description. Imported CAD includes faces without mesh triangulation; collision checking uses the available triangulated geometry.

The hardware interface follows the existing gripper bridge/mock/Gazebo selection. Motor torque and speed limits are provisional (1.9Nm, 2rad/s). Passive sliders have no command interfaces.
