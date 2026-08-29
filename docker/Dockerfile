# One image for the mod101 bench: simulation, ros2_control, and this repo's
# packages. Nothing else.
#
# Deliberately dumb: it contains dependencies and nothing else. No workspace is
# copied in — src/ is bind-mounted at runtime from the repo, so editing a launch
# file or a Python module needs no rebuild, and `colcon build` writes back into
# the repo's own docker/.bench/ directory.
#
# NOT IN HERE, AND ON PURPOSE:
#   - the agent and the robocore SDK. They ship as `robocore-agent:latest`,
#     built by their own repo; compose.yaml runs that image alongside this one.
#     They used to be bind-mounted and built here, which made this repo's build
#     depend on two repos it does not own.
#   - nav2 (nine packages), slam_toolbox, laser_filters, twist_mux,
#     robot_localization, diff_drive_controller. This robot is bolted to a
#     bench: it cannot drive, has no lidar and has nothing to localise against.
#     An unused Nav2 in the image is an invitation to wire it up by accident.
#   - MoveIt, pick_ik and the pincopen tool driver. The two packages that need
#     them (mod101_moveit_config, mod101_tool_pincopen) are skipped by the
#     workspace build — see compose.yaml.
#
# Rebuild this image only when a package.xml gains a new dependency.
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

# Collected from the package.xml of every package this stack actually builds:
# mod101_description, mod101_control, mod101_gazebo, mod101_harness and the
# jaws/parallel/none tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-yaml \
      ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2 \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-controller-manager ros-jazzy-hardware-interface \
      ros-jazzy-joint-state-broadcaster \
      ros-jazzy-joint-trajectory-controller ros-jazzy-position-controllers \
      ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-pluginlib \
      ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
      ros-jazzy-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# GUI runtime for the Gazebo client and RViz. ros:jazzy-ros-base is headless, so
# Qt6's xcb platform plugin has nothing to load against and gz sim dies with
# "no Qt platform plugin could be initialized" — libxcb-cursor0 in particular is
# required by Qt6 and ships in no ROS image.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libxcb-cursor0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
      libxcb-randr0 libxcb-render-util0 libxcb-shape0 libxcb-xinerama0 \
      libxcb-xkb1 libxkbcommon-x11-0 libgl1 libegl1 libglx-mesa0 \
      libxkbcommon0 x11-utils \
    && rm -rf /var/lib/apt/lists/*

# One colcon workspace, /ws, mounted from docker/.bench. Everything the stack
# runs is relative to it, which is why the compose commands need no paths.
WORKDIR /ws

# Sources ROS (and the workspace overlay, once it has been built) so every
# service command is the bare `ros2 ...` / `colcon ...` line it should be.
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
