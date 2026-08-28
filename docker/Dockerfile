# One image for the whole local sim stack.
#
# Deliberately dumb: it contains dependencies and nothing else. No workspace
# is copied in — every source directory is bind-mounted at runtime from the
# repo, so editing a launch file or a Python module needs no rebuild, and
# `colcon build` writes back into the repo's own .sim/ directory.
#
# Rebuild this image only when a package.xml gains a new dependency.
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

# Dependencies collected from every package.xml in the stack. MoveIt,
# pick_ik and the pincopen tool driver are deliberately absent: v0.5 drops
# MoveIt from the SDK, and the sim stack does not build those packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-pip python3-yaml \
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

# DELIBERATELY ABSENT, and this is the whole reason the bench has its own
# image rather than reusing base101's: nav2 (nine packages), slam_toolbox,
# laser_filters, twist_mux, robot_localization and diff_drive_controller.
# This robot is bolted to a bench. It cannot drive, has no lidar and has
# nothing to localise against, so every one of those would be a dependency
# that exists only to be unused — and an unused Nav2 in the image is an
# invitation to wire it up by accident.

# GUI runtime for the Gazebo client and RViz. ros:jazzy-ros-base is headless,
# so Qt6's xcb platform plugin has nothing to load against and gz sim dies
# with "no Qt platform plugin could be initialized" — libxcb-cursor0 in
# particular is required by Qt6 and ships in no ROS image.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libxcb-cursor0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
      libxcb-randr0 libxcb-render-util0 libxcb-shape0 libxcb-xinerama0 \
      libxcb-xkb1 libxkbcommon-x11-0 libgl1 libegl1 libglx-mesa0 \
      libxkbcommon0 x11-utils \
    && rm -rf /var/lib/apt/lists/*

# robocore needs pydantic v2. robocore_agent/package.xml asks for
# `python3-pydantic`, which on Ubuntu 24.04 is pydantic 1.10 — a compiled .so
# that shadows anything installed later into /usr/lib. Installing into
# /usr/local/lib/python3.12/dist-packages puts v2 ahead of it on sys.path,
# because /usr/local precedes /usr/lib in Debian's site config.
RUN pip install --no-cache-dir --break-system-packages \
      'pydantic>=2.5' 'websockets>=12' 'numpy>=1.24'

# rosboard's web server. Its package.xml does not declare these, so rosdep
# never installs them and the node dies with ModuleNotFoundError: tornado.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-tornado python3-simplejson python3-pil \
    && rm -rf /var/lib/apt/lists/*

# `robocore` itself is never installed: /bpe/engine is bind-mounted and put on
# PYTHONPATH by the compose file, so the client SDK and the agent always agree
# with the working tree.
ENV PYTHONPATH=/bpe/engine:$PYTHONPATH
WORKDIR /bpe
