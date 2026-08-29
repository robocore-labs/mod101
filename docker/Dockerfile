# One image for the mod101 bench simulation: ROS 2, Gazebo, ros2_control and
# this repo's packages, workspace built in.
#
# Build from the REPO ROOT (compose does this for you):
#   docker build -f docker/Dockerfile -t mod101-bench:jazzy .
#
# It used to ship dependencies only and bind-mount src/ from the host, with a
# second `workspace` service that ran colcon and a BUILD.DONE sentinel the
# simulation service waited on. That handshake could not work — the simulation
# service had no volumes at all, so it waited on a file in its own empty /ws
# forever — and it bought edit-without-rebuild at the price of a stack nobody
# could start. One image, one service. `docker compose up --build` after a
# source change; every package here is install-only, so that is seconds.
#
# NOT IN HERE, AND ON PURPOSE:
#   - the agent and the robocore SDK. They ship as their own image, which
#     compose.yaml pulls and runs alongside this one. Bind-mounting and
#     building them here made this repo's build depend on a repo it does not
#     own.
#   - MoveIt and pick_ik. mod101_moveit_config BUILDS without them (it is an
#     install-only package, like every package in this workspace) and only
#     needs them to run, which the bench stack never does.
#   - the pincopen tool driver, for the same reason.
#   - nav2, slam_toolbox, laser_filters, twist_mux, robot_localization,
#     diff_drive_controller. This robot is bolted to a bench: it cannot drive,
#     has no lidar and has nothing to localise against. An unused Nav2 in the
#     image is an invitation to wire it up by accident.

ARG ROS_DISTRO=jazzy


# ------------------------------------------------------------------ base --
# Everything the stack needs to RUN. Both stages below start here, so the
# dependency list is written once.
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

# Collected from the package.xml of every package in src/.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-yaml \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-controller-manager \
      ros-${ROS_DISTRO}-hardware-interface \
      ros-${ROS_DISTRO}-joint-state-broadcaster \
      ros-${ROS_DISTRO}-joint-trajectory-controller \
      ros-${ROS_DISTRO}-position-controllers \
      ros-${ROS_DISTRO}-joint-state-publisher \
      ros-${ROS_DISTRO}-joint-state-publisher-gui \
      ros-${ROS_DISTRO}-pluginlib \
      ros-${ROS_DISTRO}-ros-gz-sim \
      ros-${ROS_DISTRO}-ros-gz-bridge \
      ros-${ROS_DISTRO}-ros-gz-image \
      ros-${ROS_DISTRO}-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# GUI runtime for the Gazebo client and RViz. ros-base is headless, so Qt6's
# xcb platform plugin has nothing to load against and gz sim dies with "no Qt
# platform plugin could be initialized" — libxcb-cursor0 in particular is
# required by Qt6 and ships in no ROS image.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libxcb-cursor0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
      libxcb-randr0 libxcb-render-util0 libxcb-shape0 libxcb-xinerama0 \
      libxcb-xkb1 libxkbcommon-x11-0 libgl1 libegl1 libglx-mesa0 \
      libxkbcommon0 x11-utils \
    && rm -rf /var/lib/apt/lists/*


# --------------------------------------------------------------- builder --
# The toolchain, which nothing needs at runtime, so it stays in this stage.
FROM base AS builder
ARG ROS_DISTRO
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build
COPY src ./src
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --merge-install --install-base /opt/mod101 \
         --cmake-args -DCMAKE_BUILD_TYPE=Release


# --------------------------------------------------------------- runtime --
FROM base AS runtime
ARG ROS_DISTRO

LABEL org.opencontainers.image.title="mod101-bench" \
      org.opencontainers.image.description="mod101 arm on the camera-tower harness: Gazebo bench simulation" \
      org.opencontainers.image.source="https://github.com/robocore-labs/mod101"

COPY --from=builder /opt/mod101 /opt/mod101

# Sources ROS and the workspace overlay, so every compose `command:` is the
# bare `ros2 ...` line it should be rather than a bash -lc folded scalar.
COPY --chmod=0755 docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "mod101_harness", "sim.launch.py"]
