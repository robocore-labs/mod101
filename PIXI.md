# Running mod101 with pixi

[pixi](https://pixi.sh) gives you a complete ROS 2 Jazzy toolchain in a project-local
folder, pulled from the [RoboStack](https://robostack.github.io) conda channel. No
`apt`, no `/opt/ros`, no system-wide install - delete `.pixi/` and it's gone.

This is the fastest way to build and run the workspace on a dev machine. The
`docker/` stack is still the reference for the bench simulation and for the real
robot; see [`docker/`](docker/) for that.

## Install pixi

```bash
curl -fsSL https://pixi.sh/install.sh | bash
exec $SHELL          # pick up the PATH change
```

## Quick start

```bash
pixi install         # solve + download ROS 2 Jazzy (first run: a few minutes)
pixi run build       # colcon build the workspace
pixi shell           # enter the environment - workspace overlay included
pixi run sim         # ros2 launch mod101_harness sim.launch.py
```

`pixi run <task>` runs a single command in the environment and exits. `pixi shell`
gives you an interactive shell with `ros2`, `rviz2`, `colcon`, etc. on `PATH` and
the built workspace sourced. Leave it with `exit`.

## Environments

| Environment | Enter with            | Contents                                                        |
|-------------|-----------------------|----------------------------------------------------------------|
| `default`   | `pixi shell`          | Bench sim: description, `ros2_control`, Gazebo, the `ros_gz` bridges |
| `full`      | `pixi shell -e full`  | `default` + MoveIt 2 (`moveit`, `pick_ik`) + camera drivers (`realsense2_camera`, `usb_cam`, image transports) |

The split mirrors `docker/Dockerfile`: the bench sim never plans or looks through a
camera, so MoveIt and the RealSense stack are opt-in. Run planning demos or
`mod101_hw_bringup` from `-e full`.

Prefix any task with `-e full` too: `pixi run -e full build`.

## Tasks

Defined in `pixi.toml` under `[tasks]`:

| Task                  | Command                                                             |
|-----------------------|--------------------------------------------------------------------|
| `pixi run build`      | `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` |
| `pixi run test`       | `colcon test && colcon test-result --verbose` (builds first)        |
| `pixi run clean`      | `rm -rf build install log`                                          |
| `pixi run sim`        | `ros2 launch mod101_harness sim.launch.py`                          |
| `pixi run configurator` | `python configurator/server.py` - the web configurator on :8001   |

Anything not listed still works inside `pixi shell` / `pixi run --`:

```bash
pixi run -- ros2 launch mod101_moveit_config demo.launch.py     # needs -e full
pixi shell -e full
```

## How the workspace overlay works

`pixi.toml` registers `pixi/activate.sh` as an activation script. Every time you
`pixi shell` or `pixi run`, it sources `install/setup.sh` **if it exists**. So:

- Before the first `pixi run build` it's a no-op - the environment still works,
  you just can't `ros2 run` the mod101 packages yet.
- After a build, the overlay is live automatically. No manual `source install/setup.bash`.

Re-run `pixi run build` after changing any `CMakeLists.txt`, `package.xml`, or C++
source. `--symlink-install` means Python and launch/config edits are picked up
without a rebuild.

## Gotchas

- **Don't mix with system ROS.** Never `source /opt/ros/jazzy/setup.bash` inside a
  pixi shell - the combined `PYTHONPATH` breaks both. Close other ROS terminals
  *before* `pixi shell`. (The `configurator` section of the README tells you to
  source `/opt/ros`; use `pixi run configurator` instead when you're on pixi.)
- **Gazebo needs a GPU + display.** Same as the Docker stack: no headless mode. On
  a machine with an NVIDIA card make sure `glxinfo` works; if the Gazebo window
  won't open over X, run `xhost +local:` once.
- **`ROS_DOMAIN_ID`.** The Docker bench isolates itself on domain 1 with
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. pixi sets nothing - export those
  yourself if you're on a network with a real robot on the ROS graph.
- **Lockfile.** `pixi.lock` pins exact package versions and is committed. Commit it
  when you change dependencies so everyone resolves the same graph. `pixi install`
  regenerates it if `pixi.toml` is newer.

## Packages not on RoboStack

These are pulled from GitHub, not conda - clone them into `src/` as the main
README describes, then `pixi run build`:

- `st3215_manager`, `ros2_control_bridge` - the Feetech servo drivers (`mod101_hw_bringup`)
- `pinc_open_driver` - only for the PincOpen tool on real hardware (`mod101_tool_pincopen`)
- the robocore agent - ships as its own container image; not built here

## Platforms

`pixi.toml` lists `linux-64` only. To build on Apple Silicon or an ARM SBC, add the
platform and re-solve:

```bash
pixi workspace platform add osx-arm64      # or linux-aarch64
pixi install
```

RoboStack publishes Jazzy for all three. Gazebo GUI support on macOS is limited -
expect to use it for description/MoveIt work, not the full bench sim.
