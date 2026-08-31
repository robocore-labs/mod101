#!/usr/bin/env python3
"""The mod101 harness on real hardware.

    ros2 launch mod101_hw_bringup bringup.launch.py

Four things, in one graph:

    st3215_manager      the Feetech bus. Owns the serial port, converts
                        ticks <-> radians, publishes /motor_manager/joint_states
    ros2_control_node   the controllers, over ros2_control_bridge/TopicBridge,
                        which turns controller commands into the topics above
    robot_state_publisher   TF from the same URDF the sim uses
    cameras             RealSense + wrist camera (cameras:=false to skip)

WHY THE BUS IS A SEPARATE NODE FROM ros2_control. A SystemInterface's read() and
write() run inside controller_manager's update loop, so a blocking serial read
there stalls every controller on the robot — and eight servos over one 1 Mbaud
line is milliseconds per cycle, not microseconds. Splitting them puts the serial
latency on a topic instead of in the control loop, at the cost of a hop.

THREE FILES MAKE UP THE SERVO CONFIG, and which one owns what is the whole
design:

  config/servos.yaml            authored. Bus port, rates, which groups exist,
                                speeds, accelerations, safety flags.
  config/servos.generated.yaml  written by the configurator. Motor IDs, joint
                                names, directions, array indices — the half
                                that is discovered by scanning the bus.
  mod101_control/config/calibration.yaml
                                written by the configurator. Home tick and
                                travel per joint.

This launch loads the first, overlays the second, injects the third, and hands
the result to the driver as parameters. So calibrating and then launching is
enough to get a working arm, and no generator ever rewrites a commented file.

Without the calibration file the driver runs uncalibrated — home at mid-scale,
travel the full encoder range — which is safe on a bare motor and wrong on an
assembled arm, so it is a loud warning, not a silent default.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

# Passed straight through to the description, exactly as in sim.launch.py.
BUILD_ARGS = ('tool', 'shoulder_ext_length', 'elbow_ext_length',
              'shoulder_mount', 'elbow_mount')

# Spawned at boot. The trajectory twins stay declared-not-spawned: they claim
# the same joints, and controller_manager allows one active claim per joint.
BOOT_CONTROLLERS = ('arm_controller', 'pan_tilt_controller', 'gripper_controller')

# Dropped along with the head when head:=false — it claims joints that are no
# longer in the ros2_control block, and a spawner for those just fails.
HEAD_CONTROLLERS = ('pan_tilt_controller',)
HEAD_GROUP = 'head'


def _drop_unset(mappings):
    return {k: v for k, v in mappings.items() if v != ''}


def _load_calibration(path, logger_msgs):
    """joint name -> (home, min, max) in ticks, from the configurator's file.

    Returns {} when there is no file. The generated YAML is a ROS parameter
    file (`mod101_calibration: ros__parameters: <joint>: ...`), so it is read
    for its shape rather than passed to a node.
    """
    if not os.path.exists(path):
        logger_msgs.append(
            f'NO CALIBRATION at {path} — servos will run with home at mid-scale '
            f'and travel over the full encoder range. Every joint angle will be '
            f'offset by however far its servo horn is from centre. Run the '
            f'configurator (Calibrate -> Write to repo) before trusting motion.')
        return {}
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    params = (doc.get('mod101_calibration') or {}).get('ros__parameters') or {}
    out = {}
    for joint, v in params.items():
        try:
            out[str(joint)] = (int(v['ticks_home']), int(v['ticks_min']), int(v['ticks_max']))
        except (KeyError, TypeError, ValueError):
            logger_msgs.append(f'calibration for "{joint}" is incomplete — skipped')
    logger_msgs.append(f'calibration: {len(out)} joint(s) from {path}')
    return out


def _merge_generated(params, generated_path, logger_msgs):
    """Overlay the configurator's wiring map onto the authored config.

    servos.yaml is hand-authored — port, rates, which groups exist, speeds,
    accelerations, safety flags. servos.generated.yaml is written by
    configurator/calibrate.html and carries only the half that comes off the
    bus: motor IDs, joint names, directions and the array indices. Merging at
    launch is what keeps a generator from having to rewrite a commented file.

    Only the keys the generator owns are replaced. Anything else in a group
    block stays as authored, so adding a tuning parameter to servos.yaml does
    not require the generator to know about it.
    """
    if not os.path.exists(generated_path):
        logger_msgs.append(
            f'no {os.path.basename(generated_path)} — motor IDs and directions '
            f'are whatever servos.yaml says. Write one from the configurator '
            f'(Calibrate -> Write to repo) so the bus map comes from the bus.')
        return params
    with open(generated_path) as f:
        doc = yaml.safe_load(f) or {}
    gen = (doc.get('servo_manager_node') or {}).get('ros__parameters') or {}
    owned = ('motor_ids', 'joint_names', 'direction',
             'command_indices', 'state_indices')
    touched = []
    for group, block in gen.items():
        if not isinstance(block, dict):
            continue
        if group not in params:
            logger_msgs.append(
                f'generated config has group "{group}", which servos.yaml does '
                f'not declare — ignored')
            continue
        for key in owned:
            if key in block:
                params[group][key] = block[key]
        touched.append(group)
    if touched:
        logger_msgs.append(f'wiring from {os.path.basename(generated_path)}: '
                           f'{", ".join(touched)}')
    return params


def _drop_head(params, logger_msgs):
    """Take the head out of the servo config entirely.

    Not just `enable: false`: `group_names` is what the driver iterates, and
    `require_motors` then still counts the head's motors and refuses to start
    when they do not answer. The head is harness hardware, fitted and
    calibrated on its own schedule from the arm — the configurator does not
    even know about it — so a bring-up without it has to be a first-class
    option rather than a config edit.
    """
    if HEAD_GROUP not in params.get('group_names', []):
        return params
    params['group_names'] = [g for g in params['group_names'] if g != HEAD_GROUP]
    head = params.pop(HEAD_GROUP, None)
    n = len(head.get('motor_ids', [])) if isinstance(head, dict) else 0
    logger_msgs.append(
        f'head:=false — group "{HEAD_GROUP}" ({n} motor(s)) dropped, and its '
        f'joints are out of the ros2_control block. The arm runs on its own.')
    return params


def _servo_params(servos_path, generated_path, calib, logger_msgs, head=True):
    """The servo config: authored, overlaid with the generated wiring map, then
    with calibration merged into each group.

    Returned as a dict rather than a file path because the arrays are assembled
    here; a node given both a file and overrides would take the file's values
    for anything the overrides missed, which is the failure mode this avoids.
    """
    with open(servos_path) as f:
        doc = yaml.safe_load(f)
    params = doc['servo_manager_node']['ros__parameters']
    params = _merge_generated(params, generated_path, logger_msgs)
    if not head:
        params = _drop_head(params, logger_msgs)

    ticks_per_rev = int(params.get('ticks_per_rev', 4096))
    for group in params.get('group_names', []):
        g = params.get(group)
        if not isinstance(g, dict):
            continue
        joints = g.get('joint_names', [])
        home, lo, hi = [], [], []
        missing = []
        for j in joints:
            if j in calib:
                h, a, b = calib[j]
            else:
                # Uncalibrated joints keep the driver's own fallback rather
                # than borrowing a neighbour's numbers.
                h, a, b = ticks_per_rev // 2, 0, ticks_per_rev - 1
                missing.append(j)
            home.append(h)
            lo.append(a)
            hi.append(b)
        g['home_ticks'], g['min_ticks'], g['max_ticks'] = home, lo, hi
        if missing and calib:
            logger_msgs.append(
                f'group "{group}": no calibration for {", ".join(missing)} — '
                f'those joints run uncalibrated')
    return params


def _build(context):
    msgs = []

    pkg_bringup = get_package_share_directory('mod101_hw_bringup')
    pkg_harness = get_package_share_directory('mod101_harness')
    pkg_control = get_package_share_directory('mod101_control')

    lc = {k: LaunchConfiguration(k).perform(context)
          for k in (*BUILD_ARGS, 'calibration_file', 'servos_file',
                    'controllers_file', 'generated_servos_file',
                    'serial_port', 'cameras', 'hardware', 'head', 'bus')}

    # --- the description -----------------------------------------------------
    # use_sim=false picks the hardware ros2_control block; `hardware` picks the
    # plugin inside it. hardware:=mock is the dry run — the whole graph comes
    # up, the servo driver still opens the bus, but commands loop back as state
    # instead of reaching a motor.
    urdf = os.path.join(pkg_harness, 'urdf', 'mod101_harness_arm.xacro')
    robot_description = xacro.process_file(
        urdf,
        mappings={**_drop_unset({k: lc[k] for k in BUILD_ARGS}),
                  'use_sim': 'false',
                  'head': lc['head'],
                  'hardware': lc['hardware']}).toxml()

    # --- servo bus -----------------------------------------------------------
    calib_path = lc['calibration_file'] or os.path.join(
        pkg_control, 'config', 'calibration.yaml')
    servos_path = lc['servos_file'] or os.path.join(
        pkg_bringup, 'config', 'servos.yaml')
    generated_path = lc['generated_servos_file'] or os.path.join(
        pkg_bringup, 'config', 'servos.generated.yaml')

    head_on = lc['head'].lower() not in ('false', '0', 'no')
    servo_params = _servo_params(servos_path, generated_path,
                                 _load_calibration(calib_path, msgs), msgs,
                                 head=head_on)
    if lc['serial_port']:
        servo_params['port'] = lc['serial_port']
    servo_params['use_sim_time'] = False

    # bus:=false is the only genuinely dry run. hardware:=mock swaps the
    # ros2_control plugin, but the servo driver is a separate node and starts
    # regardless — and st3215_manager ARMS every position motor at init
    # (StartServo + MoveTo its own current position, to hold where it is). That
    # is the safe form of energising, but it is still energising, so "nothing
    # moves" has to mean "the driver does not run".
    bus_on = lc['bus'].lower() not in ('false', '0', 'no')
    if not bus_on:
        msgs.append('bus:=false — the servo driver is NOT started. Nothing is '
                    'energised and no joint states are published, so the bridge '
                    'will sit waiting to be seeded. This checks the URDF, the '
                    'plugin and the controllers, and nothing else.')

    servo_manager = Node(
        package='st3215_manager',
        executable='servo_manager_node',
        name='servo_manager_node',
        parameters=[servo_params],
        output='screen',
        emulate_tty=True,
        # The bus is the one thing whose loss takes the robot with it, and the
        # usual cause is a USB re-enumeration that fixes itself.
        respawn=True,
        respawn_delay=2.0,
    )

    # --- ros2_control --------------------------------------------------------
    controllers = lc['controllers_file'] or os.path.join(
        pkg_bringup, 'config', 'controllers.yaml')

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers],
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
    )

    def spawner(name, delay):
        return TimerAction(period=delay, actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60'],
            output='screen',
        )])

    # The bridge holds its commands until every joint it owns has reported a
    # position, so the controllers must not be spawned before the servo driver
    # has published a joint state — a controller activating against unseeded
    # commands is the moment an arm would otherwise jump to zero.
    boot = [c for c in BOOT_CONTROLLERS if head_on or c not in HEAD_CONTROLLERS]
    spawners = [spawner('joint_state_broadcaster', 4.0)]
    spawners += [spawner(c, 6.0) for c in boot]

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'cameras.launch.py')),
        condition=IfCondition(lc['cameras']),
    )

    return [*[LogInfo(msg=f'[mod101_hw_bringup] {m}') for m in msgs],
            rsp, *([servo_manager] if bus_on else []),
            control_node, *spawners, cameras]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value='') for k in BUILD_ARGS],
        DeclareLaunchArgument(
            'hardware', default_value='bridge',
            description='ros2_control plugin: "bridge" drives the real servo '
                        'bus; "mock" loops commands back as state for a dry run.'),
        DeclareLaunchArgument(
            'cameras', default_value='true',
            description='Start the RealSense and wrist cameras.'),
        DeclareLaunchArgument(
            'head', default_value='true',
            description='Include the harness pan/tilt. false drops the head '
                        'group, its controller, and its joints from the '
                        'ros2_control block — for an arm on the bus without '
                        'the harness servos.'),
        DeclareLaunchArgument(
            'bus', default_value='true',
            description='Start the servo driver. false leaves the chain '
                        'untouched and unpowered — the real dry run, since the '
                        'driver arms every motor to hold position at init.'),
        DeclareLaunchArgument(
            'serial_port', default_value='',
            description='Override the Feetech bus device from servos.yaml.'),
        DeclareLaunchArgument(
            'calibration_file', default_value='',
            description='Override mod101_control/config/calibration.yaml.'),
        DeclareLaunchArgument(
            'servos_file', default_value='',
            description='Override this package\'s config/servos.yaml.'),
        DeclareLaunchArgument(
            'generated_servos_file', default_value='',
            description='Override config/servos.generated.yaml, the wiring map '
                        'the configurator writes.'),
        DeclareLaunchArgument(
            'controllers_file', default_value='',
            description='Override this package\'s config/controllers.yaml.'),
        OpaqueFunction(function=_build),
    ])
