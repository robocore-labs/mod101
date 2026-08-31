#!/usr/bin/env python3
"""The harness's two cameras: the RealSense on the pan/tilt head, and the USB
wrist camera.

Ported from llmy_camera, with the parts that did nothing removed. That package
declared eight launch arguments for a depth-to-laserscan conversion and an
`enable_compressed` transport, and then returned a LaunchDescription containing
neither node — so `enable_laser_scan:=true` was accepted and ignored. It also
hard-coded `camera_namespace` to 'head_camera' while taking a `camera_name`
argument, so renaming the camera renamed half of its topics.

TOPICS ARE REMAPPED ONTO THE NAMES THE SIM USES. profiles/mod101_harness.yaml
names four head topics and two wrist topics, and those names came from the
Gazebo bridge. Publishing the driver's native names instead
(/head_camera/head_camera/color/image_raw ...) would mean the profile — and
anything written against it — needs a hardware variant and a sim variant that
drift apart. The remappings below are what keep one profile honest on both.

The RealSense node is run directly rather than through realsense2_camera's
rs_launch.py, because remapping a node included from another launch file means
matching that file's own argument names, which change between releases.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The sim's topic names (harness/config/gz_bridge.yaml), which the profile
# reads. Kept next to each other so the two halves cannot drift apart quietly.
HEAD_NS = '/harness/depth_camera'
WRIST_NS = '/wrist_camera'


def generate_launch_description():
    head = LaunchConfiguration('head_camera')
    wrist = LaunchConfiguration('wrist_camera')
    wrist_device = LaunchConfiguration('wrist_camera_device')
    pointcloud = LaunchConfiguration('pointcloud')
    serial_no = LaunchConfiguration('head_camera_serial')

    args = [
        DeclareLaunchArgument(
            'head_camera', default_value='true',
            description='Start the RealSense on the pan/tilt head.'),
        DeclareLaunchArgument(
            'wrist_camera', default_value='true',
            description='Start the USB wrist camera.'),
        DeclareLaunchArgument(
            'wrist_camera_device', default_value='/dev/video0',
            description='V4L2 device for the wrist camera. Prefer a stable '
                        '/dev/v4l/by-id/... path: /dev/video0 is whichever '
                        'camera enumerated first, which changes with the '
                        'RealSense plugged in.'),
        DeclareLaunchArgument(
            'head_camera_serial', default_value='',
            description='RealSense serial number. Only needed with more than '
                        'one RealSense attached; empty takes the first found.'),
        DeclareLaunchArgument(
            'pointcloud', default_value='false',
            description='Publish the registered point cloud. Off by default — '
                        'it is the most expensive stream the D435 produces, '
                        'and the profile only needs it for deproject().'),
    ]

    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='head_camera',
        condition=IfCondition(head),
        parameters=[{
            'serial_no': serial_no,
            # Matches the sim's 640x480x30 (harness.gazebo.xacro), so image
            # geometry and camera_info are the same shape either way.
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'enable_depth': True,
            'enable_color': True,
            # The IR pair is not modelled in sim and nothing consumes it.
            'enable_infra1': False,
            'enable_infra2': False,
            # Depth registered into the colour frame. Without this, depth and
            # colour pixels do not correspond and deproject() reads the wrong
            # distance for a pixel it located in the RGB image.
            'align_depth.enable': True,
            'pointcloud.enable': pointcloud,
            'enable_sync': True,
            # Hang the camera's frames off the URDF link the harness declares,
            # so TF from the arm to the camera is the model's, not the driver's
            # invention. The driver still stamps images with its own optical
            # frames (head_camera_color_optical_frame and friends) — if you
            # deproject against the profile, its `frame:` for the head camera
            # has to name that frame on hardware.
            'base_frame_id': 'hn_realsense_1',
            'publish_tf': True,
            'tf_publish_rate': 0.0,   # static; the head's motion comes from TF
        }],
        remappings=[
            ('/head_camera/color/image_raw', f'{HEAD_NS}/image'),
            ('/head_camera/color/camera_info', f'{HEAD_NS}/camera_info'),
            ('/head_camera/aligned_depth_to_color/image_raw', f'{HEAD_NS}/depth_image'),
            ('/head_camera/depth/color/points', f'{HEAD_NS}/points'),
        ],
        output='screen',
    )

    wrist_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='wrist_camera',
        namespace=WRIST_NS.lstrip('/'),
        condition=IfCondition(wrist),
        parameters=[{
            'video_device': wrist_device,
            'image_width': 640,
            'image_height': 480,
            # MJPEG off the wire, decoded on the host: the same camera at
            # 640x480x30 in raw YUYV is more USB bandwidth than a shared bus
            # has to spare next to a RealSense.
            'pixel_format': 'mjpeg2rgb',
            'framerate': 30.0,
            'camera_name': 'wrist_camera',
            # The URDF's optical frame for this camera, so images are stamped
            # with a frame TF actually knows (mod101_macro.xacro declares it).
            'camera_frame_id': 'wrist_camera_optical_frame',
            'io_method': 'mmap',
        }],
        output='screen',
    )

    return LaunchDescription([*args, realsense, wrist_cam])
