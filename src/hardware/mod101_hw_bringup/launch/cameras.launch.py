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
            # 'link', WHICH IS THE DEFAULT, AND IT IS A SUFFIX NOT A FRAME.
            # realsense2_camera builds every frame it publishes as
            # <camera_name>_<base_frame_id>, and camera_name defaults to
            # 'camera'. This used to say 'hn_realsense_1' on the assumption it
            # named the frame to hang the camera off; what it actually did was
            # rename the camera's own root to `camera_hn_realsense_1` -- a
            # frame with a URDF-looking name that is in no URDF, still rooted
            # in its own private tree, and confusing precisely because it
            # looks like it worked. Verified with `ros2 param get camera_name`
            # against the running node.
            #
            # Back to 'link' so the root is the conventional `camera_link`,
            # which is what the static transform below attaches to the robot.
            'base_frame_id': 'link',
            'publish_tf': True,
            'tf_publish_rate': 0.0,   # static; the head's motion comes from TF

            # ---- COMPRESSED TRANSPORTS ------------------------------------
            # These are image_transport's parameters, not realsense2_camera's.
            # Any node publishing through image_transport advertises
            # <topic>/compressed, /compressedDepth and /theora alongside the
            # raw stream, and declares the knobs below on itself. They were
            # already there at their defaults; setting them here makes the
            # choice deliberate and reviewable instead of inherited.
            #
            # NOTHING IS ENCODED UNTIL SOMEONE SUBSCRIBES. image_transport's
            # publishers are lazy, so this costs no CPU on a bench with no
            # viewer attached, and costs it only for the transports actually
            # in use.
            #
            # WHY IT MATTERS HERE: raw colour at 640x480x3 at 30 fps is about
            # 27 MB/s per subscriber. The compose stack runs network_mode:
            # host with ipc: host, so that traffic is real shared-memory and
            # real bandwidth the servo bus's process is competing for. JPEG at
            # quality 80 is roughly a twentieth of it.
            #
            # THE NAMES EMBED THE NODE NAME AND ARE RELATIVE TO ITS NAMESPACE:
            # image_transport builds them from the topic's fully-qualified
            # name with '/' -> '.', minus the node's namespace, which is what
            # produces that leading dot. Rename this node from `head_camera`
            # and these silently stop matching anything — the parameters are
            # simply never read, with no warning. Verified against the running
            # node with `ros2 param get` rather than guessed.
            #
            # Colour: 95 is near-lossless and most of the bandwidth back. 80
            # is the usual robot-camera trade — artefacts invisible at a
            # glance, a third of the bytes of 95.
            '.head_camera.color.image_raw.compressed.format': 'jpeg',
            '.head_camera.color.image_raw.compressed.jpeg_quality': 80,
            #
            # DEPTH IS NOT JPEG, and this is the trap worth spelling out.
            # Aligned depth is 16UC1 millimetres; a lossy codec on it does not
            # blur an image, it invents distances. `compressedDepth` is the
            # transport that understands that — PNG on the 16-bit data, which
            # is lossless. png_level 1 over the default 3: at 30 fps the
            # encoder is the cost, not the wire, and level 1 is materially
            # faster for a few percent more bytes.
            #
            # depth_max stays at its 10.0 m default deliberately. Lowering it
            # to the bench's ~1 m workspace WOULD buy quantisation resolution,
            # but it also silently clips everything beyond it, and a depth
            # image that reads "nothing there" past 4 m is worse than a
            # coarser one that is honest.
            '.head_camera.aligned_depth_to_color.image_raw.compressedDepth.png_level': 1,
            '.head_camera.aligned_depth_to_color.image_raw.compressedDepth.depth_max': 10.0,
            #
            # theora left at its default. Nothing in this stack subscribes to
            # it, and a lazy publisher nobody subscribes to encodes nothing.
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

    # ATTACHES THE CAMERA TO THE ROBOT. Without this the RealSense is invisible
    # to TF: the driver publishes its own camera_link -> camera_*_optical_frame
    # tree, nothing joins it to the URDF, and tf2 reports "two or more
    # unconnected trees". Images still stream perfectly, which is what makes it
    # easy to miss — what fails is everything that turns a pixel into a place.
    # deproject(), get_cloud(), and pointing the head at something the camera
    # saw all need a path from camera_color_optical_frame out to world, and
    # there was none.
    #
    # `base_frame_id` above does NOT do this, and the first attempt at this
    # fix failed on exactly that: that parameter is a SUFFIX on the camera's
    # own root frame name, not a pointer at a frame to attach to. Setting it
    # to hn_realsense_1 produced a root called `camera_hn_realsense_1` sitting
    # in its own tree, while this transform published a childless `camera_link`
    # in the robot's -- two trees, one more frame, still disconnected.
    #
    # SAME POINT, DIFFERENT AXES — and this was identity until 2026-09-01,
    # which was wrong in the half nobody checked. harness_body.xacro does
    # place hn_realsense_1 at the camera's mounting position, so the
    # TRANSLATION is genuinely zero. The ROTATION is not:
    #
    #   hn_realsense_1  is a URDF body link, and harness_body.xacro's own
    #                   optical-frame note says "the lens looks along
    #                   realsense_1 -Y", proven there from the mesh faces.
    #   camera_link     is the RealSense convention, +X out of the lens.
    #
    # +X and -Y are a quarter turn apart, so identity left every optical
    # frame rotated 90 degrees about Z. Measured on the running robot
    # before the fix: world <- camera_color_optical_frame had its +Z, the
    # view direction, pointing along world +X — dead horizontal — while
    # the URDF's hn_realsense_optical_frame correctly pointed down and -Y
    # with the head tilted. 89.7 degrees of error.
    #
    # WHAT IT BROKE, and why it was invisible: images stream perfectly and
    # TF resolves, so nothing errors. deproject() simply returns points
    # rotated a quarter turn — objects on the bench came back 2-3 m away
    # along +X, which reads as "out of the arm's reach" rather than as a
    # frame bug.
    #
    # THE SIM ALREADY KNEW. harness.gazebo.xacro poses its rgbd sensor
    # <pose>0 0 0 0 0 -1.570796</pose> with a comment calling the yaw
    # "load-bearing", for this exact reason. The hardware transform is the
    # same quarter turn; it just never got it.
    #
    # ARGUMENT ORDER IS x y z YAW PITCH ROLL, not roll-pitch-yaw. The
    # positional form of static_transform_publisher takes the angles
    # yaw-first, so -1.5707963 below is yaw.
    #
    # Conditioned on the same argument as the driver, so `head_camera:=false`
    # does not leave a transform pointing at a frame nobody publishes.
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='head_camera_mount_tf',
        condition=IfCondition(head),
        arguments=['0', '0', '0', '-1.5707963', '0', '0',
                   'hn_realsense_1', 'camera_link'],
        output='screen',
    )

    return LaunchDescription([*args, realsense, camera_tf, wrist_cam])
