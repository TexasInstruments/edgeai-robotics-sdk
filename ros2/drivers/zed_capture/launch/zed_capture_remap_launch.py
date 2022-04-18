import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

# Launch for for ZED capture node.
# Requirement: run <zed_capture>/script/gen_rect_map.py with factory calibration
# params file as input for generating camera_info .yaml files and
# undistortion/rectification look-up table files for LDC hardware accelerator

def finalize_node(context, *args, **kwargs):
    zed_sn_str  = LaunchConfiguration("zed_sn_str").perform(context)
    camera_mode = LaunchConfiguration("camera_mode").perform(context)
    camera_info_base = "/opt/robotics_sdk/ros1/drivers/zed_capture/config"
    camera_info_left_yaml  = "file://" + os.path.join(camera_info_base, zed_sn_str+"_"+camera_mode+"_camera_info_left.yaml")
    camera_info_right_yaml = "file://" + os.path.join(camera_info_base, zed_sn_str+"_"+camera_mode+"_camera_info_right.yaml")

    topic_ns_left           = LaunchConfiguration("topic_ns_left").perform(context)
    topic_ns_right          = LaunchConfiguration("topic_ns_right").perform(context)
    image_topic_left        = topic_ns_left  + "/image_raw"
    image_topic_right       = topic_ns_right + "/image_raw"
    camera_info_topic_left  = topic_ns_left  + "/camera_info"
    camera_info_topic_right = topic_ns_right + "/camera_info"

    params = [
        {
            "frame_id_left":           "left_frame",
            "frame_id_right":          "right_frame",
            "camera_mode":             LaunchConfiguration("camera_mode"),
            "frame_rate":              LaunchConfiguration("frame_rate"),
            "device_name":             LaunchConfiguration("device_name"),
            "encoding":                LaunchConfiguration("encoding"),
            "image_topic_left":        image_topic_left,
            "image_topic_right":       image_topic_right,
            "camera_info_topic_left":  camera_info_topic_left,
            "camera_info_topic_right": camera_info_topic_right,
            "camera_info_left_yaml":   camera_info_left_yaml,
            "camera_info_right_yaml":  camera_info_right_yaml,
        }
    ]

    node = Node(
        package     = "zed_capture",
        executable  = "zed_capture",
        name        = "zed_capture",
        output      = "screen",
        parameters  = params,
        remappings = [
            ('/camera/right/image_raw', '/camera/image_raw'),
            ('/camera/right/camera_info', '/camera/camera_info'),
        ]
    )

    return [node]


def generate_launch_description():
    ld = LaunchDescription()

    # zed_sn_str: arg that can be set from the command line or a default will be used
    zed_sn_str = DeclareLaunchArgument(
        name="zed_sn_str",
        default_value=TextSubstitution(text="SN18059"),
        description='string for ZED camera serial number'
    )

    # device_name: arg that can be set from the command line or a default will be used
    device_name = DeclareLaunchArgument(
        name="device_name",
        default_value=TextSubstitution(text="/dev/video2"),
        description='''to find your device name, use ls /dev/video*
                       and look for the name begin with video'''
    )

    # camera_mode: arg that can be set from the command line or a default will be used
    camera_mode = DeclareLaunchArgument(
        name="camera_mode",
        default_value=TextSubstitution(text="HD"),
        description="""camera_mode: '2K' - 2208x1242,
                      'FHD' - 1920x1080, 'HD' - 1280x720,
                      'HD2' - 1280x720: center-cropped from 1080p"""
    )

    # frame_rate: arg that can be set from the command line or a default will be used
    frame_rate = DeclareLaunchArgument(
        name="frame_rate",
        default_value=TextSubstitution(text="15.0"),
        description='Frame rate'
    )

    # encoding: arg that can be set from the command line or a default will be used
    encoding = DeclareLaunchArgument(
        name="encoding",
        default_value=TextSubstitution(text="yuv422"),
        description='''encoding: "yuv422" (default, output in YUV422:UYVY format), "bgr8"'''
    )

    # topic_ns_left: arg that can be set from the command line or a default will be used
    topic_ns_left = DeclareLaunchArgument(
        name="topic_ns_left",
        default_value=TextSubstitution(text="camera/left"),
        description='''topic_ns_left: topic namespace (left)'''
    )

    # topic_ns_right: arg that can be set from the command line or a default will be used
    topic_ns_right = DeclareLaunchArgument(
        name="topic_ns_right",
        default_value=TextSubstitution(text="camera/right"),
        description='''topic_ns_right: topic namespace (right)'''
    )

    ld.add_action(zed_sn_str)
    ld.add_action(device_name)
    ld.add_action(camera_mode)
    ld.add_action(frame_rate)
    ld.add_action(encoding)
    ld.add_action(topic_ns_left)
    ld.add_action(topic_ns_right)
    ld.add_action(OpaqueFunction(function=finalize_node))

    return ld

