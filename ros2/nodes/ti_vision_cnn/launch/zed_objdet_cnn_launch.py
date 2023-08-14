import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

# Folder that contains camera params
config_dir = "/opt/robotics_sdk/ros1/drivers/zed_capture/config/"

def get_launch_file(pkg, file_name):
    pkg_dir = get_package_share_directory(pkg)
    return os.path.join(pkg_dir, 'launch', file_name)

def generate_launch_description():
    # cam_id: when the camera is recognized as /dev/video-usb-camX, set cam_id = X
    cam_id_arg = DeclareLaunchArgument(
        'cam_id',
        default_value='0'
    )

    # ZED camera serial number
    zed_sn_arg = DeclareLaunchArgument(
        "zed_sn", default_value=TextSubstitution(text="SN18059")
    )

    exportPerfStats_arg = DeclareLaunchArgument(
        "exportPerfStats", default_value=TextSubstitution(text="0")
    )

    detVizThreshold_arg = DeclareLaunchArgument(
        "detVizThreshold", default_value=TextSubstitution(text="0.5")
    )

    # ref: https://answers.ros.org/question/384712/ros2-launch-how-to-concatenate-launchconfiguration-with-string/?answer=384740
    lut_file_path_arg = DeclareLaunchArgument(
        "lut_file_path", default_value=[config_dir, LaunchConfiguration('zed_sn'), "_HD_LUT_right.bin"]
    )

    # Include OBJDET launch file
    cnn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_launch_file('ti_vision_cnn', 'objdet_cnn_launch.py')),
        launch_arguments={
            "zed_sn": LaunchConfiguration('zed_sn'),
            'lut_file_path': LaunchConfiguration('lut_file_path')
        }.items()
    )

    # Include ZED launch file
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_launch_file('zed_capture', 'zed_capture_launch.py')),
        launch_arguments={
            "cam_id": LaunchConfiguration('cam_id'),
            "zed_sn_str": LaunchConfiguration('zed_sn'),
            'topic_ns_right': 'camera',
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(cam_id_arg)
    ld.add_action(zed_sn_arg)
    ld.add_action(exportPerfStats_arg)
    ld.add_action(detVizThreshold_arg)
    ld.add_action(lut_file_path_arg)
    ld.add_action(cnn_launch)
    ld.add_action(zed_launch)

    return ld

