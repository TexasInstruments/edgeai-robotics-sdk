import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

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
        "zed_sn",
        default_value=TextSubstitution(text="SN18059")
    )

    # Flag for exporting the performance data to a file: 0 - disable, 1 - enable
    exportPerfStats_arg = DeclareLaunchArgument(
        "exportPerfStats",
        default_value=TextSubstitution(text="0")
    )

    # Include SDE launch file
    sde_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_launch_file('ti_sde', 'sde_launch.py')),
        launch_arguments={
            "zed_sn": LaunchConfiguration('zed_sn'),
            "enable_pc": "0",
            "exportPerfStats": LaunchConfiguration('exportPerfStats'),
        }.items()
    )

    # Include ZED launch file
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_launch_file('zed_capture', 'zed_capture_launch.py')),
        launch_arguments={
            "cam_id": LaunchConfiguration('cam_id'),
            "zed_sn_str": LaunchConfiguration('zed_sn'),
        }.items()
    )

    return LaunchDescription([
        cam_id_arg,
        zed_sn_arg,
        exportPerfStats_arg,
        sde_launch,
        zed_launch,
    ])