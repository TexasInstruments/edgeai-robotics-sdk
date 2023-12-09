import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_launch_file(pkg, file_name):
    pkg_dir = get_package_share_directory(pkg)
    return os.path.join(pkg_dir, 'launch', file_name)

def generate_launch_description():
    # declare arguments
    cam_id_arg = DeclareLaunchArgument('cam_id', default_value='0')
    zed_sn_arg = DeclareLaunchArgument('zed_sn', default_value='SN18059')
    exportPerfStats_arg = DeclareLaunchArgument('exportPerfStats', default_value='0')

    # include objdet_range_launch
    objdet_range_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ti_objdet_range'), '/launch/objdet_range_launch.py']),
        launch_arguments={
            'zed_sn': LaunchConfiguration('zed_sn'),
            'exportPerfStats': LaunchConfiguration('exportPerfStats'),
        }.items()
    )

    # include ZED launch file
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_launch_file('zed_capture', 'zed_capture_launch.py')),
        launch_arguments={
            "cam_id": LaunchConfiguration('cam_id'),
            "zed_sn_str": LaunchConfiguration('zed_sn'),
        }.items()
    )

    ld = LaunchDescription([
        cam_id_arg,
        zed_sn_arg,
        exportPerfStats_arg,
        objdet_range_launch,
        zed_launch,
    ])

    return ld
