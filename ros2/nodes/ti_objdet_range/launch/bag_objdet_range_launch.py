import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    exportPerfStats_arg = DeclareLaunchArgument('exportPerfStats', default_value='0')
    default_bagfile = os.path.join(os.environ['TI_DATA_PATH'],'ros_bag','zed1_2020-11-09-18-01-08')
    bagfile_arg = DeclareLaunchArgument('bagfile', default_value=TextSubstitution(text=default_bagfile))
    ratefactor_arg = DeclareLaunchArgument('ratefactor', default_value='1.0')

    # include objdet_range_launch
    objdet_range_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ti_objdet_range'),'/launch/objdet_range_launch.py']),
        launch_arguments={
            'zed_sn': 'SN5867575',
            'exportPerfStats': LaunchConfiguration('exportPerfStats'),
        }.items()
    )

    # include rosbag launch file
    launch_file_path = PathJoinSubstitution([FindPackageShare('ti_sde'),'launch','rosbag_launch.py'])
    rosbag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path),
        launch_arguments={
            'bagfile': LaunchConfiguration('bagfile'),
            'ratefactor': LaunchConfiguration('ratefactor'),
        }.items()
    )

    base_link_to_right_frame_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.125', '-0.06', '-0.064', '-1.5707963267948966', '0.078539816339745', '-1.5707963267948966', 'base_link', 'right_frame'],
        name='base_link_to_right_frame'
    )

    base_link_to_camera_horiz_scan_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.125', '-0.06', '-0.064', '0', '0.078539816339745', '0', 'base_link', 'camera_horiz_scan'],
        name='base_link_to_camera_horiz_scan'
    )

    ld = LaunchDescription([
        exportPerfStats_arg,
        bagfile_arg,
        ratefactor_arg,
        objdet_range_launch,
        rosbag_launch,
        base_link_to_right_frame_node,
        base_link_to_camera_horiz_scan_node,
    ])

    return ld
